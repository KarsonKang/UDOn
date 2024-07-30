
#include "rclcpp/rclcpp.hpp"

#include "dynamixel_sdk_custom_interfaces/msg/grab_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/target_position.hpp"


class pos_pub_node : public rclcpp::Node{

public:
    pos_pub_node() : Node("pos_pub_node"){

        cam_subscriber_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::TargetPosition>(
            "target_position", 5,
            std::bind(&pos_pub_node::subscrib_pos_from_cam, this, std::placeholders::_1)
            );

        pos_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::GrabPosition>("grab_position", 5);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&pos_pub_node::publish_position, this));

        RCLCPP_INFO(this->get_logger(), "Ready to publish the position");
    };

private:
    rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::TargetPosition>::SharedPtr cam_subscriber_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::GrabPosition>::SharedPtr pos_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double target_x_cam = 0, target_y_cam = 0, target_w = 0;
    int threshold = 5;

    void subscrib_pos_from_cam(const dynamixel_sdk_custom_interfaces::msg::TargetPosition::SharedPtr msg){
        double x1 = msg->target_x1;
        double y1 = msg->target_y1;
        double x2 = msg->target_x2;
        double y2 = msg->target_y2;

        // RCLCPP_INFO(this->get_logger(),"Camera data Subscribed");

        if(std::abs(x2 - x1) >= 280){
            target_x_cam = std::abs((x1 + x2) / 2 - target_x_cam) > threshold ?  (x1 + x2) / 2 : target_x_cam;
            target_y_cam = std::abs((y1 + y2) / 2 - target_y_cam) > threshold ?  (y1 + y2) / 2 : target_y_cam;
            target_w = std::abs(std::abs(x1 - x2) - target_w) > threshold ?  std::abs((x1 - x2) / 2) : target_w;
            RCLCPP_INFO(this->get_logger(),"Camera data Subscribed");
        }  
    }

    void publish_position(){
        auto msg = dynamixel_sdk_custom_interfaces::msg::GrabPosition();
        
        if(target_x_cam >= 400 && target_x_cam <= 1000){
            if(target_y_cam > 0 && target_y_cam <= 500){
                msg.target_x = 20;
                msg.target_y = 10;
                msg.target_width = 21;
                pos_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Publishing Position : x = %f, y = %f, w = %d", msg.target_x, msg.target_y, msg.target_width);

                std::this_thread::sleep_for(std::chrono::seconds(20));
            }
            // else if(target_y_cam > 280 && target_y_cam <= 320){
            //     msg.target_x = 30;
            //     msg.target_y = 0;
            //     msg.target_width = 21;
            //     pos_publisher_->publish(msg);
            //     RCLCPP_INFO(this->get_logger(), "Publishing Position : x = %f, y = %f, w = %d", msg.target_x, msg.target_y, msg.target_width);

            //     std::this_thread::sleep_for(std::chrono::seconds(20));
            // }
            
        }
    };

};


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<pos_pub_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
