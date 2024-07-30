#include "rclcpp/rclcpp.hpp"

#include "dynamixel_sdk_custom_interfaces/msg/motor_position.hpp"

class motorPos_sub_node : public rclcpp::Node{

public:
    motorPos_sub_node() : Node("motorPos_sub_node"){

        RCLCPP_INFO(this->get_logger(), "Motor position subscriber on");

        motorPos_subscriber_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::MotorPosition>(
            "motor_position", 5,
            std::bind(&motorPos_sub_node::sub_motorPos, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::MotorPosition>::SharedPtr motorPos_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    void sub_motorPos(const dynamixel_sdk_custom_interfaces::msg::MotorPosition::SharedPtr msg){

        int32_t dxl_present_position[4] = {0, 0, 0, 0};
        int32_t dxl_goal_position[4] = {0, 0, 0, 0};

        dxl_goal_position[0] = msg->dxl1_goal_pos;
        dxl_goal_position[1] = msg->dxl2_goal_pos;
        dxl_goal_position[2] = msg->dxl3_goal_pos;
        dxl_goal_position[3] = msg->dxl4_goal_pos;

        dxl_present_position[0] = msg->dxl1_present_pos;
        dxl_present_position[1] = msg->dxl2_present_pos;
        dxl_present_position[2] = msg->dxl3_present_pos;
        dxl_present_position[3] = msg->dxl4_present_pos;
        
        printf("[ID:1] GoalPos:%03d PresPos:%03d\t[ID:2] GoalPos:%03d PresPos:%03d\t[ID:3] GoalPos:%03d PresPos:%03d\t[ID:4] GoalPos:%03d PresPos:%03d\n", dxl_goal_position[0], dxl_present_position[0], dxl_goal_position[1], dxl_present_position[1], dxl_goal_position[2], dxl_present_position[2], dxl_goal_position[3], dxl_present_position[3]);

    };
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<motorPos_sub_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}