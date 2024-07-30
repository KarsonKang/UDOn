#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "serial/serial.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"


#define ADDR_TORQUE_ENABLE              64
#define ADDR_GOAL_VELOCITY              104
#define ADDR_PROFILE_ACCELERATION       108
#define ADDR_PRESENT_VELOCITY           128

#define PROFILE_ACCELERATION            1

#define BAUDRATE                        57600
#define PROTOCOL_VERSION                2.0

#define DXL5_ID                         5
#define DXL6_ID                         6
#define DXL7_ID                         7
#define DXL8_ID                         8

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

// #define DEVICENAME                  "/dev/ttyUSB0"
#define DEVICENAME                  "/dev/udon_dynamixel_usb"
#define SERIALPORT                  "/dev/ttyACM0"
#define SERIALBAUDRATE                  57600

#define PI                              3.14159265

class udon_controller : public rclcpp::Node{

public:
    udon_controller() : Node("udon_controller"), broadcaster_(this){
        //Turn on all motors
        if(is_DXL_ON()){
            RCLCPP_INFO(this->get_logger(), "udon is ready !");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to start udon.");
            // return;
        }

        // Connect to Arduino
        serialPort.setPort(SERIALPORT);
        serialPort.setBaudrate(SERIALBAUDRATE);
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
        serialPort.setTimeout(_time);

        serialPort.open();
        if(serialPort.isOpen())
            RCLCPP_INFO(this->get_logger(), "Serial port is open");
        else
            RCLCPP_ERROR(this->get_logger(), "Serial port error");

        // Subscriber

        rclcpp::QoS qos(rclcpp::KeepLast(5));
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_out", qos,
            std::bind(&udon_controller::sub_control_udon, this, std::placeholders::_1)
        );

        // Publisher
        _time_last = this->now();   //Time initialization
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("udon_odom", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&udon_controller::pub_udon_odom, this));

        RCLCPP_INFO(this->get_logger(), "udon_control node has been started");
    
    };
    
    ~udon_controller(){
        Disable_DXL();
    };

private:
    //Initialize PortHandle and PacketHandler
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    //Initialize GroupSyncWrite instance and GroupSyncRead instance
    dynamixel::GroupSyncWrite groupSyncWrite = dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, 4);
    dynamixel::GroupSyncRead groupSyncRead = dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, 4);


    int dxl_comm_result = COMM_TX_FAIL;              //Store communication result
    uint8_t dxl_error = 0;
    bool dxl_addparam_result = false;
    bool dxl_getdata_result = false;

    uint8_t DXL_ID[4] = {DXL5_ID, DXL6_ID, DXL7_ID, DXL8_ID};
    uint8_t param_goal_velocity[4] = {0, 0, 0, 0};
    int32_t dxl_present_velocity[4] = {0, 0, 0, 0};

    double goal_linear[3] = {0};
    double goal_angular[3] = {0};
    int cylinder_state = 0;
    double left_vel = 0;
    double right_vel = 0;
    double left_rmp = 0;
    double right_rmp = 0;
    double udon_state_v = 0;
    double udon_state_w = 0;

    // Distance between two wheel and wheel Radius
    double belt_dist = 0.5;
    // On carpet 2.3, on ground 3.45
    double r = 4;
    double virtual_belt_dist = belt_dist * r;
    double Radius = 0.05;

    typedef struct _Vel_Pos_Data_{
        double x;
        double y;
        double z;
    }Vel_Pos_Data;

    Vel_Pos_Data Robot_Pos = {0.0, 0.0, 0.0}; // Position of Robot
    Vel_Pos_Data Robot_Ori = {0.0, 0.0, 0.0};
    Vel_Pos_Data Robot_Vel = {0.0, 0.0, 0.0}; // Speed of Robot
    rclcpp::Time _time_now, _time_last;

    serial::Serial serialPort;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    // rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom_tf_publisher_;
    tf2_ros::TransformBroadcaster broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_DXL_ON(){
        // Open port
        if(portHandler->openPort()){
            RCLCPP_INFO(this->get_logger(), "Succeeded to open the port");
            //printf("Succeeded to open the port\n");
        }
        else{
            printf("Failed to open the port\n");
            return 0;
        }
        // Set port baudrate
        if(portHandler->setBaudRate(BAUDRATE)){
            RCLCPP_INFO(this->get_logger(), "Succeeded to set port baudrate");
            //printf("Succeeded to set port baudrate\n");
        }
        else{
            printf("Failed to set port baudrate\n");
            return 0;
        }

        for(int i = 0; i < 4; i++){
            //Write Profile Acceleration and goal velocity
            // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_Profile_Acceleration, Profile_Acceleration, &dxl_error);

            //Add parameter storage for Dynamixel present velocity value
            dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i]);

            //Enable Dynamixel motor
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        }
        return true;
    };

    void Disable_DXL(){
        groupSyncRead.clearParam();
        for(int i = 0; i < 4; i++){
            //Disable Dynamixel motor
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        }
        portHandler->closePort();
    }

    /*********************************************************
    
    Subscribe control msg (Twist msg) and control motor

    *********************************************************/
    void sub_control_udon(const geometry_msgs::msg::Twist::SharedPtr msg){
        // Get the twist data
        this->goal_linear[0] = msg->linear.x;
        this->goal_linear[1] = msg->linear.y;
        this->goal_linear[2] = msg->linear.z;

        this->goal_angular[0] = msg->angular.x;
        this->goal_angular[1] = msg->angular.y;
        this->goal_angular[2] = msg->angular.z;

        //If angular.x > 0, extension the cylinder
        if(this->goal_angular[0] > 0){
            uint8_t sendata[1] = {1};
            cylinder_state = 1;
            serialPort.write(sendata, 1);
        }
        // else if(this->goal_angular[0] == 0){
        //     uint8_t sendata[1] = {0};
        //     serialPort.write(sendata, 1);
        // }
        else if(this->goal_angular[0] < 0){
            uint8_t sendata[1] = {2};
            cylinder_state = 0;
            serialPort.write(sendata, 1);
        }

        //calculate the wheel speed
        left_vel = this->goal_linear[0] - this->goal_angular[2] * this->belt_dist / 2;
        right_vel = this->goal_linear[0] + this->goal_angular[2] * this->belt_dist / 2;
        // RCLCPP_INFO(this->get_logger(), "get speed : x = %f, z = %f", left_vel, right_vel);
        // change wheel linear speed to rmp(round per min)
        left_rmp = (left_vel * 60) / (2 * PI * Radius);
        right_rmp = (right_vel * 60) / (2 * PI * Radius);
        // RCLCPP_INFO(this->get_logger(), "get speed : x = %f, z = %f", left_rmp, right_rmp);
        //change rmp data to motor value
        int dxl_goal_velocity_l = left_rmp / 0.229;
        int dxl_goal_velocity_r = right_rmp / 0.229;

        // RCLCPP_INFO(this->get_logger(), "get speed : x = %f, z = %f", msg->linear.x, msg->angular.z);
        // RCLCPP_INFO(this->get_logger(), "get speed : wl = %d, wr = %d", dxl_goal_velocity_l, dxl_goal_velocity_r);

        int dxl_goal_velocity[4] = {dxl_goal_velocity_l, dxl_goal_velocity_r, dxl_goal_velocity_l, dxl_goal_velocity_r};

        for(int i = 0; i < 4; i++){
            param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[i]));
            param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity[i]));
            param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity[i]));
            param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity[i]));

            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_velocity);
        }

        // Move motor
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();

        // for(int i = 0; i < 4; i++){
        //     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_GOAL_VELOCITY, dxl_goal_velocity[i], &dxl_error);
        // }

    }

    void get_DXL_velocity(){
        dxl_comm_result = groupSyncRead.txRxPacket();
        for(int i = 0; i < 4; i++){
            dxl_present_velocity[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_VELOCITY, 4);
        }
        // RCLCPP_INFO(this->get_logger(), "motor velocity: %d %d %d %d", dxl_present_velocity[0], dxl_present_velocity[1], dxl_present_velocity[2], dxl_present_velocity[3]);  
    }
    
    
    /*********************************************************
    
    Publish odometry msg

    *********************************************************/
    void pub_udon_odom(){

        get_DXL_velocity();

        _time_now = this->now();    // rclcpp::Time
        rclcpp::Duration duration = _time_now - _time_last;
        double sampling_time = static_cast<double>(duration.nanoseconds()) / 1e9;

        // RCLCPP_INFO(this->get_logger(), "sampling_time = %f", sampling_time);

        // Update position using previous Robot state data
        Robot_Pos.x += Robot_Vel.x * std::cos(Robot_Ori.z) * sampling_time;
        Robot_Pos.y += Robot_Vel.x * std::sin(Robot_Ori.z) * sampling_time;
        Robot_Pos.z += Robot_Vel.x * std::sin(Robot_Ori.y) * sampling_time;

        rclcpp::Time test;

        double wl = (double)(dxl_present_velocity[0] + dxl_present_velocity[2]) * 0.229 / 2.0 ;
        double wr = (double)(dxl_present_velocity[1] + dxl_present_velocity[3]) * 0.229 / 2.0 ;

        /********************************
            find virtual belt dist
        ********************************/
        // if((wl>0 && wr<0) || (wl<0 && wr>0)){
        //     test = this->now();
        //     RCLCPP_INFO(this->get_logger(), "test beging = %f", test.seconds());
        //     RCLCPP_INFO(this->get_logger(), "wr = %f , wl ^ %f", wr, wl);
        // }


        // RCLCPP_INFO(this->get_logger(), "get rpm : wl = %f, wr = %f", wl, wr);
        udon_state_v = (wl + wr) * PI * Radius / 60;
        udon_state_w = (wr - wl) * PI * Radius / (30 * virtual_belt_dist);

        // Update current robot state
        Robot_Vel.x = udon_state_v;    // m/s
        Robot_Vel.z = udon_state_w;    // rad/s

        // RCLCPP_INFO(this->get_logger(), "Robot_Vel : x = %f, z = %f", udon_state_v, udon_state_w);

        Robot_Ori.z += Robot_Vel.z * sampling_time;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, Robot_Ori.z);
        // RCLCPP_INFO(this->get_logger(), "Quaternion : x = %f, y = %f, z = %f w = %f",quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW() );
        geometry_msgs::msg::Quaternion quat = tf2::toMsg(quaternion);


        auto msg = std::make_shared<nav_msgs::msg::Odometry>();
        msg->header.stamp = _time_now;
        msg->header.frame_id = "udon_odom";
        msg->child_frame_id = "footprint";
        msg->twist.twist.linear.x = udon_state_v;
        msg->twist.twist.angular.z = udon_state_w;
        msg->pose.pose.position.x = Robot_Pos.x;
        msg->pose.pose.position.y = Robot_Pos.y;
        msg->pose.pose.position.z = Robot_Pos.z;
        msg->pose.pose.orientation = quat;


        // geometry_msgs::msg::TransformStamped odom_tf;
        // odom_tf.header.stamp = msg->header.stamp;
        // odom_tf.header.frame_id = "udon_odom";
        // odom_tf.child_frame_id = "footprint";
        // odom_tf.transform.translation.x = Robot_Pos.x;
        // odom_tf.transform.translation.y = Robot_Pos.y;
        // odom_tf.transform.translation.z = Robot_Pos.z;
        // odom_tf.transform.rotation = quat;

        odom_publisher_->publish(std::move(*msg));  // 发布odometry话题
        // broadcaster_.sendTransform(std::move(odom_tf));

        _time_last = _time_now;

        // rclcpp::Time now = this->now();
        // duration = now - _time_now;
        // RCLCPP_INFO(this->get_logger(), "cost_time = %f", static_cast<double>(duration.seconds()) + static_cast<double>(duration.nanoseconds()) / 1e9);
    }


};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv); 

    auto node = std::make_shared<udon_controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

