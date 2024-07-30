#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/grab_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/motor_position.hpp"

#define ADDR_OPERATING_MODE         11

#define ADDR_TORQUE_ENABLE          64

#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define BAUDRATE                    1000000
#define ADDR_Profile_Acceleration   108
#define ADDR_Profile_Velocity       112

#define ADDR_Position_P_Gain        84
#define ADDR_Position_I_Gain        82
#define ADDR_Position_D_Gain        80

#define PROTOCOL_VERSION            2.0

#define LEN_GOAL_POSITION           4
#define LEN_PRESENT_POSITION        4

#define DXL_ID1                     1
#define DXL_ID2                     2
#define DXL_ID3                     3
#define DXL_ID4                     4

#define DXL1_MINIMUM_POSITION_VALUE 900
#define DXL1_MAXIMUM_POSITION_VALUE 3200
#define DXL2_MINIMUM_POSITION_VALUE 582 
#define DXL2_MAXIMUM_POSITION_VALUE 2550      
#define DXL3_MINIMUM_POSITION_VALUE 900
#define DXL3_MAXIMUM_POSITION_VALUE 3250
#define DXL4_MINIMUM_POSITION_VALUE 100
#define DXL4_MAXIMUM_POSITION_VALUE 3400

#define DEVICENAME                  "/dev/ttyUSB0"

#define Position_P_Gain             1000
#define Position_I_Gain             100
#define Position_D_Gain             50

#define Profile_Acceleration        30
#define Profile_Velocity            50
// #define Profile_Acceleration_4      30
// #define Profile_Velocity_4          30

#define TORQUE_ENABLE               1
#define TORQUE_DISABLE              0
#define DXL_MOVING_STATUS_THRESHOLD 20

#define ESC_ASCII_VALUE             0x1b

#define RoL1                        10
#define RoL2                        20
#define PI                          3.14149265

class arm_control_node: public rclcpp::Node{
    
public:
    arm_control_node(): Node("arm_control_node"), isSubscribed(true){
        if(is_DXLs_ON()){
            RCLCPP_INFO(this->get_logger(), "Arm Subscriber is On !");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to turn on the Arm.");
            return;
        }



        RCLCPP_INFO(this->get_logger(), "Ready to hear position messages");
        //keep the latest message
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        pos_subscriber_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::GrabPosition>(
            "grab_position", qos, 
            std::bind(&arm_control_node::sub_control_arm, this, std::placeholders::_1)
        );



        RCLCPP_INFO(this->get_logger(), "Motor position publishing");
        motorPos_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::MotorPosition>("motor_position", 5);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&arm_control_node::pub_motor_pos, this));

    };
    ~arm_control_node(){
        Disabled_DXL();
    };

private:

    //Initialize PortHandle and PacketHandler
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    //Initialize GroupSyncWrite instance and GroupSyncRead instance
    dynamixel::GroupSyncWrite groupSyncWrite = dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    dynamixel::GroupSyncRead groupSyncRead = dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    
    //set other parameters
    int dxl_comm_result = COMM_TX_FAIL;              //Store communication result
    bool dxl_addparam_result = false;
    bool dxl_getdata_result = false;

    uint8_t dxl_error = 0;

    uint8_t DXL_ID[4] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4};
    uint8_t param_goal_position[4] = {0, 0, 0, 0};
    int32_t dxl_present_position[4] = {0, 0, 0, 0};
    int32_t dxl_goal_position[4] = {0, 0, 0, 0};

    double object_x = 0, object_y = 0;
    int object_w = 0;
    uint8_t sub_count = 0;
    uint8_t arm_state = 0;
    
    rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::GrabPosition>::SharedPtr pos_subscriber_;
    bool isSubscribed;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::MotorPosition>::SharedPtr motorPos_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // hu chi suo
    std::mutex mtx;

    bool is_DXLs_ON(){
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
            //Write Profile Acceleration and Velocity
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_Profile_Acceleration, Profile_Acceleration, &dxl_error);
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_Profile_Velocity, Profile_Velocity, &dxl_error);

            //Write PID
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_Position_P_Gain, Position_P_Gain, &dxl_error);
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_Position_I_Gain, Position_I_Gain, &dxl_error);
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_Position_D_Gain, Position_D_Gain, &dxl_error);
        
            //Add parameter storage for Dynamixel present position value
            dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i]);

            //Enable Dynamixel motor
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        }
        this->arm_state = 1;
        RCLCPP_INFO(this->get_logger(), "robot_state : %d", this->arm_state);
        return true;
    };

    void sub_control_arm(const dynamixel_sdk_custom_interfaces::msg::GrabPosition::SharedPtr msg){

        if(this->object_x == msg->target_x){
            return;
        }

        this->object_x = msg->target_x;
        this->object_y = msg->target_y;
        this->object_w = msg->target_width;

        int32_t pose[4] = {0, 0, 0, 0};

        double angle2 = 0, angle3 = 0;

        if (this->object_x * this->object_x + this->object_y * this->object_y >= (RoL1 + RoL2) * (RoL1 + RoL2)){
            angle2 = atan(this->object_y / this->object_x) + PI / 2;          //弧度制
            angle3 = 0;
        }
        else {
            angle3 = acos((this->object_x * this->object_x + this->object_y * this->object_y - RoL1 * RoL1 - RoL2 * RoL2) / (2 * RoL1 * RoL2));
            angle2 = atan(this->object_y / this->object_x) - acos((this->object_x * this->object_x + this->object_y * this->object_y + RoL1 * RoL1 - RoL2 * RoL2) / (2 * RoL1 * pow(this->object_x * this->object_x + this->object_y * this->object_y, 0.5))) + PI / 2;
        }

        pose[0] = 1036;
        
        int32_t temp = angle2 * 2048 / PI + 582;
        if (temp > DXL2_MAXIMUM_POSITION_VALUE) pose[1] = DXL2_MAXIMUM_POSITION_VALUE;
        else if (temp < DXL2_MINIMUM_POSITION_VALUE) pose[1] = DXL2_MINIMUM_POSITION_VALUE;
        else pose[1] = temp;

        temp = angle3 * 2048 / PI + 2048;
        if (temp > DXL3_MAXIMUM_POSITION_VALUE) pose[2] = DXL3_MAXIMUM_POSITION_VALUE;
        else if (temp < DXL3_MINIMUM_POSITION_VALUE) pose[2] = DXL3_MINIMUM_POSITION_VALUE;
        else pose[2] = temp;

        pose[3] = 2500;


        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }

        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        this->arm_state = 2;


        
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);

        this->arm_state = 3;


        // Grab the object
        temp = 100 + (double(this->object_w - 18) / 20.0) * 3300;
        if(temp < DXL4_MINIMUM_POSITION_VALUE) pose[3] = DXL4_MINIMUM_POSITION_VALUE;
        else if(temp > DXL4_MAXIMUM_POSITION_VALUE) pose[3] = DXL4_MAXIMUM_POSITION_VALUE;
        else pose[3] = temp;

        dxl_goal_position[3] = pose[3];
        dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler, DXL_ID4, ADDR_GOAL_POSITION, dxl_goal_position[3], &dxl_error);

        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[3] - dxl_present_position[3]) > 2 * DXL_MOVING_STATUS_THRESHOLD);
        this->arm_state = 5;


        std::this_thread::sleep_for(std::chrono::milliseconds(200));


        //Turn around
        pose[0] = 3080;
        pose[1] = 1200;
        pose[2] = 3200;
        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }
        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();

        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD);
        this->arm_state = 7;



        //Ready to release object
        pose[1] = 1600;
        pose[2] = 2600;
        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }
        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();

        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD);
        
        dxl_goal_position[3] = pose[3] = 3000;
        dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler, DXL_ID4, ADDR_GOAL_POSITION, dxl_goal_position[3], &dxl_error);
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);


        // Ready to go back
        pose[1] = 1800;
        pose[2] = 3100;
        pose[3] = 3000;
        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }
        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);


        //Back to initial state
        pose[0] = 1036;
        pose[1] = 610;
        pose[2] = 2100;
        pose[3] = 1000;
        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }

        // move motor1 first
        dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler, DXL_ID1, ADDR_GOAL_POSITION, dxl_goal_position[0], &dxl_error);
        
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::seconds(2);
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::chrono::steady_clock::now() < end_time);


        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);

        std::this_thread::sleep_for(std::chrono::seconds(1));

    };



    void arm_send_object(){
        int32_t pose[4] = {3080, 1800, 3100, 2500};

        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }
        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);


        // Grab box
        pose[1] = 1600;
        pose[2] = 2600;

        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }
        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);

        dxl_goal_position[3] = pose[3] = 600;
        dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler, DXL_ID4, ADDR_GOAL_POSITION, dxl_goal_position[3], &dxl_error);
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);


        // Ready to go back
        pose[1] = 1800;
        pose[2] = 3100;

        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }
        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();

        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);


        //back to initial
        pose[0] = 1036;
        pose[1] = 610;
        pose[2] = 3100;

        for(int i = 0; i < 4; i++){
            dxl_goal_position[i] = pose[i];

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]));

            // Add goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position);
        }

        // move motor1 first
        dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler, DXL_ID1, ADDR_GOAL_POSITION, dxl_goal_position[0], &dxl_error);
        
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::seconds(2);
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::chrono::steady_clock::now() < end_time);

        // Move the motor 
        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();

        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        
        //Release
        dxl_goal_position[3] = pose[3] = 3100;
        dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler, DXL_ID4, ADDR_GOAL_POSITION, dxl_goal_position[3], &dxl_error);
        do{
            // Read present position
            dxl_comm_result = groupSyncRead.txRxPacket();
            for(int i = 0; i < 4; i++){
                dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            }
        } while(std::abs(dxl_goal_position[0] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[1] - dxl_present_position[1]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[2] - dxl_present_position[2]) > DXL_MOVING_STATUS_THRESHOLD && std::abs(dxl_goal_position[3] - dxl_present_position[3]) > DXL_MOVING_STATUS_THRESHOLD);


    }

    void pub_motor_pos(){
        auto msg = dynamixel_sdk_custom_interfaces::msg::MotorPosition();

        // Read present position
        dxl_comm_result = groupSyncRead.txRxPacket();


        for(int i = 0; i < 4; i++){
            dxl_present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        }
        msg.dxl1_goal_pos = dxl_goal_position[0];
        msg.dxl2_goal_pos = dxl_goal_position[1];
        msg.dxl3_goal_pos = dxl_goal_position[2];
        msg.dxl4_goal_pos = dxl_goal_position[3];

        msg.dxl1_present_pos = dxl_present_position[0];
        msg.dxl2_present_pos = dxl_present_position[1];
        msg.dxl3_present_pos = dxl_present_position[2];
        msg.dxl4_present_pos = dxl_present_position[3];
        motorPos_publisher_->publish(msg);
        // printf("[ID:%03d] GoalPos:%03d PresPos:%03d\t[ID:%03d] GoalPos:%03d PresPos:%03d\t[ID:%03d] GoalPos:%03d PresPos:%03d\t[ID:%03d] GoalPos:%03d PresPos:%03d\n", DXL_ID[0], dxl_goal_position[0], dxl_present_position[0], DXL_ID[1], dxl_goal_position[1], dxl_present_position[1], DXL_ID[2], dxl_goal_position[2], dxl_present_position[2], DXL_ID[3], dxl_goal_position[3], dxl_present_position[3]);
    };

    void Disabled_DXL(){

        groupSyncRead.clearParam();
        
        for(int i = 0; i < 4; i++){
            //Disable Dynamixel motor
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        }

        portHandler->closePort();
    };
};

int main(int argc, char ** argv){
    
    rclcpp::init(argc, argv); 

    auto node = std::make_shared<arm_control_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}