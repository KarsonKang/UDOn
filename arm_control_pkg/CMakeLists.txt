cmake_minimum_required(VERSION 3.5)
project(arm_control_pkg)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)


# Build
add_executable(arm_control_node src/arm_control_node.cpp)
ament_target_dependencies(arm_control_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk 
  rclcpp
)

add_executable(pos_pub_node src/pos_pub_node.cpp)
ament_target_dependencies(pos_pub_node
  dynamixel_sdk_custom_interfaces
  rclcpp
)

add_executable(motorPos_sub_node src/motorPos_sub_node.cpp)
ament_target_dependencies(motorPos_sub_node
  dynamixel_sdk_custom_interfaces
  rclcpp
)

# install
install (DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  arm_control_node
  pos_pub_node
  motorPos_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
