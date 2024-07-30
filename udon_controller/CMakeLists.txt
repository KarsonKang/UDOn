cmake_minimum_required(VERSION 3.5)
project(udon_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(udon_controller src/udon_controller.cpp)
ament_target_dependencies(
  udon_controller
  dynamixel_sdk
  serial
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  rclcpp
)

install (DIRECTORY
  urdf launch rviz config map
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  udon_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
