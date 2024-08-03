cmake_minimum_required(VERSION 3.8)
project(asbr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ur_description REQUIRED)
find_package(robotiq_description REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(initial_joint_state_publisher src/initial_joint_state_publisher.cpp)
ament_target_dependencies(initial_joint_state_publisher rclcpp sensor_msgs)

install(
  DIRECTORY urdf launch rviz config meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
