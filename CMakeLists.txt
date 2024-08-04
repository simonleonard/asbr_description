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
find_package(ur_robot_driver REQUIRED)


install(
  DIRECTORY urdf launch rviz config 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
