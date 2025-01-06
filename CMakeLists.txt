cmake_minimum_required(VERSION 3.5)
project(robosysasgmt2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(launch_ros REQUIRED)
find_package(requests REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_package()

