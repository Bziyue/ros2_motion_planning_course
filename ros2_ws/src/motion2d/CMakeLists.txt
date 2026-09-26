cmake_minimum_required(VERSION 3.20)
project(motion2d)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(hello_scene_node src/nodes/hello_scene_node.cpp)
# Lyrical requires C++20 at its ROS boundary; later pure math targets use C++17.
target_compile_features(hello_scene_node PRIVATE cxx_std_20)
target_compile_options(hello_scene_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hello_scene_node PRIVATE
  rclcpp::rclcpp visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
install(TARGETS hello_scene_node DESTINATION lib/motion2d)

ament_package()
