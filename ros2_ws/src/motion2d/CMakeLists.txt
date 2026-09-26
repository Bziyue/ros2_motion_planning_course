cmake_minimum_required(VERSION 3.20)
project(motion2d)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(motion2d_core src/geometry/se2.cpp)
target_compile_features(motion2d_core PUBLIC cxx_std_17)
target_compile_options(motion2d_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motion2d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motion2d_core PUBLIC Eigen3::Eigen)

add_executable(se2_demo src/examples/se2_demo.cpp)
target_link_libraries(se2_demo PRIVATE motion2d_core)
install(TARGETS motion2d_core EXPORT motion2d_core_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS se2_demo DESTINATION lib/motion2d)
ament_export_targets(motion2d_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

add_executable(hello_scene_node src/nodes/hello_scene_node.cpp)
# Lyrical requires C++20 at its ROS boundary; later pure math targets use C++17.
target_compile_features(hello_scene_node PRIVATE cxx_std_20)
target_compile_options(hello_scene_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hello_scene_node PRIVATE
  rclcpp::rclcpp visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
install(TARGETS hello_scene_node DESTINATION lib/motion2d)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_se2 test/test_se2.cpp)
  target_link_libraries(test_se2 motion2d_core)
endif()

ament_package()
