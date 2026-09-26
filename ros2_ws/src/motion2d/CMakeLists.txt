cmake_minimum_required(VERSION 3.20)
project(motion2d)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(motion2d_core
  src/geometry/se2.cpp src/geometry/obstacles.cpp
  src/sim/sim_clock.cpp src/sim/world.cpp src/sim/world_generator.cpp
  src/sim/ideal_model.cpp src/sim/swept_collision.cpp src/sim/velocity_model.cpp
  src/sim/inertial_model.cpp src/sim/reference_model.cpp src/sim/raycast.cpp src/sim/lidar_cpu.cpp
  src/sim/imu.cpp src/sim/sensor_scheduler.cpp)
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
add_executable(inertia_demo src/examples/inertia_demo.cpp)
target_link_libraries(inertia_demo PRIVATE motion2d_core)
install(TARGETS inertia_demo DESTINATION lib/motion2d)
add_executable(raycast_demo src/examples/raycast_demo.cpp)
target_link_libraries(raycast_demo PRIVATE motion2d_core)
install(TARGETS raycast_demo DESTINATION lib/motion2d)
add_executable(lidar_noise_demo src/examples/lidar_noise_demo.cpp)
target_link_libraries(lidar_noise_demo PRIVATE motion2d_core)
install(TARGETS lidar_noise_demo DESTINATION lib/motion2d)
add_executable(lidar_benchmark src/examples/lidar_benchmark.cpp)
target_link_libraries(lidar_benchmark PRIVATE motion2d_core)
install(TARGETS lidar_benchmark DESTINATION lib/motion2d)
add_executable(imu_demo src/examples/imu_demo.cpp)
target_link_libraries(imu_demo PRIVATE motion2d_core)
install(TARGETS imu_demo DESTINATION lib/motion2d)
add_executable(imu_noise_demo src/examples/imu_noise_demo.cpp)
target_link_libraries(imu_noise_demo PRIVATE motion2d_core)
install(TARGETS imu_noise_demo DESTINATION lib/motion2d)
ament_export_targets(motion2d_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

add_executable(hello_scene_node src/nodes/hello_scene_node.cpp)
# Lyrical requires C++20 at its ROS boundary; later pure math targets use C++17.
target_compile_features(hello_scene_node PRIVATE cxx_std_20)
target_compile_options(hello_scene_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hello_scene_node PRIVATE
  rclcpp::rclcpp visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
install(TARGETS hello_scene_node DESTINATION lib/motion2d)

add_executable(frame_demo_node src/nodes/frame_demo_node.cpp)
target_compile_features(frame_demo_node PRIVATE cxx_std_20)
target_compile_options(frame_demo_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(frame_demo_node PRIVATE motion2d_core rclcpp::rclcpp tf2_ros::tf2_ros
  visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp)
install(TARGETS frame_demo_node DESTINATION lib/motion2d)

add_executable(world_scene_node src/nodes/world_scene_node.cpp)
target_compile_features(world_scene_node PRIVATE cxx_std_20)
target_compile_options(world_scene_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(world_scene_node PRIVATE motion2d_core rclcpp::rclcpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
install(TARGETS world_scene_node DESTINATION lib/motion2d)

add_executable(simulator_node src/nodes/simulator_node.cpp src/ros/lidar_messages.cpp
  src/ros/imu_messages.cpp src/ros/odometry_messages.cpp)
target_compile_features(simulator_node PRIVATE cxx_std_20)
target_compile_options(simulator_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(simulator_node PRIVATE motion2d_core rclcpp::rclcpp tf2_ros::tf2_ros
  visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp)
install(TARGETS simulator_node DESTINATION lib/motion2d)

add_executable(truth_odometry_node src/nodes/truth_odometry_node.cpp)
target_compile_features(truth_odometry_node PRIVATE cxx_std_20)
target_compile_options(truth_odometry_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(truth_odometry_node PRIVATE rclcpp::rclcpp tf2_ros::tf2_ros
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
install(TARGETS truth_odometry_node DESTINATION lib/motion2d)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_se2 test/test_se2.cpp)
  target_link_libraries(test_se2 motion2d_core)
  ament_add_gtest(test_sim_clock test/test_sim_clock.cpp)
  target_link_libraries(test_sim_clock motion2d_core)
  ament_add_gtest(test_world test/test_world.cpp)
  target_link_libraries(test_world motion2d_core)
  ament_add_gtest(test_robot_model test/test_robot_model.cpp)
  target_link_libraries(test_robot_model motion2d_core)
  ament_add_gtest(test_raycast test/test_raycast.cpp)
  target_link_libraries(test_raycast motion2d_core)
  ament_add_gtest(test_lidar test/test_lidar.cpp)
  target_link_libraries(test_lidar motion2d_core)
  ament_add_gtest(test_imu test/test_imu.cpp)
  target_link_libraries(test_imu motion2d_core)
  ament_add_gtest(test_sensor_scheduler test/test_sensor_scheduler.cpp)
  target_link_libraries(test_sensor_scheduler motion2d_core)
  ament_add_gtest(test_odometry test/test_odometry.cpp src/ros/odometry_messages.cpp)
  target_compile_features(test_odometry PRIVATE cxx_std_20)
  target_link_libraries(test_odometry motion2d_core nav_msgs::nav_msgs__rosidl_typesupport_cpp)
endif()

ament_package()
