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
find_package(motion2d_interfaces REQUIRED)

add_library(motion2d_core
  src/geometry/se2.cpp src/geometry/obstacles.cpp
  src/sim/sim_clock.cpp src/sim/world.cpp src/sim/world_generator.cpp
  src/sim/ideal_model.cpp src/sim/swept_collision.cpp src/sim/velocity_model.cpp
  src/sim/inertial_model.cpp src/sim/reference_model.cpp src/sim/raycast.cpp src/sim/lidar_cpu.cpp
  src/sim/imu.cpp src/sim/sensor_scheduler.cpp src/mapping/scan_projection.cpp
  src/mapping/occupancy_grid.cpp src/mapping/distance_transform.cpp src/mapping/esdf.cpp src/evaluation/map_evaluation.cpp
  src/estimation/scan_matcher.cpp src/estimation/lidar_odometry.cpp src/estimation/imu_predictor.cpp
  src/estimation/planar_ekf.cpp src/estimation/fusion_timeline.cpp src/estimation/pose_graph.cpp src/estimation/loop_closure.cpp src/estimation/keyframes.cpp
  src/planning/inflation.cpp src/planning/astar.cpp src/planning/path_simplifier.cpp src/planning/corridor.cpp src/planning/replanner.cpp
  src/trajectory/polynomial.cpp src/trajectory/quintic.cpp src/trajectory/execution.cpp src/trajectory/reference_schedule.cpp src/trajectory/banded_lu.cpp src/trajectory/minco2d.cpp src/trajectory/trajectory_cost.cpp
  src/optimization/bfgs.cpp src/trajectory/trajectory_optimizer.cpp src/trajectory/spline2d.cpp src/trajectory/bezier_bounds.cpp src/control/pd_tracker.cpp src/control/tracking_reference.cpp src/control/linear_mpc.cpp src/optimization/box_qp.cpp)
target_compile_features(motion2d_core PUBLIC cxx_std_17)
target_compile_options(motion2d_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(motion2d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motion2d_core PUBLIC Eigen3::Eigen)

# CPU remains the default; CUDA is an optional, separate teaching backend.
option(MOTION2D_ENABLE_CUDA "Build optional ch20 CUDA lidar" OFF)
if(MOTION2D_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    message(FATAL_ERROR "CUDA requested but nvcc was not found; CPU builds use MOTION2D_ENABLE_CUDA=OFF")
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_library(motion2d_cuda src/sim/lidar_cuda.cu)
  set_target_properties(motion2d_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
  target_compile_options(motion2d_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
  target_link_libraries(motion2d_cuda PUBLIC motion2d_core PRIVATE CUDA::cudart_static)
  install(TARGETS motion2d_cuda ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
  add_executable(lidar_cuda_benchmark src/examples/lidar_cuda_benchmark.cpp)
  target_link_libraries(lidar_cuda_benchmark PRIVATE motion2d_cuda CUDA::cudart_static)
  install(TARGETS lidar_cuda_benchmark DESTINATION lib/motion2d)
endif()

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
add_executable(mapping_demo src/examples/mapping_demo.cpp)
target_compile_options(mapping_demo PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mapping_demo PRIVATE motion2d_core)
install(TARGETS mapping_demo DESTINATION lib/motion2d)
ament_export_targets(motion2d_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

add_executable(scan_matcher_demo src/examples/scan_matcher_demo.cpp)
target_link_libraries(scan_matcher_demo PRIVATE motion2d_core)
install(TARGETS scan_matcher_demo DESTINATION lib/motion2d)

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
  src/ros/imu_messages.cpp src/ros/odometry_messages.cpp src/ros/trajectory_messages.cpp)
target_compile_features(simulator_node PRIVATE cxx_std_20)
target_compile_options(simulator_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(simulator_node PRIVATE motion2d_interfaces::motion2d_interfaces__rosidl_typesupport_cpp motion2d_core rclcpp::rclcpp tf2_ros::tf2_ros
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

add_executable(mapping_node src/nodes/mapping_node.cpp src/ros/mapping_messages.cpp)
target_compile_features(mapping_node PRIVATE cxx_std_20)
target_compile_options(mapping_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mapping_node PRIVATE motion2d_core rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp)
install(TARGETS mapping_node DESTINATION lib/motion2d)

add_executable(lidar_odometry_demo src/examples/lidar_odometry_demo.cpp)
target_link_libraries(lidar_odometry_demo PRIVATE motion2d_core)
install(TARGETS lidar_odometry_demo DESTINATION lib/motion2d)

foreach(node lidar_odometry estimated_odometry fusion slam planner esdf trajectory tracker navigation)
  add_executable(${node}_node src/nodes/${node}_node.cpp src/ros/mapping_messages.cpp)
  target_compile_features(${node}_node PRIVATE cxx_std_20)
  target_compile_options(${node}_node PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(${node}_node PRIVATE motion2d_core rclcpp::rclcpp tf2_ros::tf2_ros
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
  install(TARGETS ${node}_node DESTINATION lib/motion2d)
endforeach()

target_sources(navigation_node PRIVATE src/ros/navigation_messages.cpp src/ros/trajectory_messages.cpp src/ros/control_messages.cpp src/ros/planning_messages.cpp)
target_link_libraries(navigation_node PRIVATE motion2d_interfaces::motion2d_interfaces__rosidl_typesupport_cpp)

target_sources(planner_node PRIVATE src/ros/planning_messages.cpp src/ros/trajectory_messages.cpp)
target_link_libraries(planner_node PRIVATE motion2d_interfaces::motion2d_interfaces__rosidl_typesupport_cpp)
target_sources(tracker_node PRIVATE src/ros/trajectory_messages.cpp src/ros/control_messages.cpp src/ros/navigation_messages.cpp)
target_link_libraries(tracker_node PRIVATE motion2d_interfaces::motion2d_interfaces__rosidl_typesupport_cpp std_srvs::std_srvs__rosidl_typesupport_cpp)

target_sources(trajectory_node PRIVATE src/ros/trajectory_messages.cpp)
target_link_libraries(trajectory_node PRIVATE motion2d_interfaces::motion2d_interfaces__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp)

add_executable(planar_ekf_demo src/examples/planar_ekf_demo.cpp)
target_link_libraries(planar_ekf_demo PRIVATE motion2d_core)
install(TARGETS planar_ekf_demo DESTINATION lib/motion2d)

add_executable(lidar_imu_demo src/examples/lidar_imu_demo.cpp)
target_link_libraries(lidar_imu_demo PRIVATE motion2d_core)
install(TARGETS lidar_imu_demo DESTINATION lib/motion2d)

add_executable(pose_graph_demo src/examples/pose_graph_demo.cpp)
target_link_libraries(pose_graph_demo PRIVATE motion2d_core)
install(TARGETS pose_graph_demo DESTINATION lib/motion2d)

add_executable(slam_demo src/examples/slam_demo.cpp)
target_link_libraries(slam_demo PRIVATE motion2d_core)
install(TARGETS slam_demo DESTINATION lib/motion2d)

add_executable(astar_demo src/examples/astar_demo.cpp)
target_link_libraries(astar_demo PRIVATE motion2d_core)
install(TARGETS astar_demo DESTINATION lib/motion2d)

add_executable(esdf_demo src/examples/esdf_demo.cpp)
target_link_libraries(esdf_demo PRIVATE motion2d_core)
install(TARGETS esdf_demo DESTINATION lib/motion2d)

add_executable(corridor_demo src/examples/corridor_demo.cpp)
target_link_libraries(corridor_demo PRIVATE motion2d_core)
install(TARGETS corridor_demo DESTINATION lib/motion2d)

add_executable(quintic_demo src/examples/quintic_demo.cpp)
target_link_libraries(quintic_demo PRIVATE motion2d_core)
install(TARGETS quintic_demo DESTINATION lib/motion2d)

add_executable(minco_demo src/examples/minco_demo.cpp)
target_link_libraries(minco_demo PRIVATE motion2d_core)
install(TARGETS minco_demo DESTINATION lib/motion2d)

add_executable(trajectory_optimization_demo src/examples/trajectory_optimization_demo.cpp)
target_link_libraries(trajectory_optimization_demo PRIVATE motion2d_core)
install(TARGETS trajectory_optimization_demo DESTINATION lib/motion2d)

add_executable(spline_demo src/examples/spline_demo.cpp)
target_link_libraries(spline_demo PRIVATE motion2d_core)
install(TARGETS spline_demo DESTINATION lib/motion2d)

add_executable(bezier_demo src/examples/bezier_demo.cpp)
target_link_libraries(bezier_demo PRIVATE motion2d_core)
install(TARGETS bezier_demo DESTINATION lib/motion2d)

add_executable(pd_tracking_demo src/examples/pd_tracking_demo.cpp)
target_link_libraries(pd_tracking_demo PRIVATE motion2d_core)
install(TARGETS pd_tracking_demo DESTINATION lib/motion2d)

add_executable(mpc_demo src/examples/mpc_demo.cpp)
target_link_libraries(mpc_demo PRIVATE motion2d_core)
install(TARGETS mpc_demo DESTINATION lib/motion2d)

add_executable(reference_handover_demo src/examples/reference_handover_demo.cpp)
target_link_libraries(reference_handover_demo PRIVATE motion2d_core)
install(TARGETS reference_handover_demo DESTINATION lib/motion2d)

add_executable(replanner_demo src/examples/replanner_demo.cpp)
target_link_libraries(replanner_demo PRIVATE motion2d_core)
install(TARGETS replanner_demo DESTINATION lib/motion2d)

add_executable(navigation_clearance src/examples/navigation_clearance.cpp)
target_link_libraries(navigation_clearance PRIVATE motion2d_core)
install(TARGETS navigation_clearance DESTINATION lib/motion2d)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  if(MOTION2D_ENABLE_CUDA)
    ament_add_gtest(test_lidar_cuda test/test_lidar_cuda.cpp)
    target_link_libraries(test_lidar_cuda motion2d_cuda CUDA::cudart_static)
  endif()
  ament_add_gtest(test_trajectory_messages test/test_trajectory_messages.cpp src/ros/trajectory_messages.cpp)
  target_compile_features(test_trajectory_messages PRIVATE cxx_std_20)
  target_link_libraries(test_trajectory_messages motion2d_core motion2d_interfaces::motion2d_interfaces__rosidl_typesupport_cpp)
  ament_add_gtest(test_trajectory_cost test/test_trajectory_cost.cpp)
  target_link_libraries(test_trajectory_cost motion2d_core)
  ament_add_gtest(test_navigation_messages test/test_navigation_messages.cpp src/ros/navigation_messages.cpp src/ros/trajectory_messages.cpp)
  target_compile_features(test_navigation_messages PRIVATE cxx_std_20)
  target_link_libraries(test_navigation_messages motion2d_core motion2d_interfaces::motion2d_interfaces__rosidl_typesupport_cpp)
  ament_add_gtest(test_control_messages test/test_control_messages.cpp src/ros/control_messages.cpp src/ros/mapping_messages.cpp)
  target_compile_features(test_control_messages PRIVATE cxx_std_20)
  target_link_libraries(test_control_messages motion2d_core nav_msgs::nav_msgs__rosidl_typesupport_cpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
  ament_add_gtest(test_replanner test/test_replanner.cpp)
  target_link_libraries(test_replanner motion2d_core)
  ament_add_gtest(test_reference_schedule test/test_reference_schedule.cpp)
  target_link_libraries(test_reference_schedule motion2d_core)
  ament_add_gtest(test_linear_mpc test/test_linear_mpc.cpp)
  target_link_libraries(test_linear_mpc motion2d_core)
  ament_add_gtest(test_pd_tracker test/test_pd_tracker.cpp)
  target_link_libraries(test_pd_tracker motion2d_core)
  ament_add_gtest(test_bezier test/test_bezier.cpp)
  target_link_libraries(test_bezier motion2d_core)
  ament_add_gtest(test_spline test/test_spline.cpp)
  target_link_libraries(test_spline motion2d_core)
  ament_add_gtest(test_minco test/test_minco.cpp)
  target_link_libraries(test_minco motion2d_core)
  ament_add_gtest(test_polynomial test/test_polynomial.cpp)
  target_link_libraries(test_polynomial motion2d_core)
  ament_add_gtest(test_corridor test/test_corridor.cpp)
  target_link_libraries(test_corridor motion2d_core)
  ament_add_gtest(test_esdf test/test_esdf.cpp)
  target_link_libraries(test_esdf motion2d_core)
  ament_add_gtest(test_planning_grid test/test_planning_grid.cpp)
  target_link_libraries(test_planning_grid motion2d_core)
  ament_add_gtest(test_keyframes test/test_keyframes.cpp)
  target_link_libraries(test_keyframes motion2d_core)
  ament_add_gtest(test_pose_graph test/test_pose_graph.cpp)
  target_link_libraries(test_pose_graph motion2d_core)
  ament_add_gtest(test_fusion_timeline test/test_fusion_timeline.cpp)
  target_link_libraries(test_fusion_timeline motion2d_core)
  ament_add_gtest(test_planar_ekf test/test_planar_ekf.cpp)
  target_link_libraries(test_planar_ekf motion2d_core)
  ament_add_gtest(test_lidar_odometry test/test_lidar_odometry.cpp)
  target_link_libraries(test_lidar_odometry motion2d_core)
  ament_add_gtest(test_scan_matcher test/test_scan_matcher.cpp)
  target_link_libraries(test_scan_matcher motion2d_core)
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
  ament_add_gtest(test_scan_projection test/test_scan_projection.cpp)
  target_link_libraries(test_scan_projection motion2d_core)
  ament_add_gtest(test_occupancy_grid test/test_occupancy_grid.cpp)
  target_link_libraries(test_occupancy_grid motion2d_core)
  ament_add_gtest(test_map_evaluation test/test_map_evaluation.cpp)
  target_link_libraries(test_map_evaluation motion2d_core)
  ament_add_gtest(test_mapping_messages test/test_mapping_messages.cpp src/ros/mapping_messages.cpp)
  target_compile_features(test_mapping_messages PRIVATE cxx_std_20)
  target_link_libraries(test_mapping_messages motion2d_core
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp)
endif()

ament_package()
