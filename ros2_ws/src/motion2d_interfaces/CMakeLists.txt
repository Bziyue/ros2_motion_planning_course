cmake_minimum_required(VERSION 3.20)
project(motion2d_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuinticPiece2D.msg" "msg/Trajectory2D.msg" "msg/MpcStatus.msg" "msg/ConvexRegion2D.msg" "msg/NavigationReference.msg" "msg/NavigationStatus.msg" "msg/LidarTiming.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
