"""Sensor-only rosbag front end: player owns /clock, SLAM owns map->odom."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def required_node_exited(event, context):
    """End the replay stack when a required estimator fails or exits."""
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError(f"Required replay node failed with exit {event.returncode}")
    return [EmitEvent(event=Shutdown(reason="Required replay node exited"))]


def generate_launch_description():
    share = Path(get_package_share_directory("motion2d_bringup"))
    config = LaunchConfiguration("config")
    nodes = [
        Node(package="motion2d", executable="lidar_odometry_node", name="lidar_odometry",
             parameters=[config], remappings=[("/odometry/estimated", "/odometry/lidar")], output="screen"),
        Node(package="motion2d", executable="fusion_node", name="fusion", parameters=[config], output="screen"),
        Node(package="motion2d", executable="estimated_odometry_node", name="estimated_odometry",
             parameters=[{"use_sim_time": True, "publish_map_alignment": False}], output="screen"),
        Node(package="motion2d", executable="slam_node", name="slam", parameters=[config], output="screen"),
    ]
    # This course's sensors coincide with the disk centre. Real bags need calibrated extrinsics.
    for frame in ["laser", "imu_link"]:
        nodes.append(Node(package="tf2_ros", executable="static_transform_publisher",
                          name=f"replay_{frame}_extrinsic", parameters=[{"use_sim_time": True}],
                          arguments=["--frame-id", "base_link", "--child-frame-id", frame], output="screen"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/ch10.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        *[RegisterEventHandler(OnProcessExit(target_action=n, on_exit=required_node_exited)) for n in nodes],
        *nodes,
        Node(package="rviz2", executable="rviz2", name="course_rviz",
             parameters=[{"use_sim_time": True}], arguments=["-d", str(share / "rviz/ch10.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
