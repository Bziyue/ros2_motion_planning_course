"""Truth localization is explicit; only its adapter owns the moving TF edge."""
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
    """Preserve a required node's failure instead of leaving an empty RViz scene."""
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError(f"Required ch07 node failed with exit {event.returncode}")
    return [EmitEvent(event=Shutdown(reason="Required ch07 node exited"))]


def generate_launch_description():
    share = Path(get_package_share_directory("motion2d_bringup"))
    config = LaunchConfiguration("config")
    nodes = [
        Node(package="motion2d", executable="simulator_node", name="simulator",
             parameters=[config, {"model": LaunchConfiguration("model"),
                                  "publish_truth_tf": False}], output="screen"),
        Node(package="motion2d", executable="world_scene_node", name="world_scene",
             parameters=[config], output="screen"),
        Node(package="motion2d", executable="truth_odometry_node", name="truth_odometry",
             parameters=[{"use_sim_time": True}], output="screen"),
        Node(package="motion2d", executable="mapping_node", name="mapping",
             parameters=[config], output="screen"),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("model", default_value="reference"),
        DeclareLaunchArgument("config", default_value=str(share / "config/ch07.yaml")),
        *[RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=required_node_exited)) for node in nodes],
        *nodes,
        Node(package="rviz2", executable="rviz2", name="course_rviz",
             parameters=[{"use_sim_time": True}],
             arguments=["-d", str(share / "rviz/ch07.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
