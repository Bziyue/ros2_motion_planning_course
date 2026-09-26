"""Chapter 04: one physical simulator, one matching world and optional RViz."""
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
    """Stop the scene if its simulator or world exits, preserving failure."""
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError(f"Required scene node failed with exit {event.returncode}")
    return [EmitEvent(event=Shutdown(reason="Required scene node exited"))]


def generate_launch_description():
    share = Path(get_package_share_directory("motion2d_bringup"))
    config = LaunchConfiguration("config")
    simulator = Node(
        package="motion2d", executable="simulator_node", name="simulator",
        parameters=[config, {"model": LaunchConfiguration("model")}], output="screen")
    world = Node(package="motion2d", executable="world_scene_node", name="world_scene",
                 parameters=[config], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("model", default_value="ideal"),
        DeclareLaunchArgument("config", default_value=str(share / "config/ch04.yaml")),
        *[RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=required_node_exited)) for node in (simulator, world)],
        simulator, world,
        Node(package="rviz2", executable="rviz2", name="course_rviz",
             parameters=[{"use_sim_time": True}],
             arguments=["-d", str(share / "rviz/ch04.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
