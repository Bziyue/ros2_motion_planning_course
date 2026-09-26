"""Chapter 03: deterministic world geometry and the shared chapter 02 clock."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def world_exited(event, context):
    """Propagate a failed world as a failed launch, rather than an empty scene."""
    if context.is_shutdown:
        return []  # Ctrl-C already shuts down the whole launch.
    if event.returncode != 0:
        raise RuntimeError(f"World node failed with exit {event.returncode}; inspect its error above")
    return [EmitEvent(event=Shutdown(reason="World node exited"))]


def generate_launch_description():
    share = Path(get_package_share_directory("motion2d_bringup"))
    config = LaunchConfiguration("config")
    world = Node(package="motion2d", executable="world_scene_node", name="world_scene",
                 parameters=[config], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("config", default_value=str(share / "config/ch03.yaml")),
        RegisterEventHandler(OnProcessExit(
            target_action=world,
            on_exit=world_exited,
        )),
        world,
        Node(package="motion2d", executable="frame_demo_node", name="frame_demo",
             parameters=[config], output="screen"),
        Node(package="rviz2", executable="rviz2", name="course_rviz",
             parameters=[{"use_sim_time": True}],
             arguments=["-d", str(share / "rviz/ch03.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
