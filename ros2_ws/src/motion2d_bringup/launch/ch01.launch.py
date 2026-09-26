"""Chapter 01: one stationary disk, optionally shown in RViz2."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("motion2d_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("config", default_value=str(share / "config/ch01.yaml")),
        Node(
            package="motion2d", executable="hello_scene_node", name="hello_scene",
            parameters=[LaunchConfiguration("config")], output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", name="course_rviz",
            arguments=["-d", str(share / "rviz/ch01.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")), output="screen",
        ),
    ])
