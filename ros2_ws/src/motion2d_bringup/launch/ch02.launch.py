"""Chapter 02: deterministic simulated time and the map/odom/body/sensor TF tree."""
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
        DeclareLaunchArgument("config", default_value=str(share / "config/ch02.yaml")),
        Node(package="motion2d", executable="frame_demo_node", name="frame_demo",
             parameters=[LaunchConfiguration("config")], output="screen"),
        Node(package="rviz2", executable="rviz2", name="course_rviz",
             parameters=[{"use_sim_time": True}],
             arguments=["-d", str(share / "rviz/ch02.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
