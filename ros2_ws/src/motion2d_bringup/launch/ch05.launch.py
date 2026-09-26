"""Chapter 05 reuses the physical simulator with CPU snapshot lidar enabled."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("motion2d_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("model", default_value="reference"),
        DeclareLaunchArgument("config", default_value=str(share / "config/ch05.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/ch04.launch.py")),
            launch_arguments={
                "rviz": LaunchConfiguration("rviz"),
                "model": LaunchConfiguration("model"),
                "config": LaunchConfiguration("config"),
                "rviz_config": str(share / "rviz/ch05.rviz"),
            }.items()),
    ])
