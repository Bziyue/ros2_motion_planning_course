"""Chapter 05 world/trajectory, selectable scanner, explicit publication profiling."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('motion2d_bringup'))
    settings=yaml.safe_load((share/'config/ch05.yaml').read_text())
    backend=LaunchConfiguration('backend').perform(context)
    if backend not in ('cpu','cuda'):raise ValueError('backend must be cpu or cuda')
    beams=int(LaunchConfiguration('beams').perform(context))
    parameters={**settings['/**']['ros__parameters'],**settings['simulator']['ros__parameters'],
                'model':'reference','start_paused':True,'lidar.backend':backend,'lidar.beams':beams,
                'lidar.profile':True,'lidar.visualize_beams':False}
    return [Node(package='motion2d',executable='simulator_node',name='simulator',parameters=[parameters],output='screen'),
            Node(package='rviz2',executable='rviz2',parameters=[{'use_sim_time':True}],
                 arguments=['-d',str(share/'rviz/ch05.rviz')],condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('backend',default_value='cpu'),DeclareLaunchArgument('beams',default_value='720'),
                              DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
