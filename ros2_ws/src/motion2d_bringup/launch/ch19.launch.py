"""A: truth + ideal; B: truth + force MPC; C: laser/IMU SLAM + force MPC."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def required_exit(event, context):
    if context.is_shutdown: return []
    if event.returncode: raise RuntimeError(f'Required navigation node exited with {event.returncode}')
    return [EmitEvent(event=Shutdown(reason='Required navigation node exited'))]


def setup(context):
    share = Path(get_package_share_directory('motion2d_bringup'))
    route = LaunchConfiguration('route').perform(context)
    if route not in ('A', 'B', 'C'): raise ValueError('route must be A, B or C')
    config = LaunchConfiguration('config').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))
    settings = yaml.safe_load(Path(config).read_text())
    if route=='C' and settings.get('simulator', {}).get('ros__parameters', {}).get('yaw', 0.) != 0.:
        raise ValueError('Route C currently requires initial yaw=0 so estimated odom and physical force axes agree')
    def node(executable, name, params=None, remappings=None):
        return Node(package='motion2d', executable=executable+'_node', name=name,
                    parameters=[{**settings.get('/**', {}).get('ros__parameters', {}),
                                 **settings.get(name, {}).get('ros__parameters', {}),
                                 'seed': seed, **(params or {})}], remappings=remappings or [], output='screen')
    nodes = [node('simulator', 'simulator', {'model': 'ideal' if route=='A' else 'inertial', 'imu.enabled': route!='A', 'publish_truth_tf': False}),
             node('tracker', 'tracker', {'control.controller': 'ideal' if route=='A' else 'mpc', 'control.rate_hz': 200. if route=='A' else 50.}),
             node('navigation', 'navigation')]
    if route=='C':
        nodes += [node('lidar_odometry', 'lidar_odometry', remappings=[('/odometry/estimated', '/odometry/lidar')]),
                  node('fusion', 'fusion'), node('estimated_odometry', 'estimated_odometry', {'publish_map_alignment': False}), node('slam', 'slam')]
    else:
        nodes += [node('truth_odometry', 'truth_odometry'), node('mapping', 'mapping')]
    return [*[RegisterEventHandler(OnProcessExit(target_action=n, on_exit=required_exit)) for n in nodes], *nodes,
            Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
                 arguments=['-d', str(share/'rviz/ch19.rviz')], condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    share = Path(get_package_share_directory('motion2d_bringup'))
    return LaunchDescription([DeclareLaunchArgument('route', default_value='B'), DeclareLaunchArgument('seed', default_value='42'),
                              DeclareLaunchArgument('rviz', default_value='true'),
                              DeclareLaunchArgument('config', default_value=str(share/'config/ch19.yaml')), OpaqueFunction(function=setup)])
