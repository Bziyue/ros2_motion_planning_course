"""One YAML chooses matching plant, flatness planner and controller; observed map only."""
from pathlib import Path
import math
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
    if event.returncode: raise RuntimeError(f'Required flat vehicle node exited with {event.returncode}')
    return [EmitEvent(event=Shutdown(reason='Required flat vehicle node exited'))]


def setup(context):
    share = Path(get_package_share_directory('motion2d_bringup'))
    settings = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    v = settings['vehicle']
    model, localization = v['model'], v['localization']
    if model not in ('inertial', 'ackermann'): raise ValueError('vehicle.model: inertial or ackermann')
    if localization not in ('truth', 'slam'): raise ValueError('vehicle.localization: truth or slam')
    radius, mass = float(v['radius']), float(v['mass'])
    inertia = float(v.get('inertia_z', .5*mass*radius**2))
    physical = {'radius': radius, 'mass': mass, 'linear_drag': float(v['linear_drag']),
                'inertia_z': inertia, 'angular_drag': float(v.get('angular_drag', .01)),
                'force_max': float(v['force_max']), 'torque_max': float(v.get('torque_max', .2))}
    if not all(math.isfinite(x) for x in physical.values()): raise ValueError('Finite physical constants required')
    ack = {'ackermann.'+k: float(v[k]) for k in ('mass', 'linear_drag', 'force_max', 'wheelbase', 'steering_max', 'steering_rate_max')}
    if localization == 'slam' and settings.get('simulator', {}).get('ros__parameters', {}).get('yaw', 0.) != 0.:
        raise ValueError('This SLAM lesson uses initial yaw=0 to align odom and plant force axes')

    def node(executable, name, extra=None, remappings=None):
        params = {**settings.get('/**', {}).get('ros__parameters', {}),
                  **settings.get(name, {}).get('ros__parameters', {}), **(extra or {})}
        return Node(package='motion2d', executable=executable+'_node', name=name,
                    parameters=[params], remappings=remappings or [], output='screen')

    nodes = [node('simulator', 'simulator', {**physical, **ack, 'model': model, 'publish_truth_tf': False}),
             node('world_scene', 'world_scene', {'radius': radius}),
             node('flat_vehicle', 'flat_vehicle', {**physical, **ack, 'vehicle.model': model})]
    if localization == 'truth':
        nodes += [node('truth_odometry', 'truth_odometry'), node('mapping', 'mapping')]
    else:
        nodes += [node('lidar_odometry', 'lidar_odometry', remappings=[('/odometry/estimated', '/odometry/lidar')]),
                  node('fusion', 'fusion'), node('estimated_odometry', 'estimated_odometry', {'publish_map_alignment': False}), node('slam', 'slam')]
    return [*[RegisterEventHandler(OnProcessExit(target_action=n, on_exit=required_exit)) for n in nodes], *nodes,
            Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
                 arguments=['-d', str(share/'rviz/flat_vehicle.rviz')], condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    share = Path(get_package_share_directory('motion2d_bringup'))
    return LaunchDescription([DeclareLaunchArgument('rviz', default_value='true'),
                              DeclareLaunchArgument('config', default_value=str(share/'config/flat_vehicle.yaml')),
                              OpaqueFunction(function=setup)])
