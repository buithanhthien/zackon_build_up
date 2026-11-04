"""Docking only: reuse the hardware workspace's existing sensors, TF and Nav2."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def docking_nodes(context):
    map_file = Path(LaunchConfiguration('map_file').perform(context)).expanduser().resolve()
    if not map_file.is_file():
        raise RuntimeError(f'Không tìm thấy map: {map_file}')
    profile_file = str(map_file.with_suffix('.docking.yaml'))
    config = LaunchConfiguration('config_file').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    common = {'use_sim_time': False, 'profile_file': profile_file, 'expected_map_file': str(map_file),
              'require_safety_permit': True}
    remaps = [('/cmd_vel', '/cmd_vel_sources/docking'), ('/odom', odom_topic)]
    return [
        Node(package='dock_shape_sim', executable='v_docking',
             name='v_docking', output='screen',
             parameters=[config, common], remappings=remaps),
        Node(package='dock_shape_sim', executable='dock_mission',
             name='dock_mission', output='screen',
             parameters=[common, {'auto_start': False, 'demo_route': False,
                                  'max_mission_time': 600.0}],
             remappings=remaps),
        Node(package='dock_shape_sim', executable='dock_calibration',
             name='dock_calibration', output='screen',
             parameters=[{'use_sim_time': False, 'map_file': str(map_file)}],
             remappings=[('/odom', odom_topic)]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('dock_shape_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('map_file', description='Map YAML currently loaded in AMCL'),
        DeclareLaunchArgument('config_file', default_value=str(share/'config/docking.yaml')),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered',
                              description='Odometry topic used by the running Nav2 controller'),
        OpaqueFunction(function=docking_nodes),
    ])
