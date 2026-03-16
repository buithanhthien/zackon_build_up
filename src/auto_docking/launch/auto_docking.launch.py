from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

pkg = get_package_share_directory('auto_docking')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_docking',
            executable='dock_detector',
            name='dock_detector_node',
            output='screen',
            parameters=[{
                'intensity_threshold': 48.0,
                'cluster_distance_threshold': 0.05,
                'expected_strip_spacing': 0.37,
                'spacing_tolerance': 0.025
            }]
        ),
        Node(
            package='auto_docking',
            executable='dock_controller',
            name='dock_controller_node',
            output='screen',
            parameters=[{
                'target_reverse_distance': 0.4,
                'kp_y': 1.0,
                'reverse_speed': 0.17
            }]
        ),
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'docking_server.yaml'),
                {'dock_database': os.path.join(pkg, 'config', 'dock_database.yaml')}
            ]
        ),
    ])
