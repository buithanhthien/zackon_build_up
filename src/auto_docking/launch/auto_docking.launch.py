from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_docking',
            executable='dock_detector',
            name='dock_detector_node',
            output='screen',
            parameters=[{
                'intensity_threshold': 50.0,
                'cluster_distance_threshold': 0.05,
                'expected_strip_spacing': 0.32,
                'spacing_tolerance': 0.05
            }]
        ),
        Node(
            package='auto_docking',
            executable='dock_controller',
            name='dock_controller_node',
            output='screen',
            parameters=[{
                'target_reverse_distance': 0.45,
                'kp_y': 1.0,
                'reverse_speed': 0.15
            }]
        )
    ])
