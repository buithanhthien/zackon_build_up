import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('view_robot_pkg')
    config_file = os.path.join(pkg_dir, 'config', 'rear_lidar_filter.yaml')

    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_rear_filter',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('scan', '/rear_lidar/scan'),
                ('scan_filtered', '/scan_rear_lidar_filter'),
            ],
        )
    ])
