from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('view_robot_pkg')
    config_file = os.path.join(pkg_dir, 'config', 'front_lidar_filter.yaml')

    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_front_filter',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('scan', '/front_lidar/scan'),
                ('scan_filtered', '/scan_front_lidar_filter'),
            ],
        )
    ])
