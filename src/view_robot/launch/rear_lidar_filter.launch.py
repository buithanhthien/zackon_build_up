from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_rear_filter',
            output='screen',
            parameters=['/home/khoaiuh/zackon_ws/src/MangoMobileRobot/src/view_robot_pkg/config/rear_lidar_filter.yaml'],
            remappings=[
                ('scan', '/rear_lidar/scan'),
                ('scan_filtered', '/scan_rear_lidar_filter'),
            ],
        )
    ])
