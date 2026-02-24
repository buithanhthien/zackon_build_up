from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_front_filter',
            output='screen',
            parameters=['/home/khoaiuh/zackon_ws/src/MangoMobileRobot/src/view_robot_pkg/config/lidar_filter.yaml'],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_front_filter'),
            ],
        )
    ])
