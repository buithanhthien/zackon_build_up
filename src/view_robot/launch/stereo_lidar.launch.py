#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sllidar_launch = PathJoinSubstitution([
        FindPackageShare('sllidar_ros2'),
        'launch',
        'sllidar_s2_launch.py'
    ])

    front_lidar = GroupAction([
        PushRosNamespace('front'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'channel_type': 'serial',
                'serial_port': '/dev/lidar',
                'serial_baudrate': '1000000',
                'frame_id': 'front_lidar_link',
                'inverted': 'false',
                'angle_compensate': 'true',
                'scan_mode': 'Standard',
            }.items()
        )
    ])

    rear_lidar = GroupAction([
        PushRosNamespace('rear'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'channel_type': 'serial',
                'serial_port': '/dev/lidar_rear',
                'serial_baudrate': '1000000',
                'frame_id': 'rear_lidar_link',
                'inverted': 'false',
                'angle_compensate': 'true',
                'scan_mode': 'Standard',
            }.items()
        )
    ])

    return LaunchDescription([
        # front_lidar,
        rear_lidar,
    ])