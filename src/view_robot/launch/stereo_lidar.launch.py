#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sllidar_launch = PathJoinSubstitution([
        FindPackageShare('sllidar_ros2'),
        'launch',
        'sllidar_s2e_launch.py'
    ])

    # ============================================================
    # FRONT LiDAR
    # PC:
    #   192.168.3.150
    #
    # LiDAR:
    #   192.168.3.45
    # ============================================================

    front_lidar = GroupAction([
        PushRosNamespace('front_lidar'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'channel_type': 'udp',
                'udp_ip': '192.168.3.45',
                'udp_port': '8089',
                'frame_id': 'front_lidar_link',
                'inverted': 'false',
                'angle_compensate': 'true',
                'scan_mode': 'Sensitivity',
            }.items()
        )
    ])

    # ============================================================
    # REAR LiDAR
    # PC:
    #   192.168.2.150
    #
    # LiDAR:
    #   192.168.2.46
    # ============================================================

    rear_lidar = GroupAction([
        PushRosNamespace('rear_lidar'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'channel_type': 'udp',
                'udp_ip': '192.168.2.46',
                'udp_port': '8089',
                'frame_id': 'rear_lidar_link',
                'inverted': 'false',
                'angle_compensate': 'true',
                'scan_mode': 'Sensitivity',
            }.items()
        )
    ])

    return LaunchDescription([
        front_lidar,
        rear_lidar,
    ])