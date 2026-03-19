# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():
#     params = os.path.join(
#         get_package_share_directory('lidar_dock_detector'),
#         'config', 'docking_params.yaml'
#     )

#     return LaunchDescription([
#         Node(
#             package='lidar_dock_detector',
#             executable='docking_node',
#             name='docking_node',
#             output='screen',
#             parameters=[params],
#         ),
#     ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'lidar_dock_detector'
    )

    params_file = os.path.join(
        pkg_share, 'config', 'docking_params.yaml'
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['docking_server'],
        }],
    )

    return LaunchDescription([
        docking_server,
        lifecycle_manager,
    ])
