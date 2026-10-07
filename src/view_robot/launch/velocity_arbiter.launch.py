"""Start once per robot; navigation bringup includes this automatically."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package='view_robot_pkg', executable='velocity_arbiter', name='velocity_arbiter',
        output='screen', parameters=[os.path.join(
            get_package_share_directory('view_robot_pkg'), 'config', 'twist_mux.yaml')])])
