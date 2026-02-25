from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='human_following',
            executable='human_following_node',
            name='human_following_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])
