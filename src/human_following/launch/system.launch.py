import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_url_arg = DeclareLaunchArgument(
        'camera_url',
        default_value='1',
        description='Camera URL for human following'
    )
    
    view_robot_dir = get_package_share_directory('view_robot_pkg')
    nav2_params_file = os.path.join(view_robot_dir, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        camera_url_arg,
        
        Node(
            package='human_following',
            executable='human_following_node',
            name='human_following_node',
            parameters=[{
                'camera_url': LaunchConfiguration('camera_url')
            }],
            output='screen'
        ),
        
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['controller_server']
            }]
        ),
    ])
