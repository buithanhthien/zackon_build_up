from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_url_arg = DeclareLaunchArgument(
        'camera_url',
        default_value='http://10.67.199.145:8080/video',
        description='Camera URL for human following'
    )
    
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
    ])
