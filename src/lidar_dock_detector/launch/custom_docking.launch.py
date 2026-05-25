from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('home_dock_x', default_value='-0.5155331150'),
        DeclareLaunchArgument('home_dock_y', default_value='1.5446702719'),
        DeclareLaunchArgument('home_dock_yaw', default_value='-1.6477'),
        DeclareLaunchArgument('staging_x_offset', default_value='0.8'),
        DeclareLaunchArgument('target_range', default_value='0.245'),
        DeclareLaunchArgument('backward_velocity', default_value='-0.3'),
        
        Node(
            package='lidar_dock_detector',
            executable='custom_range_dock_node',
            name='custom_range_dock',
            output='screen',
            parameters=[{
                'home_dock.x': LaunchConfiguration('home_dock_x'),
                'home_dock.y': LaunchConfiguration('home_dock_y'),
                'home_dock.yaw': LaunchConfiguration('home_dock_yaw'),
                'staging_x_offset': LaunchConfiguration('staging_x_offset'),
                'target_range': LaunchConfiguration('target_range'),
                'backward_velocity': LaunchConfiguration('backward_velocity'),
                'range_tolerance': 0.01,
            }]
        ),
    ])
