from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'camera_url': ('realsense', str),
        'camera_serial': ('', str),
        'camera_width': ('640', int),
        'camera_height': ('480', int),
        'camera_fps': ('30', int),
        'camera_format': ('YUYV', str),
    }
    node = Node(
        package='human_following', executable='camera_preview_node',
        name='camera_preview_node', output='screen',
        parameters=[{
            name: ParameterValue(LaunchConfiguration(name), value_type=kind)
            for name, (_, kind) in defaults.items()
        }])
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, (value, _) in defaults.items()],
        node,
    ])
