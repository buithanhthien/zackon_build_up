from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
        'enable_motion': ('false', bool),
    }
    node = Node(
        package='human_following', executable='human_following_node',
        name='human_following_node', output='screen',
        parameters=[{**{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                        for name, (_, kind) in defaults.items()}, 'use_nav2': False}])
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, (value, _) in defaults.items()],
        RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            EmitEvent(event=Shutdown(reason='Human following node exited; inspect its log'))])),
        node,
    ])
