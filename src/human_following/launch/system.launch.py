from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
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
        'use_nav2': ('false', bool),
        'max_frame_age': ('0.5', float),
        'desired_distance': ('1.0', float),
        'depth_min': ('0.3', float),
        'depth_max': ('3.0', float),
        'max_linear': ('0.2', float),
        'max_angular': ('0.5', float),
        'camera_frame': ('tracking_camera_color_optical_frame', str),
        'nav_frame': ('odom', str),
    }
    node = Node(
        package='human_following', executable='human_following_node',
        name='human_following_node', output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                        for name, (_, kind) in defaults.items()}])
    # Only publish after the operator has measured the mounting pose. RPY is
    # the optical frame orientation relative to base_link, in radians.
    tf_defaults = dict(publish_camera_tf='false', camera_x='0.0', camera_y='0.0',
                       camera_z='0.0', camera_roll='-1.5707963267948966',
                       camera_pitch='0.0', camera_yaw='-1.5707963267948966')
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tracking_camera_tf', condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
        arguments=[arg for flag, value in [
            ('--x', 'camera_x'), ('--y', 'camera_y'), ('--z', 'camera_z'),
            ('--roll', 'camera_roll'), ('--pitch', 'camera_pitch'), ('--yaw', 'camera_yaw'),
            ('--child-frame-id', 'camera_frame')]
            for arg in (flag, LaunchConfiguration(value))] + ['--frame-id', 'base_link'])
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in tf_defaults.items()],
        *[DeclareLaunchArgument(name, default_value=value)
          for name, (value, _) in defaults.items()],
        camera_tf,
        RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            EmitEvent(event=Shutdown(reason='Human following node exited; inspect its log'))])),
        node,
    ])
