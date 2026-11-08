"""Real Zackon: hardware SLAM, live keepout Nav2, explorer and mission."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, EmitEvent)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from bounded_exploration_real.nav_config import write_session_config


def launch_real(context):
    robot_share = Path(get_package_share_directory('view_robot_pkg'))
    share = Path(get_package_share_directory('bounded_exploration_real'))
    hardware_config = robot_share / 'config' / 'nav2_params.yaml'
    session_config = write_session_config(hardware_config)
    nav_enabled = IfCondition(LaunchConfiguration('start_nav2'))
    nav_specs = [
        ('nav2_controller', 'controller_server',
         [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server',
         [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', []),
        ('nav2_bt_navigator', 'bt_navigator', []),
    ]
    nav_nodes = [Node(package=package, executable=executable, name=executable,
                      parameters=[session_config], remappings=remaps,
                      condition=nav_enabled, output='screen')
                 for package, executable, remaps in nav_specs]
    nav_nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_bounded_exploration_real',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'bond_timeout': 4.0,
                     'node_names': [name for _, name, _ in nav_specs]}],
        condition=nav_enabled, output='screen'))

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(robot_share / 'launch' /
                                          'MAP_GENERATING.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'start_robot': LaunchConfiguration('start_hardware'),
            'start_slam': LaunchConfiguration('start_slam'),
        }.items())
    explorer = Node(
        package='frontier_exploration_ros2', executable='frontier_explorer',
        name='frontier_explorer',
        parameters=[str(share / 'config' / 'explorer.yaml'), {'use_sim_time': False}],
        output='screen')
    mission = Node(
        package='bounded_exploration_real', executable='mission',
        name='bounded_exploration_mission', parameters=[{
            'use_sim_time': False,
            'half_length_m': ParameterValue(LaunchConfiguration('half_length_m'),
                                            value_type=float),
            'half_width_m': ParameterValue(LaunchConfiguration('half_width_m'),
                                           value_type=float),
            'observation_points_enabled': ParameterValue(
                LaunchConfiguration('observation_points_enabled'), value_type=bool),
            'observation_spacing_m': ParameterValue(
                LaunchConfiguration('observation_spacing_m'), value_type=float),
            'observation_max_points': ParameterValue(
                LaunchConfiguration('observation_max_points'), value_type=int),
            'map_output_dir': LaunchConfiguration('map_output_dir'),
        }], output='screen')
    # If an owned navigation component dies, stop this launch. The arbiter's
    # command timeout also stops robot output when navigation goes silent.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Exploration Nav2 node exited'))]))
        for node in nav_nodes]
    return [hardware, *nav_nodes, explorer, mission, *handlers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_hardware', default_value='true',
                              description='Start view_robot drivers, TF, EKF and arbiter'),
        DeclareLaunchArgument('start_slam', default_value='true',
                              description='Start view_robot live SLAM on /merged'),
        DeclareLaunchArgument('start_nav2', default_value='true',
                              description='Start this mission\'s Nav2 with live ROI filters'),
        DeclareLaunchArgument('half_length_m', default_value='5.0'),
        DeclareLaunchArgument('half_width_m', default_value='5.0'),
        DeclareLaunchArgument('observation_points_enabled', default_value='true'),
        DeclareLaunchArgument('observation_spacing_m', default_value='2.0'),
        DeclareLaunchArgument('observation_max_points', default_value='6'),
        DeclareLaunchArgument('map_output_dir',
                              default_value='~/zackon_maps/bounded_exploration'),
        OpaqueFunction(function=launch_real),
    ])
