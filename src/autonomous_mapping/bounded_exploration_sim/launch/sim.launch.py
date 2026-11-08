"""Gazebo + live SLAM + bounded Nav2 exploration; no physical drivers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('bounded_exploration_sim')
    robot_share = get_package_share_directory('view_robot_pkg')
    robot_file = os.path.join(share, 'urdf', 'zackon_sim.urdf')
    with open(robot_file, encoding='utf-8') as stream:
        robot_description = stream.read()
    world = os.path.join(share, 'worlds', 'exploration_room.sdf')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'),
                               'launch', 'online_async_launch.py')
    params = os.path.join(share, 'config', 'nav2_explore.yaml')
    remaps = [('cmd_vel', '/cmd_vel_sources/navigation'),
              ('/cmd_vel', '/cmd_vel_sources/navigation')]
    nav_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
    ]
    actions = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sim_domain_id', default_value='77'),
        DeclareLaunchArgument('sim_partition', default_value='bounded_exploration_sim'),
        DeclareLaunchArgument('half_length_m', default_value='5.0'),
        DeclareLaunchArgument('half_width_m', default_value='5.0'),
        DeclareLaunchArgument('observation_points_enabled', default_value='true'),
        DeclareLaunchArgument('observation_spacing_m', default_value='2.0'),
        DeclareLaunchArgument('observation_max_points', default_value='6'),
        DeclareLaunchArgument('world_file', default_value=world),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('sim_domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('sim_partition')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [
            os.path.dirname(share), os.pathsep, os.path.dirname(robot_share),
            os.pathsep, EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world_file')]}.items(),
                                 condition=UnlessCondition(LaunchConfiguration('headless'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r -s ', LaunchConfiguration('world_file')]}.items(),
                                 condition=IfCondition(LaunchConfiguration('headless'))),
        Node(package='ros_gz_sim', executable='create', name='spawn_zackon',
             arguments=['-world', 'exploration_room', '-file', robot_file,
                        '-name', 'zackon_explorer', '-z', '0.022'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/rear_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'], output='screen'),
        Node(package='bounded_exploration_sim', executable='scan_merger',
             parameters=[{'use_sim_time': True}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch),
                                 launch_arguments={'use_sim_time': 'true',
                                                   'slam_params_file': os.path.join(
                                                       share, 'config', 'slam_sim.yaml')}.items()),
        Node(package='view_robot_pkg', executable='velocity_arbiter',
             parameters=[os.path.join(robot_share, 'config', 'twist_mux.yaml')],
             output='screen'),
    ]
    for package, executable in nav_nodes:
        actions.append(Node(package=package, executable=executable,
                            name=executable, parameters=[params], remappings=remaps,
                            output='screen'))
    actions.extend([
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_exploration', parameters=[{
                 'use_sim_time': True, 'autostart': True,
                 'node_names': [name for _, name in nav_nodes]}], output='screen'),
        Node(package='frontier_exploration_ros2', executable='frontier_explorer',
             name='frontier_explorer', parameters=[os.path.join(
                 share, 'config', 'explorer.yaml'), {'use_sim_time': True}],
             output='screen'),
        Node(package='bounded_exploration_sim', executable='mission',
             parameters=[{'use_sim_time': True,
                          'half_length_m': ParameterValue(
                              LaunchConfiguration('half_length_m'), value_type=float),
                          'half_width_m': ParameterValue(
                              LaunchConfiguration('half_width_m'), value_type=float),
                          'observation_points_enabled': ParameterValue(
                              LaunchConfiguration('observation_points_enabled'), value_type=bool),
                          'observation_spacing_m': ParameterValue(
                              LaunchConfiguration('observation_spacing_m'), value_type=float),
                          'observation_max_points': ParameterValue(
                              LaunchConfiguration('observation_max_points'), value_type=int)}],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='bounded_exploration_rviz',
             arguments=['-d', os.path.join(share, 'rviz', 'exploration.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
    return LaunchDescription(actions)
