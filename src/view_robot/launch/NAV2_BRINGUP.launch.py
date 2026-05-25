# =============================================================================
# NAV2_BRINGUP.launch.py
# =============================================================================
# Master launch file for the Zackon Mobile Robot full navigation stack.
#
# PIPELINE OVERVIEW:
#
#   [ESP32 / STM32]
#       │  UDP (port 8888)
#       ▼
#   micro_ros_agent          → publishes /odomfromSTM32, /cmd_vel subscriber
#       │
#       ▼
#   ekf_filter_node          → fuses /odomfromSTM32 → publishes /odom (filtered)
#       │                       also broadcasts TF: odom → base_link
#       ▼
#   [LiDAR front + rear]
#       │  /scan_front, /scan_rear
#       ▼
#   lidar_filters            → remove robot body from scan
#       │  /scan_front_filter, /scan_rear_filter
#       ▼
#   merge_lidar              → combines both scans → /merged
#       │
#       ▼
#   AMCL (localization)      → uses /merged + /odom → publishes TF: map → odom
#       │
#       ▼
#   Nav2 (navigation stack)  → path planning + controller → /cmd_vel
#
# HOW TO RUN:
#   ros2 launch view_robot_pkg NAV2_BRINGUP.launch.py
#
# CHANGE MAP:
#   ros2 launch view_robot_pkg NAV2_BRINGUP.launch.py map:=<full_path_to_map.yaml>
#
# NOTES:
#   - Do NOT run odom_tf_broadcaster manually. EKF handles odom→base_link TF.
#   - micro_ros_agent uses UDP (port 8888). Make sure ESP32 is on the same network.
#   - Default map: X5_08042026.yaml — change map_file_path below if needed.
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================================
    # 1. PACKAGE DIRECTORIES
    # =========================================================================
    pkg_dir = get_package_share_directory('view_robot_pkg')

    # =========================================================================
    # 2. FILE PATHS
    # =========================================================================

    # CHANGE THIS when switching maps
    map_file_path = PathJoinSubstitution([pkg_dir, 'maps', 'X5_08042026.yaml'])

    # Nav2 parameter file (AMCL, planner, controller, etc.)
    nav2_params_file = PathJoinSubstitution([pkg_dir, 'config', 'nav2_params.yaml'])

    # EKF parameter file (sensor fusion config)
    ekf_yaml = PathJoinSubstitution([pkg_dir, 'config', 'ekf.yaml'])

    # =========================================================================
    # 3. LAUNCH ARGUMENTS
    # =========================================================================

    # Map file — override at runtime: map:=<path>
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map yaml file to load'
    )

    # Set true only when running in Gazebo simulation
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock (Gazebo). Set false for real robot.'
    )

    # LiDAR hardware arguments (used if switching to single-lidar mode)
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='lidar_link')
    lidar_port_arg  = DeclareLaunchArgument('lidar_port',  default_value='/dev/lidar')
    lidar_baud_arg  = DeclareLaunchArgument('lidar_baud',  default_value='1000000')

    # =========================================================================
    # 4. HARDWARE NODES
    # =========================================================================

    # 4.1 Robot State Publisher + RViz
    #     Reads URDF and publishes robot TF tree (base_link → lidar_link, wheels, etc.)
    robot_state_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'view_robot.launch.py'])
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 4.2 micro-ROS Agent
    #     Bridges ESP32/STM32 firmware ↔ ROS 2 over UDP port 8888.
    #     Publishes: /odomfromSTM32
    #     Subscribes: /cmd_vel (forwards to firmware)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        output='screen'
    )

    # 4.3 Dual LiDAR drivers (front + rear RPLIDAR)
    #     Publishes: /scan_front, /scan_rear
    sllidar_driver_front_and_rear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'stereo_lidar.launch.py'])
        ),
    )

    # 4.4 LiDAR filters
    #     Removes robot body blind-spot from raw scans.
    #     Publishes: /scan_front_filter, /scan_rear_filter
    lidar_front_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'front_lidar_filter.launch.py'])
        ),
    )
    lidar_rear_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'rear_lidar_filter.launch.py'])
        ),
    )

    # 4.5 LiDAR merger
    #     Merges /scan_front_filter + /scan_rear_filter → /merged
    #     AMCL and Nav2 costmap consume /merged
    merge_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'merge_lidar.launch.py'])
        )
    )

    # =========================================================================
    # 5. STATE ESTIMATION — EKF (robot_localization)
    # =========================================================================
    # Subscribes:  /odomfromSTM32  (raw wheel odometry from STM32)
    # Publishes:   /odom           (filtered odometry for Nav2)
    # Broadcasts:  TF odom → base_link
    #
    #     Do NOT run odom_tf_broadcaster alongside this node.
    #     EKF already handles both /odom publishing and odom→base_link TF.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        # Remap default output topic to /odom so Nav2 receives filtered odometry
        remappings=[('/odometry/filtered', '/odom')],
    )

    # =========================================================================
    # 6. LOCALIZATION — Map Server + AMCL
    # =========================================================================
    # map_server: loads the static map from yaml file
    # amcl:       particle filter localization using /merged scan + /odom
    # Broadcasts: TF map → odom
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'zackon_localization.launch.py'])
        ),
        launch_arguments={
            'map':         LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file
        }.items()
    )

    # =========================================================================
    # 7. NAVIGATION — Nav2 Stack
    # =========================================================================
    # controller_server: follows local trajectory, outputs /cmd_vel
    # planner_server:    computes global path (map → goal)
    # behavior_server:   recovery behaviors (spin, backup, wait)
    # bt_navigator:      behavior tree orchestrator
    # waypoint_follower: executes multi-waypoint missions
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'zackon_navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  nav2_params_file
        }.items()
    )

    # =========================================================================
    # 8. ASSEMBLE LAUNCH DESCRIPTION
    #    Order matters for readability but ROS 2 handles actual startup order.
    # =========================================================================
    return LaunchDescription([
        # --- Arguments ---
        map_arg,
        use_sim_time_arg,
        lidar_frame_arg,
        lidar_port_arg,
        lidar_baud_arg,

        # --- Hardware ---
        robot_state_and_rviz,           # URDF + RViz
        micro_ros_agent,                # ESP32/STM32 bridge
        sllidar_driver_front_and_rear,  # Front + rear LiDAR
        lidar_front_filter,             # Filter front scan
        lidar_rear_filter,              # Filter rear scan
        merge_lidar,                    # Merge → /merged

        # --- State Estimation (must be up before Nav2) ---
        ekf_node,                       # /odomfromSTM32 → /odom + TF odom→base_link

        # --- Localization ---
        localization_launch,            # map_server + AMCL → TF map→odom

        # --- Navigation ---
        navigation_launch,              # Nav2 full stack → /cmd_vel
    ])
