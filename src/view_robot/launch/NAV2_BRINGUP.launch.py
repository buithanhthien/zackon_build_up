import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ----------------------------------------------------
    # 1. INIT PACKAGE & DIRECTORIES
    # ----------------------------------------------------
    PACKAGE_NAME = 'view_robot_pkg' 
    pkg_dir = get_package_share_directory(PACKAGE_NAME)
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    
    # ----------------------------------------------------
    # 2. DECLARE ARGUMENTS
    # ----------------------------------------------------
    
    # File Config & Map for navigation 
    # Need to change map file name here when you change the environment to test 
    nav2_params_file = PathJoinSubstitution([pkg_dir, 'config', 'nav2_params.yaml'])
    map_file_path = PathJoinSubstitution([pkg_dir, 'maps', 'X5_08042026.yaml'])

    # Hardware Params
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='lidar_link')
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/lidar')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='1000000')
    
    # Simulation Time
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

    # Startup delay: give ekf_filter_node time to start publishing the
    # odom -> base_link transform before RViz/Nav2 start consuming scans
    # tagged with frame 'odom'. Prevents the
    # "Message Filter dropping message: frame 'odom' ... queue is full"
    # warning that occurs during the first ~2s after launch.
    ekf_startup_delay_arg = DeclareLaunchArgument(
        'ekf_startup_delay',
        default_value='3.0',
        description='Seconds to wait after EKF starts before launching localization/navigation'
    )

    # Map Argument
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map yaml file to load'
    )
    
    # ----------------------------------------------------
    # 3. HARDWARE SECTION
    # ----------------------------------------------------
    
    # 3.1 Robot State Publisher
    robot_state_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'view_robot.launch.py'])
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    # 3.2 Node Micro-ROS
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        output='screen'
    )

    # # 3.3 Node Lidar
    # sllidar_driver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([sllidar_dir, 'launch', 'sllidar_s2_launch.py']) 
    #     ),
    #     launch_arguments={
    #         'frame_id': LaunchConfiguration('lidar_frame'),
    #         'serial_port': LaunchConfiguration('lidar_port'),
    #         'serial_baudrate': LaunchConfiguration('lidar_baud')
    #     }.items()
    # )
    
    # # 3.4 Lidar filter: Filter in front of data of the robot 
    # lidar_filter = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([pkg_dir, 'launch', 'lidar_filter.launch.py'])
    #     ),
    # )


    sllidar_driver_front_and_rear=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'stereo_lidar.launch.py']) 
        ),
    )

    lidar_front_filter=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'front_lidar_filter.launch.py']) 
        ),
    )

    lidar_rear_filter=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'rear_lidar_filter.launch.py']) 
        ),
    )

    merge_lidar=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'merge_lidar.launch.py']) 
        )
    )

    # NOTE: lidar driver/filter/merge are NOT added directly to LaunchDescription
    # below anymore — they're started together with Nav2 inside the delayed
    # TimerAction group, so scan messages (frame 'odom' consumers in RViz)
    # don't start flowing before ekf_filter_node has published the
    # odom -> base_link transform. Starting Nav2 late while scans start at
    # t=0 was the actual cause of the "Message Filter... queue is full"
    # warning persisting even after delaying only localization/navigation.
    
    # ----------------------------------------------------
    # 4. LOCALIZATION (Map + AMCL)
    # ----------------------------------------------------
    ekf_yaml = PathJoinSubstitution([pkg_dir, 'config', 'ekf.yaml'])
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        #remappings=[('odom', '/odometry/filtered')],
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'zackon_localization.launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file
        }.items()
    )

    # 5. NAVIGATION (Controller + Planner)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'zackon_navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file
        }.items()
    )

    # Delay the lidar pipeline + localization + navigation until
    # ekf_filter_node has had time to come up and start publishing
    # odom -> base_link. Scans must NOT start flowing (into RViz's message
    # filters, costmaps, etc.) before that transform exists, or the
    # "Message Filter dropping message: frame 'odom' ... queue is full"
    # warning happens regardless of when Nav2 itself starts.
    delayed_sensors_localization_and_navigation = TimerAction(
        period=LaunchConfiguration('ekf_startup_delay'),
        actions=[
            sllidar_driver_front_and_rear,
            lidar_front_filter,
            lidar_rear_filter,
            merge_lidar,
            localization_launch,
            navigation_launch,
        ]
    )

    # ----------------------------------------------------
    # 6. RETURN
    # ----------------------------------------------------
    return LaunchDescription([
        # --- Arguments ---
        map_arg,
        lidar_frame_arg,
        lidar_port_arg,
        lidar_baud_arg,
        use_sim_time_arg,
        ekf_startup_delay_arg,
        
        # --- Hardware ---
        robot_state_and_rviz,
        micro_ros_agent,
        # sllidar_driver,
        # lidar_filter,

        # --- STATE ESTIMATION (MUST RUN FIRST) ---
        ekf_node,
        
        # --- SENSORS + NAVIGATION (delayed to avoid odom TF startup race) ---
        # lidar driver/filters/merge + localization + navigation all start
        # together after ekf_startup_delay seconds, once EKF is publishing
        # the odom -> base_link transform.
        delayed_sensors_localization_and_navigation,

    ])