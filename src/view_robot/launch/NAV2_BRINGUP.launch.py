import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    map_file_path = PathJoinSubstitution([pkg_dir, 'maps', 'X5_19032026.yaml'])

    # Hardware Params
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='lidar_link')
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/lidar')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='1000000')
    
    # Simulation Time
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

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
        
        # --- Hardware ---
        robot_state_and_rviz,
        micro_ros_agent,
        sllidar_driver_front_and_rear,
        lidar_front_filter,
        lidar_rear_filter,
        merge_lidar,
        # sllidar_driver,
        # lidar_filter,

        # --- STATE ESTIMATION (MUST RUN FIRST) ---
        ekf_node,
        
        # --- CHẾ ĐỘ: NAVIGATION  ---
        localization_launch,
        navigation_launch,

    ])
