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
    slam_dir = get_package_share_directory('slam_toolbox')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    
    # ----------------------------------------------------
    # 2. DECLARE ARGUMENTS
    # ----------------------------------------------------
    
    # File Config for Recording Map
    slam_params_file = PathJoinSubstitution([pkg_dir, 'config', 'mapper_params_online_async.yaml'])
    # Params for lidar
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='lidar_link')
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/lidar')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='1000000')
    
    # Simulation Time
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

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

    # 3.3 Node Lidar
    sllidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_dir, 'launch', 'sllidar_s2_launch.py']) 
        ),
        launch_arguments={
            'frame_id': LaunchConfiguration('lidar_frame'),
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baud')
        }.items()
    )
    
    # 3.4 Lidar filter: Filter in front of data of the robot
    lidar_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'lidar_filter.launch.py'])
        ),
    )

    # ----------------------------------------------------
    # 4. MAPPING SECTION (SLAM TOOLBOX)
    # ----------------------------------------------------
    # Using for recording map
    #When you want to storage map in rviz, use this cmd 'ros2 run nav2_map_server map_saver_cli -f <map_name>'
    # Change <map name> with your name you want to change to save map. Map will be saved in the folder 'maps' of this package
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_dir, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_file
        }.items()
    )

    # ----------------------------------------------------
    # 5. Estimate pose 
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

    # ----------------------------------------------------
    # 6. RETURN
    # ----------------------------------------------------
    return LaunchDescription([
        # --- Arguments ---
        lidar_frame_arg,
        lidar_port_arg,
        lidar_baud_arg,
        use_sim_time_arg,
        
        # --- Hardware  ---
        robot_state_and_rviz,
        micro_ros_agent,
        sllidar_driver,
        lidar_filter,
        
        # --- Estimate pose and record map ---
        ekf_node,
        slam_node, 

    ])
