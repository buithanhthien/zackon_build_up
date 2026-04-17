import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    # Stage 1: Send navigation goal 5 times, each waits for "Goal succeeded" before next
    send_nav_goal = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'for i in 1 2 3 4 5; do '
            'echo "=== Nav goal attempt $i/5 ==="; '
            'ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '
            "\"{pose: {header: {frame_id: 'map'}, pose: {position: {x: -0.61, y: 1.14, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.80, w: 0.59}}}}\" "
            '2>&1 | grep -m1 "Goal succeeded"; '
            'echo "Attempt $i done"; '
            'done; '
            'echo "All 5 nav goals sent"'
        ],
        output='screen',
        name='send_nav_goal'
    )

    delayed_nav_goal = TimerAction(
        period=3.0,
        actions=[send_nav_goal]
    )
    
    # Stage 3: Run debug_dock_pose_node_v4 after nav goal succeeds + 3s delay
    debug_dock_node = Node(
        package='lidar_dock_detector',
        executable='debug_dock_pose_node_v4',
        output='screen',
        name='debug_dock_node'
    )
    
    delayed_debug_node = TimerAction(
        period=3.0,
        actions=[debug_dock_node]
    )
    
    # Register handler to launch debug node after nav goal completes
    on_nav_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=send_nav_goal,
            on_exit=[delayed_debug_node]
        )
    )
    
    # Stage 4: Run rear_docking_controller_node after debug node starts + 1s delay
    controller_node = Node(
        package='lidar_dock_detector',
        executable='rear_docking_controller_node',
        output='screen',
        name='rear_docking_controller'
    )
    
    delayed_controller = TimerAction(
        period=1.0,
        actions=[controller_node]
    )
    
    # Register handler to launch controller 1s after debug node STARTS (not exits)
    on_debug_start = RegisterEventHandler(
        OnProcessStart(
            target_action=debug_dock_node,
            on_start=[delayed_controller]
        )
    )
    
    # Monitor docked status
    docked_monitor = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'ros2 topic echo /rear_docking_status | while read -r line; do '
            'if [[ "$line" == *"data: 1"* ]]; then '
            'echo "========================================"; '
            'echo "🎯 DOCKED SUCCESSFULLY! 🎯"; '
            'echo "========================================"; '
            'fi; done'
        ],
        output='screen',
        name='docked_monitor'
    )
    
    return LaunchDescription([
        delayed_nav_goal,
        on_nav_complete,
        on_debug_start,
        docked_monitor
    ])
