#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import subprocess
import time

class DockingSequence(Node):
    def __init__(self):
        super().__init__('docking_sequence')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting docking sequence.')
        self.navigate_to_dock()
    
    def navigate_to_dock(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = -1.736452305440518
        goal_msg.pose.pose.position.y = -5.823076834288289
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.9997858207623426
        goal_msg.pose.pose.orientation.w = 0.020695714594307763
        
        self.get_logger().info('Navigating to docking position...')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return
        
        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Reached docking position. Launching auto_docking...')
        time.sleep(1)
        
        subprocess.Popen([
            'gnome-terminal', '--', 'bash', '-c',
            'source ~/zackon_build_up/install/setup.bash && ros2 launch auto_docking auto_docking.launch.py; exec bash'
        ])
        
        self.get_logger().info('Auto docking launched')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = DockingSequence()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
