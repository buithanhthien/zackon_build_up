#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class DockControllerNode(Node):
    def __init__(self):
        super().__init__('dock_controller_node')
        
        self.declare_parameter('target_reverse_distance', 0.45)
        self.declare_parameter('kp_y', 1.0)
        self.declare_parameter('reverse_speed', 0.15)
        
        self.target_reverse_distance = self.get_parameter('target_reverse_distance').value
        self.kp_y = self.get_parameter('kp_y').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        
        self.dock_pose_sub = self.create_subscription(PoseStamped, '/dock_pose', self.dock_pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.dock_pose = None
        self.robot_pose = None
        self.state = 'WAITING_FOR_DOCK'
        self.reverse_start_x = None
        self.reverse_start_y = None
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Dock Controller Node started')
    
    def dock_pose_callback(self, msg):
        self.dock_pose = msg
        if self.state == 'WAITING_FOR_DOCK':
            self.state = 'REVERSING'
            self.get_logger().info('Dock detected. Starting reverse.')
    
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
    
    def control_loop(self):
        if self.robot_pose is None:
            return
        
        cmd = Twist()
        
        if self.state == 'WAITING_FOR_DOCK':
            pass
        
        elif self.state == 'REVERSING':
            if self.reverse_start_x is None:
                self.reverse_start_x = self.robot_pose.position.x
                self.reverse_start_y = self.robot_pose.position.y
            
            dx = self.robot_pose.position.x - self.reverse_start_x
            dy = self.robot_pose.position.y - self.reverse_start_y
            distance_traveled = math.sqrt(dx**2 + dy**2)
            
            if distance_traveled >= self.target_reverse_distance:
                self.state = 'DOCKED'
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(f'Docking complete. Reversed {distance_traveled:.3f}m')
            else:
                if self.dock_pose:
                    lateral_error = self.dock_pose.pose.position.y
                    cmd.angular.z = self.kp_y * lateral_error
                cmd.linear.x = -self.reverse_speed
        
        elif self.state == 'DOCKED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

