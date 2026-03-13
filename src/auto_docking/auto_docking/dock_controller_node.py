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
        
        self.declare_parameter('pre_dock_distance', 0.5)
        self.declare_parameter('kp_x', 0.5)
        self.declare_parameter('kp_y', 1.0)
        self.declare_parameter('stop_distance', 0.10)
        
        self.pre_dock_distance = self.get_parameter('pre_dock_distance').value
        self.kp_x = self.get_parameter('kp_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.stop_distance = self.get_parameter('stop_distance').value
        
        self.dock_pose_sub = self.create_subscription(PoseStamped, '/dock_pose', self.dock_pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.dock_pose = None
        self.robot_pose = None
        self.state = 'IDLE'
        self.rotated_180 = False
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Dock Controller Node started')
    
    def dock_pose_callback(self, msg):
        self.dock_pose = msg
    
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
    
    def control_loop(self):
        if self.dock_pose is None or self.robot_pose is None:
            return
        
        dock_x = self.dock_pose.pose.position.x
        dock_y = self.dock_pose.pose.position.y
        
        distance = math.sqrt(dock_x**2 + dock_y**2)
        
        cmd = Twist()
        
        if self.state == 'IDLE':
            if distance < self.pre_dock_distance + 0.1:
                self.state = 'APPROACH'
        
        elif self.state == 'APPROACH':
            if distance > self.pre_dock_distance:
                angle_to_dock = math.atan2(dock_y, dock_x)
                cmd.angular.z = 2.0 * angle_to_dock
                cmd.linear.x = 0.3 * distance
            else:
                self.state = 'ROTATE'
                self.rotated_180 = False
        
        elif self.state == 'ROTATE':
            if not self.rotated_180:
                cmd.angular.z = 0.5
                self.rotated_180 = True
                self.rotation_start_time = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
                if elapsed > 6.28:
                    self.state = 'REVERSE'
                else:
                    cmd.angular.z = 0.5
        
        elif self.state == 'REVERSE':
            if distance < self.stop_distance:
                self.state = 'DOCKED'
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                lateral_error = dock_y
                cmd.angular.z = self.kp_y * lateral_error
                cmd.linear.x = -self.kp_x * distance
        
        elif self.state == 'DOCKED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Docking complete')
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
