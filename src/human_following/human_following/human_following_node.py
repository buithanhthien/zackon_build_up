#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .camera import Camera
from .detector import HumanDetector
from .tracker import HumanTracker
from .controller import MotionController

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.camera = Camera()
        self.detector = HumanDetector()
        self.tracker = HumanTracker(frame_width=640)
        self.controller = MotionController()
        
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info('Human following node started')
        
    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return
            
        results = self.detector.detect(frame)
        zone = self.tracker.get_human_position(results)
        
        linear, angular = self.controller.compute_velocity(zone)
        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        
        if zone:
            self.get_logger().info(f'Human detected in {zone} zone - linear: {linear:.2f}, angular: {angular:.2f}')
        
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollowingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
