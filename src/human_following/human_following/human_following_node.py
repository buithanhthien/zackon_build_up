#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .camera import Camera
from .detector import HumanDetector
from .tracker import HumanTracker
import time

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        
        self.declare_parameter('camera_url', 'http://10.67.199.145:8080/video')
        camera_url = self.get_parameter('camera_url').get_parameter_value().string_value
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 20)
        
        self.camera = Camera(source=camera_url)
        self.detector = HumanDetector()
        self.tracker = HumanTracker(frame_width=640, frame_height=480)
        
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 1.0
        self.camera_connected = False
        
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info(f'Human following node started with camera: {camera_url}')
        
    def process_frame(self):
        start_time = time.time()
        ret, frame = self.camera.read()
        
        if not ret or frame is None:
            if not self.camera_connected:
                current_time = time.time()
                if current_time - self.last_reconnect_attempt >= self.reconnect_interval:
                    self.camera.reconnect()
                    self.last_reconnect_attempt = current_time
            else:
                self.camera_connected = False
                self.get_logger().warn('Camera disconnected, attempting reconnection...')
            
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            return
        
        if not self.camera_connected:
            self.camera_connected = True
            self.get_logger().info('Camera connected successfully')
            
        results = self.detector.detect(frame)
        linear, angular = self.tracker.get_human_position(results)
        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        
        if linear != 0.0 or angular != 0.0:
            elapsed_ms = (time.time() - start_time) * 1000
            self.get_logger().info(f'Tracking human - linear: {linear:.2f}, angular: {angular:.2f}, speed: {elapsed_ms:.2f}ms')
        
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
