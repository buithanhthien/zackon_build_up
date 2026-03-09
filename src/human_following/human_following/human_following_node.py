#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from .camera import Camera
from .detector import HumanDetector
from .tracker import HumanTracker
import time
import math

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        
        self.declare_parameter('camera_device', 1)
        camera_device = self.get_parameter('camera_device').get_parameter_value().integer_value
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.lock_publisher = self.create_publisher(String, '/human_lock_status', 10)
        
        self.camera = Camera(source=camera_device, width=640, height=480)
        self.detector = HumanDetector()
        self.tracker = HumanTracker(frame_width=640, frame_height=480, horizontal_fov=145.0)
        
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 1.0
        self.camera_connected = False
        self.last_locked_id = None
        
        self.frame_count = 0
        self.fps_start_time = time.time()
        self.fps_publisher = self.create_publisher(String, '/tracking_fps', 10)
        
        self.timer = self.create_timer(0.03, self.process_frame)
        self.get_logger().info(f'Human following node started with camera: {camera_device}')
        
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
        num_detections = len(results[0].boxes) if results and len(results) > 0 else 0
        
        if num_detections > 0:
            self.get_logger().info(f'Detected {num_detections} humans', throttle_duration_sec=2.0)
        
        linear, angular, tracked, human_pose = self.tracker.get_human_position(results, time.time())
        
        if len(tracked) > 0:
            self.get_logger().info(f'Tracked: {len(tracked)}, Locked: {self.tracker.locked_id}, Linear: {linear:.2f}, Angular: {angular:.2f}', throttle_duration_sec=1.0)
            if human_pose:
                self.get_logger().info(f'Human pose - distance: {human_pose["distance"]:.2f}m, angle: {human_pose["angle"]:.2f}rad', throttle_duration_sec=1.0)
        
        if human_pose is not None:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'base_link'
            goal_msg.pose.position.x = float(human_pose['x'])
            goal_msg.pose.position.y = float(human_pose['y'])
            goal_msg.pose.position.z = 0.0
            
            yaw = math.atan2(human_pose['y'], human_pose['x'])
            goal_msg.pose.orientation.z = float(math.sin(yaw / 2))
            goal_msg.pose.orientation.w = float(math.cos(yaw / 2))
            
            self.goal_publisher.publish(goal_msg)
        
        if self.tracker.locked_id and self.tracker.locked_id != self.last_locked_id:
            lock_msg = String()
            lock_msg.data = f"Human locked: ID {self.tracker.locked_id}"
            self.lock_publisher.publish(lock_msg)
            self.get_logger().info(f'Human locked: ID {self.tracker.locked_id}')
            self.last_locked_id = self.tracker.locked_id
        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.fps_start_time
            fps = self.frame_count / elapsed
            fps_msg = String()
            fps_msg.data = f"FPS: {fps:.1f}"
            self.fps_publisher.publish(fps_msg)
            self.frame_count = 0
            self.fps_start_time = time.time()
        
        if linear != 0.0 or angular != 0.0:
            elapsed_ms = (time.time() - start_time) * 1000
            track_info = ', '.join([f'ID{t[0]}' for t in tracked])
            self.get_logger().info(f'Tracking [{track_info}] - linear: {linear:.2f}, angular: {angular:.2f}, speed: {elapsed_ms:.2f}ms')
        
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
