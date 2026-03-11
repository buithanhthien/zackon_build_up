#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import String
from .camera import Camera
from .detector import HumanDetector
from .tracker import HumanTracker
import time
import math

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        
        self.declare_parameter('camera_url', 1)
        self.declare_parameter('use_nav2', True)
        camera_url = self.get_parameter('camera_url').value
        self.use_nav2 = self.get_parameter('use_nav2').value
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.lock_publisher = self.create_publisher(String, '/human_lock_status', 10)
        
        if self.use_nav2:
            self.nav2_client = ActionClient(self, FollowPath, '/follow_path')
            self.current_goal_handle = None
            self.last_goal_time = 0
            self.goal_interval = 1.0
        
        self.camera = Camera(source=camera_url)
        self.detector = HumanDetector()
        self.tracker = HumanTracker(frame_width=640, frame_height=480)
        
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 1.0
        self.camera_connected = False
        self.last_locked_id = None
        
        self.timer = self.create_timer(0.1, self.process_frame)
        mode = "Nav2" if self.use_nav2 else "Direct"
        self.get_logger().info(f'Human following node started ({mode} mode) with camera: {camera_url}')
        
    def send_nav2_goal(self, human_pose):
        current_time = time.time()
        if current_time - self.last_goal_time < self.goal_interval:
            return
            
        if self.current_goal_handle and not self.current_goal_handle.done():
            self.current_goal_handle.cancel_goal_async()
        
        goal_msg = FollowPath.Goal()
        goal_msg.path = Path()
        goal_msg.path.header.stamp = self.get_clock().now().to_msg()
        goal_msg.path.header.frame_id = 'base_link'
        
        pose = PoseStamped()
        pose.header = goal_msg.path.header
        pose.pose.position.x = float(human_pose['x'])
        pose.pose.position.y = float(human_pose['y'])
        pose.pose.position.z = 0.0
        
        yaw = math.atan2(human_pose['y'], human_pose['x'])
        pose.pose.orientation.z = float(math.sin(yaw / 2))
        pose.pose.orientation.w = float(math.cos(yaw / 2))
        
        goal_msg.path.poses.append(pose)
        
        self.nav2_client.wait_for_server(timeout_sec=0.1)
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.last_goal_time = current_time
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected, will retry')
            return
            
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status != 4:  # Not SUCCEEDED
            self.get_logger().warn(f'Nav2 goal aborted/failed (status={status}), will replan on next frame')
        
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
        linear, angular, tracked, human_pose = self.tracker.get_human_position(results, time.time())
        
        if human_pose is not None:
            if self.use_nav2:
                self.send_nav2_goal(human_pose)
            else:
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
        
        if not self.use_nav2:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.publisher.publish(msg)
        
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
