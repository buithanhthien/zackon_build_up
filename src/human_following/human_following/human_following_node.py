#!/usr/bin/env python3
import rclpy
import signal
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import String
from .camera import Camera
from .tracker import HumanTracker
import time
import math
import json

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        
        self.declare_parameter('camera_url', 'realsense')
        for name, default in [('camera_serial', ''), ('camera_width', 640),
                              ('camera_height', 480), ('camera_fps', 30),
                              ('camera_format', 'YUYV'), ('enable_motion', False)]:
            self.declare_parameter(name, default)
        self.enable_motion = self.get_parameter('enable_motion').value
        self.declare_parameter('use_nav2', False)
        camera_url = self.get_parameter('camera_url').value
        self.use_nav2 = self.get_parameter('use_nav2').value
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel_sources/following', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.lock_publisher = self.create_publisher(String, '/human_lock_status', 10)
        
        if self.use_nav2:
            self.nav2_client = ActionClient(self, FollowPath, '/follow_path')
            self.current_goal_handle = None
            self.last_goal_time = 0
            self.goal_interval = 1.0
        
        self.status_publisher = self.create_publisher(String, '/tracking_status', 10)
        self.image_publisher = self.create_publisher(
            Image, '/tracking/color/image_raw',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.camera = None
        self.camera_connected = False
        self.last_locked_id = None
        self.status = dict(camera='starting', detector='starting', tracking='waiting', error='')
        self.status_timer = self.create_timer(0.5, self.publish_status)
        try:
            self.camera = Camera(source=camera_url,
                width=self.get_parameter('camera_width').value,
                height=self.get_parameter('camera_height').value,
                fps=self.get_parameter('camera_fps').value,
                serial=self.get_parameter('camera_serial').value,
                pixel_format=self.get_parameter('camera_format').value)
            ok, frame = self.camera.read()
            if not ok:
                raise RuntimeError(f'No colour frame from {camera_url}')
            self.publish_frame(frame)
            self.status['camera'] = 'connected'
            from .detector import HumanDetector
            self.detector = HumanDetector()
            self.detector.detect(frame)  # Readiness includes actual inference.
            self.status['detector'] = 'ready'
            self.tracker = HumanTracker(frame_width=frame.shape[1], frame_height=frame.shape[0])
        except KeyboardInterrupt:
            self.stop()
            raise
        except Exception as exc:
            self.fail(exc)
            return

        self.timer = self.create_timer(0.1, self.process_frame)
        mode = "Nav2" if self.use_nav2 else "Direct"
        self.get_logger().info(f'Human following node started ({mode} mode) with camera: {camera_url}')
        
    def send_nav2_goal(self, human_pose):
        current_time = time.time()
        if current_time - self.last_goal_time < self.goal_interval:
            return
            
        if self.current_goal_handle:
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
        try:
            ret, frame = self.camera.read()
            if not ret:
                raise RuntimeError('Camera lost: no colour frame received')
            self.publish_frame(frame)
            results = self.detector.detect(frame)
        except Exception as exc:
            self.fail(exc)
            return

        linear, angular, tracked, human_pose = self.tracker.get_human_position(results, time.time())
        
        self.status['tracking'] = ('locked' if human_pose is not None else 'searching')
        if human_pose is not None and self.enable_motion:
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
        
        if self.tracker.locked_id is not None and self.tracker.locked_id != self.last_locked_id:
            lock_msg = String()
            lock_msg.data = f"Human locked: ID {self.tracker.locked_id}"
            self.lock_publisher.publish(lock_msg)
            self.get_logger().info(f'Human locked: ID {self.tracker.locked_id}')
            self.last_locked_id = self.tracker.locked_id
        
        if not self.use_nav2:
            msg = Twist()
            msg.linear.x = linear if self.enable_motion else 0.0
            msg.angular.z = angular if self.enable_motion else 0.0
            self.publisher.publish(msg)
        
        if linear != 0.0 or angular != 0.0:
            elapsed_ms = (time.time() - start_time) * 1000
            track_info = ', '.join([f'ID{t[0]}' for t in tracked])
            self.get_logger().info(f'Tracking [{track_info}] - linear: {linear:.2f}, angular: {angular:.2f}, speed: {elapsed_ms:.2f}ms')
        
    def publish_frame(self, frame):
        # Camera already supplies BGR8; no second capture or cv_bridge needed.
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tracking_camera_color_optical_frame'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.image_publisher.publish(msg)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(self.status)
        self.status_publisher.publish(msg)

    def fail(self, exc):
        self.status.update(camera='unavailable', tracking='stopped', error=str(exc))
        if self.status['detector'] == 'starting':
            self.status['detector'] = 'not_ready'
        self.get_logger().error(str(exc))
        self.publish_status()
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.stop()
        # Give the UI time to receive the failure before terminating the launch.
        self.failure_timer = self.create_timer(1.0, self.exit_failed)

    def exit_failed(self):
        raise RuntimeError(self.status['error'])

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        try:
            if rclpy.ok():
                self.publisher.publish(msg)
        finally:
            if self.camera is not None:
                self.camera.release()

def main(args=None):
    # Keep the ROS context alive until the zero velocity command is published.
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = HumanFollowingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Do not let a repeated launch signal interrupt camera release.
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        if node is not None:
            try:
                node.stop()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)

if __name__ == '__main__':
    main()
