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
import numpy as np
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        
        self.declare_parameter('camera_url', 'realsense')
        for name, default in [('camera_serial', ''), ('camera_width', 640),
                              ('camera_height', 480), ('camera_fps', 30),
                              ('camera_format', 'YUYV'), ('enable_motion', False),
                              ('desired_distance', 1.0), ('depth_min', 0.3),
                              ('depth_max', 3.0), ('max_linear', 0.2),
                              ('max_angular', 0.5),
                              ('camera_frame', 'tracking_camera_color_optical_frame'),
                              ('nav_frame', 'odom'), ('max_frame_age', 0.5)]:
            self.declare_parameter(name, default)
        self.max_frame_age = self.get_parameter('max_frame_age').value
        if not math.isfinite(self.max_frame_age) or self.max_frame_age <= 0:
            raise ValueError('max_frame_age must be positive')
        self.enable_motion = self.get_parameter('enable_motion').value
        self.declare_parameter('use_nav2', False)
        camera_url = self.get_parameter('camera_url').value
        self.use_nav2 = self.get_parameter('use_nav2').value
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel_sources/following', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.lock_publisher = self.create_publisher(String, '/human_lock_status', 10)
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.nav_frame = self.get_parameter('nav_frame').value
        if self.nav_frame == 'base_link':
            raise ValueError('nav_frame must be a fixed navigation frame such as odom')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_generation = 0
        self.goal_pending = False
        self.current_goal_handle = None
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
            self.tracker = HumanTracker(
                frame_width=frame.shape[1], frame_height=frame.shape[0],
                **{name: self.get_parameter(name).value for name in
                   ('desired_distance', 'depth_min', 'depth_max', 'max_linear', 'max_angular')})
        except KeyboardInterrupt:
            self.stop()
            raise
        except Exception as exc:
            self.fail(exc)
            return

        self.timer = self.create_timer(0.1, self.process_frame)
        mode = "Nav2" if self.use_nav2 else "Direct"
        self.get_logger().info(f'Human following node started ({mode} mode) with camera: {camera_url}')
        
    def cancel_navigation(self):
        self.goal_generation += 1
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

    def send_nav2_goal(self, human_pose):
        distance = human_pose['distance']
        remaining = max(0.0, distance - self.tracker.desired_distance)
        if remaining <= .05:
            self.cancel_navigation()
            self.publisher.publish(Twist())
            return
        # A moving base_link path would move with the robot. Freeze both endpoints
        # in the controller's fixed frame. Use the latest bounded-age TF because
        # odometry typically arrives just before the image-receive timestamp.
        try:
            transform = self.tf_buffer.lookup_transform(self.nav_frame, 'base_link', Time())
            stamp = Time.from_msg(transform.header.stamp)
            age = (self.frame_stamp.nanoseconds - stamp.nanoseconds) / 1e9
            if stamp.nanoseconds and not (-.1 <= age <= self.max_frame_age):
                raise TransformException('Navigation TF is stale')
            fixed = transform_matrix(transform.transform)
        except TransformException:
            self.status.update(tracking='tf_unavailable', error='Missing navigation TF')
            self.cancel_navigation()
            self.publisher.publish(Twist())
            return
        now = time.monotonic()
        if self.goal_pending or now-self.last_goal_time < self.goal_interval:
            return
        if self.current_goal_handle is not None:
            # Wait for completion of cancellation before submitting a replacement.
            self.current_goal_handle.cancel_goal_async()
            return
        if not self.nav2_client.server_is_ready():
            self.status.update(tracking='nav_unavailable', error='FollowPath server unavailable')
            self.publisher.publish(Twist())
            return
        end = np.array([human_pose['x']*remaining/distance,
                        human_pose['y']*remaining/distance, 0.0, 1.0])
        origin = fixed @ np.array([0., 0., 0., 1.])
        target = fixed @ end
        yaw = math.atan2(target[1]-origin[1], target[0]-origin[0])
        goal = FollowPath.Goal()
        goal.path = Path()
        goal.path.header.stamp = self.frame_stamp.to_msg()
        goal.path.header.frame_id = self.nav_frame
        for fraction in np.linspace(0., 1., max(2, int(remaining/.1)+1)):
            pose = PoseStamped()
            pose.header = goal.path.header
            position = origin + fraction*(target-origin)
            pose.pose.position.x, pose.pose.position.y = map(float, position[:2])
            pose.pose.orientation.z = math.sin(yaw/2)
            pose.pose.orientation.w = math.cos(yaw/2)
            goal.path.poses.append(pose)
        generation = self.goal_generation
        self.goal_pending = True
        future = self.nav2_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self.goal_response_callback(result, generation))
        self.last_goal_time = now

    def goal_response_callback(self, future, generation):
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(str(exc))
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 goal rejected, will retry')
            return
        self.current_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result: self.goal_result_callback(result, handle))
        if generation != self.goal_generation:
            handle.cancel_goal_async()

    def goal_result_callback(self, future, handle):
        if self.current_goal_handle is handle:
            self.current_goal_handle = None

    def process_frame(self):
        start_time = time.monotonic()
        try:
            ret, frame = self.camera.read()
            if not ret:
                raise RuntimeError('Camera lost: no colour frame received')
            acquired_at = time.monotonic()
            self.frame_stamp = self.get_clock().now()
            self.publish_frame(frame)
            results = self.detector.detect(frame)
        except Exception as exc:
            self.fail(exc)
            return

        try:
            options = {}
            if self.camera.requires_depth is True:
                try:
                    matrix = transform_matrix(self.tf_buffer.lookup_transform(
                        'base_link', self.camera_frame, self.frame_stamp).transform)
                except TransformException:
                    matrix = None
                options = dict(depth_m=self.camera.depth_m,
                               deproject=self.camera.deproject if self.camera.intrinsics else None,
                               camera_to_base=matrix, require_depth=True)
            if time.monotonic() - acquired_at > getattr(self, 'max_frame_age', .5):
                linear, angular, tracked, human_pose = self.tracker._stop([], 'frame_stale')
            else:
                linear, angular, tracked, human_pose = self.tracker.get_human_position(
                    results, time.monotonic(), **options)
        except Exception as exc:
            self.fail(exc)
            return
        
        self.status['tracking'] = ('locked' if human_pose is not None else 'searching')
        measurement = self.tracker.measurement_status
        if isinstance(measurement, str):
            self.status.update(measurement=measurement,
                               distance=human_pose.get('distance') if human_pose else None,
                               error='')
            if human_pose is None:
                self.status['tracking'] = measurement
        if human_pose is None or not self.enable_motion:
            linear = angular = 0.0
            if self.use_nav2:
                self.cancel_navigation()
                self.publisher.publish(Twist())
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
            elapsed_ms = (time.monotonic() - start_time) * 1000
            track_info = ', '.join([f'ID{t[0]}' for t in tracked])
            self.get_logger().info(f'Tracking [{track_info}] - linear: {linear:.2f}, angular: {angular:.2f}, speed: {elapsed_ms:.2f}ms')
        
    def publish_frame(self, frame):
        # Camera already supplies BGR8; no second capture or cv_bridge needed.
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = getattr(self, 'camera_frame', 'tracking_camera_color_optical_frame')
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
        if getattr(self, 'use_nav2', False):
            self.cancel_navigation()
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
                # Allow pending action responses/cancellation to reach Nav2 before
                # destroying the action client; bound shutdown if the server is gone.
                deadline = time.monotonic() + .5
                while (node.use_nav2 and rclpy.ok() and time.monotonic() < deadline
                       and (node.goal_pending or node.current_goal_handle is not None)):
                    rclpy.spin_once(node, timeout_sec=.05)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)

if __name__ == '__main__':
    main()
