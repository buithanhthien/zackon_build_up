#!/usr/bin/env python3
import subprocess
import time
import sys
import os
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, DockRobot
from action_msgs.msg import GoalStatus

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH

_PARAMS_FILE = os.path.join(SOURCE_PATH, 'src/lidar_dock_detector/config/docking_params.yaml')
with open(_PARAMS_FILE) as _f:
    _p = yaml.safe_load(_f)['docking_server']['ros__parameters']

DOCK_ID        = _p['docks'][0]
DOCK_X, DOCK_Y, DOCK_YAW = _p[DOCK_ID]['pose']
STAGING_OFFSET = _p['reflective_tape_dock']['staging_x_offset']
STAGING_X      = DOCK_X + STAGING_OFFSET * math.cos(DOCK_YAW)
STAGING_Y      = DOCK_Y + STAGING_OFFSET * math.sin(DOCK_YAW)
STAGING_YAW    = DOCK_YAW + _p['reflective_tape_dock']['staging_yaw_offset']

# Lateral alignment tuning
LATERAL_ALIGN_THRESHOLD = 0.02   # 2 cm — acceptable lateral error before DockRobot
LATERAL_ALIGN_MAX_ITER  = 5      # max correction iterations
LATERAL_DETECTION_TIMEOUT = 3.0  # seconds to wait for /detected_dock_pose per iteration


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class DockingSequenceNode(Node):
    def __init__(self):
        super().__init__('docking_sequence_node')
        self._nav_client  = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._dock_client = ActionClient(self, DockRobot,      '/dock_robot')
        self._latest_dock_pose = None

    def _dock_pose_callback(self, msg: PoseStamped):
        self._latest_dock_pose = msg

    def _navigate_to(self, x: float, y: float, yaw: float, label: str = '') -> bool:
        """Send a NavigateToPose goal and block until complete. Returns True on success."""
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if label:
            self.get_logger().info(
                f'Navigating to {label} ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')

        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Navigation goal "{label}" rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def _lateral_align(self):
        """
        Iteratively correct lateral offset relative to the two reflective tapes.

        Reads /detected_dock_pose (published by LidarIntensityDock plugin).
        The pose.position.y in the scan frame is the lateral error: how far
        the robot is off-centre from the tape midpoint.

        Projects this error into the map frame and sends a corrective
        NavigateToPose, keeping the forward distance constant.
        Repeats until error < LATERAL_ALIGN_THRESHOLD or max iterations reached.
        """
        sub = self.create_subscription(
            PoseStamped, '/detected_dock_pose', self._dock_pose_callback, 10)

        current_x = STAGING_X
        current_y = STAGING_Y

        for i in range(LATERAL_ALIGN_MAX_ITER):
            # Wait for a fresh detection
            self._latest_dock_pose = None
            deadline = time.time() + LATERAL_DETECTION_TIMEOUT
            while self._latest_dock_pose is None and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

            if self._latest_dock_pose is None:
                self.get_logger().warn(
                    f'[Align {i+1}] No /detected_dock_pose received — skipping lateral align')
                break

            # pose.position.y in scan frame = lateral offset from tape midpoint
            y_lateral = self._latest_dock_pose.pose.position.y
            self.get_logger().info(
                f'[Align {i+1}/{LATERAL_ALIGN_MAX_ITER}] '
                f'lateral error = {y_lateral * 100:.1f} cm')

            if abs(y_lateral) < LATERAL_ALIGN_THRESHOLD:
                self.get_logger().info('✅ Lateral alignment complete')
                break

            # Project the lateral correction into the map frame.
            # y_lateral tells WHERE the dock center is in the laser frame.
            # To move the robot TOWARD the dock center, the sign is negated:
            #   dock center to the LEFT (y > 0) → robot is to the RIGHT → move LEFT (−)
            #   dock center to the RIGHT (y < 0) → robot is to the LEFT  → move RIGHT (+)
            dx = -y_lateral * (-math.sin(STAGING_YAW))
            dy = -y_lateral *   math.cos(STAGING_YAW)
            current_x += dx
            current_y += dy

            self._navigate_to(current_x, current_y, STAGING_YAW,
                               label=f'lateral correction #{i + 1}')
            time.sleep(0.5)   # let robot settle before next measurement

        self.destroy_subscription(sub)

    def run(self):
        # ── 1. Launch docking server first so detection runs during alignment ──
        self.get_logger().info('Launching docking server...')
        subprocess.Popen(
            ['bash', '-c',
             f'source {SOURCE_PATH}/install/setup.bash && '
             f'ros2 launch lidar_dock_detector docking.launch.py'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

        self.get_logger().info('Waiting for Nav2 server...')
        self._nav_client.wait_for_server()

        # ── 2. Navigate to staging pose (facing dock, front LiDAR sees tapes) ──
        self._navigate_to(STAGING_X, STAGING_Y, STAGING_YAW, label='staging pose')
        time.sleep(1.0)   # settle before measuring lateral offset

        # ── 3. Wait for docking server to be fully alive ──
        self.get_logger().info('Waiting for docking server...')
        self._dock_client.wait_for_server()
        time.sleep(1.0)

        # ── 4. Precise lateral alignment loop ──
        self.get_logger().info('Starting lateral alignment...')
        self._lateral_align()

        # ── 5. Hand off to docking server: rotate 180° → reverse into dock ──
        self.get_logger().info(f'Sending DockRobot goal: {DOCK_ID}')
        dock_goal = DockRobot.Goal()
        dock_goal.dock_id = DOCK_ID

        dock_future = self._dock_client.send_goal_async(dock_goal)
        rclpy.spin_until_future_complete(self, dock_future)

        dock_handle = dock_future.result()
        if not dock_handle.accepted:
            self.get_logger().error('Dock goal rejected')
            return

        self.get_logger().info('Docking in progress...')
        dock_result_future = dock_handle.get_result_async()
        rclpy.spin_until_future_complete(self, dock_result_future)

        result = dock_result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Docking SUCCEEDED ✅')
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f'Docking ABORTED — error_code={result.result.error_code}')
        else:
            self.get_logger().error(f'Docking FAILED — status={result.status}')


def main():
    rclpy.init()
    node = DockingSequenceNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
