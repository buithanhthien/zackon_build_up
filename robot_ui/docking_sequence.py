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
# STAGING_YAW    = DOCK_YAW + _p['reflective_tape_dock']['staging_yaw_offset']
STAGING_YAW    = DOCK_YAW 

def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class DockingSequenceNode(Node):
    def __init__(self):
        super().__init__('docking_sequence_node')
        self._nav_client  = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._dock_client = ActionClient(self, DockRobot,      '/dock_robot')

    def run(self):
        self.get_logger().info('Waiting for Nav2 action server...')
        self._nav_client.wait_for_server()

        self.get_logger().info(f'Navigating to staging pose ({STAGING_X:.2f}, {STAGING_Y:.2f}, yaw={STAGING_YAW:.2f})')

        qx, qy, qz, qw = yaw_to_quaternion(STAGING_YAW)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = STAGING_X
        goal.pose.pose.position.y = STAGING_Y
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return

        self.get_logger().info('Navigating to staging pose...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Reached staging pose — waiting for robot to settle...')
        time.sleep(1.0)

        self.get_logger().info('Launching docking server...')
        subprocess.Popen(
            ['bash', '-c',
             f'source {SOURCE_PATH}/install/setup.bash && '
             f'ros2 launch lidar_dock_detector docking.launch.py'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        
        self.get_logger().info('Waiting for docking server to come up...')
        self._dock_client.wait_for_server()
        time.sleep(2.0)

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
        if result.status == 4:
            self.get_logger().info('Docking SUCCEEDED')
        elif result.status == 6:
            self.get_logger().error(f'Docking ABORTED — error_code={result.result.error_code}')
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
