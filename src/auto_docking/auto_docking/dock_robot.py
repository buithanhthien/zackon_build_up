#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot


class DockRobotClient(Node):
    def __init__(self):
        super().__init__('dock_robot_client')
        self._client = ActionClient(self, DockRobot, '/dock_robot')

    def send_goal(self):
        self._client.wait_for_server()
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = 'home_dock'
        goal.navigate_to_staging_pose = True

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _feedback(self, feedback):
        self.get_logger().info(f'State: {feedback.feedback.state}')

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Docking succeeded')
        else:
            self.get_logger().error(f'Docking failed: {result.error_code}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DockRobotClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
