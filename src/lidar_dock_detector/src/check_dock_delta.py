#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


class DockDeltaChecker(Node):
    def __init__(self):
        super().__init__('dock_delta_checker')

        self.sub = self.create_subscription(
            PoseStamped,
            '/debug_dock_pose_lidar',
            self.pose_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.latest_cmd_vel = None
        self.get_logger().info("Dock delta checker started...")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd_vel = msg

    def pose_callback(self, msg: PoseStamped):
        dx = msg.pose.position.x
        dy = msg.pose.position.y

        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        dyaw = 2.0 * math.atan2(qz, qw)
        dyaw = self.normalize_angle(dyaw)

        dist = math.sqrt(dx * dx + dy * dy)

        print("\n==============================")
        print(f"frame      : {msg.header.frame_id}")
        print(f"delta_x    : {dx:.3f} m")
        print(f"delta_y    : {dy:.3f} m")
        print(f"delta_yaw  : {dyaw:.3f} rad")
        print(f"distance   : {dist:.3f} m")
        
        if self.latest_cmd_vel:
            print(f"cmd_vel.linear.x  : {self.latest_cmd_vel.linear.x:.3f} m/s")
            print(f"cmd_vel.angular.z : {self.latest_cmd_vel.angular.z:.3f} rad/s")
        else:
            print("cmd_vel    : (no data yet)")
        
        print("==============================")


def main():
    rclpy.init()
    node = DockDeltaChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()