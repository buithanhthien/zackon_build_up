#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


INTENSITY_THRESHOLD = 35.0


class RearLidarIntensityChecker(Node):
    def __init__(self):
        super().__init__('rear_lidar_intensity_checker')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_rear_lidar_filter',
            self.scan_callback,
            10
        )

        self.msg_count = 0
        self.get_logger().info(
            f'Subscribed to /scan_rear_lidar_filter | threshold > {INTENSITY_THRESHOLD}'
        )

    def scan_callback(self, msg: LaserScan):
        self.msg_count += 1

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Message #{self.msg_count}")

        found_points = []

        for i, intensity in enumerate(msg.intensities):
            if not math.isfinite(intensity):
                continue

            # CHỈ LẤY intensity > 40
            if intensity > INTENSITY_THRESHOLD:
                angle = msg.angle_min + i * msg.angle_increment

                if i < len(msg.ranges):
                    r = msg.ranges[i]
                else:
                    r = float('nan')

                found_points.append((i, angle, r, intensity))

        if not found_points:
            self.get_logger().info(
                f'No points found with intensity > {INTENSITY_THRESHOLD}'
            )
            return

        self.get_logger().info(
            f'Found {len(found_points)} points with intensity > {INTENSITY_THRESHOLD}'
        )

        for idx, angle, r, intensity in found_points:
            self.get_logger().info(
                f'idx={idx:4d} | angle={angle:+.4f} rad | range={r:.3f} m | intensity={intensity:.1f}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = RearLidarIntensityChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()