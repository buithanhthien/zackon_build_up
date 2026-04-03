#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

'''
/scan_front_lidar_filter
/scan_rear_lidar_filter
'''
class CheckRearFilterNode(Node):
    def __init__(self):
        super().__init__('check_front_filter_node')

        self.raw_msg = None
        self.filtered_msg = None

        self.lower_angle = -math.pi / 2.0
        self.upper_angle =  math.pi / 2.0

        self.sub_raw = self.create_subscription(
            LaserScan,
            '/front_lidar/scan',
            self.raw_callback,
            10
        )

        self.sub_filtered = self.create_subscription(
            LaserScan,
            '/scan_front_lidar_filter',
            self.filtered_callback,
            10
        )

        self.get_logger().info('CheckFrontFilterNode started')

    def raw_callback(self, msg: LaserScan):
        self.raw_msg = msg
        self.try_compare()

    def filtered_callback(self, msg: LaserScan):
        self.filtered_msg = msg
        self.try_compare()

    def is_valid(self, x: float) -> bool:
        return math.isfinite(x) and x > 0.0

    def try_compare(self):
        if self.raw_msg is None or self.filtered_msg is None:
            return

        raw = self.raw_msg
        fil = self.filtered_msg

        if len(raw.ranges) != len(fil.ranges):
            self.get_logger().error(
                f'Length mismatch: raw={len(raw.ranges)}, filtered={len(fil.ranges)}'
            )
            return

        total_inside = 0
        total_outside = 0
        inside_kept = 0
        outside_removed = 0

        suspicious_inside_removed = []
        suspicious_outside_kept = []

        for i in range(len(raw.ranges)):
            angle = raw.angle_min + i * raw.angle_increment
            raw_r = raw.ranges[i]
            fil_r = fil.ranges[i]

            inside = self.lower_angle <= angle <= self.upper_angle

            raw_valid = self.is_valid(raw_r)
            fil_valid = self.is_valid(fil_r)

            if inside:
                total_inside += 1
                if fil_valid:
                    inside_kept += 1
                elif raw_valid:
                    suspicious_inside_removed.append((i, angle, raw_r, fil_r))
            else:
                total_outside += 1
                if not fil_valid:
                    outside_removed += 1
                elif raw_valid:
                    suspicious_outside_kept.append((i, angle, raw_r, fil_r))

        self.get_logger().info('================ FILTER CHECK RESULT ================')
        self.get_logger().info(
            f'Inside region count      : {total_inside}'
        )
        self.get_logger().info(
            f'Inside region kept       : {inside_kept}'
        )
        self.get_logger().info(
            f'Outside region count     : {total_outside}'
        )
        self.get_logger().info(
            f'Outside region removed   : {outside_removed}'
        )

        if total_inside > 0:
            self.get_logger().info(
                f'Inside kept ratio        : {inside_kept / total_inside:.3f}'
            )
        if total_outside > 0:
            self.get_logger().info(
                f'Outside removed ratio    : {outside_removed / total_outside:.3f}'
            )

        self.get_logger().info('----- Suspicious beams inside but removed -----')
        for item in suspicious_inside_removed[:10]:
            i, angle, raw_r, fil_r = item
            self.get_logger().info(
                f'beam={i}, angle={math.degrees(angle):.1f} deg, raw={raw_r:.3f}, filtered={fil_r}'
            )

        self.get_logger().info('----- Suspicious beams outside but still kept -----')
        for item in suspicious_outside_kept[:10]:
            i, angle, raw_r, fil_r = item
            self.get_logger().info(
                f'beam={i}, angle={math.degrees(angle):.1f} deg, raw={raw_r:.3f}, filtered={fil_r:.3f}'
            )

        # chỉ kiểm tra 1 lần rồi thoát
        self.get_logger().info('Done. Shutting down...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CheckRearFilterNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()