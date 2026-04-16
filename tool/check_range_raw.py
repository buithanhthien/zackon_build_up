#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RearLidarReader(Node):
    def __init__(self):
        super().__init__('rear_lidar_reader')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_rear_lidar_filter',
            self.scan_callback,
            10
        )
        self.get_logger().info('Rear lidar reader node started...')
        self.get_logger().info('Subscribing to topic: /scan_rear_lidar_filter')

    min_range = 0.0;
    
    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        beam_count = len(ranges)

        self.get_logger().info(f'Nhan duoc scan moi: {beam_count} beams')

        if beam_count != 1800:
            self.get_logger().warn(
                f'So beam hien tai = {beam_count}, khong phai 1800'
            )

        # In toàn bộ dữ liệu range
        for i, r in enumerate(ranges):
            self.get_logger().info(f'Beam {i}: {r:.4f} m')

        # Nếu muốn xem nhanh toàn bộ mảng trên 1 dòng thì bỏ comment dòng dưới
        # self.get_logger().info(f'Ranges = {ranges}')

        # In thêm vài thông tin scan
        self.get_logger().info(
            f'angle_min={msg.angle_min:.6f}, '
            f'angle_max={msg.angle_max:.6f}, '
            f'angle_increment={msg.angle_increment:.6f}'
        )
        self.get_logger().info('-----------------------------------')


def main(args=None):
    rclpy.init(args=args)
    node = RearLidarReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()