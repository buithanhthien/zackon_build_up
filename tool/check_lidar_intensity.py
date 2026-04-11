#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarIntensityChecker(Node):
    def __init__(self):
        super().__init__('lidar_intensity_checker')
        self.declare_parameter('scan_topic', '/scan_rear_lidar_filter')
        self.declare_parameter('intensity_threshold', 40.0)
        
        topic = self.get_parameter('scan_topic').value
        self.threshold = self.get_parameter('intensity_threshold').value
        
        self.sub = self.create_subscription(LaserScan, topic, self.scan_callback, 10)
        self.get_logger().info(f'Listening to {topic} | Threshold: {self.threshold}')

    def scan_callback(self, msg):
        if not msg.intensities:
            self.get_logger().warn('No intensity data')
            return
        
        high_intensity = [(i, val) for i, val in enumerate(msg.intensities) if val >= self.threshold]
        
        if high_intensity:
            self.get_logger().info(f'Found {len(high_intensity)} points above {self.threshold}:')
            for idx, intensity in high_intensity:
                angle = msg.angle_min + idx * msg.angle_increment
                distance = msg.ranges[idx]
                self.get_logger().info(f'  [{idx}] I={intensity:.1f} | angle={angle:.3f}rad | dist={distance:.3f}m')
        else:
            self.get_logger().info(f'No points above threshold {self.threshold}')

def main():
    rclpy.init()
    node = LidarIntensityChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
