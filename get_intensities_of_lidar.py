import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class ScanIntensity(Node):
    a = [1, 10]
    z = 0
    def __init__(self):
        super().__init__('scan_intensity')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_front_filter',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        max = 0
        intensities = msg.intensities

        for i, intensity in enumerate(intensities):
            if max < intensity:
                max = intensity
            print(f"Beam {i}: {intensity}")
            if i == 1799:
                print(f"Max light intensity: {max}")
                
                # for i, intensity in enumerate(intensities):
                #     if intensity >= max-20: 
                #         print (f"Beam {i}: {intensity}")
                time.sleep(99999)

def main():
    rclpy.init()
    node = ScanIntensity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()