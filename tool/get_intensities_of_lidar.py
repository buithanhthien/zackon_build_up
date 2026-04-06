import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from collections import Counter

SAMPLE_COUNT = 1000

class ScanIntensity(Node):
    def __init__(self):
        super().__init__('scan_intensity')
        self.counter = Counter()
        self.samples = 0
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_front_filter',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        if self.samples >= SAMPLE_COUNT:
            return

        intensities = msg.intensities
        max_gap = 0.0
        best_peak = 0.0
        best_valley = 0.0
        prev = intensities[0] if intensities[0] != 0 else None

        for i in range(1, len(intensities)):
            curr = intensities[i]
            if curr == 0 or prev is None:
                prev = curr if curr != 0 else None
                continue
            gap = curr - prev
            if gap > max_gap:
                max_gap = gap
                best_valley = prev
                best_peak = curr
            prev = curr

        key = (round(best_peak), round(best_valley))
        self.counter[key] += 1
        self.samples += 1
        print(f"[{self.samples}/{SAMPLE_COUNT}] peak={key[0]}, valley={key[1]}")

        if self.samples >= SAMPLE_COUNT:
            self.print_results()
            rclpy.shutdown()

    def print_results(self):
        print("\n=== Results after 100 scans ===")
        print(f"{'Couple (peak-valley)':<25} {'Count':<8} {'Percentage'}")
        print("-" * 45)
        for (peak, valley), count in self.counter.most_common():
            pct = count / SAMPLE_COUNT * 100
            print(f"{peak}-{valley:<20} {count:<8} {pct:.1f}%")

def main():
    rclpy.init()
    node = ScanIntensity()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
