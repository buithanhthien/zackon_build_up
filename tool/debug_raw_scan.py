"""
Quick diagnostic: prints the top-N intensity beams from /scan_front_filter
so you can see what the LiDAR actually measures before any filtering.
"""
import math, os, sys, yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH

_PARAMS_FILE = os.path.join(SOURCE_PATH, 'src/lidar_dock_detector/config/docking_params.yaml')
with open(_PARAMS_FILE) as f:
    _rp = yaml.safe_load(f)['docking_server']['ros__parameters']['reflective_tape_dock']

SCAN_TOPIC = _rp['scan_topic']
I_PEAK     = float(_rp['i_peak'])
I_VALLEY   = float(_rp['i_valley'])
MIN_ANGLE  = float(_rp['min_detection_angle_deg'])
MAX_ANGLE  = float(_rp['max_detection_angle_deg'])
TOP_N      = 30  # show top-N brightest beams


class RawScanDebug(Node):
    def __init__(self):
        super().__init__('debug_raw_scan')
        self.create_subscription(LaserScan, SCAN_TOPIC, self._cb, 10)
        print(f"Listening on {SCAN_TOPIC} ...")
        print(f"Thresholds: i_peak={I_PEAK}  i_valley={I_VALLEY}  angle=[{MIN_ANGLE},{MAX_ANGLE}]deg\n")

    def _cb(self, msg):
        print("\033[2J\033[H", end="")
        N = len(msg.ranges)
        pairs = [(msg.intensities[i], msg.ranges[i],
                  math.degrees(msg.angle_min + i * msg.angle_increment), i)
                 for i in range(N) if len(msg.intensities) == N
                 and msg.ranges[i] < msg.range_max]

        # top-N by intensity
        top = sorted(pairs, key=lambda x: x[0], reverse=True)[:TOP_N]
        print(f"{'rank':<5} {'idx':<6} {'angle(deg)':<12} {'intensity':<12} {'range(m)':<10}")
        print("-" * 50)
        for rank, (intensity, rng, angle_deg, idx) in enumerate(top, 1):
            above = " >= i_peak" if intensity >= I_PEAK else f" < i_peak({I_PEAK})"
            print(f"{rank:<5} {idx:<6} {angle_deg:<12.2f} {intensity:<12.1f} {rng:<10.4f}{above}")


def main():
    rclpy.init()
    rclpy.spin(RawScanDebug())


if __name__ == '__main__':
    main()
