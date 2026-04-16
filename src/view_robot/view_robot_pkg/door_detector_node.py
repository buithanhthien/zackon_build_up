#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# ── Detection zone (metres, robot frame) ──────────────────────────────────────
X_MIN, X_MAX = 0.6, 1.0
Y_MIN, Y_MAX = -0.45, 0.45

# ── Thresholds ─────────────────────────────────────────────────────────────────
# Points drop on elevator doors ~ 318 points
THRESHOLD_OPEN  = 30
THRESHOLD_CLOSE = 350

# ── Temporal filter ────────────────────────────────────────────────────────────
BUFFER_SIZE = 5


class DoorDetectorNode(Node):
    def __init__(self):
        super().__init__('door_detector_node')
        self._prev_state = False          # default: closed
        self._buffer: deque[bool] = deque(maxlen=BUFFER_SIZE)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._pub = self.create_publisher(Bool, '/door_state', 10)
        self.get_logger().info('door_detector_node started')

    def _scan_cb(self, msg: LaserScan):
        count = 0
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
            theta = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX:
                count += 1

        # Threshold logic
        if count <= THRESHOLD_OPEN:
            state = True
        elif count >= THRESHOLD_CLOSE:
            state = False
        else:
            state = self._prev_state

        # Temporal filter — majority vote
        self._buffer.append(state)
        final_state = sum(self._buffer) > len(self._buffer) / 2

        self._prev_state = final_state
        self._pub.publish(Bool(data=final_state))
        print(f"DOOR OPEN: {final_state} | points: {count}")


def main(args=None):
    rclpy.init(args=args)
    node = DoorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
