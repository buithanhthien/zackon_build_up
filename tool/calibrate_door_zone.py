#!/usr/bin/env python3
"""
Door zone calibration tool.
Draws a live top-down view of the LiDAR scan with the detection zone overlay.
Prints point count inside the zone every scan so you can read OPEN and CLOSED values.

Usage:
    python3 calibrate_door_zone.py

Adjust the zone constants below, then read the printed counts:
    - Point door at closed elevator → note count  → set THRESHOLD_CLOSE below that
    - Point door at open  elevator → note count  → set THRESHOLD_OPEN  above that
"""

import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# ── Zone to calibrate (edit these) ───────────────────────────────────────────
X_MIN, X_MAX = 0.5, 1.0
Y_MIN, Y_MAX = -0.8, 0.8
# ─────────────────────────────────────────────────────────────────────────────

_lock = threading.Lock()
_points_all = []   # all (x, y) from latest scan
_points_in  = []   # (x, y) inside zone
_count = 0


class ScanListener(Node):
    def __init__(self):
        super().__init__('calibrate_door_zone')
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, msg: LaserScan):
        global _points_all, _points_in, _count
        all_pts, in_pts = [], []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or not math.isfinite(r):
                continue
            theta = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            all_pts.append((x, y))
            if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX:
                in_pts.append((x, y))
        with _lock:
            _points_all = all_pts
            _points_in  = in_pts
            _count      = len(in_pts)
        print(f"Points in zone: {_count:4d}   (total scan pts: {len(all_pts)})")


def _ros_thread():
    rclpy.init()
    node = ScanListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    threading.Thread(target=_ros_thread, daemon=True).start()

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_xlabel('X (m) — forward')
    ax.set_ylabel('Y (m) — left')
    ax.set_title('Door Zone Calibration  |  robot at origin →')
    ax.axhline(0, color='gray', lw=0.5)
    ax.axvline(0, color='gray', lw=0.5)

    # Detection zone rectangle
    zone_rect = patches.Rectangle(
        (X_MIN, Y_MIN),
        X_MAX - X_MIN, Y_MAX - Y_MIN,
        linewidth=2, edgecolor='cyan', facecolor='cyan', alpha=0.15,
        label='Detection zone'
    )
    ax.add_patch(zone_rect)

    # Robot marker
    ax.plot(0, 0, 'r^', markersize=12, label='Robot')

    scatter_all = ax.scatter([], [], s=2,  color='gray',  alpha=0.4, label='All points')
    scatter_in  = ax.scatter([], [], s=10, color='lime',  label='In zone')
    count_text  = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                          fontsize=13, verticalalignment='top',
                          color='white',
                          bbox=dict(boxstyle='round', facecolor='black', alpha=0.6))
    ax.legend(loc='lower right')
    ax.set_facecolor('#0d0f12')
    fig.patch.set_facecolor('#0d0f12')
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')

    def update(_frame):
        with _lock:
            all_pts = list(_points_all)
            in_pts  = list(_points_in)
            cnt     = _count

        if all_pts:
            xs, ys = zip(*all_pts)
            scatter_all.set_offsets(list(zip(xs, ys)))
        else:
            scatter_all.set_offsets([])

        if in_pts:
            xs, ys = zip(*in_pts)
            scatter_in.set_offsets(list(zip(xs, ys)))
        else:
            scatter_in.set_offsets([])

        count_text.set_text(
            f'Zone: X[{X_MIN},{X_MAX}]  Y[{Y_MIN},{Y_MAX}]\n'
            f'Points in zone: {cnt}'
        )
        return scatter_all, scatter_in, count_text

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)  # noqa: F841
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
