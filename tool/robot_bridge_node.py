#!/usr/bin/env python3
"""
Robot → VPS bridge node.
- Converts /map (OccupancyGrid) → PNG, uploads to VPS when map changes.
- Streams /amcl_pose to VPS via WebSocket at ~10 Hz.

Usage:
    python3 robot_bridge_node.py --vps http://YOUR_VPS_IP:8000

Dependencies:
    pip install numpy pillow requests websocket-client
"""
import argparse, io, json, math, threading, time
import numpy as np
from PIL import Image
import requests, websocket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


def quat_to_yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class BridgeNode(Node):
    def __init__(self, vps_url: str):
        super().__init__('robot_bridge_node')
        self._vps = vps_url.rstrip('/')
        self._ws = None
        self._ws_lock = threading.Lock()
        self._last_map_hash = None

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, qos)
        threading.Thread(target=self._ws_loop, daemon=True).start()
        self.get_logger().info(f'Bridge started → {self._vps}')

    def _ws_loop(self):
        ws_url = self._vps.replace('http', 'ws') + '/ws/pose'
        while True:
            try:
                ws = websocket.create_connection(ws_url, timeout=5)
                with self._ws_lock:
                    self._ws = ws
                self.get_logger().info('WebSocket connected')
                # Stay alive until _pose_cb clears _ws on send failure
                while True:
                    with self._ws_lock:
                        if self._ws is None:
                            break
                    time.sleep(0.5)
            except Exception as e:
                self.get_logger().warn(f'WS error: {e}, retrying in 3s')
                with self._ws_lock:
                    self._ws = None
                time.sleep(3)

    def _map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        map_hash = hash(data.tobytes())
        if map_hash == self._last_map_hash:
            return
        self._last_map_hash = map_hash

        img = np.full((h, w), 128, dtype=np.uint8)
        img[data == 0]   = 255  # free  → white
        img[data == 100] = 0    # wall  → black
        img = np.flipud(img)    # ROS origin is bottom-left

        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format='PNG')
        buf.seek(0)

        meta = json.dumps({
            'resolution': msg.info.resolution,
            'origin_x':   msg.info.origin.position.x,
            'origin_y':   msg.info.origin.position.y,
            'width': w, 'height': h,
        })
        try:
            requests.post(f'{self._vps}/map',
                          files={'file': ('map.png', buf, 'image/png')},
                          data={'meta': meta}, timeout=10)
            self.get_logger().info('Map uploaded')
        except Exception as e:
            self.get_logger().warn(f'Map upload failed: {e}')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        payload = json.dumps({'x': p.position.x, 'y': p.position.y,
                               'yaw': quat_to_yaw(p.orientation)})
        with self._ws_lock:
            if self._ws:
                try:
                    self._ws.send(payload)
                except Exception:
                    try:
                        self._ws.close()
                    except Exception:
                        pass
                    self._ws = None  # _ws_loop detects None and reconnects immediately


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vps', default='http://localhost:8000')
    args = parser.parse_args()
    rclpy.init()
    rclpy.spin(BridgeNode(args.vps))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
