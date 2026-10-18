#!/usr/bin/env python3
"""Publish colour frames for the standalone camera preview window."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .camera import Camera


class CameraPreviewNode(Node):
    def __init__(self, context=None):
        super().__init__('camera_preview_node', context=context)
        self.declare_parameter('camera_url', 'realsense')
        self.declare_parameter('camera_serial', '')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('camera_format', 'YUYV')
        self.publisher = self.create_publisher(
            Image, '/camera_test/image_raw',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.camera = Camera(
            source=self.get_parameter('camera_url').value,
            serial=self.get_parameter('camera_serial').value,
            width=self.get_parameter('camera_width').value,
            height=self.get_parameter('camera_height').value,
            fps=self.get_parameter('camera_fps').value,
            pixel_format=self.get_parameter('camera_format').value)
        self.timer = self.create_timer(
            1.0 / self.get_parameter('camera_fps').value, self.publish_frame)

    def publish_frame(self):
        ok, frame = self.camera.read()
        if not ok:
            raise RuntimeError('Camera không trả về frame màu')
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tracking_camera_color_optical_frame'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.publisher.publish(msg)

    def destroy_node(self):
        self.timer.cancel()
        self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPreviewNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
