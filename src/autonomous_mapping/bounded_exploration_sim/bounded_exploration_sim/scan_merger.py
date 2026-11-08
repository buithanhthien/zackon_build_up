"""Simulation adapter: merge both scans into base_link for SLAM/Nav2."""

from math import atan2, cos, hypot, inf, isfinite, pi, sin

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class MergeLidar(Node):
    def __init__(self):
        super().__init__('bounded_scan_merger')
        self.declare_parameter('self_filter_range', 0.35)
        self.declare_parameter('publish_period', 0.3)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._scans = {}
        self._last_publish_ns = None
        self._publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        for name in ('front', 'rear'):
            self.create_subscription(
                LaserScan, f'/{name}_lidar/scan',
                lambda scan, side=name: self._callback(side, scan),
                qos_profile_sensor_data)

    def _callback(self, side, scan):
        self._scans[side] = scan
        if len(self._scans) != 2:
            return
        stamps = [Time.from_msg(message.header.stamp).nanoseconds
                  for message in self._scans.values()]
        # Both simulated sensors run at 10 Hz. Wait for their matching pair;
        # merging the previous rear scan while turning creates ghost walls.
        if abs(stamps[0] - stamps[1]) > 5_000_000:
            return
        scan = self._scans['front']
        stamp_ns = Time.from_msg(scan.header.stamp).nanoseconds
        period_ns = int(self.get_parameter('publish_period').value * 1e9)
        if (self._last_publish_ns is not None and
                stamp_ns - self._last_publish_ns < period_ns):
            return

        transforms = {}
        try:
            for key, message in self._scans.items():
                transforms[key] = self._tf.lookup_transform(
                    'base_link', message.header.frame_id, Time())
        except TransformException:
            return

        beam_count = 720
        output = LaserScan()
        output.header.stamp = scan.header.stamp
        output.header.frame_id = 'base_link'
        output.angle_min = -pi
        output.angle_max = pi
        output.angle_increment = 2.0 * pi / (beam_count - 1)
        output.scan_time = scan.scan_time
        output.range_min = self.get_parameter('self_filter_range').value
        output.range_max = min(message.range_max for message in self._scans.values())
        ranges = [inf] * beam_count

        for key, message in self._scans.items():
            transform = transforms[key].transform
            q = transform.rotation
            yaw = atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            origin_x = transform.translation.x
            origin_y = transform.translation.y
            for index, distance in enumerate(message.ranges):
                if (not isfinite(distance) or
                        distance < output.range_min or
                        distance > message.range_max):
                    continue
                angle = message.angle_min + index * message.angle_increment + yaw
                x = origin_x + distance * cos(angle)
                y = origin_y + distance * sin(angle)
                base_range = hypot(x, y)
                if not output.range_min <= base_range <= output.range_max:
                    continue
                base_angle = atan2(y, x)
                output_index = round((base_angle + pi) / output.angle_increment)
                output_index = max(0, min(beam_count - 1, output_index))
                ranges[output_index] = min(ranges[output_index], base_range)

        output.ranges = ranges
        self._publisher.publish(output)
        self._last_publish_ns = stamp_ns


def main():
    rclpy.init()
    node = MergeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
