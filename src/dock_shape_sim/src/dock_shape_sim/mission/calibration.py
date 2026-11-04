"""Hardware teaching services. No Qt, no simulation, no velocity publishers."""
import json
from math import atan2
from pathlib import Path
import time
import uuid

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .profile import compose, inverse, rear_pose, load_profile, save_profile, update_reference
from ..perception.v_shape import detect_profile, scan_points


def pose_dict(pose):
    q = pose.orientation
    return dict(x=float(pose.position.x), y=float(pose.position.y),
                yaw=atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


def teach_pre(profile, sample, found):
    if found is None or not found.full_shape:
        raise ValueError('Pre-docking phải nhìn thấy đầy đủ hình chữ V.')
    if found.x < .65 or abs(found.y) > .15 or abs(found.yaw) > .30:
        raise ValueError('Căn đuôi về trạm, cách trạm ít nhất 0.65 m và thấy đủ chữ V.')
    if profile.get('terminal_sample', {}).get('session_id') != sample['session_id']:
        profile.pop('terminal_sample', None)
    profile['pre_docking'] = dict(sample['map_pose'], odom_pose=sample['odom_pose'],
        stamp=sample['stamp'], session_id=sample['session_id'], marker_in_odom=compose(
            rear_pose(sample['odom_pose']), dict(x=found.x, y=found.y, yaw=found.yaw)))
    profile['map_to_odom'] = compose(sample['map_pose'], inverse(sample['odom_pose']))
    profile.pop('docked_reference', None)
    update_reference(profile)
    return profile


def teach_terminal(profile, sample, points, found):
    # Hardware integration starts with an observable terminal target. Do not
    # silently enable the upstream bounded odometry-only finish mode.
    if (found is None or not found.full_shape or found.x > .8
            or abs(found.y) > .1 or abs(found.yaw) > .3):
        raise ValueError('Mẫu sát trạm cần còn thấy đầy đủ chữ V, khoảng cách <=0.8 m và đã căn thẳng.')
    near = points[(points[:, 0] > .28) & (points[:, 0] < .9) & (abs(points[:, 1]) < .25)]
    if len(near) < 3:
        raise ValueError('Không đủ điểm LiDAR sát trạm để xác nhận dừng.')
    sample.update(near_points=near[::max(1, len(near)//220)].tolist(),
                  full_shape_visible=True,
                  observed_profile=dict(x=found.x, y=found.y, yaw=found.yaw))
    if profile.get('pre_docking', {}).get('session_id') != sample['session_id']:
        profile.pop('pre_docking', None)
    profile['terminal_sample'] = sample
    profile['map_to_odom'] = compose(sample['map_pose'], inverse(sample['odom_pose']))
    profile['docked_reference'] = dict(x=found.x, y=found.y, yaw=found.yaw,
        points=sample['near_points'], use_near_signature=True, blind_finish=False)
    return profile


class DockCalibration(Node):
    def __init__(self):
        super().__init__('dock_calibration')
        self.map_file = str(Path(self.declare_parameter('map_file', '').value).resolve())
        self._session = str(uuid.uuid4())
        self._scan = self._odom = None
        self._scan_time = self._odom_time = None
        self._mission_state = None
        self._mission_time = None
        self._mission_cancel_pending = True
        self._map_verified = False
        self._tf = Buffer(node=self)
        self._listener = TransformListener(self._tf, self)
        self.create_subscription(
            LaserScan, '/scan_rear_lidar_filter', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/dock_mission/status', self._mission_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/dock_calibration/save_pre', self._pre)
        self.create_service(Trigger, '/dock_calibration/save_terminal', self._terminal)

    def _scan_cb(self, message):
        self._scan, self._scan_time = message, time.monotonic()

    def _odom_cb(self, message):
        self._odom, self._odom_time = message, time.monotonic()

    def _mission_cb(self, message):
        try:
            data = json.loads(message.data)
            self._mission_state = data['state']
            self._mission_cancel_pending = data.get('cancel_pending', True)
            self._map_verified = data.get('map_verified', False)
            self._mission_time = time.monotonic()
        except (ValueError, KeyError, TypeError):
            pass

    def _sample(self):
        now = time.monotonic()
        if self._mission_time is None or now-self._mission_time > 1.:
            raise ValueError('Chưa xác nhận được mission đang dừng.')
        if not self._map_verified:
            raise ValueError('Map chọn khác map đang nạp hoặc map_server chưa sẵn sàng.')
        if self._mission_cancel_pending:
            raise ValueError('Chưa xác nhận đã hủy Nav2.')
        if (self._mission_state not in ('IDLE', 'DOCKED_GEOMETRY')
                and not self._mission_state.startswith('FAILED')):
            raise ValueError('Hủy docking và chờ dừng trước khi dạy mẫu.')
        if (self._scan_time is None or self._odom_time is None
                or now-self._scan_time > .8 or now-self._odom_time > .8):
            raise ValueError('Thiếu LiDAR sau hoặc odometry mới.')
        twist = self._odom.twist.twist
        if abs(twist.linear.x) > .01 or abs(twist.angular.z) > .03:
            raise ValueError('Robot phải đứng yên trước khi lưu mẫu.')
        scan_stamp = Time.from_msg(self._scan.header.stamp).nanoseconds
        odom_stamp = Time.from_msg(self._odom.header.stamp).nanoseconds
        if abs(scan_stamp-odom_stamp)/1e9 > .25 or abs(self.get_clock().now().nanoseconds-scan_stamp)/1e9 > .8:
            raise ValueError('LiDAR và odometry chưa đồng bộ.')
        tf = self._tf.lookup_transform('map', 'base_link', Time())
        if abs(Time.from_msg(tf.header.stamp).nanoseconds-odom_stamp)/1e9 > 2.5:
            raise ValueError('TF map/base_link đã cũ.')
        transform = self._tf.lookup_transform('base_link', self._scan.header.frame_id, Time()).transform
        q = tf.transform.rotation
        sample = dict(map_pose=dict(x=float(tf.transform.translation.x), y=float(tf.transform.translation.y),
            yaw=atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))),
            odom_pose=pose_dict(self._odom.pose.pose), stamp=scan_stamp/1e9, session_id=self._session)
        return sample, scan_points(self._scan, transform, True)

    def _save(self, response, terminal):
        try:
            sample, points = self._sample()
            profile = load_profile(self.map_file)
            found = detect_profile(points)
            if terminal:
                teach_terminal(profile, sample, points, found)
            else:
                teach_pre(profile, sample, found)
            profile['hardware_calibration'] = True
            saved = save_profile(self.map_file, profile)
            response.success = True
            pose = sample['map_pose']
            ready = ('Đã có đủ hai điểm.' if saved['calibration_status'] == 'ready' else
                     'Cần lưu thêm điểm predock.' if terminal else 'Cần lưu thêm điểm dock.')
            response.message = ('Đã lưu '+('dock' if terminal else 'predock')+
                f" trên map: x={pose['x']:.3f} m, y={pose['y']:.3f} m, góc={pose['yaw']:.3f} rad. "+ready)
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def _pre(self, _request, response):
        return self._save(response, False)

    def _terminal(self, _request, response):
        return self._save(response, True)


def main():
    rclpy.init()
    node = DockCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
