"""Independent ROS executor keeps UI/TTS work out of the velocity control loop."""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from motion_commands import validate_motion
from motion_controller import MotionController
from motion_navigation import NavigationTask, relative_target


class RosMotionController:
    def __init__(self, status):
        self.lock = threading.RLock()
        self.node = Node('robot_ui_motion')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel_sources/ui', 1)
        self.arbiter_state = None
        self.arbiter_stamp = 0.0
        self._pending = None
        self._claim_future = None
        self._release_future = None
        self._leased = False
        self._claim_started = 0.0
        self._control = self.node.create_client(SetBool, '/velocity_arbiter/ui_control')
        self.node.create_subscription(String, '/velocity_arbiter/state', self._state, 1)
        self.controller = MotionController(self._publish, status)
        self.status = status
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self.node)
        self.navigation = NavigationTask(
            ActionClient(self.node, NavigateToPose, '/navigate_to_pose'), status, self.lock)
        self.ui_heartbeat = time.monotonic()
        self.node.create_subscription(Odometry, '/odomfromSTM32', self._odom, qos_profile_sensor_data)
        for side in ('front', 'rear'):
            self.node.create_subscription(LaserScan, f'/{side}_lidar/scan',
                                          lambda msg, side=side: self._scan(side, msg), qos_profile_sensor_data)
        self.node.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._run, name='robot-motion', daemon=True)
        self.thread.start()

    def _publish(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.publisher.publish(msg)

    def _odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        with self.lock:
            self.controller.update_odom(pose.position.x, pose.position.y, yaw)

    def _scan(self, side, msg):
        with self.lock:
            self.controller.update_scan(side, msg.ranges, msg.range_min, msg.range_max)

    def _state(self, msg):
        try:
            state = json.loads(msg.data)
            if (not isinstance(state, dict) or not isinstance(state.get('owner'), str)
                    or any(type(state.get(key)) is not bool
                           for key in ('healthy', 'ui_granted', 'emergency_stop'))):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.arbiter_state = state
            self.arbiter_stamp = time.monotonic()

    def _ownership_error(self):
        outputs = self.node.get_publishers_info_by_topic('/cmd_vel')
        if not outputs:
            return 'Bộ điều khiển chuyển động chưa chạy. Hãy khởi động hệ thống điều hướng trước khi ra lệnh.'
        if len(outputs) != 1 or outputs[0].node_name != 'velocity_arbiter':
            return 'Đầu ra /cmd_vel chưa đi qua duy nhất velocity_arbiter; cần khởi động lại các nguồn đã remap.'
        state = self.arbiter_state
        if state is None or time.monotonic() - self.arbiter_stamp > 0.3:
            return 'Mất heartbeat của bộ phân xử vận tốc.'
        if not state['healthy'] or state['emergency_stop']:
            return 'Bộ phân xử chưa sẵn sàng hoặc đang dừng khẩn cấp.'
        if self.publisher.get_subscription_count() == 0:
            return 'Không có bộ nhận đầu vào vận tốc giao diện.'
        if time.monotonic() - self.ui_heartbeat > 0.75:
            return 'Mất kết nối với giao diện điều khiển.'
        return None

    def heartbeat(self):
        self.ui_heartbeat = time.monotonic()

    def start(self, data):
        validated = validate_motion(data)
        if validated['actions'][0]['type'] == 'stop':
            self.stop()
            return
        with self.lock:
            if not self.thread.is_alive():
                raise ValueError('Luồng ROS điều khiển đã ngừng hoạt động.')
            if (self.navigation.active or self.controller.active or self._pending is not None or self._claim_future is not None
                    or self._release_future is not None or self._leased):
                raise ValueError('Đang chạy hoặc đang chuyển quyền điều khiển; hãy chờ/dừng trước.')
            if validated['actions'][0]['type'] == 'navigate_move':
                self._start_navigation(validated['actions'][0])
                return
            error = self._ownership_error() or self.controller.safety_error()
            if error:
                raise ValueError(error)
            if not self._control.service_is_ready():
                raise ValueError('Dịch vụ cấp quyền vận tốc chưa sẵn sàng.')
            self._pending = validated
            self._claim_started = time.monotonic()
            try:
                self._claim_future = self._control.call_async(SetBool.Request(data=True))
                self._claim_future.add_done_callback(self._claimed)
            except Exception:
                self._pending = None
                raise
            self.status('Đang xin quyền chuyển động từ bộ phân xử.')

    @property
    def busy(self):
        with self.lock:
            return (self.navigation.active or self.controller.active or self._pending is not None
                    or self._claim_future is not None or self._release_future is not None or self._leased)

    def _start_navigation(self, action):
        error = self._ownership_error()
        if error:
            raise ValueError(error)
        if (self.arbiter_state['ui_granted'] or self.arbiter_state['owner']
                or self.arbiter_state.get('navigation_busy', False)):
            raise ValueError('Nguồn vận tốc khác đang điều khiển; hãy dừng trước.')
        publishers = self.node.get_publishers_info_by_topic('/cmd_vel_sources/navigation')
        if not any(p.node_name == 'controller_server' for p in publishers):
            raise ValueError('Nav2 controller chưa nối vào nhánh navigation của arbiter.')
        try:
            transform = self._tf.lookup_transform('map', 'base_link', Time())
        except Exception as exc:
            raise ValueError(f'Không lấy được TF map → base_link: {exc}') from exc
        now = self.node.get_clock().now()
        stamp = Time.from_msg(transform.header.stamp, clock_type=now.clock_type)
        age = (now - stamp).nanoseconds / 1e9
        if not 0 <= age <= 0.75:
            raise ValueError('TF pose hiện tại đã cũ hoặc sai thời gian.')
        p, q = transform.transform.translation, transform.transform.rotation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-6:
            raise ValueError('Quaternion TF không hợp lệ.')
        qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        x, y, yaw = relative_target(p.x, p.y, yaw, action['direction'], action['distance_meters'])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.navigation.start(goal)

    def _claimed(self, future):
        with self.lock:
            self._claim_future = None
            try:
                result = future.result()
                if not result.success:
                    self._pending = None
                    self.status(f'Không thể thực hiện: {result.message}')
                    return
                self._leased = True
                if self._pending is None:
                    self._release()  # Stop arrived while the service was pending.
            except Exception as exc:
                self._pending = None
                self.status(f'Lỗi cấp quyền điều khiển: {exc}')

    def _release(self):
        if self._leased and self._release_future is None:
            self._leased = False
            self._release_future = self._control.call_async(SetBool.Request(data=False))
            self._release_future.add_done_callback(self._released)

    def _released(self, future):
        with self.lock:
            self._release_future = None
            try:
                future.result()
            except Exception as exc:
                self.status(f'Lỗi trả quyền; arbiter sẽ dừng theo timeout: {exc}')

    def stop(self):
        with self.lock:
            self.navigation.stop()
            if self._pending is not None:
                self.status('Đã hủy yêu cầu chuyển động đang chờ cấp quyền.')
            self._pending = None
            try:
                if self.controller.active:
                    self.controller.stop()
            except Exception as exc:
                self.status(f'Lỗi gửi vận tốc dừng: {exc}')
            finally:
                self._release()

    def _tick(self):
        with self.lock:
            if self.navigation.active:
                error = self._ownership_error()
                if not error and (self.arbiter_state['ui_granted']
                                  or self.arbiter_state['owner'] not in ('', 'navigation')):
                    error = 'Nguồn vận tốc khác đã giành quyền điều khiển.'
                if error:
                    self.navigation.stop(f'{error} Đang hủy Nav2.')
                self.navigation.tick()
            if self.controller.active or self._pending is not None:
                error = self._ownership_error()
                if error:
                    self.status(error)
                    self.stop()
                    return
            if self._pending is not None:
                state = self.arbiter_state
                if self._leased and state['ui_granted'] and state['owner'] == 'ui':
                    data, self._pending = self._pending, None
                    try:
                        self.controller.start(data)
                    except Exception as exc:
                        self.status(f'Không thể thực hiện: {exc}')
                        self._release()
                        return
                elif time.monotonic() - self._claim_started > 0.75:
                    self.status('Hết thời gian chờ quyền điều khiển.')
                    self.stop()
                    return
                else:
                    return
            if self.controller.active:
                state = self.arbiter_state
                if not state['ui_granted'] or state['owner'] != 'ui':
                    self.status(f"Đã mất quyền điều khiển cho {state['owner'] or 'nguồn khác'}; hủy chuỗi.")
                    self.stop()
                    return
            self.controller.tick()
            if not self.controller.active:
                self._release()

    def _run(self):
        try:
            self.executor.spin()
        except Exception as exc:
            with self.lock:
                try:
                    self.stop()
                finally:
                    self.status(f'Lỗi kết nối điều khiển: {exc}')

    def close(self):
        self.stop()
        deadline = time.monotonic() + 5.0
        while self.navigation.active and self.thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.navigation.active:
            self.status('Chưa xác nhận Nav2 đã dừng; giữ giao diện mở để tiếp tục xử lý hủy.')
            return False
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        return True
