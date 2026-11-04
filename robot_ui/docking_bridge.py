"""Qt/ROS bridge for docking. Service requests never block the Qt event loop."""
import json
from pathlib import Path
import time

from PyQt6.QtCore import QObject, QTimer, pyqtSignal
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger


class DockingBridge(QObject):
    updated = pyqtSignal(str, object)
    replied = pyqtSignal(str, bool, str)
    error = pyqtSignal(str)
    SERVICES = {
        'start': '/dock_mission/start', 'cancel': '/dock_mission/cancel',
        'controller_cancel': '/v_dock/cancel',
        'pre': '/dock_calibration/save_pre', 'terminal': '/dock_calibration/save_terminal',
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node('docking_ui_bridge', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.permit = False
        self.latest, self.received, self.pending = {}, {}, {}
        self._closed = False
        self.clients = {key: self.node.create_client(Trigger, topic)
                        for key, topic in self.SERVICES.items()}
        self.heartbeat = self.node.create_publisher(Bool, '/dock_ui/heartbeat', 1)
        self.estop = self.node.create_publisher(Bool, '/velocity_arbiter/emergency_stop', 1)
        self.charge = self.node.create_publisher(Bool, '/charge_command', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, topic in (('mission', '/dock_mission/status'), ('controller', '/v_dock/status')):
            self.node.create_subscription(String, topic,
                lambda msg, key=name: self._json(key, msg), qos)
        self.node.create_subscription(String, '/velocity_arbiter/state',
                                      lambda msg: self._json('arbiter', msg), 1)
        self._odom_topic = None
        self._odom_subscription = None
        self._nav_queries = {}
        self._nav_clients = {
            'nav2_map': self.node.create_client(GetParameters, '/map_server/get_parameters'),
            'nav2_odom': self.node.create_client(GetParameters, '/controller_server/get_parameters'),
        }
        self._last_nav_check = 0.
        self.node.create_subscription(Bool, '/charging_status',
                                      lambda msg: self._update('charging', msg.data), 1)
        self.node.create_subscription(Float32, '/battery_level',
                                      lambda msg: self._update('battery', msg.data), 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)
        self._last_heartbeat = 0.

    def _update(self, key, data):
        self.latest[key], self.received[key] = data, time.monotonic()
        self.updated.emit(key, data)

    def _json(self, key, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self._update(key, data)
        except (ValueError, TypeError):
            pass

    def _odom(self, message):
        twist = message.twist.twist
        self._update('odom', {'linear': twist.linear.x, 'angular': twist.angular.z})

    def _check_navigation(self):
        now = time.monotonic()
        if now-self._last_nav_check < 1.:
            return
        self._last_nav_check = now
        for key, name in (('nav2_map', 'yaml_filename'), ('nav2_odom', 'odom_topic')):
            pending = self._nav_queries.get(key)
            if pending is not None:
                future, started = pending
                if now-started <= 3.:
                    continue
                self._nav_queries.pop(key)
                future.cancel()
            client = self._nav_clients[key]
            if client.service_is_ready():
                future = client.call_async(GetParameters.Request(names=[name]))
                self._nav_queries[key] = (future, now)
                future.add_done_callback(lambda result, name=key: self._navigation_result(name, result))

    def _navigation_result(self, key, future):
        pending = self._nav_queries.get(key)
        if self._closed or pending is None or pending[0] is not future:
            return
        self._nav_queries.pop(key)
        try:
            value = future.result().values[0].string_value.strip()
            if not value:
                raise ValueError('Nav2 chưa cung cấp '+key)
            if key == 'nav2_map':
                path = Path(value).expanduser()
                if not path.is_absolute():
                    from ament_index_python.packages import get_package_share_directory
                    path = Path(get_package_share_directory('view_robot_pkg'))/path
                path = path.resolve(strict=True)
                if not path.is_file():
                    raise ValueError('Không tìm thấy map Nav2: '+str(path))
                value = str(path)
            else:
                value = self.node.resolve_topic_name(value)
                if value != self._odom_topic:
                    if self._odom_subscription is not None:
                        self.node.destroy_subscription(self._odom_subscription)
                    self.latest.pop('odom', None)
                    self.received.pop('odom', None)
                    self._odom_subscription = self.node.create_subscription(
                        Odometry, value, self._odom, qos_profile_sensor_data)
                    self._odom_topic = value
            self._update(key, value)
        except Exception as exc:
            self.received.pop(key, None)
            self._update('navigation_error', str(exc))

    def fresh(self, key, timeout=1.):
        return key in self.received and time.monotonic()-self.received[key] < timeout

    def request(self, key):
        if key in self.pending:
            return False
        client = self.clients[key]
        if not client.service_is_ready():
            self.replied.emit(key, False, 'Dịch vụ chưa sẵn sàng: '+self.SERVICES[key])
            return False
        try:
            future = client.call_async(Trigger.Request())
            self.pending[key] = (future, time.monotonic())
            future.add_done_callback(lambda result, name=key: self._done(name, result))
            return True
        except Exception as exc:
            self.replied.emit(key, False, str(exc))
            return False

    def _done(self, key, future):
        # Keep late replies observable so a cancelled start cannot resume motion.
        self.pending.pop(key, None)
        try:
            response = future.result()
            self.replied.emit(key, response.success, response.message)
        except Exception as exc:
            self.replied.emit(key, False, str(exc))

    def set_permit(self, enabled):
        self.permit = enabled
        self.heartbeat.publish(Bool(data=enabled))

    def emergency_stop(self, enabled):
        self.estop.publish(Bool(data=enabled))

    def _tick(self):
        try:
            for _ in range(8):
                self.executor.spin_once(timeout_sec=0)
            self._check_navigation()
            now = time.monotonic()
            if now-self._last_heartbeat >= .1:
                self.heartbeat.publish(Bool(data=self.permit))
                self._last_heartbeat = now
            for key, (future, started) in list(self.pending.items()):
                if now-started > 8.:
                    self.pending[key] = (future, float('inf'))
                    self.replied.emit(key, False, 'Quá hạn phản hồi; chưa xác nhận thao tác hoàn tất.')
        except Exception as exc:
            self.permit = False
            self.error.emit(str(exc))

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.timer.stop()
        self.set_permit(False)
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1.)
        self.node.destroy_node()
        self.context.try_shutdown()
