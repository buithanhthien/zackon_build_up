"""Hardware permission expires independently of ROS /clock and UI requests."""
import json
import time
from pathlib import Path

from std_msgs.msg import Bool, String
from rcl_interfaces.srv import GetParameters
from .mission.profile import map_identity


class DockingSafety:
    def __init__(self, node, clock=time.monotonic):
        self.required = node.declare_parameter('require_safety_permit', False).value
        self.clock = clock
        self.heartbeat = None
        self.arbiter_stamp = None
        self.arbiter = None
        self.expected_map = node.declare_parameter('expected_map_file', '').value
        self.map_error = 'Chưa xác nhận map đang chạy trên map_server.' if self.expected_map else None
        self.map_stamp = None
        self.map_pending = False
        if self.required:
            node.create_subscription(Bool, '/dock_ui/heartbeat', self._heartbeat, 1)
            node.create_subscription(String, '/velocity_arbiter/state', self._arbiter, 1)
            if self.expected_map:
                self.map_client = node.create_client(GetParameters, '/map_server/get_parameters')
                node.create_timer(1., self._check_map)

    def _check_map(self):
        if self.map_pending or not self.map_client.service_is_ready():
            return
        self.map_pending = True
        self.map_client.call_async(GetParameters.Request(names=['yaml_filename'])).add_done_callback(self._map_result)

    def _map_result(self, future):
        self.map_pending = False
        self.map_stamp = self.clock()
        try:
            filename = future.result().values[0].string_value
            actual = Path(filename)
            if not actual.is_absolute():
                from ament_index_python.packages import get_package_share_directory
                actual = Path(get_package_share_directory('view_robot_pkg'))/actual
            if not filename or map_identity(actual) != map_identity(self.expected_map):
                raise ValueError('Map chọn trong UI khác map đang nạp trên map_server.')
            self.map_error = None
        except Exception as exc:
            self.map_error = 'Chưa xác nhận đúng map: '+str(exc)

    def _heartbeat(self, message):
        self.heartbeat = self.clock() if message.data else None

    def _arbiter(self, message):
        try:
            state = json.loads(message.data)
            if (not isinstance(state, dict) or not isinstance(state.get('owner'), str)
                    or type(state.get('healthy')) is not bool
                    or type(state.get('emergency_stop')) is not bool):
                return
        except (ValueError, TypeError):
            return
        self.arbiter, self.arbiter_stamp = state, self.clock()

    def error(self):
        if not self.required:
            return None
        now = self.clock()
        if self.expected_map:
            if self.map_error:
                return self.map_error
            if self.map_stamp is None or now-self.map_stamp > 3.:
                return 'Quá hạn xác nhận map_server.'
        if self.heartbeat is None or now-self.heartbeat > .6:
            return 'Mất quyền/heartbeat giao diện docking.'
        if self.arbiter_stamp is None or now-self.arbiter_stamp > .5:
            return 'Mất heartbeat velocity_arbiter.'
        if not self.arbiter['healthy'] or self.arbiter['emergency_stop']:
            return 'Arbiter chưa sẵn sàng hoặc đang dừng khẩn cấp.'
        if self.arbiter['owner'] not in ('', 'navigation', 'docking'):
            return 'Nguồn khác đã giành quyền: '+self.arbiter['owner']
        return None
