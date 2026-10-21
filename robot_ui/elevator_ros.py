"""ROS adapter for the opt-in elevator sequence; uses the existing arbiter UI lease."""
import json
import math
from pathlib import Path
import time

from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from elevator_detector import ElevatorDetector, MODEL_DIR
from elevator_sequence import ElevatorSequence


class ElevatorMixin:
    def init_elevator(self):
        self.elevator = ElevatorSequence(self)
        self.detector = None
        self._elevator_hold = False
        self._elevator_map_valid = False
        self._map_client = self.node.create_client(GetMap, '/map_server/map')
        self._map_parameters = self.node.create_client(GetParameters, '/map_server/get_parameters')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(OccupancyGrid, '/map', self._elevator_map, qos)
        self.node.create_subscription(Image, '/camera_test/image_raw', self._elevator_frame, qos_profile_sensor_data)

    def prepare_elevator(self):
        with self.lock:
            if self.detector is None:
                self.detector = ElevatorDetector()

    def _elevator_frame(self, msg):
        if self.detector is None:
            return
        try:
            import numpy as np
            if msg.encoding not in ('rgb8', 'bgr8') or msg.height <= 0 or msg.width <= 0:
                raise ValueError('Camera cần rgb8 hoặc bgr8.')
            if msg.step < msg.width * 3 or len(msg.data) < msg.height * msg.step:
                raise ValueError('Kích thước dữ liệu frame camera không hợp lệ.')
            rgb = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.step).reshape(msg.height, msg.step)
            rgb = rgb[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                rgb = rgb[:, :, ::-1]
            # Reject delayed frames, not merely recent receipt of old camera data.
            from rclpy.time import Time
            now = self.node.get_clock().now()
            age = (now - Time.from_msg(msg.header.stamp, clock_type=now.clock_type)).nanoseconds / 1e9
            if not 0 <= age <= 0.75:
                self.detector.reject_frame(f'Frame camera sai thời gian hoặc quá cũ: {age:.2f} s.')
                return
            self.detector.submit(rgb.copy(), time.monotonic() - age)
        except Exception as exc:
            self.detector.reject_frame(f'Lỗi frame camera: {exc}')

    def _elevator_map(self, msg):
        with self.lock:
            if hasattr(self, '_expected_map'):
                self._elevator_map_valid = self._matches_elevator_map(msg)

    def _matches_elevator_map(self, msg):
        import numpy as np
        cells, resolution, origin = self._expected_map
        info = msg.info
        q = info.origin.orientation
        return (msg.header.frame_id == 'map' and info.width == cells.shape[1]
                and info.height == cells.shape[0] and abs(info.resolution - resolution) < 1e-6
                and abs(info.origin.position.x - origin[0]) < 1e-5
                and abs(info.origin.position.y - origin[1]) < 1e-5
                and abs(q.x) < 1e-6 and abs(q.y) < 1e-6
                and abs(q.z - math.sin(origin[2]/2)) < 1e-5
                and abs(q.w - math.cos(origin[2]/2)) < 1e-5
                and np.array_equal(np.asarray(msg.data), cells.ravel()))

    def elevator_preflight(self):
        if self.busy:
            raise ValueError('Có tác vụ điều khiển đang chạy; hãy dừng trước.')
        if not self.thread.is_alive():
            raise ValueError('Luồng ROS không hoạt động.')
        error = self._ownership_error() or self.controller.safety_error()
        if error:
            raise ValueError(error)
        if self.detector is None:
            raise ValueError('Chưa chuẩn bị detector.')
        ready, error, _, stamp = self.detector.snapshot()
        if not ready or time.monotonic() - stamp > 1:
            raise ValueError(error or 'Camera chưa sẵn sàng / frame đã cũ.')
        if not all((self._control.service_is_ready(), self.navigation.client.server_is_ready(),
                    self._map_client.service_is_ready(), self._map_parameters.service_is_ready())):
            raise ValueError('Nav2 / map_server / arbiter chưa sẵn sàng.')
        root = MODEL_DIR.parents[1]
        waypoints = json.loads((root / 'robot_ui/waypoints.json').read_text())
        self._elevator_waypoint = waypoints['X5_elevator']
        if self._elevator_waypoint['map_name'] != 'X5_19102026':
            raise ValueError('Waypoint không thuộc map X5_19102026.')
        import numpy as np
        import yaml
        from PIL import Image as PILImage
        map_path = root / 'src/view_robot/maps/X5_19102026.yaml'
        metadata = yaml.safe_load(map_path.read_text())
        if metadata.get('mode', 'trinary') != 'trinary':
            raise ValueError('Map test phải ở chế độ trinary.')
        pixels = np.asarray(PILImage.open(map_path.parent / metadata['image']).convert('L'))
        probability = pixels / 255.0 if metadata['negate'] else (255.0 - pixels) / 255.0
        cells = np.full(pixels.shape, -1, dtype=np.int8)
        cells[probability > metadata['occupied_thresh']] = 100
        cells[probability < metadata['free_thresh']] = 0
        self._expected_map = (np.flipud(cells), metadata['resolution'], metadata['origin'])
        self._elevator_map_valid = False
        self._map_future = self._map_client.call_async(GetMap.Request())
        self._parameter_future = self._map_parameters.call_async(GetParameters.Request(names=['yaml_filename']))

    def elevator_map_ready(self):
        if not self._map_future.done() or not self._parameter_future.done():
            return False
        values = self._parameter_future.result().values
        if len(values) != 1 or Path(values[0].string_value).stem != 'X5_19102026':
            raise ValueError('Map server không nạp X5_19102026.')
        self._elevator_map_valid = self._matches_elevator_map(self._map_future.result().map)
        if not self._elevator_map_valid:
            raise ValueError('Nội dung map ROS không khớp X5_19102026.')
        return True

    def elevator_navigate(self):
        self._start_navigation(None, waypoint=self._elevator_waypoint)

    def elevator_hold(self):
        state = self.arbiter_state
        if self._leased:
            return state['ui_granted'] and state['owner'] == 'ui'
        if self._claim_future is None and not state.get('navigation_busy') and not state['owner']:
            self._elevator_hold = True
            self._pending = 'elevator_hold'
            self._claim_started = time.monotonic()
            self._claim_future = self._control.call_async(SetBool.Request(data=True))
            self._claim_future.add_done_callback(self._claimed)
        return False

    def elevator_move(self):
        self.controller.start({'intent': 'motion', 'actions': [
            {'type': 'move', 'direction': 'forward', 'distance_meters': 2.0},
            {'type': 'rotate', 'direction': 'left', 'degrees': 180.0}]})

    def elevator_safety(self):
        error = self._ownership_error() or self.controller.safety_error()
        if error:
            return error
        if self.elevator.state in ('preflight', 'navigating', 'acquiring'):
            ready, detector_error, _, stamp = self.detector.snapshot()
            if not ready or time.monotonic() - stamp > 1:
                return detector_error or 'Mất camera; hủy hành trình.'
        if self.elevator.state not in ('preflight',) and not self._elevator_map_valid:
            return 'Map đã thay đổi hoặc không còn hợp lệ.'
        if self.elevator.state in ('waiting', 'moving') and (
                not self._leased or not self.arbiter_state['ui_granted']
                or self.arbiter_state['owner'] != 'ui'):
            return 'Đã mất quyền giữ robot đứng yên.'
        return None

    def elevator_halt(self):
        self._elevator_hold = False
        if self.detector:
            self.detector.disable()
        self._stop_motion()
        self._publish(0.0, 0.0)

    def elevator_released(self):
        state = self.arbiter_state
        return (not self.navigation.active and not self.controller.active
                and self._pending is None and self._claim_future is None
                and self._release_future is None and not self._leased
                and state is not None and time.monotonic() - self.arbiter_stamp < 0.3
                and state['healthy'] and not state['emergency_stop']
                and not state['ui_granted'] and not state.get('navigation_busy', True)
                and state['owner'] in ('', 'teleop'))
