"""Sequential motion state machine. All methods must share the adapter lock."""
import math
import time
from motion_commands import validate_motion

LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.314
SENSOR_TIMEOUT = 0.75
OBSTACLE_CLEARANCE = 0.45  # metres from either lidar, conservative full-scan check


class MotionController:
    def __init__(self, publish, status, clock=time.monotonic):
        self.publish = publish
        self.status = status
        self.clock = clock
        self.queue = []
        self.current = None
        self.odom = None
        self.scans = {}
        self.last_tick = clock()
        self.progress = 0.0

    @property
    def active(self):
        return self.current is not None or bool(self.queue)

    def update_odom(self, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.odom = None
            return
        now = self.clock()
        if self.active and self.odom:
            old_x, old_y, old_yaw, stamp = self.odom
            distance = math.hypot(x - old_x, y - old_y)
            delta = math.atan2(math.sin(yaw - old_yaw), math.cos(yaw - old_yaw))
            if distance > 0.5 or abs(delta) > 0.5:
                self.stop('Lỗi: odometry thay đổi bất thường.')
            elif self.current:
                action = self.current
                if action['type'] == 'move':
                    sign = 1 if action['direction'] == 'forward' else -1
                    self.progress += sign * ((x - old_x) * math.cos(old_yaw) + (y - old_y) * math.sin(old_yaw))
                else:
                    self.progress += delta * (1 if action['direction'] == 'left' else -1)
        self.odom = (x, y, yaw, now)

    def update_scan(self, side, ranges, minimum, maximum):
        values = [r for r in ranges if math.isfinite(r) and minimum <= r <= maximum]
        # An all-invalid scan is not evidence that the space is clear.
        self.scans[side] = (min(values) if values else 0.0, self.clock())

    def safety_error(self):
        now = self.clock()
        if self.odom is None or now - self.odom[3] > SENSOR_TIMEOUT:
            return 'Mất dữ liệu odometry.'
        for side in ('front', 'rear'):
            scan = self.scans.get(side)
            if scan is None or now - scan[1] > SENSOR_TIMEOUT:
                return f'Mất dữ liệu lidar {side}.'
            if scan[0] < OBSTACLE_CLEARANCE:
                return f'Có vật cản hoặc dữ liệu lidar {side} không hợp lệ.'
        return None

    def start(self, data):
        actions = validate_motion(data)['actions']
        if actions[0]['type'] == 'stop':
            self.stop()
            return
        if self.active:
            raise ValueError('Robot đang chạy; hãy dừng trước khi gửi chuỗi mới.')
        error = self.safety_error()
        if error:
            raise ValueError(error)
        self.queue = actions
        self.last_tick = self.clock()
        self.status('Bắt đầu chuỗi chuyển động.')

    def stop(self, reason='Đã dừng chuyển động và hủy các bước còn lại.'):
        was_active = self.active
        self.queue = []
        self.current = None
        try:
            self.publish(0.0, 0.0)
        except Exception:
            self.status('Lỗi: không gửi được lệnh dừng; hàng đợi đã bị hủy.')
            raise
        if was_active:
            self.status(reason)

    def tick(self):
        now = self.clock()
        if not self.active:
            self.last_tick = now
            return
        error = self.safety_error()
        if now - self.last_tick > SENSOR_TIMEOUT:
            error = 'Vòng điều khiển bị gián đoạn.'
        self.last_tick = now
        if error:
            self.stop(error)
            return
        try:
            if self.current is None:
                self.current = self.queue.pop(0)
                self.started = now
                self.progress = 0.0
                self.status(f"Đang thực hiện: {self.current}")
            action = self.current
            kind = action['type']
            elapsed = now - self.started
            target = None
            if 'distance_meters' in action:
                target = action['distance_meters']
            elif kind == 'rotate':
                target = action.get('revolutions', 0) * 2 * math.pi + math.radians(action.get('degrees', 0))
            completed = (target is not None and self.progress >= target) or (
                'duration_seconds' in action and elapsed >= action['duration_seconds'])
            if completed:
                self.publish(0.0, 0.0)
                self.current = None
                self.status('Hoàn thành bước.' if self.queue else 'Hoàn thành chuỗi chuyển động.')
                return
            if target is not None and elapsed > target / (LINEAR_SPEED if kind == 'move' else ANGULAR_SPEED) * 2 + 5:
                self.stop('Lỗi: không đạt mục tiêu trong thời gian cho phép.')
                return
            linear = LINEAR_SPEED * (1 if action['direction'] == 'forward' else -1) if kind == 'move' else 0.0
            angular = ANGULAR_SPEED * (1 if action['direction'] == 'left' else -1) if kind != 'move' else 0.0
            self.publish(linear, angular)
        except Exception as exc:
            self.stop(f'Lỗi điều khiển: {exc}')
