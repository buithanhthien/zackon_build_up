"""Elevator test transitions, driven on the existing motion executor under its lock."""
import time
from elevator_detector import freshness_error


class ElevatorSequence:
    def __init__(self, port, clock=time.monotonic):
        self.port, self.clock = port, clock
        self.state = 'idle'
        self.message = 'Chưa bắt đầu.'
        self.last_sample = 0.0
        self.streak = 0

    @property
    def active(self):
        return self.state not in ('idle', 'done')

    def report(self, message):
        if message != self.message:
            self.message = message
            self.port.status('Thang máy: ' + message)

    def start(self):
        if self.active:
            raise ValueError('Test thang máy đang chạy hoặc đang dừng.')
        self.port.elevator_preflight()
        self.state = 'preflight'
        self.started = self.clock()
        self.streak = 0
        self.last_sample = 0.0
        self.report('Đang kiểm tra map trên ROS; chưa di chuyển.')

    def stop(self, reason='Đã yêu cầu dừng.'):
        if not self.active:
            return
        self.state = 'stopping'
        self.port.elevator_halt()
        self.report(reason + ' Chờ Nav2 và arbiter xác nhận trả quyền manual.')

    def tick(self):
        if not self.active:
            return
        try:
            if self.state == 'stopping':
                if self.port.elevator_released():
                    self.state = 'done'
                    self.report('Đã dừng; arbiter đã nhả quyền tự động, manual khả dụng.')
                return
            error = self.port.elevator_safety()
            if error:
                self.stop(error)
                return
            if self.state == 'preflight':
                if self.port.elevator_map_ready():
                    self.port.elevator_navigate()
                    self.state = 'navigating'
                    self.report('Đang đến X5_elevator.')
                elif self.clock() - self.started > 5:
                    self.stop('Không xác nhận được map đang nạp.')
            elif self.state == 'navigating':
                outcome = self.port.navigation.outcome
                if not self.port.navigation.active:
                    if outcome != 'succeeded':
                        self.stop('Nav2 không hoàn tất thành công.')
                    else:
                        self.state = 'acquiring'
                        self.started = self.clock()
            elif self.state == 'acquiring':
                if self.port.elevator_hold():
                    self.state = 'waiting'
                    self.port.detector.enable()
                    self.last_sample = self.clock()
                    self.report('Đã đến waypoint; đang chờ nhận diện.')
                elif self.clock() - self.started > 5:
                    self.stop('Không lấy được quyền giữ robot đứng yên.')
            elif self.state in ('waiting', 'moving'):
                ready, error, (label, confidence, stamp), frame_stamp = self.port.detector.snapshot()
                now = self.clock()
                freshness = freshness_error(stamp, frame_stamp, now)
                valid = ready and not freshness
                threshold = self.port.detector.threshold
                available = valid and label == 'available' and confidence >= threshold
                if self.state == 'moving':
                    if not available:
                        self.stop(error or 'Mất xác nhận available; đã ngắt chuyển động.')
                    elif not self.port.controller.active:
                        self.stop('Hoàn thành đi 2 m và quay 180°.' if
                                  self.port.controller.outcome == 'succeeded' else 'Chuyển động bị ngắt.')
                else:
                    if not available:
                        self.streak = 0
                        self.report(error or freshness or (
                            'unavailable — tiếp tục đứng yên, không timeout.'
                            if valid and label == 'unavailable' else
                            f'unknown: điểm dự đoán chưa đạt ngưỡng {threshold:.0%} — giữ đứng yên.'))
                    elif stamp > self.last_sample:
                        self.streak += 1
                        if self.streak >= 3:
                            self.port.elevator_move()
                            self.state = 'moving'
                            self.report('available: đi thẳng 2 m, sau đó quay 180°.')
                    self.last_sample = max(self.last_sample, stamp)
        except Exception as exc:
            self.stop(f'Lỗi: {exc}')
