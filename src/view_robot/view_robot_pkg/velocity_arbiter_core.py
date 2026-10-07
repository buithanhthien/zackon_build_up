"""Velocity arbitration without ROS dependencies; time is monotonic seconds."""
import math
import time

PRIORITIES = {'navigation': 20, 'following': 40, 'ui': 60,
              'docking': 70, 'localization': 80, 'teleop': 100}
TIMEOUTS = {source: 0.5 for source in PRIORITIES}
STOP_BURST_SECONDS = 0.15


class VelocityArbiterCore:
    def __init__(self, priorities=None, timeouts=None, clock=time.monotonic):
        self.priorities = dict(PRIORITIES if priorities is None else priorities)
        self.timeouts = dict(TIMEOUTS if timeouts is None else timeouts)
        self.clock = clock
        self.owner = ''
        self.velocity = (0.0, 0.0)
        self.deadline = 0.0
        self.ui_granted = False
        self.emergency_stop = False
        self.healthy = False
        self.navigation_busy = False
        self._was_forwarding = False
        self._stop_until = 0.0

    def clear(self):
        # Never replay buffered commands from a displaced controller.
        self.owner = ''
        self.velocity = (0.0, 0.0)
        self.deadline = 0.0
        self.ui_granted = False

    def expire(self):
        if self.owner and self.clock() >= self.deadline:
            self.clear()

    def set_health(self, healthy):
        self.healthy = healthy
        if not healthy:
            self.clear()

    def set_emergency_stop(self, enabled):
        self.emergency_stop = enabled
        self.clear()

    def claim_ui(self):
        self.expire()
        if not self.healthy or self.emergency_stop:
            return False, 'Arbiter chưa sẵn sàng hoặc đang dừng khẩn cấp.'
        if self.navigation_busy:
            return False, 'Nav2 đang thực hiện action; hãy hủy hành trình trước.'
        if self.owner:
            return False, f'Nguồn {self.owner} đang điều khiển; hãy dừng nguồn đó trước.'
        self.owner = 'ui'
        self.ui_granted = True
        self.deadline = self.clock() + self.timeouts['ui']
        return True, 'Đã cấp quyền chuyển động cho giao diện.'

    def release(self, source):
        if self.owner == source:
            self.clear()

    def receive(self, source, linear, angular):
        self.expire()
        if source not in self.priorities or not self.healthy or self.emergency_stop:
            return
        if not all(math.isfinite(value) for value in (linear, angular)):
            self.release(source)
            return
        if source == 'ui' and not self.ui_granted:
            return
        if self.owner and self.priorities[source] < self.priorities[self.owner]:
            return
        # Idle zeroes from Nav2/following must not acquire ownership. Teleop
        # publishes only with the deadman held, so centered sticks still own it.
        if not linear and not angular and source not in ('ui', 'teleop'):
            self.release(source)
            return
        if source != self.owner:
            self.clear()
            self.owner = source
        self.velocity = (linear, angular)
        self.deadline = self.clock() + self.timeouts[source]

    def output(self):
        self.expire()
        return self.velocity if self.healthy and not self.emergency_stop else (0.0, 0.0)

    def next_output(self):
        """None means silence, allowing the STM32's local PS2 watchdog takeover.

        Any Twist (including zero) sets autonomous_MODE in the firmware. Send
        a bounded stop burst on relinquishing a ROS command, never idle zeroes.
        An explicit software emergency stop intentionally keeps zero streaming.
        """
        velocity = self.output()
        if self.emergency_stop or (self.healthy and self.owner):
            self._was_forwarding = True
            self._stop_until = 0.0
            return velocity
        if self._was_forwarding:
            self._was_forwarding = False
            self._stop_until = self.clock() + STOP_BURST_SECONDS
        if self.clock() < self._stop_until:
            return (0.0, 0.0)
        return None
