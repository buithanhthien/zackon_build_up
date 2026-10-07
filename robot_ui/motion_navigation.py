"""Nav2 goal lifecycle; caller serializes methods and callbacks with its lock."""
import math
import time


def relative_target(x, y, yaw, direction, distance):
    if direction not in ('forward', 'backward') or not all(
            math.isfinite(v) for v in (x, y, yaw, distance)) or not 0 < distance <= 5:
        raise ValueError('Pose/hướng/khoảng cách điều hướng không hợp lệ.')
    signed = distance if direction == 'forward' else -distance
    return x + signed * math.cos(yaw), y + signed * math.sin(yaw), yaw


class NavigationTask:
    def __init__(self, client, status, lock, clock=time.monotonic):
        self.client, self.status, self.lock, self.clock = client, status, lock, clock
        self.active = False
        self.handle = None
        self.cancel_requested = False
        self.cancel_future = None
        self.cancel_stamp = 0.0
        self.warned = False

    def start(self, goal):
        if self.active:
            raise ValueError('Nav2 đang chạy hoặc đang chờ hủy.')
        if not self.client.server_is_ready():
            raise ValueError('Nav2 chưa sẵn sàng: /navigate_to_pose.')
        self.active = True
        self.handle = None
        self.cancel_requested = self.warned = False
        self.started = self.clock()
        self.cancel_future = None
        self.status('Đang chờ Nav2 nhận mục tiêu né vật cản.')
        try:
            self.client.send_goal_async(goal).add_done_callback(self._accepted)
        except Exception:
            self.active = False
            raise

    def _accepted(self, future):
        with self.lock:
            try:
                handle = future.result()
            except Exception as exc:
                self.active = False
                self.status(f'Lỗi gửi goal Nav2: {exc}')
                return
            if not handle.accepted:
                self.active = False
                self.status('Nav2 từ chối mục tiêu.')
                return
            self.handle = handle
            self.accepted_at = self.clock()
            try:
                handle.get_result_async().add_done_callback(self._result)
            except Exception as exc:
                self.status(f'Lỗi đăng ký kết quả Nav2; chưa xác nhận dừng: {exc}')
                self.stop()
                return
            if self.cancel_requested:
                self._cancel()
            else:
                self.status('Đang điều hướng né vật cản bằng Nav2.')

    def _result(self, future):
        with self.lock:
            try:
                result = future.result()
            except Exception as exc:
                # Unknown terminal state: retain ownership guard and request cancellation.
                self.status(f'Lỗi nhận kết quả Nav2; chưa xác nhận dừng: {exc}')
                self.stop()
                return
            self.active = False
            self.handle = None
            self.cancel_future = None
            messages = {4: 'Điều hướng Nav2 thành công.', 5: 'Nav2 đã hủy mục tiêu và dừng hành trình.'}
            self.status(messages.get(result.status, f'Điều hướng Nav2 thất bại, status={result.status}.'))

    def stop(self, reason='Đang yêu cầu hủy goal Nav2; chờ kết quả dừng.'):
        if not self.active:
            return
        if not self.cancel_requested:
            self.status(reason)
        self.cancel_requested = True
        if self.handle is not None and self.cancel_future is None:
            self._cancel()

    def _cancel(self):
        self.cancel_stamp = self.clock()
        try:
            self.cancel_future = self.handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self._canceled)
        except Exception as exc:
            self.status(f'Lỗi yêu cầu hủy Nav2; chưa xác nhận dừng: {exc}')

    def _canceled(self, future):
        with self.lock:
            try:
                response = future.result()
                if not response.goals_canceling:
                    self.status('Nav2 chưa nhận yêu cầu hủy; chờ kết quả hoặc thử lại.')
            except Exception as exc:
                self.status(f'Lỗi hủy Nav2; chưa xác nhận dừng: {exc}')
            # Cancellation acknowledgement is not a terminal action result.

    def tick(self):
        if not self.active:
            return
        now = self.clock()
        timeout = 5.0 if self.handle is None else 120.0
        since = self.started if self.handle is None else self.accepted_at
        if not self.cancel_requested and now - since > timeout:
            self.stop('Nav2 timeout; đang yêu cầu hủy, chưa xác nhận dừng.')
        if self.cancel_requested and self.handle is not None and now - self.cancel_stamp > 2.0:
            if not self.warned:
                self.status('Chưa có kết quả dừng Nav2; tiếp tục chờ và thử hủy lại.')
                self.warned = True
            if self.cancel_future is None or self.cancel_future.done():
                self._cancel()
