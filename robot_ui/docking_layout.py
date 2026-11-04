#!/usr/bin/env python3
"""PyQt6 docking controls for the backend in this workspace's src directory."""
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (QApplication, QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QPlainTextEdit)
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config import SOURCE_PATH, shell_source_workspace
from styles import DIALOG_STYLESHEET
from docking_bridge import DockingBridge


TERMINAL = ('IDLE', 'DOCKED_GEOMETRY')
LABELS = {
    'IDLE': 'Sẵn sàng', 'WAIT_NAV2': 'Chờ Nav2',
    'NAVIGATING_STAGING': 'Đang đến điểm chờ', 'WAIT_V_DOCK': 'Bàn giao điều khiển',
    'STARTING_V_DOCK': 'Khởi động nhận dạng', 'DOCKING': 'Đang docking',
    'SEARCH': 'Tìm hình trạm', 'PLAN_QUINTIC': 'Lập đường bậc 5',
    'APPROACH': 'Lùi theo quỹ đạo', 'ALIGN_FINAL': 'Căn góc cuối',
    'WAIT_DETECTION': 'Dừng chờ nhận dạng lại', 'DOCKED_GEOMETRY': 'Đã vào vị trí hình học',
    'FAILED_OBSTACLE': 'Dừng do vùng di chuyển hoặc xoay bị chặn',
    'FAILED_NO_V_SHAPE': 'Chưa nhận dạng được hình trạm',
}


def terminal(state):
    return state in TERMINAL or state.startswith('FAILED')


class DockingUI(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Trạm sạc · Docking theo hình dạng')
        self.setStyleSheet(DIALOG_STYLESHEET)
        self.resize(950, 740)
        self._cleaned = False
        self._active = self._start_pending = self._stopping = False
        self._saw_active = self._close_when_stopped = False
        self._started = self._stop_time = None
        self._process = self._log_stream = None
        self._log_path = None
        self._map_file = None
        self._odom_topic = None
        self._launch_attempt = None
        self._navigation_changed = False
        self._last_states = {}
        self._estop_by_us = False
        self._docked_confirmed = False
        self._build_ui()
        self.bridge = DockingBridge(self)
        self.bridge.updated.connect(self._updated)
        self.bridge.replied.connect(self._replied)
        self.bridge.error.connect(self._bridge_error)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(200)
        self.log('Đang kết nối Nav2. Hai điểm sẽ được lưu theo bản đồ và vị trí robot đang chạy.')

    @property
    def busy(self):
        if self._active or self._start_pending or self._stopping:
            return True
        for key in ('mission', 'controller'):
            state = self.bridge.latest.get(key, {}).get('state', '')
            if state and not terminal(state):
                return True
        return False

    def _build_ui(self):
        layout = QVBoxLayout(self)
        title = QLabel('Về trạm sạc bằng LiDAR · Nav2 → điểm chờ → lùi vào trạm')
        title.setObjectName('header-title')
        layout.addWidget(title)
        self.map_label = QLabel('Bản đồ Nav2: đang kết nối…')
        self.map_label.setWordWrap(True)
        layout.addWidget(self.map_label)
        row = QHBoxLayout()
        self.pre_btn = QPushButton('Lưu điểm predock')
        self.pre_btn.clicked.connect(lambda: self._teach('pre'))
        row.addWidget(self.pre_btn)
        self.terminal_btn = QPushButton('Lưu điểm dock (mẫu sát trạm)')
        self.terminal_btn.clicked.connect(lambda: self._teach('terminal'))
        row.addWidget(self.terminal_btn)
        layout.addLayout(row)
        guide = QLabel('Đưa robot tới điểm chờ rồi lưu predock; đưa robot tới vị trí sát trạm rồi lưu dock. '
                       'Mỗi lần lưu lấy tọa độ hiện tại trên bản đồ Nav2 và mẫu LiDAR khi robot đứng yên. '
                       'Lưu cả hai điểm trong cùng lần mở cửa sổ, có thể lưu dock trước.')
        guide.setWordWrap(True)
        layout.addWidget(guide)
        self.status = QLabel('Đang kết nối Nav2…')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.health = QLabel('Chờ trạng thái cảm biến / arbiter')
        self.health.setWordWrap(True)
        layout.addWidget(self.health)
        self.metrics = QLabel('Sai số x / y / góc: chưa có dữ liệu')
        self.metrics.setWordWrap(True)
        layout.addWidget(self.metrics)
        self.battery = QLabel('Pin: chưa có dữ liệu · Sạc: chưa có dữ liệu')
        layout.addWidget(self.battery)
        row = QHBoxLayout()
        self.start_btn = QPushButton('Bắt đầu về trạm')
        self.start_btn.clicked.connect(self.execute_docking)
        row.addWidget(self.start_btn)
        self.cancel_btn = QPushButton('Hủy docking')
        self.cancel_btn.clicked.connect(self.cancel_docking)
        row.addWidget(self.cancel_btn)
        self.stop_btn = QPushButton('DỪNG')
        self.stop_btn.setStyleSheet('background: #b42332; color: white; font-weight: bold; padding: 12px;')
        self.stop_btn.clicked.connect(self._emergency_stop)
        row.addWidget(self.stop_btn)
        self.reset_btn = QPushButton('Nhả dừng')
        self.reset_btn.clicked.connect(self._reset_stop)
        row.addWidget(self.reset_btn)
        layout.addLayout(row)
        row = QHBoxLayout()
        self.charge_btn = QPushButton('Yêu cầu bắt đầu sạc')
        self.charge_btn.clicked.connect(self.send_charge_command)
        row.addWidget(self.charge_btn)
        undock = QPushButton('Rời trạm — chưa có điều khiển')
        undock.setEnabled(False)
        row.addWidget(undock)
        back = QPushButton('Quay lại')
        back.clicked.connect(self.close)
        row.addWidget(back)
        layout.addLayout(row)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1000)
        layout.addWidget(self.log_view, 1)

    def log(self, message):
        self.log_view.appendPlainText(time.strftime('%H:%M:%S')+' '+message)

    def _profile(self):
        return str(Path(self._map_file).with_suffix('.docking.yaml')) if self._map_file else None

    def _navigation_ready(self):
        return (self.bridge.fresh('nav2_map', 3.) and self.bridge.fresh('nav2_odom', 3.)
                and self.bridge.latest.get('nav2_map') == self._map_file
                and self.bridge.latest.get('nav2_odom') == self._odom_topic)

    def _matching_backend(self):
        if not self._navigation_ready():
            return False
        if not self.bridge.fresh('mission') or not self.bridge.fresh('controller'):
            return False
        return all(self.bridge.latest[key].get('profile_file') == self._profile()
                   and self.bridge.latest[key].get('odom_topic') == self._odom_topic
                   for key in ('mission', 'controller'))

    def _sync_navigation(self):
        if not (self.bridge.fresh('nav2_map', 3.) and self.bridge.fresh('nav2_odom', 3.)):
            return
        map_file = self.bridge.latest['nav2_map']
        odom_topic = self.bridge.latest['nav2_odom']
        if (map_file, odom_topic) != (self._map_file, self._odom_topic):
            if self.busy or self.bridge.pending:
                if not self._navigation_changed:
                    self.log('Nav2 đã đổi bản đồ/odometry; đang hủy tác vụ trước khi kết nối lại.')
                self._navigation_changed = True
                if self.busy:
                    self.cancel_docking()
                return
            self._stop_backend()
            self._map_file, self._odom_topic = map_file, odom_topic
            self._navigation_changed = False
            self._docked_confirmed = False
            self._launch_attempt = None
            self.map_label.setText('Bản đồ Nav2: '+Path(map_file).stem)
            self.log('Đã nhận bản đồ Nav2: '+map_file)
        self._launch_backend()

    def _launch_backend(self):
        if self._process is not None or self.busy or not self._navigation_ready():
            return
        if self.bridge.clients['start'].service_is_ready():
            return
        attempt = (self._map_file, self._odom_topic)
        if self._launch_attempt == attempt:
            return
        self._launch_attempt = attempt
        map_file = Path(self._map_file)
        if not map_file.is_file():
            self.log('Không tìm thấy map: '+str(map_file))
            return
        try:
            directory = Path(SOURCE_PATH)/'log/docking'
            directory.mkdir(parents=True, exist_ok=True)
            self._log_path = directory/('backend_'+time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()}.log')
            self._log_stream = self._log_path.open('w')
            command = shell_source_workspace('exec ros2 launch dock_shape_sim hardware_docking.launch.py '
                'map_file:='+shlex.quote(str(map_file))+' odom_topic:='+shlex.quote(self._odom_topic))
            self._process = subprocess.Popen(['bash', '-c', command], start_new_session=True,
                stdout=self._log_stream, stderr=subprocess.STDOUT)
            self.log('Đang kết nối điều khiển docking với Nav2…')
        except Exception as exc:
            if self._log_stream:
                self._log_stream.close()
                self._log_stream = None
            self.log('Không mở được điều khiển docking: '+str(exc))

    def _stop_backend(self):
        if self._process is not None and self._process.poll() is None:
            try:
                os.killpg(self._process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(self._process.pid, signal.SIGKILL)
                self._process.wait(timeout=1.)
        self._process = None
        if self._log_stream:
            self._log_stream.close()
            self._log_stream = None

    def _teach(self, key):
        if self.busy or self.bridge.pending or not self._matching_backend():
            self.log('Chờ kết nối Nav2 và hủy docking trước khi lưu điểm.')
            return
        if not self._stationary():
            self.log('Chờ robot đứng yên và có odometry từ Nav2 ('+str(self._odom_topic)+').')
            return
        self.bridge.request(key)

    def _stationary(self):
        odom = self.bridge.latest.get('odom', {})
        return (self.bridge.fresh('odom', .7) and abs(odom.get('linear', 1.)) <= .01
                and abs(odom.get('angular', 1.)) <= .03)

    def execute_docking(self):
        if self.busy or self.bridge.pending or not self._matching_backend():
            self.log('Chưa kết nối đúng bản đồ Nav2 hoặc đang có nhiệm vụ.')
            return
        parent = self.parent()
        if parent is not None:
            localization = getattr(parent, 'localization_thread', None)
            if parent._navigation_busy() or (localization and localization.is_alive()):
                self.log('Dừng tác vụ chuyển động/định vị của màn hình chính trước khi docking.')
                return
        arbiter = self.bridge.latest.get('arbiter', {})
        if (not self.bridge.fresh('arbiter', .5) or not arbiter.get('healthy')
                or arbiter.get('emergency_stop') or arbiter.get('navigation_busy')
                or arbiter.get('owner') not in ('',)):
            self.log('Dừng các nguồn khác, chờ arbiter sẵn sàng và nhả dừng trước khi docking.')
            return
        if not self._stationary():
            self.log('Chưa xác nhận robot đứng yên.')
            return
        self._active = self._start_pending = True
        self._docked_confirmed = False
        self._saw_active = False
        self._started = time.monotonic()
        self.bridge.set_permit(True)
        self.log('Bắt đầu về trạm: Nav2 → predock → lùi tới điểm dock.')
        QTimer.singleShot(350, self._send_start)

    def _send_start(self):
        if self._active and self._start_pending and not self._stopping:
            self.bridge.request('start')

    def cancel_docking(self):
        if self._cleaned:
            return
        self.bridge.set_permit(False)
        if not self.busy:
            return
        if not self._stopping:
            self._stopping = True
            self._stop_time = time.monotonic()
            self.log('Đang hủy; chờ mission, controller, arbiter và odometry xác nhận dừng.')
        self.bridge.request('cancel')
        self.bridge.request('controller_cancel')

    def _emergency_stop(self):
        self._estop_by_us = True
        self.bridge.emergency_stop(True)
        self.cancel_docking()
        self.log('Đã gửi DỪNG tới velocity_arbiter. Nhả dừng là thao tác riêng.')

    def _reset_stop(self):
        if self.busy or not self._stationary():
            self.log('Chờ xác nhận dừng trước khi nhả dừng.')
            return
        self.bridge.emergency_stop(False)
        self._estop_by_us = False
        self.log('Đã yêu cầu nhả dừng; robot không tự khởi động lại docking.')

    def send_charge_command(self):
        controller = self.bridge.latest.get('controller', {})
        if (self.busy or not self._stationary() or not self._docked_confirmed
                or controller.get('state') != 'DOCKED_GEOMETRY'):
            return
        if (not self.bridge.fresh('controller') or not self.bridge.fresh('charging', 5.)
                or not self.bridge.charge.get_subscription_count()):
            self.log('Chưa có kết nối/phản hồi phần cứng sạc.')
            return
        self.bridge.charge.publish(Bool(data=True))
        self.log('Đã gửi yêu cầu sạc; chỉ xác nhận sạc khi có phản hồi phần cứng.')

    def _updated(self, key, data):
        if key == 'odom' and (abs(data['linear']) > .01 or abs(data['angular']) > .03):
            self._docked_confirmed = False
        if key in ('mission', 'controller'):
            state = data.get('state', '')
            if state != self._last_states.get(key):
                if key == 'controller' and state == 'DOCKED_GEOMETRY' and self._active:
                    self._docked_confirmed = True
                self.log(key+': '+state+' '+str(data.get('detail', '')))
                self._last_states[key] = state
            if key == 'mission' and self._active and not self._stopping:
                if state and not terminal(state):
                    self._saw_active = True
                elif state and terminal(state) and self._saw_active:
                    if state == 'DOCKED_GEOMETRY':
                        self._docked_confirmed = True
                    self.bridge.set_permit(False)
                    self._active = self._start_pending = False

    def _replied(self, key, success, message):
        self.log(key+': '+('OK — ' if success else 'LỖI — ')+message)
        if key == 'start':
            self._start_pending = False
            if self._stopping or not self._active:
                self.bridge.set_permit(False)
                self.bridge.request('cancel')
                self.bridge.request('controller_cancel')
            elif not success:
                self.cancel_docking()

    def _bridge_error(self, message):
        self.log('Lỗi ROS: '+message)
        if self.busy:
            self._emergency_stop()

    def _stop_confirmed(self):
        if self._stop_time is None or not self._stationary():
            return False
        for key in ('mission', 'controller', 'arbiter', 'odom'):
            if not self.bridge.fresh(key) or self.bridge.received[key] <= self._stop_time:
                return False
        mission = self.bridge.latest['mission']
        arbiter = self.bridge.latest['arbiter']
        return (terminal(mission.get('state', '')) and not mission.get('cancel_pending', True)
                and terminal(self.bridge.latest['controller'].get('state', ''))
                and not arbiter.get('navigation_busy', True) and not arbiter.get('owner')
                and 'start' not in self.bridge.pending)

    def _refresh(self):
        if self._process is not None and self._process.poll() is not None:
            code = self._process.returncode
            self.log(f'Điều khiển docking đã ngừng (code={code}). Xem log: {self._log_path}')
            self._process = None
            self._log_stream.close()
            self._log_stream = None
            if self.busy:
                self._emergency_stop()
        self._sync_navigation()
        if self._active and not self._stopping:
            if (time.monotonic()-self._started > 2. and not self._matching_backend()
                    or time.monotonic()-self._started > 5. and not self._saw_active):
                self.log('Mất kết nối docking/Nav2 hoặc chưa xác nhận bắt đầu; đang hủy.')
                self.cancel_docking()
        if self._stopping and self._stop_confirmed():
            self._stopping = self._active = self._start_pending = False
            self.log('Đã xác nhận dừng.')
            if self._close_when_stopped:
                QTimer.singleShot(0, self.close)
        elif self._stopping and time.monotonic()-self._stop_time > 3.:
            self.bridge.emergency_stop(True)
            self._estop_by_us = True
        mission = self.bridge.latest.get('mission', {})
        controller = self.bridge.latest.get('controller', {})
        arbiter = self.bridge.latest.get('arbiter', {})
        state = controller.get('state') if mission.get('state') == 'DOCKING' else mission.get('state', '')
        if not self.busy and not self._matching_backend():
            state = ''
        detail = controller.get('detail') if state.startswith('FAILED') else ''
        self.status.setText(('Đang chờ xác nhận hủy… ' if self._stopping else '')+
                            LABELS.get(state, state or 'Đang kết nối Nav2…')+
                            (' · '+detail if detail else ''))
        self.health.setText(f"Kết nối Nav2: {'OK' if self._matching_backend() else 'chưa sẵn sàng'} · "
            f"TF: {controller.get('tf_ok', '?')} · tuổi scan: {controller.get('scan_age')} · "
            f"tuổi odom: {controller.get('odom_age')} · nguồn: {arbiter.get('owner', '?') or 'rảnh'} · "
            f"dừng: {arbiter.get('emergency_stop', '?')}")
        if self.busy and mission.get('safety_error'):
            self.health.setText(self.health.text()+' · '+mission['safety_error'])
        if not self._navigation_ready():
            detail = self.bridge.latest.get('navigation_error',
                'Chờ bản đồ từ map_server và odometry từ controller_server của Nav2.')
            self.health.setText(self.health.text()+' · '+detail)
        elif not self._matching_backend() and self.bridge.clients['start'].service_is_ready():
            self.health.setText(self.health.text()+' · Điều khiển docking đang chạy khác map hoặc nguồn odometry; '
                                'hãy đóng phiên docking cũ rồi mở lại cửa sổ này.')
        metrics = controller.get('metrics') or {}
        self.metrics.setText('Sai số x/y/góc: '+str([metrics.get(k) for k in ('error_x','error_y','error_yaw')])+
            f" · Xác nhận: {controller.get('verification_count', 0)}/{controller.get('verification_required', 5)}")
        battery = self.bridge.latest.get('battery')
        battery_text = (f'{battery:.0f}%' if self.bridge.fresh('battery', 5.)
                        and isinstance(battery, (int, float)) and math.isfinite(battery) else 'chưa có dữ liệu')
        charge_text = ('đang sạc' if self.bridge.latest.get('charging') else 'chưa sạc') if self.bridge.fresh('charging', 5.) else 'chưa xác nhận'
        self.battery.setText('Pin: '+battery_text+' · Sạc: '+charge_text)
        busy = self.busy
        self.start_btn.setEnabled(not busy and not self.bridge.pending and self._matching_backend())
        self.pre_btn.setEnabled(not busy and not self.bridge.pending and self._matching_backend())
        self.terminal_btn.setEnabled(not busy and not self.bridge.pending and self._matching_backend())
        self.cancel_btn.setEnabled(busy)
        self.reset_btn.setEnabled(not busy and bool(arbiter.get('emergency_stop')))
        self.charge_btn.setEnabled(not busy and self._docked_confirmed and self.bridge.fresh('charging', 5.)
            and bool(self.bridge.charge.get_subscription_count()) and self.bridge.fresh('controller')
            and controller.get('state') == 'DOCKED_GEOMETRY' and self._stationary())

    def closeEvent(self, event):
        if self._cleaned:
            event.accept()
            return
        if self.busy:
            self._close_when_stopped = True
            self.cancel_docking()
            event.ignore()
            return
        self.timer.stop()
        self.bridge.close()
        self._stop_backend()
        self._cleaned = True
        event.accept()

    def reject(self):
        # Escape must follow the same cancellation path as the window close button.
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DockingUI()
    window.show()
    sys.exit(app.exec())
