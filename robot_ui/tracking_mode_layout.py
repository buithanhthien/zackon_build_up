#!/usr/bin/env python3
import sys
import os
import shlex
import json
import time
import re
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from concurrent.futures import ThreadPoolExecutor
from camera_devices import discover_cameras
from std_msgs.msg import String
from PyQt6.QtWidgets import (QApplication, QDialog, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QComboBox, QCheckBox)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QPixmap, QImage
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH, shell_source_workspace
from styles import DIALOG_STYLESHEET
from tracking_process import TrackingProcess


class VideoWidget(QLabel):
    def __init__(self):
        super().__init__()
        self.frame = None
        self.setMinimumSize(320, 260)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setText('Đang chờ camera')

    def show_status(self, text):
        self.frame = None
        self.clear()
        self.setText(text)

    def show_frame(self, frame):
        self.frame = frame
        self.render_frame()

    def render_frame(self):
        if self.frame is not None:
            self.setPixmap(QPixmap.fromImage(self.frame).scaled(
                self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.render_frame()


class TrackingModeUI(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setModal(False)
        self.resize(820, 640)
        self.ros_context = Context()
        self.ros_executor = None
        self.tracking_process = TrackingProcess()
        self._cleaned = False
        self._motion_active = False
        self.last_frame_time = None
        self._stopping = None
        self._restart_pending = False
        self._stop_message = ''
        self.stop_worker = ThreadPoolExecutor(max_workers=1)
        self.last_status_time = None
        self.last_error = ''
        self._launch_output = ''
        self.ros_node = None
        self.init_ui()
        try:
            self.init_ros()
            self.launch_tracking()
        except Exception as exc:
            self.status_label.setText(f'Không thể khởi tạo theo dõi: {exc}')
            self.log(str(exc))
        self.process_timer = QTimer(self)
        self.process_timer.timeout.connect(self.check_tracking)
        self.process_timer.start(250)

    def init_ui(self):
        self.setWindowTitle('Theo dõi')
        self.setStyleSheet(DIALOG_STYLESHEET)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(14)
        title = QLabel('Theo dõi')
        title.setFont(QFont('JetBrains Mono', 18, QFont.Weight.Bold))
        layout.addWidget(title)

        self.video = VideoWidget()
        layout.addWidget(self.video, 1)

        self.status_label = QLabel('Camera: đang chờ frame\nDetector: đang khởi tạo\nChuyển động: tắt')
        self.status_label.setWordWrap(True)
        self.status_label.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.status_label)
        layout.addWidget(QLabel('Nguồn camera'))
        self.camera_source = QComboBox()
        self.refresh_button = QPushButton('Làm mới camera')
        self.refresh_button.clicked.connect(self.refresh_cameras)
        camera_row = QHBoxLayout()
        camera_row.addWidget(self.camera_source, 1)
        camera_row.addWidget(self.refresh_button)
        layout.addLayout(camera_row)
        self.camera_hint = QLabel()
        self.camera_hint.setWordWrap(True)
        layout.addWidget(self.camera_hint)
        self.motion_enabled = QCheckBox('Cho phép robot di chuyển (áp dụng khi khởi động lại)')
        layout.addWidget(self.motion_enabled)
        buttons = QHBoxLayout()
        self.restart_button = QPushButton('Khởi động lại theo dõi')
        self.restart_button.setObjectName('primary-btn')
        self.restart_button.clicked.connect(self.launch_tracking)
        buttons.addWidget(self.restart_button)
        self.stop_button = QPushButton('Dừng theo dõi')
        self.stop_button.clicked.connect(self.stop_tracking)
        buttons.addWidget(self.stop_button)
        self.btn_back = QPushButton('Đóng')
        self.btn_back.setObjectName('secondary-btn')
        self.btn_back.clicked.connect(self.close)
        buttons.addWidget(self.btn_back)
        layout.addLayout(buttons)
        self.camera_source.currentIndexChanged.connect(self.update_controls)
        self.refresh_cameras()

    def log(self, message):
        # Diagnostic output stays in the terminal / ROS log file, outside the popup.
        print(f'[Tracking] {message}')

    def init_ros(self):
        rclpy.init(context=self.ros_context)
        self.ros_node = Node('tracking_ui_node', context=self.ros_context)
        self.ros_executor = SingleThreadedExecutor(context=self.ros_context)
        self.ros_executor.add_node(self.ros_node)
        self.image_sub = self.ros_node.create_subscription(
            Image, '/tracking/color/image_raw', self.image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.lock_sub = self.ros_node.create_subscription(
            String, '/human_lock_status', self.lock_callback, 10)

        self.status_sub = self.ros_node.create_subscription(
            String, '/tracking_status', self.status_callback, 10)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: self.ros_executor.spin_once(timeout_sec=0))
        self.ros_timer.start(50)

        self.log("Đã chuyển sang chế độ theo dõi")

    def refresh_cameras(self):
        previous = self.camera_source.currentData()
        devices, warnings = discover_cameras()
        self.camera_source.clear()
        for label, data in devices:
            self.camera_source.addItem(label, data)
        index = self.camera_source.findData(previous)
        if index >= 0:
            self.camera_source.setCurrentIndex(index)
        self.camera_hint.setText(('' if devices else 'Không tìm thấy camera. Cắm camera rồi làm mới. ')
                                 + '\n'.join(warnings))
        self.update_controls()

    def update_controls(self, *_):
        busy = self._stopping is not None
        self.restart_button.setEnabled(not busy and self.camera_source.currentData() is not None)
        self.stop_button.setEnabled(not busy and self.tracking_process.process is not None)
        self.camera_source.setEnabled(not busy)
        self.refresh_button.setEnabled(not busy)
        self.motion_enabled.setEnabled(not busy)

    def image_callback(self, msg):
        # spin_once runs on the Qt timer: all widget access stays on the GUI thread.
        if self.tracking_process.process is None or self._stopping is not None:
            return
        formats = {'bgr8': QImage.Format.Format_BGR888, 'rgb8': QImage.Format.Format_RGB888}
        if (msg.encoding not in formats or msg.width <= 0 or msg.height <= 0
                or msg.step < msg.width * 3 or len(msg.data) < msg.step * msg.height):
            self.video.show_status('Frame camera không hợp lệ')
            return
        frame = QImage(bytes(msg.data), msg.width, msg.height, msg.step, formats[msg.encoding]).copy()
        self.last_frame_time = time.monotonic()
        self.video.show_frame(frame)

    def stop_tracking(self, restart=False, message=''):
        if self._stopping is not None:
            return
        self._restart_pending = restart
        self._stop_message = message
        self._motion_active = False
        self.last_frame_time = None
        self.video.show_status('Đang dừng theo dõi…')
        self.status_label.setText('Đang dừng theo dõi…')
        self._stopping = self.stop_worker.submit(self.tracking_process.stop)
        self.update_controls()

    def lock_callback(self, msg):
        self.log(msg.data)

    def status_callback(self, msg):
        if self.tracking_process.process is None or self._stopping is not None:
            return
        try:
            status = json.loads(msg.data)
            camera = {'connected': 'đã nhận frame màu', 'starting': 'đang kết nối',
                      'error': 'lỗi', 'unavailable': 'không khả dụng'}
            detector = {'ready': 'sẵn sàng', 'starting': 'đang khởi tạo', 'not_ready': 'chưa sẵn sàng'}
            tracking = {'locked': 'đang khóa người', 'searching': 'chưa phát hiện người',
                        'waiting': 'đang chờ', 'stopped': 'đã dừng'}
            self.last_error = status.get('error', '')
            if status['camera'] in ('error', 'unavailable'):
                self.video.show_status('Camera không khả dụng: ' + self.last_error)
            self.last_status_time = time.monotonic()
            self.status_label.setText(
                f"Camera: {camera.get(status['camera'], status['camera'])}\n"
                f"Detector: {detector.get(status['detector'], status['detector'])}\n"
                f"Theo dõi: {tracking.get(status['tracking'], status['tracking'])}\n"
                f"Chuyển động: {'bật' if self._motion_active else 'tắt'}\n{status.get('error', '')}")
        except (ValueError, KeyError, TypeError) as exc:
            self.log(f'Trạng thái ROS không hợp lệ: {exc}')

    def launch_tracking(self):
        if self.ros_node is None:
            self.status_label.setText('ROS UI chưa sẵn sàng; hãy quay lại startup và thử lại.')
            return
        if self._stopping is not None:
            return
        device = self.camera_source.currentData()
        if not device or not device.get('source'):
            self.status_label.setText('Chưa có camera hợp lệ. Hãy làm mới danh sách.')
            self.video.show_status('Không có camera được chọn')
            return
        if self.tracking_process.process is not None:
            self.stop_tracking(restart=True)
            return
        self.tracking_process = TrackingProcess()
        self.last_status_time = None
        self.last_error = ''
        self._launch_output = ''
        self.launch_started = time.monotonic()
        self._motion_active = self.motion_enabled.isChecked()
        self.last_frame_time = None
        self.video.show_status('Đang chờ frame màu…')
        source = device['source']
        arguments = ['camera_url:=' + source,
                     'enable_motion:=' + str(self._motion_active).lower()]
        serial = device.get('serial', '')
        if serial:
            arguments.append('camera_serial:=' + serial)
        command = shell_source_workspace(
            'exec ros2 launch ' + shlex.quote(os.path.join(
                SOURCE_PATH, 'src/human_following/launch/system.launch.py')) +
            ' ' + shlex.join(arguments))
        try:
            self.tracking_process.start(command)
            self.status_label.setText('Đang khởi động ROS; chờ frame màu và detector…')
            self.log(f'Đã tạo tiến trình ROS; log: {self.tracking_process.path}')
        except Exception as exc:
            self.status_label.setText(f'Không thể khởi động ROS: {exc}')
            self.log(str(exc))
            self.video.show_status('Không thể khởi động camera')
        self.update_controls()

    def check_tracking(self):
        if self._stopping is not None:
            if not self._stopping.done():
                return
            future, self._stopping = self._stopping, None
            try:
                future.result()
            except Exception as exc:
                self._restart_pending = False
                self.status_label.setText(f'Không thể dừng theo dõi: {exc}')
                self.update_controls()
                return
            self.status_label.setText(self._stop_message or 'Đã dừng\nChuyển động: tắt')
            self.video.show_status('Đã dừng theo dõi')
            self.update_controls()
            if self._restart_pending:
                self._restart_pending = False
                self.launch_tracking()
            return
        if (self.tracking_process.process is not None and
                time.monotonic() - (self.last_frame_time or self.launch_started) > 3):
            self.video.show_status('Không nhận được frame màu; đang chờ camera…')
        output = self.tracking_process.read_log()
        if output:
            self.log(output)
            self._launch_output = (self._launch_output + output)[-8192:]
        proc = self.tracking_process.process
        if proc is None:
            return
        code = proc.poll()
        if code is not None:
            # Launch parser/import failures happen before the status topic exists.
            output = re.sub(r'\x1b\[[0-9;]*m', '', self._launch_output)
            lines = [line.strip() for line in output.splitlines() if line.strip()]
            errors = [line for line in lines if any(word in line.lower()
                      for word in ('error', 'exception', 'malformed', 'failed', 'not found'))]
            causes = [line for line in errors if 'process has died' not in line]
            detail = self.last_error or ((causes or errors or lines)[-1] if lines else '')
            detail = re.sub(r'^\[human_following_node-\d+\]\s*', '', detail)[:400]
            if not detail:
                detail = f'Không thể tiếp tục theo dõi. Chi tiết: {self.tracking_process.path}'
            headline = (f'Hệ thống theo dõi đã dừng (mã {code}).' if code
                        else 'Hệ thống theo dõi đã dừng.')
            self.status_label.setText(f'{headline}\n{detail}')
            self.log(f'ROS đã thoát với mã {code}')
            self.stop_tracking(message=f'{headline}\n{detail}')
        elif time.monotonic() - (self.last_status_time or self.launch_started) > (
                5 if self.last_status_time else 60):
            self.status_label.setText('ROS không gửi trạng thái camera/detector đúng hạn. Đã dừng theo dõi.')
            self.log('Timeout chờ /tracking_status; dừng toàn bộ launch')
            self.stop_tracking(message='ROS mất trạng thái; đã dừng theo dõi.')

    def go_back(self):
        self.close()

    def cleanup(self):
        if self._cleaned:
            return
        self.process_timer.stop()
        if hasattr(self, 'ros_timer'):
            self.ros_timer.stop()
        self.stop_worker.shutdown(wait=True)
        self.tracking_process.stop()
        if self.ros_executor is not None:
            self.ros_executor.shutdown()
        if self.ros_node is not None:
            self.ros_node.destroy_node()
        if self.ros_context.ok():
            self.ros_context.shutdown()
        self._cleaned = True

    def done(self, result):
        # Escape also stops the owned launch and releases only this dialog's ROS context.
        try:
            self.cleanup()
        except Exception as exc:
            self.status_label.setText(f'Không thể dừng theo dõi: {exc}')
            return
        super().done(result)

    def closeEvent(self, event):
        try:
            self.cleanup()
        except Exception as exc:
            self.status_label.setText(f'Không thể dừng theo dõi: {exc}')
            event.ignore()
            return
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    window = TrackingModeUI()
    window.show()
    sys.exit(app.exec())
