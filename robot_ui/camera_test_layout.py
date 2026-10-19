#!/usr/bin/env python3
"""Standalone colour-camera preview without the detector or robot motion."""
import os
import shlex
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QImage
from PyQt6.QtWidgets import (QComboBox, QDialog, QHBoxLayout, QLabel,
                             QPushButton, QVBoxLayout)
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera_devices import discover_cameras
from config import SOURCE_PATH, shell_source_workspace
from styles import DIALOG_STYLESHEET
from tracking_mode_layout import VideoWidget
from tracking_process import TrackingProcess


class CameraTestUI(QDialog):
    capture_directory = Path('/home/khoaiuh/images')

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setModal(False)
        self.setWindowTitle('Kiểm tra camera')
        self.resize(800, 600)
        self._cleaned = False
        self.tracking_process = TrackingProcess()
        self.ros_context = Context()
        self.ros_node = None
        self.ros_executor = None
        self.last_frame_time = None
        self.init_ui()
        try:
            self.init_ros()
            self.start_preview()
        except Exception as exc:
            self.status_label.setText(f'Không thể khởi động preview: {exc}')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_preview)
        self.timer.start(250)

    def init_ui(self):
        self.setStyleSheet(DIALOG_STYLESHEET)
        layout = QVBoxLayout(self)
        title = QLabel('Kiểm tra tầm nhìn camera')
        title.setFont(QFont('JetBrains Mono', 18, QFont.Weight.Bold))
        layout.addWidget(title)
        self.video = VideoWidget()
        layout.addWidget(self.video, 1)
        self.status_label = QLabel('Đang chờ hình ảnh từ camera…')
        self.status_label.setWordWrap(True)
        self.status_label.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.status_label)
        row = QHBoxLayout()
        self.camera_source = QComboBox()
        row.addWidget(self.camera_source, 1)
        self.refresh_button = QPushButton('Làm mới camera')
        self.refresh_button.clicked.connect(self.refresh_cameras)
        row.addWidget(self.refresh_button)
        layout.addLayout(row)
        buttons = QHBoxLayout()
        self.start_button = QPushButton('Mở camera')
        self.start_button.clicked.connect(self.start_preview)
        buttons.addWidget(self.start_button)
        self.stop_button = QPushButton('Dừng camera')
        self.stop_button.clicked.connect(self.stop_preview)
        buttons.addWidget(self.stop_button)
        self.capture_button = QPushButton('Chụp ảnh')
        self.capture_button.clicked.connect(self.capture_image)
        self.capture_button.setEnabled(False)
        buttons.addWidget(self.capture_button)
        close_button = QPushButton('Đóng')
        close_button.clicked.connect(self.close)
        buttons.addWidget(close_button)
        layout.addLayout(buttons)
        self.camera_source.currentIndexChanged.connect(self.update_controls)
        self.refresh_cameras()

    def init_ros(self):
        rclpy.init(context=self.ros_context)
        self.ros_node = Node('camera_test_ui_node', context=self.ros_context)
        self.ros_executor = SingleThreadedExecutor(context=self.ros_context)
        self.ros_executor.add_node(self.ros_node)
        self.image_sub = self.ros_node.create_subscription(
            Image, '/camera_test/image_raw', self.image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: self.ros_executor.spin_once(timeout_sec=0))
        self.ros_timer.start(50)

    def refresh_cameras(self):
        previous = self.camera_source.currentData()
        devices, warnings = discover_cameras()
        self.camera_source.clear()
        for label, data in devices:
            self.camera_source.addItem(label, data)
        index = self.camera_source.findData(previous)
        if index >= 0:
            self.camera_source.setCurrentIndex(index)
        if not devices:
            self.video.show_status('Không tìm thấy camera')
        if warnings:
            self.status_label.setText('\n'.join(warnings))
        self.update_controls()

    def start_preview(self):
        device = self.camera_source.currentData()
        if not device or not device.get('source'):
            self.status_label.setText('Chưa có camera hợp lệ. Hãy làm mới danh sách.')
            self.video.show_status('Không có camera được chọn')
            self.update_controls()
            return
        if self.tracking_process.process is not None:
            self.status_label.setText('Camera preview đang chạy.')
            return
        self.last_frame_time = None
        self.video.show_status('Đang chờ hình ảnh từ camera…')
        arguments = ['camera_url:=' + device['source']]
        if device.get('serial'):
            arguments.append('camera_serial:=' + device['serial'])
        launch = os.path.join(
            SOURCE_PATH, 'src/human_following/launch/camera_preview.launch.py')
        command = shell_source_workspace(
            'exec ros2 launch ' + shlex.quote(launch) + ' ' + shlex.join(arguments))
        try:
            self.tracking_process.start(command)
            self.status_label.setText('Đang kết nối camera…')
        except Exception as exc:
            self.status_label.setText(f'Không thể mở camera: {exc}')
            self.video.show_status('Không thể mở camera')
        self.update_controls()

    def stop_preview(self):
        self.tracking_process.stop()
        self.last_frame_time = None
        self.video.show_status('Đã dừng camera')
        self.status_label.setText('Đã dừng camera')
        self.update_controls()

    def image_callback(self, msg):
        if self.tracking_process.process is None:
            return
        if (msg.encoding != 'bgr8' or msg.width <= 0 or msg.height <= 0 or
                msg.step < msg.width * 3 or len(msg.data) < msg.step * msg.height):
            self.video.show_status('Frame camera không hợp lệ')
            self.update_controls()
            return
        frame = QImage(bytes(msg.data), msg.width, msg.height, msg.step,
                       QImage.Format.Format_BGR888).copy()
        self.last_frame_time = time.monotonic()
        self.video.show_frame(frame)
        self.status_label.setText(f'Đang nhận hình ảnh màu · {msg.width} × {msg.height}')
        self.update_controls()

    def capture_image(self):
        frame = self.video.frame
        if self.tracking_process.process is None or frame is None or frame.isNull():
            self.status_label.setText('Chưa có hình ảnh hợp lệ để chụp.')
            self.update_controls()
            return
        try:
            self.capture_directory.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            output_path = self.capture_directory / f'camera_{timestamp}.png'
            suffix = 1
            while output_path.exists():
                output_path = self.capture_directory / f'camera_{timestamp}_{suffix}.png'
                suffix += 1
            if not frame.save(str(output_path), 'PNG'):
                raise OSError('Không thể ghi ảnh PNG')
        except OSError as exc:
            self.status_label.setText(f'Không thể lưu ảnh: {exc}')
            return
        self.status_label.setText(f'Đã lưu ảnh: {output_path}')

    def check_preview(self):
        output = self.tracking_process.read_log()
        process = self.tracking_process.process
        if process is None:
            return
        exit_code = process.poll()
        if exit_code is not None:
            detail = output.strip().splitlines()
            self.status_label.setText(
                f'Camera đã dừng (mã {exit_code}). ' +
                (detail[-1] if detail else 'Kiểm tra camera và log ROS.'))
            self.tracking_process.stop()
            self.video.show_status('Camera đã dừng')
            self.update_controls()
        elif (self.last_frame_time is not None and
              time.monotonic() - self.last_frame_time > 3):
            self.video.show_status('Không nhận được hình ảnh camera')
            self.update_controls()

    def update_controls(self, *_):
        running = self.tracking_process.process is not None
        self.camera_source.setEnabled(not running)
        self.refresh_button.setEnabled(not running)
        self.start_button.setEnabled(not running and self.camera_source.currentData() is not None)
        self.stop_button.setEnabled(running)
        self.capture_button.setEnabled(running and self.video.frame is not None)

    def cleanup(self):
        if self._cleaned:
            return
        self.timer.stop()
        if hasattr(self, 'ros_timer'):
            self.ros_timer.stop()
        self.tracking_process.stop()
        if self.ros_executor is not None:
            self.ros_executor.shutdown()
        if self.ros_node is not None:
            self.ros_node.destroy_node()
        if self.ros_context.ok():
            self.ros_context.shutdown()
        self._cleaned = True

    def closeEvent(self, event):
        try:
            self.cleanup()
        except Exception as exc:
            self.status_label.setText(f'Không thể dừng camera: {exc}')
            event.ignore()
            return
        super().closeEvent(event)


if __name__ == '__main__':
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    window = CameraTestUI()
    window.show()
    sys.exit(app.exec())
