"""Non-modal elevator controls; closing is a stop request, never silent abandonment."""
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import QCheckBox, QDialog, QLabel, QPushButton, QVBoxLayout


class ElevatorTestUI(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.owner = parent
        self.motion = parent._motion
        self.setWindowTitle('Test thang máy · X5_elevator')
        self.setModal(False)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('Map: X5_19102026\nĐến waypoint → chờ available → đi 2 m → quay 180°'))
        self.status = QLabel('')
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.detection_status = QLabel('')
        self.detection_status.setTextFormat(Qt.TextFormat.PlainText)
        self.detection_status.setWordWrap(True)
        layout.addWidget(self.detection_status)
        camera = QPushButton('Mở / kiểm tra camera')
        camera.clicked.connect(parent.open_camera_test_dialog)
        layout.addWidget(camera)
        self.preview = QCheckBox('Chỉ kiểm tra nhận diện tại chỗ (không chạy chuỗi chuyển động)')
        self.preview.toggled.connect(self.toggle_preview)
        layout.addWidget(self.preview)
        self.start_button = QPushButton('Bắt đầu')
        self.start_button.clicked.connect(self.start)
        layout.addWidget(self.start_button)
        stop = QPushButton('Dừng / Trả quyền manual')
        stop.clicked.connect(self.stop)
        layout.addWidget(stop)
        self.motion.prepare_elevator()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)
        self.start_error = ''
        self.resize(520, 260)

    def start(self):
        try:
            if self.owner._navigation_busy():
                raise ValueError('Đang có tác vụ điều khiển; hãy dừng trước.')
            self.preview.setChecked(False)
            with self.motion.lock:
                self.motion.elevator.start()
            self.start_error = ''
        except Exception as exc:
            self.start_error = str(exc)
        self.refresh()

    def toggle_preview(self, enabled):
        with self.motion.lock:
            if self.motion.elevator.active:
                return
            if enabled:
                self.motion.detector.enable()
            else:
                self.motion.detector.disable()

    def stop(self):
        self.preview.setChecked(False)
        self.owner.cancel_voice_navigation()

    def refresh(self):
        with self.motion.lock:
            sequence = self.motion.elevator
            ready, error, _, stamp = self.motion.detector.snapshot()
            self.preview.setEnabled(not self.motion.busy)
            if sequence.active:
                self.preview.setChecked(False)
            self.detection_status.setText(self.motion.detector.describe())
            self.start_button.setEnabled(not self.motion.busy)
            self.status.setText(self.start_error or (sequence.message if sequence.active or sequence.state == 'done'
                                                    else error or 'Detector sẵn sàng. Mở camera trước khi bắt đầu.'))

    def closeEvent(self, event):
        with self.motion.lock:
            if self.motion.elevator.active:
                self.motion.stop()
                event.ignore()
                return
            self.preview.setChecked(False)
            self.motion.detector.disable()
        event.accept()
