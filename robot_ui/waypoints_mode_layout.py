#!/usr/bin/env python3
import sys
import json
import subprocess
import os
import math
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                              QLabel, QListWidget, QListWidgetItem, QTextEdit,
                              QApplication, QMainWindow, QSizePolicy, QDialog, QLineEdit,
                              QMessageBox, QSplitter, QTabWidget)
from PyQt6.QtCore import Qt, QTimer, QPointF, QMetaObject, Q_ARG, pyqtSignal
from PyQt6.QtCore import pyqtSlot
from PyQt6.QtGui import QFont, QPixmap, QPainter, QPen, QColor, QTransform, QFontDatabase, QTextCursor
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH
from chat_panel_widget import ChatPanel, _AIChatWorker
from load_map_dialog import LoadMapDialog
from PyQt6.QtCore import QThread
from styles import MAIN_STYLESHEET, DIALOG_STYLESHEET
from ui_utils import append_log, setup_clock_timer
from map_utils import get_current_map_path, load_map_yaml, get_current_map_name, update_map_files
from process_manager import ProcessManager


class WaypointsNode(Node):
    def __init__(self):
        super().__init__('waypoints_node')
        _amcl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, _amcl_qos)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_pose = None
        self.current_goal_handle = None
        self.nav_server_available = False
        self.nav_server_available = self.nav_client.wait_for_server(timeout_sec=2.0)
        
    def pose_callback(self, msg):
        self.current_pose = msg


class MapWidget(QWidget):
    def __init__(self, map_path, yaml_data):
        super().__init__()
        self.map_image = QPixmap(map_path)
        self.resolution = float(yaml_data.get('resolution', 1.0))
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            self.resolution = 1.0
        self.origin = yaml_data.get('origin', [0.0, 0.0, 0.0])
        if not isinstance(self.origin, (list, tuple)) or len(self.origin) < 2:
            self.origin = [0.0, 0.0, 0.0]
        self.robot_pose = None
        self.waypoints = {}
        self.setMinimumSize(320, 260)
        
    def set_robot_pose(self, pose):
        self.robot_pose = pose
        self.update()

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.update()
        
    def world_to_pixel(self, x, y):
        dx = x - self.origin[0]
        dy = y - self.origin[1]
        origin_yaw = self.origin[2] if len(self.origin) > 2 else 0.0
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        px = int(local_x / self.resolution)
        py = int(self.map_image.height() - local_y / self.resolution)
        return px, py
        
    def paintEvent(self, event):
        painter = QPainter(self)

        if self.map_image.isNull():
            painter.drawText(
                self.rect(),
                Qt.AlignmentFlag.AlignCenter,
                "Không tải được bản đồ",
            )
            return
        
        scaled_map = self.map_image.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        x_offset = (self.width() - scaled_map.width()) // 2
        y_offset = (self.height() - scaled_map.height()) // 2
        painter.drawPixmap(x_offset, y_offset, scaled_map)

        scale_x = scaled_map.width() / self.map_image.width()
        scale_y = scaled_map.height() / self.map_image.height()

        # Draw waypoint arrows
        for slot, wp in self.waypoints.items():
            px, py = self.world_to_pixel(wp['x'], wp['y'])
            px = int(px * scale_x + x_offset)
            py = int(py * scale_y + y_offset)
            self._draw_arrow(painter, px, py, slot)

        if self.robot_pose:
            px, py = self.world_to_pixel(
                self.robot_pose.pose.pose.position.x,
                self.robot_pose.pose.pose.position.y
            )
            px = int(px * scale_x + x_offset)
            py = int(py * scale_y + y_offset)
            painter.setPen(QPen(QColor(255, 0, 0), 3))
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(px - 5, py - 5, 10, 10)

    def _draw_arrow(self, painter, px, py, label):
        painter.setPen(QPen(QColor(252, 181, 37), 2))
        painter.setBrush(QColor(252, 181, 37))
        painter.drawLine(px, py - 20, px, py)
        from PyQt6.QtGui import QPolygon
        from PyQt6.QtCore import QPoint
        tip = QPoint(px, py)
        left = QPoint(px - 6, py - 12)
        right = QPoint(px + 6, py - 12)
        painter.drawPolygon(QPolygon([tip, left, right]))
        painter.setPen(QPen(QColor(26, 42, 94), 1))
        painter.drawText(px + 8, py - 10, label)


class NewWaypointDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("New Waypoint")
        self.setModal(True)
        self.resize(400, 200)
        self.setStyleSheet("""
            QDialog { background-color: #f0f4ff; color: #1a2a5e; }
            QLabel { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 8px; }
            QLineEdit {
                background-color: #ffffff; color: #1a2a5e;
                border: 1px solid #c8d4f0; border-radius: 8px;
                font-size: 15px; padding: 10px 14px;
            }
            QLineEdit:focus { border: 1px solid #214196; }
            QPushButton#ok-btn {
                background-color: #214196; color: #ffffff;
                border: none; border-radius: 8px;
                font-size: 15px; min-height: 44px;
            }
            QPushButton#ok-btn:hover { background-color: #1a3278; }
            QPushButton#cancel-btn {
                background-color: transparent; color: #5a7abf;
                border: 1px solid #c8d4f0; border-radius: 8px;
                font-size: 15px; min-height: 44px;
            }
            QPushButton#cancel-btn:hover { background-color: #e8f0ff; color: #214196; border: 1px solid #214196; }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        label = QLabel("WAYPOINT NAME")
        label.setFont(QFont("DM Sans", 11))
        layout.addWidget(label)

        self.name_input = QLineEdit()
        self.name_input.setFont(QFont("JetBrains Mono", 15))
        self.name_input.setPlaceholderText("Enter name...")
        self.name_input.returnPressed.connect(self.accept)
        layout.addWidget(self.name_input)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)
        btn_confirm = QPushButton("Xác nhận")
        btn_confirm.setObjectName("ok-btn")
        btn_confirm.setFont(QFont("JetBrains Mono", 15))
        btn_confirm.clicked.connect(self.accept)
        btn_back = QPushButton("Hủy")
        btn_back.setObjectName("cancel-btn")
        btn_back.setFont(QFont("JetBrains Mono", 15))
        btn_back.clicked.connect(self.reject)
        btn_layout.addWidget(btn_confirm)
        btn_layout.addWidget(btn_back)
        layout.addLayout(btn_layout)

    def get_name(self):
        return self.name_input.text().strip()


class WaypointPickerDialog(QDialog):
    def __init__(self, waypoints, current_map_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Chọn địa điểm")
        self.setModal(True)
        self.resize(480, 560)
        self._selected_key = None
        self.setStyleSheet("""
            QDialog { background-color: #f0f4ff; color: #1a2a5e; }
            QLabel#title { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 8px; }
            QListWidget {
                background-color: #ffffff; color: #1a2a5e;
                border: 1px solid #c8d4f0; border-radius: 8px;
                font-size: 15px; outline: none;
            }
            QListWidget::item { padding: 12px 16px; border-bottom: 1px solid #e8f0ff; }
            QListWidget::item:hover { background-color: #e8f0ff; color: #214196; }
            QListWidget::item:selected { background-color: #214196; color: #ffffff; border-left: 3px solid #fcb525; }
            QPushButton#ok-btn {
                background-color: #214196; color: #ffffff;
                border: none; border-radius: 8px; font-size: 15px; min-height: 44px;
            }
            QPushButton#ok-btn:hover { background-color: #1a3278; }
            QPushButton#ok-btn:disabled { color: #a8bce8; background-color: #e0e8f8; }
            QPushButton#cancel-btn {
                background-color: transparent; color: #5a7abf;
                border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; min-height: 44px;
            }
            QPushButton#cancel-btn:hover { background-color: #e8f0ff; color: #214196; border: 1px solid #214196; }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel("ĐỊA ĐIỂM")
        title.setObjectName("title")
        title.setFont(QFont("DM Sans", 11))
        layout.addWidget(title)

        self.list_widget = QListWidget()
        self.list_widget.setFont(QFont("JetBrains Mono", 15))
        for key, data in waypoints.items():
            if data.get('map_name') == current_map_name:
                self.list_widget.addItem(QListWidgetItem(key))
        self.list_widget.itemClicked.connect(lambda item: setattr(self, '_selected_key', item.text()))
        self.list_widget.itemDoubleClicked.connect(lambda item: (setattr(self, '_selected_key', item.text()), self.accept()))
        layout.addWidget(self.list_widget)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self.btn_ok = QPushButton("Đi tới")
        self.btn_ok.setObjectName("ok-btn")
        self.btn_ok.setFont(QFont("JetBrains Mono", 15))
        self.btn_ok.setEnabled(False)
        self.btn_ok.clicked.connect(self.accept)
        self.list_widget.itemClicked.connect(lambda: self.btn_ok.setEnabled(True))
        btn_cancel = QPushButton("Hủy")
        btn_cancel.setObjectName("cancel-btn")
        btn_cancel.setFont(QFont("JetBrains Mono", 15))
        btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(self.btn_ok)
        btn_row.addWidget(btn_cancel)
        layout.addLayout(btn_row)

    def get_selected_key(self):
        return self._selected_key


DIALOG_STYLE = """
    QDialog { background-color: #f0f4ff; color: #1a2a5e; }
    QLabel#title { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 4px; }
    QLabel#seq-label { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 4px; }
    QListWidget {
        background-color: #ffffff; color: #1a2a5e;
        border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; outline: none;
    }
    QListWidget::item { padding: 10px 16px; border-bottom: 1px solid #e8f0ff; }
    QListWidget::item:hover { background-color: #e8f0ff; color: #214196; }
    QListWidget::item:selected { background-color: #214196; color: #ffffff; border-left: 3px solid #fcb525; }
    QTextEdit { background-color: #ffffff; color: #1a2a5e; border: 1px solid #c8d4f0; border-radius: 8px; font-size: 14px; padding: 8px; }
    QLineEdit {
        background-color: #ffffff; color: #1a2a5e;
        border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; padding: 10px 14px;
    }
    QLineEdit:focus { border: 1px solid #214196; }
    QPushButton#primary-btn {
        background-color: #214196; color: #ffffff;
        border: none; border-radius: 8px; font-size: 15px; min-height: 44px;
    }
    QPushButton#primary-btn:hover { background-color: #1a3278; }
    QPushButton#primary-btn:disabled { color: #a8bce8; background-color: #e0e8f8; }
    QPushButton#secondary-btn {
        background-color: transparent; color: #5a7abf;
        border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; min-height: 44px;
    }
    QPushButton#secondary-btn:hover { background-color: #e8f0ff; color: #214196; border: 1px solid #214196; }
"""


class PathManagerDialog(QDialog):
    """Shows saved paths from multi_waypoints.json. Run path / New path / Back."""
    run_path_requested = pyqtSignal(list)   # emits sequence of keys

    def __init__(self, multi_wp_file, current_map, parent=None):
        super().__init__(parent)
        self.multi_wp_file = multi_wp_file
        self.current_map = current_map
        self.setWindowTitle("Tạo lộ trình")
        self.setModal(True)
        self.resize(480, 520)
        self.setStyleSheet(DIALOG_STYLE)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel("LỘ TRÌNH ĐÃ LƯU")
        title.setObjectName("title")
        title.setFont(QFont("DM Sans", 11))
        layout.addWidget(title)

        self.list_widget = QListWidget()
        self.list_widget.setFont(QFont("JetBrains Mono", 15))
        self._reload_list()
        layout.addWidget(self.list_widget)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        self.btn_run = QPushButton("Chạy lộ trình")
        self.btn_run.setObjectName("primary-btn")
        self.btn_run.setFont(QFont("JetBrains Mono", 15))
        self.btn_run.setEnabled(False)
        self.btn_run.clicked.connect(self._on_run)

        btn_new = QPushButton("Lộ trình mới")
        btn_new.setObjectName("primary-btn")
        btn_new.setFont(QFont("JetBrains Mono", 15))
        btn_new.clicked.connect(self._on_new)

        btn_remove = QPushButton("Xóa lộ trình")
        btn_remove.setObjectName("secondary-btn")
        btn_remove.setFont(QFont("JetBrains Mono", 15))
        btn_remove.clicked.connect(self._on_remove)

        btn_back = QPushButton("Quay lại")
        btn_back.setObjectName("secondary-btn")
        btn_back.setFont(QFont("JetBrains Mono", 15))
        btn_back.clicked.connect(self.reject)

        btn_row.addWidget(self.btn_run)
        btn_row.addWidget(btn_new)
        btn_row.addWidget(btn_remove)
        btn_row.addWidget(btn_back)
        layout.addLayout(btn_row)

        self.list_widget.itemClicked.connect(lambda: self.btn_run.setEnabled(True))

    def _reload_list(self):
        self.list_widget.clear()
        try:
            with open(self.multi_wp_file) as f:
                data = json.load(f)
            for name, info in data.items():
                if info.get('map_name') == self.current_map:
                    seq = ', '.join(info.get('sequence', []))
                    self.list_widget.addItem(QListWidgetItem(f"{name}  [{seq}]"))
        except Exception:
            pass

    def _on_run(self):
        item = self.list_widget.currentItem()
        if not item:
            return
        path_name = item.text().split('  [')[0]
        try:
            with open(self.multi_wp_file) as f:
                data = json.load(f)
            sequence = data[path_name]['sequence']
            self.run_path_requested.emit(sequence)
            self.accept()
        except Exception:
            pass

    def _on_new(self):
        self.done(2)   # custom code 2 = open NewPathDialog

    def _on_remove(self):
        item = self.list_widget.currentItem()
        if not item:
            return
        path_name = item.text().split('  [')[0]
        from PyQt6.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self, "Xác nhận xóa",
            f"Bạn có chắc muốn xóa lộ trình '{path_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            with open(self.multi_wp_file) as f:
                data = json.load(f)
            if path_name in data:
                del data[path_name]
                with open(self.multi_wp_file, 'w') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                self._reload_list()
                self.btn_run.setEnabled(False)
        except Exception:
            pass


class NewPathDialog(QDialog):
    """Select goals in sequence, name the path, confirm to save."""

    def __init__(self, waypoints, current_map, multi_wp_file, parent=None):
        super().__init__(parent)
        self.waypoints = waypoints
        self.current_map = current_map
        self.multi_wp_file = multi_wp_file
        self.sequence = []
        self.setWindowTitle("Lộ trình mới")
        self.setModal(True)
        self.resize(700, 520)
        self.setStyleSheet(DIALOG_STYLE)
        self._build_ui()

    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(16)

        # ── Left: sequence preview ──
        left = QVBoxLayout()
        seq_label = QLabel("THỨ TỰ LỘ TRÌNH")
        seq_label.setObjectName("seq-label")
        seq_label.setFont(QFont("DM Sans", 11))
        left.addWidget(seq_label)

        self.seq_text = QTextEdit()
        self.seq_text.setReadOnly(True)
        self.seq_text.setFont(QFont("JetBrains Mono", 14))
        self.seq_text.setPlaceholderText("(chưa chọn)")
        left.addWidget(self.seq_text, 1)

        name_label = QLabel("TÊN LỘ TRÌNH")
        name_label.setObjectName("seq-label")
        name_label.setFont(QFont("DM Sans", 11))
        left.addWidget(name_label)

        self.name_input = QLineEdit()
        self.name_input.setFont(QFont("JetBrains Mono", 14))
        self.name_input.setPlaceholderText("Nhập tên...")
        left.addWidget(self.name_input)

        root.addLayout(left, 1)

        # ── Right: goal list + buttons ──
        right = QVBoxLayout()
        goal_label = QLabel("ĐỊA ĐIỂM")
        goal_label.setObjectName("title")
        goal_label.setFont(QFont("DM Sans", 11))
        right.addWidget(goal_label)

        self.goal_list = QListWidget()
        self.goal_list.setFont(QFont("JetBrains Mono", 14))
        for key, data in self.waypoints.items():
            if data.get('map_name') == self.current_map:
                self.goal_list.addItem(QListWidgetItem(key))
        self.goal_list.itemClicked.connect(self._add_goal)
        right.addWidget(self.goal_list, 1)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        btn_confirm = QPushButton("Xác nhận")
        btn_confirm.setObjectName("primary-btn")
        btn_confirm.setFont(QFont("JetBrains Mono", 14))
        btn_confirm.clicked.connect(self._confirm)

        btn_undo = QPushButton("Hoàn tác")
        btn_undo.setObjectName("secondary-btn")
        btn_undo.setFont(QFont("JetBrains Mono", 14))
        btn_undo.clicked.connect(self._undo)

        btn_back = QPushButton("Hủy")
        btn_back.setObjectName("secondary-btn")
        btn_back.setFont(QFont("JetBrains Mono", 14))
        btn_back.clicked.connect(self.reject)

        btn_row.addWidget(btn_confirm)
        btn_row.addWidget(btn_undo)
        btn_row.addWidget(btn_back)
        right.addLayout(btn_row)

        root.addLayout(right, 1)

    def _add_goal(self, item):
        self.sequence.append(item.text())
        self._refresh_preview()

    def _undo(self):
        if self.sequence:
            self.sequence.pop()
            self._refresh_preview()

    def _refresh_preview(self):
        lines = [f"{i+1}. {k}" for i, k in enumerate(self.sequence)]
        self.seq_text.setPlainText('\n'.join(lines))

    def _confirm(self):
        name = self.name_input.text().strip()
        if not name:
            self.name_input.setPlaceholderText("⚠ Nhập tên trước!")
            return
        if not self.sequence:
            return

        data = {}
        if os.path.exists(self.multi_wp_file):
            try:
                with open(self.multi_wp_file, encoding='utf-8') as file:
                    data = json.load(file)
            except (OSError, json.JSONDecodeError) as exc:
                QMessageBox.warning(
                    self,
                    "Không thể lưu lộ trình",
                    f"Không đọc được dữ liệu lộ trình: {exc}",
                )
                return
            if not isinstance(data, dict):
                QMessageBox.warning(
                    self,
                    "Không thể lưu lộ trình",
                    "File lộ trình không đúng định dạng.",
                )
                return

        if name in data:
            answer = QMessageBox.question(
                self,
                "Lộ trình đã tồn tại",
                f"Bạn muốn thay thế lộ trình '{name}' không?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return

        data[name] = {'map_name': self.current_map, 'sequence': self.sequence}
        temp_path = self.multi_wp_file + '.tmp'
        try:
            with open(temp_path, 'w', encoding='utf-8') as file:
                json.dump(data, file, indent=2, ensure_ascii=False)
            os.replace(temp_path, self.multi_wp_file)
        except OSError as exc:
            QMessageBox.warning(
                self,
                "Không thể lưu lộ trình",
                f"Không ghi được file lộ trình: {exc}",
            )
            return
        self.accept()


class WaypointsModeLayout(QMainWindow):
    def __init__(self):
        super().__init__()
        self.process_mgr = ProcessManager()
        try:
            rclpy.init()
        except:
            pass
        self.ros_node = WaypointsNode()
        self.waypoints_file = f'{SOURCE_PATH}/robot_ui/waypoints.json'
        self.waypoints = self.load_waypoints()
        self.current_mode = None
        self.selected_sequence = []
        self.running_sequence = False
        self.current_sequence_index = 0
        self._current_nav_target = None
        self._navigation_generation = 0
        self._announce_thread = None
        self.init_ui()

        self.log("[Giọng nói] Đang lắng nghe — hãy nói để điều hướng")

        QTimer.singleShot(500, self._auto_start_voice)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_and_update)
        self.timer.start(50)
        
    def init_ui(self):
        self.setWindowTitle('Waypoints Mode')
        self.showMaximized()

        self.setStyleSheet(MAIN_STYLESHEET)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ── Left panel ────────────────────────────────────────────────────────
        left_panel = QWidget()
        left_panel.setObjectName("left-panel")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)

        wordmark = QLabel("ĐIỂM ĐẾN")
        wordmark.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        wordmark.setStyleSheet("color: #fcb525; padding: 24px 24px 16px 24px;")
        left_layout.addWidget(wordmark)

        mono = QFont("DM Sans", 14)
        self.load_map_btn   = QPushButton("Tải bản đồ")
        self.waypoints_btn  = QPushButton("Địa điểm")
        self.lo_trinh_btn   = QPushButton("Tạo lộ trình")
        self.new_btn        = QPushButton("Tạo địa điểm mới")
        self.stop_btn       = QPushButton("Dừng")
        self.back_btn       = QPushButton("Quay lại")

        for btn in [self.load_map_btn, self.waypoints_btn, self.lo_trinh_btn,
                    self.new_btn, self.stop_btn, self.back_btn]:
            btn.setObjectName("action-btn")
            btn.setFont(mono)
            btn.setMinimumHeight(52)
            btn.setCheckable(False)
            left_layout.addWidget(btn)

        # Position section
        pos_title = QLabel("VỊ TRÍ")
        pos_title.setObjectName("section-title")
        pos_title.setFont(QFont("DM Sans", 11))
        left_layout.addWidget(pos_title)

        self.pos_label = QLabel("x: 0.00   y: 0.00   z: 0.00")
        self.pos_label.setObjectName("pos-value")
        self.pos_label.setFont(QFont("JetBrains Mono", 13))
        left_layout.addWidget(self.pos_label)

        orient_title = QLabel("HƯỚNG")
        orient_title.setObjectName("section-title")
        orient_title.setFont(QFont("DM Sans", 11))
        left_layout.addWidget(orient_title)

        self.orient_label = QLabel("qx: 0.00   qy: 0.00\nqz: 0.00   qw: 1.00")
        self.orient_label.setObjectName("pos-value")
        self.orient_label.setFont(QFont("JetBrains Mono", 13))
        left_layout.addWidget(self.orient_label)

        left_layout.addStretch()

        self.load_map_btn.clicked.connect(self.load_map)
        self.waypoints_btn.clicked.connect(self.open_waypoint_picker)
        self.lo_trinh_btn.clicked.connect(self.open_path_manager)
        self.new_btn.clicked.connect(self.open_new_waypoint_dialog)
        self.stop_btn.clicked.connect(self.stop_navigation)
        self.back_btn.clicked.connect(self.go_back)

        # ── Right area ────────────────────────────────────────────────────────
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        # Header bar
        header = QWidget()
        header.setObjectName("header-bar")
        header.setFixedHeight(68)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(20, 0, 20, 0)

        header_title = QLabel("ĐIỀU HƯỚNG ĐIỂM ĐẾN")
        header_title.setObjectName("header-title")
        header_title.setFont(QFont("JetBrains Mono", 15, QFont.Weight.Bold))
        header_title.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)

        self.clock_label = QLabel()
        self.clock_label.setObjectName("clock")
        self.clock_label.setFont(QFont("JetBrains Mono", 15))

        header_layout.addWidget(header_title)
        header_layout.addStretch()
        header_layout.addWidget(self.clock_label)
        right_layout.addWidget(header)

        # Map
        map_container = QWidget()
        map_container.setStyleSheet("background-color: #f0f4ff; padding: 12px;")
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(12, 12, 12, 0)
        map_layout.setSpacing(0)

        map_load_error = None
        try:
            map_yaml_path = get_current_map_path()
            map_dir = os.path.dirname(map_yaml_path)
            yaml_data = load_map_yaml(map_yaml_path)
            map_image_path = os.path.join(map_dir, yaml_data['image'])
        except Exception as exc:
            yaml_data = {'resolution': 1.0, 'origin': [0.0, 0.0, 0.0]}
            map_image_path = ''
            map_load_error = str(exc)

        self.map_widget = MapWidget(map_image_path, yaml_data)
        self.map_widget.set_waypoints(self.waypoints)

        self.chat_widget = ChatPanel()
        self.chat_widget.waypoint_command.connect(self.voice_navigate_to_waypoint)
        self.chat_widget.set_pose_provider(
            lambda: self.ros_node.current_pose.pose.pose if self.ros_node.current_pose else None
        )
        self.chat_widget.set_waypoints_provider(
            self._get_current_map_waypoint_descriptors
        )

        map_layout.addWidget(self.map_widget)

        mic_btn = self.chat_widget.voice_btn
        mic_btn.setText("Mic")
        mic_btn.setToolTip("Bắt đầu hoặc dừng nghe")
        mic_btn.setFixedSize(58, 44)
        mic_btn.setStyleSheet("""
            QPushButton {
                background-color: #214196;
                color: #ffffff;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: 600;
            }
            QPushButton:hover { background-color: #1a3278; }
            QPushButton:checked { background-color: #b42318; }
        """)

        self.navigation_status = QLabel()
        self.navigation_status.setMinimumWidth(150)
        self._set_navigation_status(
            "Nav2 sẵn sàng" if self.ros_node.nav_server_available else "Nav2 chưa sẵn sàng",
            "ready" if self.ros_node.nav_server_available else "error",
        )

        voice_status = self.chat_widget.voice_status_label
        voice_status.setAlignment(Qt.AlignmentFlag.AlignVCenter)
        voice_status.setMinimumWidth(130)

        stop_voice_btn = self.chat_widget.interrupt_btn
        stop_voice_btn.setText("Dừng")
        stop_voice_btn.setFixedSize(58, 44)
        stop_voice_btn.setToolTip("Dừng nghe hoặc phát âm")

        voice_controls = QWidget()
        voice_controls_layout = QHBoxLayout(voice_controls)
        voice_controls_layout.setContentsMargins(0, 0, 0, 0)
        voice_controls_layout.setSpacing(8)
        voice_controls_layout.addWidget(self.navigation_status)
        voice_controls_layout.addWidget(voice_status)
        voice_controls_layout.addWidget(mic_btn)
        voice_controls_layout.addWidget(stop_voice_btn)
        header_layout.insertWidget(1, voice_controls)
        self.chat_widget.hide()

        # Log panel
        self.log_panel = QWidget()
        self.log_panel.setObjectName("log-panel")
        log_layout = QVBoxLayout(self.log_panel)
        log_layout.setContentsMargins(16, 12, 16, 12)
        log_layout.setSpacing(6)

        log_header = QLabel("NHẬT KÝ HỆ THỐNG")
        log_header.setFont(QFont("DM Sans", 11, QFont.Weight.Bold))
        log_header.setStyleSheet("color: #214196; padding: 4px 0;")
        log_layout.addWidget(log_header)

        self.log_text = QTextEdit()
        self.log_text.setObjectName("log-text")
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Code", 13))
        log_layout.addWidget(self.log_text)

        self.chat_history_box = QTextEdit()
        self.chat_history_box.setReadOnly(True)
        self.chat_history_box.setPlaceholderText("Hội thoại sẽ xuất hiện ở đây")
        self.chat_history_box.setStyleSheet("""
            QTextEdit {
                background-color: #ffffff;
                color: #1a2a5e;
                border: none;
                padding: 12px;
                font-family: "DM Sans";
                font-size: 15px;
            }
        """)

        chat_panel = QWidget()
        chat_panel.setObjectName("chat-panel")
        chat_panel_layout = QVBoxLayout(chat_panel)
        chat_panel_layout.setContentsMargins(8, 8, 8, 8)
        chat_panel_layout.addWidget(self.chat_history_box)

        self.activity_tabs = QTabWidget()
        self.activity_tabs.addTab(chat_panel, "Hội thoại")
        self.activity_tabs.addTab(self.log_panel, "Nhật ký")
        self.chat_widget.log_signal.connect(self._append_chat_message)

        workspace_splitter = QSplitter(Qt.Orientation.Vertical)
        workspace_splitter.setChildrenCollapsible(False)
        workspace_splitter.addWidget(map_container)
        workspace_splitter.addWidget(self.activity_tabs)
        workspace_splitter.setStretchFactor(0, 4)
        workspace_splitter.setStretchFactor(1, 1)
        workspace_splitter.setSizes([680, 240])
        right_layout.addWidget(workspace_splitter, 1)

        main_layout.addWidget(left_panel, 22)
        main_layout.addWidget(right_widget, 78)

        self.clock_timer = setup_clock_timer(self.clock_label)

        self.update_map_waypoints()
        self.log("Đã chuyển sang chế độ điểm đến")

        if not self.ros_node.nav_server_available:
            self.log("[CẢNH BÁO] Máy chủ Nav2 chưa sẵn sàng")
        if map_load_error:
            self.log(f'[ERROR] Không tải được bản đồ: {map_load_error}')

    def _auto_start_voice(self):
        self.chat_widget._voice_enabled = True

    def _set_navigation_status(self, message, state='ready'):
        status_label = getattr(self, 'navigation_status', None)
        if status_label is None:
            return

        colors = {
            'ready': '#18794e',
            'active': '#214196',
            'checking': '#8a6100',
            'error': '#b42318',
        }
        status_label.setText(message)
        status_label.setStyleSheet(
            f'color: {colors.get(state, colors["ready"])}; font-weight: 600;'
        )

    def _append_chat_message(self, message):
        if not message:
            return

        if message.startswith(('[Bạn]', '[Bé Son]')):
            target = self.chat_history_box
            self.activity_tabs.setCurrentIndex(0)
        else:
            target = self.log_text

        cursor = target.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        cursor.insertText(message.rstrip() + '\n')
        target.setTextCursor(cursor)
        target.ensureCursorVisible()

    def log(self, message):
        append_log(self.log_text, message)
        
    def spin_and_update(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        if self.ros_node.current_pose:
            msg = self.ros_node.current_pose
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            self.pos_label.setText(f"x: {p.x:.2f}   y: {p.y:.2f}   z: {p.z:.2f}")
            self.orient_label.setText(f"qx: {o.x:.2f}   qy: {o.y:.2f}\nqz: {o.z:.2f}   qw: {o.w:.2f}")
            self.map_widget.set_robot_pose(msg)
    
    def navigate_to_waypoint(self, slot_num, generation=None):
        if generation is None:
            generation = self._navigation_generation
        if generation != self._navigation_generation:
            return

        target = str(slot_num)
        waypoint = self.waypoints.get(target)
        if waypoint is None:
            self.log(f'[NAV] Không tìm thấy địa điểm "{target}"')
            self._set_navigation_status('Không tìm thấy điểm đến', 'error')
            self.running_sequence = False
            return

        required_fields = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
        missing_fields = [field for field in required_fields if field not in waypoint]
        if missing_fields:
            self.log(
                f'[NAV] Địa điểm "{target}" thiếu dữ liệu: '
                f'{", ".join(missing_fields)}'
            )
            self._set_navigation_status('Dữ liệu điểm đến không hợp lệ', 'error')
            self.running_sequence = False
            return
        
        self._set_navigation_status('Đang kết nối Nav2', 'checking')
        if not self.ros_node.nav_server_available:
            self.log('[NAV] Đang chờ máy chủ Nav2...')
            self.ros_node.nav_server_available = self.ros_node.nav_client.wait_for_server(timeout_sec=3.0)
        if not self.ros_node.nav_server_available:
            self.log('[NAV] [LỖI] Máy chủ Nav2 không khả dụng')
            self._set_navigation_status('Nav2 chưa sẵn sàng', 'error')
            self.running_sequence = False
            return
        
        if self.ros_node.current_goal_handle is not None:
            self.log('Đang hủy điều hướng trước...')
            self.ros_node.current_goal_handle.cancel_goal_async()
            self.ros_node.current_goal_handle = None
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(waypoint['x'])
        goal_msg.pose.pose.position.y = float(waypoint['y'])
        goal_msg.pose.pose.position.z = float(waypoint['z'])
        goal_msg.pose.pose.orientation.x = float(waypoint['qx'])
        goal_msg.pose.pose.orientation.y = float(waypoint['qy'])
        goal_msg.pose.pose.orientation.z = float(waypoint['qz'])
        goal_msg.pose.pose.orientation.w = float(waypoint['qw'])
        if waypoint.get('yaw_tolerance'):
            self._set_yaw_tolerance(waypoint['yaw_tolerance'])
        
        send_goal_future = self.ros_node.nav_client.send_goal_async(goal_msg)
        self._current_nav_target = target
        self._set_navigation_status(f'Đang gửi điểm đến: {target}', 'active')
        send_goal_future.add_done_callback(
            lambda future, target=target, generation=generation:
            self._goal_response_callback(future, target, generation)
        )
        
    def _goal_response_callback(self, future, target, generation):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if generation == self._navigation_generation:
                self.log(f'[NAV] Không nhận được phản hồi goal: {exc}')
                self.running_sequence = False
                self._set_navigation_status('Không nhận được phản hồi từ Nav2', 'error')
            return

        if generation != self._navigation_generation:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if goal_handle.accepted:
            self.ros_node.current_goal_handle = goal_handle
            self.log(f'Đang điều hướng đến {target}')
            self._set_navigation_status(f'Đang tới: {target}', 'active')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future, target=target, generation=generation:
                self._goal_result_callback(future, target, generation)
            )
        else:
            self.log(f'Mục tiêu {target} bị từ chối')
            self.ros_node.current_goal_handle = None
            self.running_sequence = False
            self._set_navigation_status('Nav2 từ chối điểm đến', 'error')

    def _set_yaw_tolerance(self, value: float):
        """Dynamically set yaw_goal_tolerance on controller_server via ros2 param."""
        subprocess.Popen([
            'ros2', 'param', 'set', '/controller_server',
            'goal_checker.yaw_goal_tolerance', str(value)
        ])

    def _goal_result_callback(self, future, target, generation):
        if generation != self._navigation_generation:
            return

        try:
            status = future.result().status
        except Exception as exc:
            self.ros_node.current_goal_handle = None
            self.running_sequence = False
            self.log(f'[NAV] Không nhận được kết quả goal: {exc}')
            self._set_navigation_status('Không nhận được kết quả từ Nav2', 'error')
            return
        print(f"[Nav] result target={target}, status={status} (SUCCEEDED={GoalStatus.STATUS_SUCCEEDED})")
        waypoint = self.waypoints.get(target, {})
        self.ros_node.current_goal_handle = None
        if waypoint.get('yaw_tolerance'):
            self._set_yaw_tolerance(0.25)
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.log(f'[NAV] Mục tiêu {target} không thành công (status={status}) — dừng chuỗi')
            self.running_sequence = False
            self._set_navigation_status(f'Không tới được: {target}', 'error')
            return

        self.log(f'Đã đến {target}')
        self._announce_arrival(target)

        if self.running_sequence and self.current_sequence_index < len(self.selected_sequence) - 1:
            self.current_sequence_index += 1
            self.navigate_to_waypoint(
                self.selected_sequence[self.current_sequence_index],
                generation,
            )
        else:
            self.running_sequence = False
            self.log('Đã hoàn thành chuỗi điều hướng')
            self._set_navigation_status(f'Đã tới: {target}', 'ready')

    def _announce_arrival(self, target: str):
        print(f"[Announce] _announce_arrival called with target='{target}'")
        QMetaObject.invokeMethod(
            self.chat_widget._voice_engine,
            "speak",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(str, "Đã tới nơi rồi")
        )
    
    def run_sequence(self):
        if not self.selected_sequence:
            self.log('Chưa chọn địa điểm nào.')
            return
        
        self._navigation_generation += 1
        generation = self._navigation_generation
        self.log(f'Bắt đầu chuỗi: {self.selected_sequence}')
        self.running_sequence = True
        self.current_sequence_index = 0
        self.navigate_to_waypoint(self.selected_sequence[0], generation)
    
    def reset_sequence(self):
        self._navigation_generation += 1
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
            self.ros_node.current_goal_handle = None
        self.selected_sequence = []
        self.running_sequence = False
        self.current_sequence_index = 0
        self.log('Đã đặt lại chuỗi')
        self._set_navigation_status('Lộ trình đã xóa', 'ready')
        
    def stop_navigation(self):
        self._navigation_generation += 1
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
            self.ros_node.current_goal_handle = None
            self.log('Đã hủy điều hướng')
        else:
            self.log('Không có điều hướng nào đang chạy')
        
        self.running_sequence = False
        self.selected_sequence = []
        self.current_sequence_index = 0
        self.log('Đã xóa chuỗi')
        self._set_navigation_status('Đã dừng điều hướng', 'ready')

    def _get_current_map_waypoint_descriptors(self):
        current_map = get_current_map_name()
        return [
            {'key': key, 'aliases': waypoint.get('aliases', [])}
            for key, waypoint in self.waypoints.items()
            if waypoint.get('map_name') == current_map
        ]
        
    def update_map_waypoints(self):
        """Show only waypoints belonging to the current map on the map widget."""
        current_map = get_current_map_name()
        filtered = {k: v for k, v in self.waypoints.items()
                    if isinstance(v, dict) and v.get('map_name') == current_map}
        self.map_widget.set_waypoints(filtered)

    def open_waypoint_picker(self):
        dialog = WaypointPickerDialog(self.waypoints, get_current_map_name(), self)
        if dialog.exec():
            key = dialog.get_selected_key()
            if key:
                self.log(f'Đang điều hướng đến: {key}')
                self.selected_sequence = [key]
                self.running_sequence = True
                self.current_sequence_index = 0
                self._navigation_generation += 1
                self.navigate_to_waypoint(key, self._navigation_generation)

    def open_path_manager(self):
        multi_wp_file = f'{SOURCE_PATH}/robot_ui/multi_waypoints.json'
        current_map = get_current_map_name()
        while True:
            dlg = PathManagerDialog(multi_wp_file, current_map, self)
            dlg.run_path_requested.connect(self._run_multi_path)
            result = dlg.exec()
            if result == 2:   # New path
                new_dlg = NewPathDialog(self.waypoints, current_map, multi_wp_file, self)
                new_dlg.exec()
                # loop back to PathManagerDialog with refreshed list
                continue
            break   # accepted (run) or rejected (back)

    def _run_multi_path(self, sequence):
        if not isinstance(sequence, list) or not sequence:
            self.log('[CẢNH BÁO] Lộ trình không có điểm đến hợp lệ')
            return
        missing_waypoints = [key for key in sequence if key not in self.waypoints]
        if missing_waypoints:
            self.log(
                '[CẢNH BÁO] Lộ trình có điểm chưa tồn tại: '
                + ', '.join(str(key) for key in missing_waypoints)
            )
            return
        self.log(f'Đang chạy lộ trình: {sequence}')
        self.selected_sequence = list(sequence)
        self.running_sequence = True
        self.current_sequence_index = 0
        self._navigation_generation += 1
        self.navigate_to_waypoint(sequence[0], self._navigation_generation)

    def load_waypoints(self):
        try:
            with open(self.waypoints_file, 'r', encoding='utf-8') as file:
                content = file.read().strip()
                if not content:
                    return {}
                data = json.loads(content)
            if not isinstance(data, dict):
                raise ValueError('Waypoint file must contain a JSON object')

            required_fields = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'map_name')
            valid_waypoints = {}
            for key, waypoint in data.items():
                if not isinstance(key, str) or not isinstance(waypoint, dict):
                    continue
                if any(field not in waypoint for field in required_fields):
                    continue
                if not isinstance(waypoint['map_name'], str):
                    continue
                try:
                    coordinates = {
                        field: float(waypoint[field])
                        for field in required_fields[:-1]
                    }
                except (TypeError, ValueError):
                    continue
                if not all(math.isfinite(value) for value in coordinates.values()):
                    continue
                clean_waypoint = dict(waypoint)
                clean_waypoint.update(coordinates)
                aliases = clean_waypoint.get('aliases', [])
                clean_waypoint['aliases'] = (
                    [alias for alias in aliases if isinstance(alias, str)]
                    if isinstance(aliases, list) else []
                )
                valid_waypoints[key] = clean_waypoint

            skipped_count = len(data) - len(valid_waypoints)
            if skipped_count:
                print(f'[WAYPOINTS] Bỏ qua {skipped_count} waypoint không hợp lệ')
            return valid_waypoints
        except FileNotFoundError:
            return {}
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            print(f'[WAYPOINTS] Không đọc được waypoint: {exc}')
            return {}
            
    def save_waypoints(self):
        temp_path = self.waypoints_file + '.tmp'
        try:
            with open(temp_path, 'w', encoding='utf-8') as file:
                json.dump(self.waypoints, file, indent=2, ensure_ascii=False)
            os.replace(temp_path, self.waypoints_file)
        except OSError as exc:
            self.log(f'[ERROR] Không lưu được waypoint: {exc}')
            return False
        return True
            
    def log(self, message):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        if "[ERROR]" in message:
            color = "#ff3b3b"
        elif "[WARN]" in message or "WARNING" in message:
            color = "#ffb300"
        elif "✓" in message or "completed" in message.lower():
            color = "#00c853"
        else:
            color = "#1a2a5e"
        self.log_text.append(
            f'<span style="color:#8fa3cc">[{ts}]</span> <span style="color:{color}">{message}</span>'
        )
        
    def voice_navigate_to_waypoint(self, slots: str):
        slot_list = [s.strip() for s in slots.split(',') if s.strip()]
        # case-insensitive key resolution
        key_map = {k.lower(): k for k in self.waypoints}
        resolved = []
        for s in slot_list:
            if s.startswith('__return_here__:'):
                # Inject synthetic waypoint from encoded coordinates
                _, coords = s.split(':', 1)
                parts = coords.split(';')
                rx, ry = float(parts[0]), float(parts[1])
                rqz = float(parts[2]) if len(parts) > 2 else 0.0
                rqw = float(parts[3]) if len(parts) > 3 else 1.0
                _RETURN_KEY = '__return_here__'
                self.waypoints[_RETURN_KEY] = {
                    'x': rx, 'y': ry, 'z': 0.0,
                    'qx': 0.0, 'qy': 0.0, 'qz': rqz, 'qw': rqw,
                    'yaw_tolerance': 3.14,
                }
                resolved.append(_RETURN_KEY)
            elif s == '__return_here__':
                # Pose was unavailable when command was issued — capture it now
                pose = self.ros_node.current_pose
                if pose:
                    _RETURN_KEY = '__return_here__'
                    self.waypoints[_RETURN_KEY] = {
                        'x': pose.pose.pose.position.x,
                        'y': pose.pose.pose.position.y,
                        'z': 0.0,
                        'qx': pose.pose.pose.orientation.x,
                        'qy': pose.pose.pose.orientation.y,
                        'qz': pose.pose.pose.orientation.z,
                        'qw': pose.pose.pose.orientation.w,
                        'yaw_tolerance': 3.14,
                    }
                    resolved.append(_RETURN_KEY)
                else:
                    resolved.append(None)  # will be caught as missing
            else:
                resolved.append(key_map.get(s.lower()))
        missing = [slot_list[i] for i, r in enumerate(resolved) if r is None]
        if missing:
            has_return_sentinel = any(s.startswith('__return_here__') for s in missing)
            if has_return_sentinel:
                msg = 'Không thể xác định vị trí hiện tại để quay về — robot chưa được định vị.'
            else:
                msg = f'Vị trí chưa được lưu: {", ".join(m for m in missing if not m.startswith("__return_here__"))}'
            self.log(f'[Voice] {msg}')
            self.chat_widget._voice_engine.speak(msg)
            return
        self.log(f'[Voice] Tour: {resolved}')
        self.chat_widget._voice_engine.speak(f'Bắt đầu tham quan {len(resolved)} địa điểm')
        self.selected_sequence = resolved
        self.running_sequence = True
        self.current_sequence_index = 0
        self._navigation_generation += 1
        self.navigate_to_waypoint(resolved[0], self._navigation_generation)

    def open_new_waypoint_dialog(self):
        if not self.ros_node.current_pose:
            self.log('[CẢNH BÁO] Chưa có dữ liệu vị trí — không thể lưu địa điểm')
            return
        dialog = NewWaypointDialog(self)
        if dialog.exec():
            name = dialog.get_name()
            if not name:
                self.log('[CẢNH BÁO] Tên địa điểm không được để trống')
                return
            if name in self.waypoints:
                self.log(f'[CẢNH BÁO] Địa điểm "{name}" đã tồn tại — hãy chọn tên khác')
                return
            pose = self.ros_node.current_pose
            self.waypoints[name] = {
                'x': pose.pose.pose.position.x,
                'y': pose.pose.pose.position.y,
                'z': pose.pose.pose.position.z,
                'qx': pose.pose.pose.orientation.x,
                'qy': pose.pose.pose.orientation.y,
                'qz': pose.pose.pose.orientation.z,
                'qw': pose.pose.pose.orientation.w,
                'map_name': get_current_map_name(),
            }
            if self.save_waypoints():
                self.update_map_waypoints()
                self.log(f'Đã lưu địa điểm mới: {name}')
            else:
                del self.waypoints[name]

    def load_map(self):
        dialog = LoadMapDialog(self)
        if dialog.exec():
            map_name = dialog.get_selected_map()
            if map_name:
                self.log(f"Đang tải bản đồ: {map_name}")
                if self.running_sequence or self.ros_node.current_goal_handle is not None:
                    self.stop_navigation()
                if not update_map_files(map_name, self.log):
                    return
                try:
                    map_yaml_path = get_current_map_path()
                    map_dir = os.path.dirname(map_yaml_path)
                    yaml_data = load_map_yaml(map_yaml_path)
                    map_image_path = os.path.join(map_dir, yaml_data['image'])
                except (OSError, KeyError, TypeError, ValueError, SyntaxError) as exc:
                    self.log(f'[ERROR] Không tải được bản đồ mới: {exc}')
                    return

                self.map_widget.map_image = QPixmap(map_image_path)
                self.map_widget.resolution = float(yaml_data['resolution'])
                self.map_widget.origin = yaml_data['origin']
                if self.map_widget.map_image.isNull():
                    self.log('[ERROR] Không đọc được ảnh bản đồ đã chọn')
                self.update_map_waypoints()
                self.map_widget.update()

    def go_back(self):
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
        subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def closeEvent(self, event):
        self._navigation_generation += 1
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
        if self.ros_node:
            self.ros_node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass
        self.chat_widget.cleanup()
        self.process_mgr.cleanup_all()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WaypointsModeLayout()
    # Auto-navigate if launched with --go-to <slot>
    if '--go-to' in sys.argv:
        idx = sys.argv.index('--go-to')
        if idx + 1 < len(sys.argv):
            slots = sys.argv[idx + 1]
            QTimer.singleShot(1500, lambda: window.voice_navigate_to_waypoint(slots))
    sys.exit(app.exec())
