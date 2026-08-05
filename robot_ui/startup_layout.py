#!/usr/bin/env python3
import sys
import subprocess
import os
import threading
import time
import math
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QTextEdit, QLabel,
                             QSizePolicy)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from load_map_dialog import LoadMapDialog
from chat_panel_widget import ChatPanel
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH
from styles import MAIN_STYLESHEET
from ui_utils import setup_clock_timer
from map_utils import update_map_files
from process_manager import ProcessManager


# ── Tuning constants ──────────────────────────────────────────────────────────
ANGULAR_SPEED        = 0.314
HALF_ROTATION_RAD    = math.pi
HALF_ROTATION_TIME   = HALF_ROTATION_RAD / ANGULAR_SPEED

RAMP_STEPS           = 5
RAMP_INTERVAL        = 0.05

GOOD_COV_THRESHOLD   = 0.10
ACCEPTABLE_COV       = 0.20
MAX_RETRIES          = 2
SPIN_TICK            = 0.05
COV_LOCK_TIMEOUT     = 15.0
# ─────────────────────────────────────────────────────────────────────────────

# ══════════════════════════════════════════════════════════════════════════════
#  Localization Worker  (unchanged from original)
# ══════════════════════════════════════════════════════════════════════════════

class LocalizationWorker(QObject):
    log_signal      = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._stop_event    = threading.Event()
        self._cov_lock      = threading.Lock()
        self._current_cov   = float('inf')
        self._best_cov      = float('inf')
        self._received_pose = threading.Event()

    def stop(self):
        self._stop_event.set()

    def _pose_callback(self, msg):
        cov = msg.pose.covariance
        total = cov[0] + cov[7] + cov[35]
        with self._cov_lock:
            self._current_cov = total
            if total < self._best_cov:
                self._best_cov = total
        self._received_pose.set()

    def _ramp_velocity(self, publisher, target_z: float):
        twist = Twist()
        for i in range(1, RAMP_STEPS + 1):
            if self._stop_event.is_set():
                break
            twist.angular.z = target_z * (i / RAMP_STEPS)
            publisher.publish(twist)
            time.sleep(RAMP_INTERVAL)

    def _stop_robot(self, publisher):
        self._ramp_velocity(publisher, 0.0)
        twist = Twist()
        twist.angular.z = 0.0
        publisher.publish(twist)

    def _spin_arc(self, publisher, node, arc_time: float, label: str):
        self._ramp_velocity(publisher, ANGULAR_SPEED)
        twist = Twist()
        twist.angular.z = ANGULAR_SPEED
        elapsed    = 0.0
        arc_best   = float('inf')
        tick_count = max(1, int(arc_time / SPIN_TICK))

        for i in range(tick_count):
            if self._stop_event.is_set():
                self.log_signal.emit(f"[{label}] Stop requested — aborting arc")
                break
            publisher.publish(twist)
            rclpy.spin_once(node, timeout_sec=SPIN_TICK)
            elapsed += SPIN_TICK
            with self._cov_lock:
                cov = self._current_cov
            if cov < arc_best:
                arc_best = cov
            progress = int(elapsed / arc_time * 100)
            if i > 0 and i % int(tick_count / 5 or 1) == 0:
                self.log_signal.emit(
                    f"[{label}] {progress}% — current σ2={cov:.4f}, best={arc_best:.4f}"
                )
            if cov < GOOD_COV_THRESHOLD:
                self.log_signal.emit(
                    f"[{label}] Early exit at {elapsed:.1f}s — covariance {cov:.4f} "
                    f"< threshold {GOOD_COV_THRESHOLD}"
                )
                break
        return elapsed, arc_best

    def run(self):
        try:
            rclpy.init()
        except Exception:
            pass
        node = Node('localization_worker')
        cmd_vel_pub        = node.create_publisher(Twist, '/cmd_vel', 10)
        _pose_sub          = node.create_subscription(          # noqa: F841
            PoseWithCovarianceStamped, '/amcl_pose',
            self._pose_callback, 10
        )
        global_loc_client  = node.create_client(Empty, '/reinitialize_global_localization')
        clear_local_client = node.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        try:
            self._run_sequence(node, cmd_vel_pub, global_loc_client, clear_local_client)
        except Exception as exc:
            self.log_signal.emit(f"[ERROR] Localization sequence failed: {exc}")
            self._stop_robot(cmd_vel_pub)
        finally:
            node.destroy_node()
            self.finished_signal.emit()

    def _run_sequence(self, node, cmd_vel_pub, global_loc_client, clear_local_client):
        self.log_signal.emit("Waiting for /reinitialize_global_localization service...")
        if not global_loc_client.wait_for_service(timeout_sec=10.0):
            self.log_signal.emit("[ERROR] Reinitialize service unavailable — aborting")
            return
        self.log_signal.emit("Calling /reinitialize_global_localization")
        global_loc_client.call_async(Empty.Request())
        rclpy.spin_once(node, timeout_sec=1.0)
        self.log_signal.emit(f"Waiting up to {COV_LOCK_TIMEOUT}s for first AMCL pose...")
        deadline = time.monotonic() + COV_LOCK_TIMEOUT
        while not self._received_pose.is_set() and not self._stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() > deadline:
                self.log_signal.emit("[ERROR] No AMCL pose received — check Nav2/AMCL — aborting")
                return
        if self._stop_event.is_set():
            return

        for attempt in range(1, MAX_RETRIES + 2):
            if self._stop_event.is_set():
                break
            with self._cov_lock:
                pre_cov = self._current_cov
            if pre_cov < GOOD_COV_THRESHOLD:
                self.log_signal.emit(
                    f"Covariance already {pre_cov:.4f} < {GOOD_COV_THRESHOLD} "
                    f"— skipping rotation {attempt}"
                )
                break
            self.log_signal.emit(
                f"── Rotation attempt {attempt}/{MAX_RETRIES + 1} "
                f"(pre-spin σ2={pre_cov:.4f}) ──"
            )
            t1, best_1 = self._spin_arc(cmd_vel_pub, node, HALF_ROTATION_TIME, f"R{attempt} first 180°")
            self.log_signal.emit(f"First 180° done in {t1:.1f}s — best σ2={best_1:.4f}")
            with self._cov_lock:
                mid_cov = self._current_cov
            if mid_cov < GOOD_COV_THRESHOLD:
                self.log_signal.emit("Well-localised after first half — stopping spin")
                break
            t2, best_2 = self._spin_arc(cmd_vel_pub, node, HALF_ROTATION_TIME, f"R{attempt} second 180°")
            self.log_signal.emit(f"Second 180° done in {t2:.1f}s — best σ2={best_2:.4f}")
            delta = abs(best_1 - best_2)
            self.log_signal.emit(
                f"Half-covariance delta: {delta:.4f} "
                f"({'asymmetric occlusion suspected' if delta > 0.15 else 'symmetric'})"
            )
            with self._cov_lock:
                post_cov = self._best_cov
            self.log_signal.emit(f"Best overall σ2 after attempt {attempt}: {post_cov:.4f}")
            if post_cov < ACCEPTABLE_COV:
                self.log_signal.emit("Acceptable covariance reached — done")
                break
            elif attempt <= MAX_RETRIES:
                self.log_signal.emit(
                    f"Covariance {post_cov:.4f} still above {ACCEPTABLE_COV} "
                    f"— retrying ({attempt}/{MAX_RETRIES})..."
                )
            else:
                self.log_signal.emit(
                    f"[WARN] Max retries reached — best σ2={post_cov:.4f}. "
                    f"Manual intervention may be needed."
                )

        self._stop_robot(cmd_vel_pub)
        self.log_signal.emit("Robot stopped")
        if clear_local_client.wait_for_service(timeout_sec=2.0):
            clear_local_client.call_async(Empty.Request())
            self.log_signal.emit("Cleared local costmap")
        else:
            self.log_signal.emit("[WARN] local_costmap clear service not available")
        rclpy.spin_once(node, timeout_sec=1.0)
        with self._cov_lock:
            final_cov = self._best_cov
        self.log_signal.emit(f"Relocalization complete — final best σ2={final_cov:.4f}")


# ══════════════════════════════════════════════════════════════════════════════
#  Main UI
# ══════════════════════════════════════════════════════════════════════════════

class RobotUI(QMainWindow):
    def __init__(self, skip_micro_ros=False):
        super().__init__()
        self.process_mgr = ProcessManager()
        self.prev_stm32_status       = None
        self.prev_lidar_status       = None
        self.prev_lidar_rear_status  = None
        self.localization_worker     = None
        self.localization_thread     = None
        self._latest_pose            = None
        self._stm32_last_msg_time        = None
        self._front_lidar_last_msg_time  = None
        self._rear_lidar_last_msg_time   = None
        self._switching_layout       = False  # Track if switching to another layout

        try:
            rclpy.init()
        except Exception:
            pass
        self._ros_node = Node('robot_ui_node')
        _amcl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._ros_node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback, _amcl_qos
        )
        self._ros_node.create_subscription(
            Odometry, '/odomfromSTM32', self._stm32_odom_callback, 10
        )
        self._ros_node.create_subscription(
            LaserScan, '/front_lidar/scan', lambda msg: self._lidar_callback('front'), 10
        )
        self._ros_node.create_subscription(
            LaserScan, '/rear_lidar/scan', lambda msg: self._lidar_callback('rear'), 10
        )
        self._ros_spin_timer = QTimer()
        self._ros_spin_timer.timeout.connect(self._ros_spin_once)
        self._ros_spin_timer.start(100)

        self.init_ui()
        self.chat_panel._voice_enabled = True
        if not skip_micro_ros:
            self.start_micro_ros()

    def _ros_spin_once(self):
        try:
            if rclpy.ok():
                rclpy.spin_once(self._ros_node, timeout_sec=0)
        except Exception:
            self._ros_spin_timer.stop()

    def _amcl_callback(self, msg):
        self._latest_pose = msg.pose.pose

    def _stm32_odom_callback(self, msg):
        self._stm32_last_msg_time = time.monotonic()

    def _lidar_callback(self, which):
        if which == 'front':
            self._front_lidar_last_msg_time = time.monotonic()
        else:
            self._rear_lidar_last_msg_time = time.monotonic()

    # ══════════════════════════════════════════════════════════════════════════
    #  UI construction
    # ══════════════════════════════════════════════════════════════════════════

    def init_ui(self):
        self.setWindowTitle("IUH Robot – Giao diện điều khiển")

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

        wordmark = QLabel("IUH ROBOT")
        wordmark.setFont(QFont("JetBrains Mono", 16, QFont.Weight.Bold))
        wordmark.setStyleSheet("color: #fcb525; padding: 24px 24px 16px 24px;")
        left_layout.addWidget(wordmark)

        self.btn_waypoints  = QPushButton("Điểm đến")
        self.btn_docking    = QPushButton("Về trạm sạc")
        self.btn_load_map   = QPushButton("Tải bản đồ")
        self.btn_new_map    = QPushButton("Bản đồ mới")
        self.btn_tracking   = QPushButton("Theo dõi")
        self.btn_reestimate = QPushButton("Định vị lại")
        self.btn_nav2       = QPushButton("Nav2")

        mono = QFont("JetBrains Mono", 22)
        for btn in [self.btn_waypoints, self.btn_docking, self.btn_load_map,
                    self.btn_new_map, self.btn_tracking, self.btn_reestimate, self.btn_nav2]:
            btn.setObjectName("mode-btn")
            btn.setFont(mono)
            btn.setMinimumHeight(72)
            btn.setCheckable(False)
            left_layout.addWidget(btn)

        left_layout.addStretch()

        self.btn_dev = QPushButton("⚙ Developer")
        self.btn_dev.setObjectName("mode-btn")
        self.btn_dev.setFont(QFont("JetBrains Mono", 15))
        self.btn_dev.setMinimumHeight(56)
        self.btn_dev.setCheckable(False)
        self.btn_dev.setStyleSheet(
            "QPushButton#mode-btn { color: #fcb525; border-left: 4px solid #fcb52544; }"
            "QPushButton#mode-btn:hover { background-color: #1a3278; border-left: 4px solid #fcb525; }"
        )
        left_layout.addWidget(self.btn_dev)

        self.btn_tracking.clicked.connect(lambda: self.mode_changed("Tracking"))
        self.btn_waypoints.clicked.connect(lambda: self.mode_changed("Waypoints"))
        self.btn_reestimate.clicked.connect(self.start_reestimate)
        self.btn_new_map.clicked.connect(self.start_new_map)
        self.btn_load_map.clicked.connect(self.load_map)
        self.btn_docking.clicked.connect(self.start_docking)
        self.btn_nav2.clicked.connect(lambda: self.mode_changed("Nav2"))
        self.btn_dev.clicked.connect(self.open_developer_mode)

        # ── Right area ────────────────────────────────────────────────────────
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        # Header bar
        header = QWidget()
        header.setObjectName("header-bar")
        header.setFixedHeight(48)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(20, 0, 20, 0)

        self.mode_label = QLabel("STARTUP")
        self.mode_label.setObjectName("mode-title")
        self.mode_label.setFont(QFont("JetBrains Mono", 15, QFont.Weight.Bold))

        self.clock_label = QLabel()
        self.clock_label.setObjectName("clock")
        self.clock_label.setFont(QFont("JetBrains Mono", 15))

        header_layout.addWidget(self.mode_label)
        header_layout.addStretch()
        header_layout.addWidget(self.clock_label)
        right_layout.addWidget(header)

        # Status cards row
        cards_widget = QWidget()
        cards_widget.setStyleSheet("background-color: #f0f4ff; padding: 12px;")
        cards_layout = QHBoxLayout(cards_widget)
        cards_layout.setContentsMargins(12, 12, 12, 12)
        cards_layout.setSpacing(12)

        self.stm32_card       = self._make_status_card("STM32")
        self.front_lidar_card = self._make_status_card("LiDAR Front")
        self.rear_lidar_card  = self._make_status_card("LiDAR Rear")
        cards_layout.addWidget(self.stm32_card["widget"])
        cards_layout.addWidget(self.front_lidar_card["widget"])
        cards_layout.addWidget(self.rear_lidar_card["widget"])
        right_layout.addWidget(cards_widget)

        # ── Voice panel fills the right content area ──────────────────────
        self.chat_panel = ChatPanel()
        self.chat_panel.hide()  # hidden — only owns the voice engine

        # ── Voice panel (right of log) ─────────────────────────────────────
        voice_panel = QWidget()
        voice_panel.setObjectName("voice-panel")
        voice_panel.setFixedWidth(560)
        voice_layout = QVBoxLayout(voice_panel)
        voice_layout.setContentsMargins(16, 12, 16, 12)
        voice_layout.setSpacing(12)
        voice_layout.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)

        mic_btn = self.chat_panel.voice_btn
        mic_btn.setFixedSize(500, 500)
        mic_btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        mic_btn.setStyleSheet("""
            QPushButton {
                background-color: #214196;
                color: #ffffff;
                border: 3px solid #a8bce8;
                border-radius: 250px;
                font-size: 60px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1a3278;
                border: 3px solid #fcb525;
            }
            QPushButton:checked {
                background-color: #ef4444;
                border: 3px solid #fca5a5;
            }
        """)

        voice_status = self.chat_panel.voice_status_label
        voice_status.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        voice_status.show()

        interrupt_btn = self.chat_panel.interrupt_btn
        interrupt_btn.setFixedSize(200, 130)

        voice_layout.addStretch()
        voice_layout.addWidget(mic_btn, 0, Qt.AlignmentFlag.AlignHCenter)
        voice_layout.addWidget(voice_status, 0, Qt.AlignmentFlag.AlignHCenter)
        voice_layout.addSpacing(16)
        voice_layout.addWidget(interrupt_btn, 0, Qt.AlignmentFlag.AlignHCenter)
        voice_layout.addStretch()

        # ── Main content row: voice panel fills full width ────────────────
        content_row = QWidget()
        content_layout = QHBoxLayout(content_row)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)
        content_layout.addWidget(voice_panel)

        right_layout.addWidget(content_row, 1)

        main_layout.addWidget(left_panel, 22)
        main_layout.addWidget(right_widget, 78)

        # Timers
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(5000)

        self.clock_timer = setup_clock_timer(self.clock_label)

        self._reestimate_pulse_timer = QTimer()
        self._reestimate_pulse_timer.timeout.connect(self._pulse_reestimate)
        self._pulse_state = False

        QTimer.singleShot(0, self.update_status)    # ══════════════════════════════════════════════════════════════════════════
    #  Existing UI helpers (unchanged)
    # ══════════════════════════════════════════════════════════════════════════

    def _make_status_card(self, device_name):
        card = QWidget()
        card.setObjectName("status-card")
        card.setMinimumHeight(100)
        card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout = QHBoxLayout(card)
        layout.setContentsMargins(16, 12, 20, 12)

        dot = QLabel("●")
        dot.setStyleSheet("color: #8fa3cc; font-size: 12px;")
        dot.setFixedWidth(20)

        info = QVBoxLayout()
        name_lbl = QLabel(device_name.upper())
        name_lbl.setObjectName("device-name")
        name_lbl.setFont(QFont("DM Sans", 11))

        state_lbl = QLabel("Checking...")
        state_lbl.setObjectName("status-checking")
        state_lbl.setFont(QFont("DM Sans", 14, QFont.Weight.Medium))

        info.addWidget(name_lbl)
        info.addWidget(state_lbl)

        layout.addWidget(dot)
        layout.addLayout(info)
        layout.addStretch()

        return {"widget": card, "dot": dot, "state": state_lbl}

    def _set_card_status(self, card, available):
        color  = "#22c55e" if available else "#ef4444"
        text   = "Hoạt động" if available else "Mất kết nối"
        obj    = "status-ok" if available else "status-error"
        card["dot"].setStyleSheet(f"color: {color}; font-size: 12px;")
        card["state"].setText(text)
        card["state"].setObjectName(obj)
        card["state"].setStyleSheet(f"color: {color}; font-size: 14px;")
        border_side = f"border-left: 4px solid {color};"
        card["widget"].setStyleSheet(
            f"QWidget#status-card {{ background-color: #ffffff; border: 1px solid #c8d4f0; "
            f"border-radius: 4px; {border_side} }}"
        )

    def _update_clock(self):
        from datetime import datetime
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S"))

    def _pulse_reestimate(self):
        self._pulse_state = not self._pulse_state
        color = "#fcb525" if self._pulse_state else "#8fa3cc"
        self.btn_reestimate.setStyleSheet(
            f"QPushButton#mode-btn {{ border-left: 4px solid {color}; color: #fcb525; background-color: #1a3278; }}"
        )

    def start_micro_ros(self):
        self.process_mgr.launch_terminal(
            'source ~/zackon_build_up/install/setup.bash && '
            'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; exec bash',
            'micro-ROS agent'
        )
        self.log("Đã khởi động micro-ROS agent trong terminal mới")

    def update_status(self):
        now = time.monotonic()

        stm32_available = (
            self._stm32_last_msg_time is not None and
            now - self._stm32_last_msg_time < 3.0
        )
        self._set_card_status(self.stm32_card, stm32_available)
        if self.prev_stm32_status is not None and self.prev_stm32_status != stm32_available:
            self.log("Mất kết nối STM32" if not stm32_available else "Đã khôi phục kết nối STM32")
        self.prev_stm32_status = stm32_available

        front_available = (
            self._front_lidar_last_msg_time is not None and
            now - self._front_lidar_last_msg_time < 3.0
        )
        self._set_card_status(self.front_lidar_card, front_available)
        if self.prev_lidar_status is not None and self.prev_lidar_status != front_available:
            self.log("Mất kết nối LiDAR trước" if not front_available else "Đã khôi phục kết nối LiDAR trước")
        self.prev_lidar_status = front_available

        rear_available = (
            self._rear_lidar_last_msg_time is not None and
            now - self._rear_lidar_last_msg_time < 3.0
        )
        self._set_card_status(self.rear_lidar_card, rear_available)
        if self.prev_lidar_rear_status is not None and self.prev_lidar_rear_status != rear_available:
            self.log("Mất kết nối LiDAR sau" if not rear_available else "Đã khôi phục kết nối LiDAR sau")
        self.prev_lidar_rear_status = rear_available

    def mode_changed(self, mode):
        self.log(f"Đã chuyển sang chế độ {mode}")
        if mode == "Tracking":
            subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/tracking_mode_layout.py'])
            self.close()
        elif mode == "Waypoints":
            subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/waypoints_mode_layout.py'])
            self.close()
        elif mode == "Nav2":
            self.process_mgr.launch_terminal(
                f'source {SOURCE_PATH}/install/setup.bash && '
                f'ros2 launch {SOURCE_PATH}/src/view_robot/launch/NAV2_BRINGUP.launch.py; exec bash',
                'Nav2'
            )
            self.log("Đã khởi động hệ thống điều hướng Nav2")

    def start_reestimate(self):
        if self.prev_stm32_status == False:
            self.log("Không thể định vị: STM32 chưa kết nối")
            return
        if self.localization_thread and self.localization_thread.is_alive():
            self.log("Quá trình định vị đang chạy")
            return
        self.log("Bắt đầu định vị toàn cục")
        self._reestimate_pulse_timer.start(600)

        self.localization_worker = LocalizationWorker()
        self.localization_worker.log_signal.connect(self.log)
        self.localization_worker.finished_signal.connect(self.localization_finished)
        self.localization_thread = threading.Thread(
            target=self.localization_worker.run, daemon=True
        )
        self.localization_thread.start()

    def localization_finished(self):
        self._reestimate_pulse_timer.stop()
        self.btn_reestimate.setStyleSheet("")
        self.localization_thread = None
        self.localization_worker = None

    def start_new_map(self):
        self.log("Chuyển sang chế độ tạo bản đồ mới")
        subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/new_map_layout.py'])
        self.close()

    def start_docking(self):
        self.log("Chuyển sang chế độ docking")
        subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/docking_layout.py'])
        self.close()

    def load_map(self):
        dialog = LoadMapDialog(self)
        if dialog.exec():
            map_name = dialog.get_selected_map()
            if map_name:
                self.log(f"Đang tải bản đồ: {map_name}")
                update_map_files(map_name, self.log)

    def log(self, message):
        print(f"[LOG] {message}")

    def open_developer_mode(self):
        self.log("Đang mở chế độ Developer")
        self.process_mgr.launch_terminal(
            f'cd {SOURCE_PATH} && source install/setup.bash && claude; exec bash',
            'Developer Mode'
        )

    def closeEvent(self, event):
        if self.localization_worker:
            self.localization_worker.stop()
        self._ros_spin_timer.stop()
        self._ros_node.destroy_node()
        # Only clear history if truly closing, not switching layouts
        if not self._switching_layout:
            self.chat_panel.cleanup()
        self.process_mgr.cleanup_all()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    skip_micro_ros = '--skip-micro-ros' in sys.argv
    window = RobotUI(skip_micro_ros)
    window.showMaximized()
    sys.exit(app.exec())