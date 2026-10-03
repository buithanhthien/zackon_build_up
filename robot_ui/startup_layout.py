#!/usr/bin/env python3
import json
import math
import os
import re
import html
import sys
import subprocess
import threading
import time
import shlex
import uuid


from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QTextEdit, QLineEdit, QLabel,
                             QSizePolicy, QMessageBox, QDialog)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import (
    QFont,
    QColor,
    QTextCursor,
    QTextBlockFormat,
    QTextCharFormat,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from load_map_dialog import LoadMapDialog
from language_dialog import LanguageDialog
from language_config import (get_language, get_ui_text)
from chat_panel_widget import ChatPanel
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH, shell_source_workspace
from styles import MAIN_STYLESHEET
from ui_utils import setup_clock_timer
from map_utils import (update_map_files, get_current_map_name,
                       get_current_map_path, load_map_yaml)
from waypoint_store import load_waypoint_file, resolve_waypoint, save_waypoint_file
from waypoint_dialogs import (DestinationDialog, WaypointPickerDialog, NewWaypointDialog,
                              PathManagerDialog, NewPathDialog, DIALOG_STYLE)
from waypoint_map_widget import MapWidget
from process_manager import ProcessManager
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus



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
        self.process_mgr                 = ProcessManager()
        # Tránh khởi động Nav2 nhiều lần
        self._nav2_started               = False
        self.prev_stm32_status           = None
        self.prev_lidar_status           = None
        self.prev_lidar_rear_status      = None
        self.localization_worker         = None
        self.localization_thread         = None

        # ============================================================
        # Vị trí cuối cùng của robot
        # ============================================================
        self._latest_pose                = None
        self._latest_amcl_msg            = None
        self._last_pose_file = os.path.join(SOURCE_PATH, "robot_ui", "last_robot_pose.json")

        # Chỉ ghi file khoảng 1 lần / giây
        self._last_pose_save_time = 0.0

        # Quan trọng:
        # Không cho AMCL mới khởi động ghi đè pose cũ
        # trước khi restore hoàn tất.
        self._allow_pose_save = False
        self._stm32_last_msg_time        = None
        self._front_lidar_last_msg_time  = None
        self._rear_lidar_last_msg_time   = None
        self._switching_layout           = False  # Track if switching to another layout

        try:
            rclpy.init()
        except Exception:
            pass
        self._ros_node = Node('robot_ui_node')

        # ============================================================
        # Publisher dùng để khôi phục vị trí cho AMCL
        # ============================================================

        self._initialpose_pub = (
            self._ros_node.create_publisher(
                PoseWithCovarianceStamped,
                "/initialpose",
                10
            )
        )

        # ============================================================
        # Voice navigation -> Nav2
        # ============================================================

        self._nav_client = ActionClient(
            self._ros_node,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self._nav_goal_handle = None
        self._navigation_generation = 0
        self._voice_nav_queue = []

        self._waypoints_file = os.path.join(SOURCE_PATH, "robot_ui", "waypoints.json")

        self._waypoints = self._load_waypoints()
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

        # ------------------------------------------------------------
        # Voice navigation wiring
        # ------------------------------------------------------------

        self.chat_panel.set_waypoints_provider(
            self._get_voice_waypoints
        )

        self.chat_panel.set_pose_provider(
            lambda: self._latest_pose
        )

        self.chat_panel.waypoint_command.connect(
            self.voice_navigate_to_waypoint
        )

        self.chat_panel.navigation_stop.connect(
            self.cancel_voice_navigation
        )

        self.chat_panel.interrupt_btn.clicked.connect(
            self.cancel_voice_navigation
        )

        self.chat_panel._voice_enabled = True
        # Tắt khởi động Agent từ UI vì đã có Systemd Service lo
        # if not skip_micro_ros:
        #     self.start_micro_ros()

        #QTimer.singleShot(
        #    5000,
        #    self.start_nav2
        #)

    def _ros_spin_once(self):
        try:
            if rclpy.ok():
                rclpy.spin_once(self._ros_node, timeout_sec=0)
        except Exception:
            self._ros_spin_timer.stop()

    def _amcl_callback(self, msg):

        self._latest_pose = msg.pose.pose
        self._latest_amcl_msg = msg
        if getattr(self, "_map_view", None) is not None:
            self._map_view.set_robot_pose(msg)

        # ========================================================
        # Chưa restore xong thì KHÔNG được ghi đè pose cũ
        # ========================================================

        if not self._allow_pose_save:
            return

        # ========================================================
        # Chỉ lưu khoảng 1 lần mỗi giây
        # ========================================================

        now = time.monotonic()

        if (
            now
            - self._last_pose_save_time
            < 1.0
        ):
            return

        self._last_pose_save_time = now

        self._save_last_robot_pose(
            msg
        )

    def _save_last_robot_pose(
        self,
        msg
    ):

        try:

            pose = msg.pose.pose

            data = {
                "frame_id": "map",

                "position": {
                    "x": float(
                        pose.position.x
                    ),
                    "y": float(
                        pose.position.y
                    ),
                    "z": float(
                        pose.position.z
                    ),
                },

                "orientation": {
                    "x": float(
                        pose.orientation.x
                    ),
                    "y": float(
                        pose.orientation.y
                    ),
                    "z": float(
                        pose.orientation.z
                    ),
                    "w": float(
                        pose.orientation.w
                    ),
                },

                "covariance": [
                    float(value)
                    for value
                    in msg.pose.covariance
                ],

                "saved_at": time.time(),
            }

            # Ghi file tạm trước.
            # Nếu máy tắt đúng lúc ghi,
            # file chính sẽ không bị hỏng.
            temp_file = (
                self._last_pose_file
                + ".tmp"
            )

            with open(
                temp_file,
                "w",
                encoding="utf-8"
            ) as f:

                json.dump(
                    data,
                    f,
                    indent=4
                )

            os.replace(
                temp_file,
                self._last_pose_file
            )

        except Exception as e:

            self.log(
                f"Lỗi lưu vị trí robot: {e}"
            )

    def _load_last_robot_pose(
        self
    ):

        if not os.path.exists(
            self._last_pose_file
        ):

            self.log(
                "Chưa có vị trí robot đã lưu"
            )

            return None

        try:

            with open(
                self._last_pose_file,
                "r",
                encoding="utf-8"
            ) as f:

                data = json.load(f)

            if (
                data.get("frame_id")
                != "map"
            ):

                self.log(
                    "Pose đã lưu không thuộc map frame"
                )

                return None

            position = data.get(
                "position",
                {}
            )

            orientation = data.get(
                "orientation",
                {}
            )

            covariance = data.get(
                "covariance",
                []
            )

            if len(covariance) != 36:

                self.log(
                    "Covariance pose đã lưu "
                    "không hợp lệ"
                )

                return None

            msg = (
                PoseWithCovarianceStamped()
            )

            msg.header.frame_id = "map"

            msg.header.stamp = (
                self._ros_node
                .get_clock()
                .now()
                .to_msg()
            )

            msg.pose.pose.position.x = float(
                position["x"]
            )

            msg.pose.pose.position.y = float(
                position["y"]
            )

            msg.pose.pose.position.z = float(
                position.get(
                    "z",
                    0.0
                )
            )

            msg.pose.pose.orientation.x = float(
                orientation["x"]
            )

            msg.pose.pose.orientation.y = float(
                orientation["y"]
            )

            msg.pose.pose.orientation.z = float(
                orientation["z"]
            )

            msg.pose.pose.orientation.w = float(
                orientation["w"]
            )

            msg.pose.covariance = [
                float(value)
                for value
                in covariance
            ]

            return msg

        except Exception as e:

            self.log(
                "Lỗi đọc vị trí robot đã lưu: "
                f"{e}"
            )

            return None

    def restore_last_robot_pose(self, attempt=0):

        max_attempts = 80

        # ========================================================
        # Chờ AMCL subscribe /initialpose
        # ========================================================

        subscriber_count = (
            self._initialpose_pub
            .get_subscription_count()
        )

        if subscriber_count == 0:

            if attempt < max_attempts:

                QTimer.singleShot(
                    500,
                    lambda: (
                        self.restore_last_robot_pose(
                            attempt + 1
                        )
                    )
                )

            else:

                self.log(
                    "Không restore được vị trí: "
                    "AMCL chưa sẵn sàng"
                )

                # Không có AMCL thì tuyệt đối
                # chưa ghi pose mới.
                self._allow_pose_save = False

            return

        # ========================================================
        # Đọc pose cũ
        # ========================================================

        msg = (
            self._load_last_robot_pose()
        )

        # ========================================================
        # Trường hợp lần đầu tiên chạy
        # chưa có file pose
        # ========================================================

        if msg is None:

            self.log(
                "Không có pose cũ, "
                "sử dụng vị trí hiện tại của AMCL"
            )

            # Cho AMCL chạy ổn một chút rồi mới lưu.
            QTimer.singleShot(
                3000,
                self._enable_pose_saving
            )

            return

        # ========================================================
        # [QUAN TRỌNG] Thêm độ trễ 1.5 giây để AMCL hoàn tất setup nội bộ
        # rồi mới tiến hành bắn bản tin initialpose đầu tiên.
        # ========================================================
        QTimer.singleShot(
            1500,
            lambda: self._execute_publish_pose(msg)
        )

    def _execute_publish_pose(self, msg):
        # ========================================================
        # Publish pose cũ vào /initialpose
        # ========================================================

        self._initialpose_pub.publish(
            msg
        )

        x = (
            msg.pose.pose.position.x
        )

        y = (
            msg.pose.pose.position.y
        )

        self.log(
            "Đã gửi vị trí cũ cho AMCL | "
            f"x={x:.3f}, "
            f"y={y:.3f}"
        )

        # Gửi lại thêm lần nữa để chắc chắn
        QTimer.singleShot(
            300,
            lambda: (
                self._initialpose_pub.publish(
                    msg
                )
            )
        )

        QTimer.singleShot(
            600,
            lambda: (
                self._initialpose_pub.publish(
                    msg
                )
            )
        )

        # Chờ AMCL hội tụ rồi mới cho phép
        # ghi file pose mới.
        QTimer.singleShot(
            2500,
            self._enable_pose_saving
        )



    def _enable_pose_saving(self):

        self._allow_pose_save = True

        self.log(
            "Đã bật lưu vị trí robot"
        )

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

        UI_TEXT = {
            "vi": {
                "waypoints": "Điểm đến",
                "docking": "Về trạm sạc",
                "load_map": "Tải bản đồ",
                "new_map": "Bản đồ mới",
                "tracking": "Theo dõi",
                "reestimate": "Định vị lại",
                "nav2": "Nav2",
                "language": "Ngôn Ngữ",
                "developer": "⚙ Developer",

                "select_language": "CHỌN NGÔN NGỮ",
                "select": "Chọn",
                "cancel": "Hủy",
            },

            "en": {
                "waypoints": "Destinations",
                "docking": "Docking",
                "load_map": "Load Map",
                "new_map": "New Map",
                "tracking": "Tracking",
                "reestimate": "Relocalize",
                "nav2": "Nav2",
                "language": "Language",
                "developer": "⚙ Developer",

                "select_language": "SELECT LANGUAGE",
                "select": "Select",
                "cancel": "Cancel",
            },
        }

        def update_language_ui(self):

            lang = get_language()
            text = UI_TEXT[lang]

            self.btn_waypoints.setText(
                text["waypoints"]
            )

            self.btn_docking.setText(
                text["docking"]
            )

            self.btn_load_map.setText(
                text["load_map"]
            )

            self.btn_new_map.setText(
                text["new_map"]
            )

            self.btn_tracking.setText(
                text["tracking"]
            )

            self.btn_reestimate.setText(
                text["reestimate"]
            )

            self.btn_nav2.setText(
                text["nav2"]
            )

            self.btn_language.setText(
                text["language"]
            )

            self.btn_dev.setText(
                text["developer"]
            )

        self.btn_waypoints  = QPushButton("Điểm đến")
        self.btn_docking    = QPushButton("Về trạm sạc")
        self.btn_load_map   = QPushButton("Tải bản đồ")
        self.btn_new_map    = QPushButton("Bản đồ mới")
        self.btn_tracking   = QPushButton("Theo dõi")
        self.btn_reestimate = QPushButton("Định vị lại")
        self.btn_nav2       = QPushButton("Nav2")
        self.btn_language   = QPushButton("Ngôn Ngữ")

        mono = QFont("JetBrains Mono", 22)
        for btn in [
            self.btn_waypoints, 
            self.btn_docking, 
            self.btn_load_map,
            self.btn_new_map, 
            self.btn_tracking, 
            self.btn_reestimate, 
            self.btn_nav2, 
            self.btn_language
        ]:
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
        self.btn_waypoints.clicked.connect(self.open_destination_dialog)
        self.btn_reestimate.clicked.connect(self.start_reestimate)
        self.btn_new_map.clicked.connect(self.start_new_map)
        self.btn_load_map.clicked.connect(self.load_map)
        self.btn_docking.clicked.connect(self.start_docking)
        self.btn_nav2.clicked.connect(lambda: self.mode_changed("Nav2"))
        self.btn_language.clicked.connect(self.open_language_dialog)
        self.btn_dev.clicked.connect(self.open_developer_mode)
        # Áp dụng ngôn ngữ đã lưu khi mở chương trình
        self.update_language_ui()

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
        self.btn_map = QPushButton("🗺")
        self.btn_map.setObjectName("map-toggle")
        self.btn_map.setFixedSize(36, 36)
        self.btn_map.setToolTip("Mở bản đồ")
        self.btn_map.setAccessibleName("Mở bản đồ")
        self.btn_map.clicked.connect(self.toggle_map_window)
        header_layout.addWidget(self.btn_map)
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
        # ================================================================
        # ChatPanel
        # ================================================================

        self.chat_panel = ChatPanel()

        # ChatPanel không tự hiển thị UI.
        # Chúng ta chỉ lấy VoiceEngine, button và signal từ nó.
        self.chat_panel.hide()


        # ================================================================
        # KHU VỰC BÊN TRÁI: VOICE CONTROL
        # ================================================================

        voice_panel = QWidget()

        voice_panel.setObjectName("voice-panel")
        voice_layout = QVBoxLayout(voice_panel)

        voice_layout.setContentsMargins(30, 20, 30, 20)

        voice_layout.setSpacing(14)

        voice_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)


        # ------------------------------------------------
        # Nút CLICK TO SPEAK
        # ------------------------------------------------

        mic_btn = (self.chat_panel.voice_btn)

        mic_btn.setFixedSize(390, 390)

        mic_btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        mic_btn.setStyleSheet("""
            QPushButton {
                background-color: #214196;
                color: #ffffff;

                border: 3px solid #a8bce8;

                border-radius: 195px;

                font-size: 46px;
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


        # ------------------------------------------------
        # Trạng thái LISTENING / THINKING / SPEAKING
        # ------------------------------------------------

        voice_status = (self.chat_panel.voice_status_label)

        voice_status.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        voice_status.show()


        # ------------------------------------------------
        # Nút Dừng
        # ------------------------------------------------

        interrupt_btn = (self.chat_panel.interrupt_btn)

        interrupt_btn.setFixedSize(180,90)


        voice_layout.addStretch()

        voice_layout.addWidget(mic_btn, 0, Qt.AlignmentFlag.AlignHCenter)

        voice_layout.addWidget(voice_status, 0, Qt.AlignmentFlag.AlignHCenter)

        voice_layout.addSpacing(10)

        voice_layout.addWidget(interrupt_btn, 0, Qt.AlignmentFlag.AlignHCenter)

        voice_layout.addStretch()


        # ================================================================
        # KHU VỰC BÊN PHẢI: HỘI THOẠI USER <-> AI
        # ================================================================

        chat_widget = QWidget()

        chat_widget.setObjectName(
            "conversation-panel"
        )

        chat_layout = QVBoxLayout(
            chat_widget
        )

        chat_layout.setContentsMargins(
            18,
            18,
            18,
            18
        )

        chat_layout.setSpacing(
            12
        )


        # ------------------------------------------------
        # Tiêu đề
        # ------------------------------------------------

        chat_title = QLabel(
            "HỘI THOẠI"
        )

        chat_title.setFont(
            QFont(
                "JetBrains Mono",
                17,
                QFont.Weight.Bold
            )
        )

        chat_title.setStyleSheet("""
            color: #17306f;
            padding-bottom: 4px;
        """)

        chat_layout.addWidget(
            chat_title
        )


        # ------------------------------------------------
        # Khung lịch sử hội thoại
        # ------------------------------------------------

        self.chat_history_box = QTextEdit()

        self.chat_history_box.setReadOnly(
            True
        )

        self.chat_history_box.setObjectName(
            "chat-history"
        )

        self.chat_history_box.setStyleSheet("""
            QTextEdit {
                background-color: #ffffff;

                color: #172554;

                border: 2px solid #c7d5f3;

                border-radius: 18px;

                padding: 16px;

                font-family: "DM Sans";

                font-size: 18px;
            }
        """)

        self.chat_history_box.setPlaceholderText("Cuộc hội thoại với Bé Son sẽ xuất hiện ở đây...")
        chat_layout.addWidget(self.chat_history_box, 1)

        self.virtual_keyboard = QWidget()
        self.virtual_keyboard.setObjectName("virtual-keyboard")
        self.virtual_keyboard.setStyleSheet("""
            QWidget#virtual-keyboard {
                background-color: #e8edf7;
                border: 1px solid #c7d5f3;
                border-radius: 10px;
            }
            QWidget#virtual-keyboard QPushButton {
                min-height: 42px;
                background-color: #ffffff;
                color: #172554;
                border: 1px solid #c7d5f3;
                border-radius: 6px;
                font-size: 16px;
                font-weight: 600;
            }
            QWidget#virtual-keyboard QPushButton:hover {
                background-color: #dbeafe;
            }
        """)
        self._virtual_keyboard_layout = QVBoxLayout(self.virtual_keyboard)
        self._virtual_keyboard_layout.setContentsMargins(8, 8, 8, 8)
        self._virtual_keyboard_layout.setSpacing(4)
        self._keyboard_symbols = False
        self._keyboard_shift = False
        self._render_virtual_keyboard()
        self.virtual_keyboard.hide()
        chat_layout.addWidget(self.virtual_keyboard)

        chat_input_row = QWidget()
        chat_input_layout = QHBoxLayout(chat_input_row)
        chat_input_layout.setContentsMargins(0, 0, 0, 0)
        chat_input_layout.setSpacing(8)

        self.chat_input = QLineEdit()
        self.chat_input.setObjectName("chat-input")
        self.chat_input.setPlaceholderText("Nhập tin nhắn...")
        self.chat_input.setFixedHeight(50)
        self.chat_input.returnPressed.connect(self._send_chat_message)
        chat_input_layout.addWidget(self.chat_input, 1)

        self.keyboard_toggle = QPushButton("⌨")
        self.keyboard_toggle.setCheckable(True)
        self.keyboard_toggle.setFixedSize(50, 50)
        self.keyboard_toggle.setToolTip("Hiện/ẩn bàn phím ảo")
        self.keyboard_toggle.setAccessibleName("Hiện hoặc ẩn bàn phím ảo")
        self.keyboard_toggle.toggled.connect(self.virtual_keyboard.setVisible)
        chat_input_layout.addWidget(self.keyboard_toggle)

        send_chat_button = QPushButton("Gửi")
        send_chat_button.setFixedSize(72, 50)
        send_chat_button.clicked.connect(self._send_chat_message)
        chat_input_layout.addWidget(send_chat_button)
        chat_layout.addWidget(chat_input_row)


        # ================================================================
        # Kết nối nội dung ChatPanel -> hộp hội thoại
        # ================================================================

        self.chat_panel.log_signal.connect(self._append_chat_message)


        # ================================================================
        # Main content row
        # ================================================================

        content_row = QWidget()
        content_layout = QHBoxLayout(content_row)
        content_layout.setContentsMargins(24, 20, 24, 20)
        content_layout.setSpacing(26)


        # Voice bên trái
        content_layout.addWidget(voice_panel, 45)

        # Hội thoại bên phải
        content_layout.addWidget(chat_widget, 55)


        right_layout.addWidget(
            content_row,
            1
        )

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

        QTimer.singleShot(0, self.update_status)    

    def _render_virtual_keyboard(self):
        while self._virtual_keyboard_layout.count():
            item = self._virtual_keyboard_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        if self._keyboard_symbols:
            rows = [
                [(digit, digit, 1) for digit in "1234567890"],
                [(symbol, symbol, 1) for symbol in "@-'?!, ." if symbol != " "],
                [("letters", "ABC", 1), ("space", "space", 4),
                 ("backspace", "⌫", 1), ("enter", "Gửi", 1)],
            ]
        else:
            rows = [
                [(letter, letter.upper() if self._keyboard_shift else letter, 1)
                 for letter in "qwertyuiop"],
                [(letter, letter.upper() if self._keyboard_shift else letter, 1)
                 for letter in "asdfghjkl"],
                [("shift", "Shift", 2)] +
                [(letter, letter.upper() if self._keyboard_shift else letter, 1)
                 for letter in "zxcvbnm"] +
                [("backspace", "⌫", 2)],
                [("symbols", "?123", 2), (",", ",", 1),
                 ("space", "space", 5), (".", ".", 1),
                 ("enter", "Gửi", 2)],
            ]

        for row in rows:
            row_layout = QHBoxLayout()
            row_layout.setSpacing(4)
            for key, label, stretch in row:
                button = QPushButton(label)
                button.setMinimumWidth(32)
                button.setSizePolicy(
                    QSizePolicy.Policy.Expanding,
                    QSizePolicy.Policy.Fixed,
                )
                button.clicked.connect(
                    lambda checked=False, key=key: self._handle_virtual_key(key)
                )
                row_layout.addWidget(button, stretch)
            self._virtual_keyboard_layout.addLayout(row_layout)

    def _handle_virtual_key(self, key):
        if key == "symbols":
            self._keyboard_symbols = True
            self._render_virtual_keyboard()
        elif key == "letters":
            self._keyboard_symbols = False
            self._render_virtual_keyboard()
        elif key == "shift":
            self._keyboard_shift = not self._keyboard_shift
            self._render_virtual_keyboard()
        elif key == "backspace":
            self.chat_input.backspace()
        elif key == "space":
            self.chat_input.insert(" ")
        elif key == "enter":
            self._send_chat_message()
        else:
            value = key.upper() if self._keyboard_shift else key
            self.chat_input.insert(value)
            self._keyboard_shift = False
            if not self._keyboard_symbols:
                self._render_virtual_keyboard()

    def _send_chat_message(self):
        message = self.chat_input.text().strip()
        if not message:
            return

        worker_thread = self.chat_panel._ai_thread
        if worker_thread and worker_thread.isRunning():
            return

        self.chat_panel._ask_ai(message)
        self.chat_input.clear()

    def _append_chat_message(self, message):

        if not message:
            return

        message = message.strip()

        # ============================================================
        # Xác định người nói
        # ============================================================

        if message.startswith("[Bạn]"):

            speaker = "BẠN"

            text = message.replace(
                "[Bạn]",
                "",
                1
            ).strip()

            alignment = (
                Qt.AlignmentFlag.AlignRight
            )

            speaker_color = QColor(
                "#64748b"
            )

            text_color = QColor(
                "#172554"
            )

            bubble_color = QColor(
                "#dbeafe"
            )

        elif message.startswith("[Bé Son]"):
            speaker = "BÉ SON"
            text = message.replace("[Bé Son]", "", 1).strip()
            alignment = (Qt.AlignmentFlag.AlignLeft)
            speaker_color = QColor("#214196")
            text_color = QColor("#172554")
            bubble_color = QColor("#eef2ff")

        else:
            return

        # ============================================================
        # Cursor cuối hộp chat
        # ============================================================

        cursor = (self.chat_history_box.textCursor())

        cursor.movePosition(QTextCursor.MoveOperation.End)

        # ============================================================
        # Tạo block mới
        # ============================================================

        block_format = QTextBlockFormat()

        block_format.setAlignment(
            alignment
        )

        block_format.setTopMargin(
            12
        )

        block_format.setBottomMargin(
            12
        )

        block_format.setLeftMargin(
            18
        )

        block_format.setRightMargin(
            18
        )

        cursor.insertBlock(
            block_format
        )

        # ============================================================
        # Tên người nói
        # ============================================================

        speaker_format = QTextCharFormat()

        speaker_format.setForeground(
            speaker_color
        )

        speaker_format.setFontWeight(
            QFont.Weight.Bold
        )

        speaker_format.setFontPointSize(
            10
        )

        cursor.insertText(
            speaker,
            speaker_format
        )

        # Xuống dòng nhưng vẫn giữ cùng căn lề
        cursor.insertText(
            "\n"
        )

        # ============================================================
        # Nội dung
        # ============================================================

        text_format = QTextCharFormat()

        text_format.setForeground(
            text_color
        )

        text_format.setBackground(
            bubble_color
        )

        text_format.setFontPointSize(
            14
        )

        cursor.insertText(
            text,
            text_format
        )

        # ============================================================
        # Cuộn xuống tin mới nhất
        # ============================================================

        self.chat_history_box.setTextCursor(
            cursor
        )

        scroll_bar = (
            self.chat_history_box
            .verticalScrollBar()
        )

        scroll_bar.setValue(
            scroll_bar.maximum()
        )
    
    # ══════════════════════════════════════════════════════════════════════════
    #  Existing UI helpers (unchanged)
    # ══════════════════════════════════════════════════════════════════════════

    def _make_status_card(self, device_name):
        card = QWidget()
        card.setObjectName("status-card")
        card.setMinimumHeight(100)
        card.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed
        )

        layout = QHBoxLayout(card)
        layout.setContentsMargins(16, 12, 20, 12)

        dot = QLabel("●")
        dot.setStyleSheet(
            "color: #8fa3cc; font-size: 12px;"
        )
        dot.setFixedWidth(20)

        info = QVBoxLayout()

        name_lbl = QLabel(device_name.upper())
        name_lbl.setObjectName("device-name")
        name_lbl.setFont(
            QFont("DM Sans", 11)
        )

        state_lbl = QLabel("Checking...")
        state_lbl.setObjectName(
            "status-checking"
        )
        state_lbl.setFont(
            QFont(
                "DM Sans",
                14,
                QFont.Weight.Medium
            )
        )

        info.addWidget(name_lbl)
        info.addWidget(state_lbl)

        layout.addWidget(dot)
        layout.addLayout(info)
        layout.addStretch()

        # Mặc định LiDAR không có nút
        detail_btn = None

        # Chỉ STM32 có nút chẩn đoán
        if device_name.upper() == "STM32":
            detail_btn = QPushButton("i")

            detail_btn.setFixedSize(
                34,
                34
            )

            detail_btn.setToolTip(
                "Kiểm tra kết nối STM32"
            )

            detail_btn.setCursor(
                Qt.CursorShape.PointingHandCursor
            )

            detail_btn.setStyleSheet("""
                QPushButton {
                    background-color: #eef2ff;
                    color: #214196;
                    border: 1px solid #c8d4f0;
                    border-radius: 17px;
                    font-size: 16px;
                    font-weight: bold;
                }

                QPushButton:hover {
                    background-color: #dbeafe;
                    border: 1px solid #214196;
                }
            """)

            detail_btn.clicked.connect(
                self.show_stm32_diagnostics
            )

            layout.addWidget(
                detail_btn,
                0,
                Qt.AlignmentFlag.AlignVCenter
            )

        # QUAN TRỌNG:
        # return này nằm ngoài if STM32
        return {
            "widget": card,
            "dot": dot,
            "state": state_lbl,
            "detail_btn": detail_btn
        }

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

        detail_btn = card.get(
            "detail_btn"
        )

        if detail_btn is not None:

            if available:
                detail_btn.setText("i")

                detail_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #eef2ff;
                        color: #214196;
                        border: 1px solid #c8d4f0;
                        border-radius: 17px;
                        font-size: 16px;
                        font-weight: bold;
                    }

                    QPushButton:hover {
                        background-color: #dbeafe;
                        border: 1px solid #214196;
                    }
                """)

            else:
                detail_btn.setText("!")

                detail_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #fee2e2;
                        color: #ef4444;
                        border: 1px solid #ef4444;
                        border-radius: 17px;
                        font-size: 17px;
                        font-weight: bold;
                    }

                    QPushButton:hover {
                        background-color: #fecaca;
                    }
                """)

    def _update_clock(self):
        from datetime import datetime
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S"))

    def _pulse_reestimate(self):
        self._pulse_state = not self._pulse_state
        color = "#fcb525" if self._pulse_state else "#8fa3cc"
        self.btn_reestimate.setStyleSheet(
            f"QPushButton#mode-btn {{ border-left: 4px solid {color}; color: #fcb525; background-color: #1a3278; }}"
        )

    def show_stm32_diagnostics(self):

        STM32_IP = os.environ.get("STM32_IP", "192.168.4.50")
        STM32_IFACE = os.environ.get("STM32_IFACE", "enx00e04c534458")

        now = time.monotonic()

        # ========================================================
        # 1. Heartbeat /odomfromSTM32
        # ========================================================

        if self._stm32_last_msg_time is None:
            odom_age = None
            odom_ok = False
        else:
            odom_age = (
                now -
                self._stm32_last_msg_time
            )

            odom_ok = odom_age < 3.0

        # ========================================================
        # 2. micro-ROS Agent UDP port 8888
        # ========================================================

        agent_port_ok = False

        try:
            result = subprocess.run(
                ["ss", "-lun"],
                capture_output=True,
                text=True,
                timeout=1.0
            )

            agent_port_ok = (
                ":8888" in result.stdout
            )

        except Exception:
            agent_port_ok = False

        # Report systemd's startup state as well as the UDP socket.
        service_state = "Không đọc được trạng thái"
        try:
            result = subprocess.run(
                ["systemctl", "show", "microros_agent.service",
                 "--property=ActiveState,SubState,Result"],
                capture_output=True, text=True, timeout=1.0
            )
            if result.returncode == 0:
                values = dict(line.split("=", 1) for line in
                              result.stdout.splitlines() if "=" in line)
                service_state = "/".join(values.get(key, "?") for key in
                                        ("ActiveState", "SubState", "Result"))
        except (OSError, subprocess.TimeoutExpired):
            pass

        # ========================================================
        # 3. micro_ros_agent process
        # ========================================================

        agent_process_ok = False

        try:
            result = subprocess.run(
                [
                    "pgrep",
                    "-af",
                    "micro_ros_agent"
                ],
                capture_output=True,
                text=True,
                timeout=1.0
            )

            agent_process_ok = (
                result.returncode == 0
                and "micro_ros_agent" in result.stdout
            )

        except Exception:
            agent_process_ok = False

        # ========================================================
        # 4. Ethernet physical carrier
        # ========================================================

        ethernet_carrier_ok = False

        try:
            carrier_path = (
                f"/sys/class/net/"
                f"{STM32_IFACE}/carrier"
            )

            with open(
                carrier_path,
                "r",
                encoding="utf-8"
            ) as f:
                carrier_value = f.read().strip()

            ethernet_carrier_ok = (
                carrier_value == "1"
            )

        except Exception:
            ethernet_carrier_ok = False

        # ========================================================
        # 5. Ping STM32 qua ĐÚNG Ethernet interface
        # ========================================================

        stm32_ping_ok = False

        try:
            result = subprocess.run(
                [
                    "ping",
                    "-I",
                    STM32_IFACE,
                    "-c",
                    "1",
                    "-W",
                    "1",
                    STM32_IP
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0
            )

            stm32_ping_ok = (
                result.returncode == 0
            )

        except Exception:
            stm32_ping_ok = False

        # ========================================================
        # 6. ROS publisher /odomfromSTM32
        # ========================================================

        try:
            publisher_count = (
                self._ros_node.count_publishers(
                    "/odomfromSTM32"
                )
            )

        except Exception:
            publisher_count = 0

        # ========================================================
        # 7. stm32_odom_node
        # ========================================================

        stm32_node_ok = False

        try:
            node_names = (
                self._ros_node.get_node_names()
            )

            stm32_node_ok = (
                "stm32_odom_node"
                in node_names
            )

        except Exception:
            stm32_node_ok = False

        # ========================================================
        # 8. Chẩn đoán
        # ========================================================

        if odom_ok:

            diagnosis = (
                "STM32 đang hoạt động bình thường."
            )

        elif not agent_process_ok:

            diagnosis = (
                "micro-ROS Agent không chạy.\n\n"
                f"Service: {service_state}.\n"
                "Nếu ở activating/start-pre: kiểm tra điều kiện "
                "ExecStartPre và IP Ethernet của mini PC."
            )

        elif not agent_port_ok:

            diagnosis = (
                "micro-ROS Agent có process nhưng "
                "UDP port 8888 không hoạt động.\n\n"
                "Kiểm tra lại Agent hoặc port."
            )

        elif not ethernet_carrier_ok:

            diagnosis = (
                "Không có Ethernet physical link "
                "tới STM32.\n\n"
                "Có thể do:\n"
                "• Cáp Ethernet bị rút\n"
                "• Jack/cáp bị lỗi\n"
                "• USB Ethernet adapter mất link\n"
                "• PHY Ethernet phía STM32 chưa lên"
            )

        elif not stm32_ping_ok:

            diagnosis = (
                "Ethernet physical link đang UP "
                "nhưng STM32 không trả lời ping.\n\n"
                "Có thể do:\n"
                "• STM32 mất nguồn hoặc bị treo\n"
                "• Ethernet MAC/DMA/LwIP trên STM32 "
                "chưa hoạt động\n"
                "• STM32 đang trong quá trình "
                "recover Ethernet"
            )

        elif publisher_count == 0:

            diagnosis = (
                "STM32 ping được và Agent đang chạy, "
                "nhưng ROS 2 không thấy publisher "
                "/odomfromSTM32.\n\n"
                "Khả năng cao micro-ROS session "
                "STM32 ↔ Agent chưa được tạo "
                "hoặc đang reconnect."
            )

        else:

            diagnosis = (
                "STM32 ping được và ROS 2 vẫn thấy "
                "publisher /odomfromSTM32, "
                "nhưng không nhận được dữ liệu.\n\n"
                "Khả năng task publish ODOM hoặc "
                "micro-ROS publisher trên STM32 "
                "đang bị dừng/kẹt."
            )

        # ========================================================
        # 9. Text hiển thị
        # ========================================================

        if odom_age is None:
            odom_text = "Chưa từng nhận dữ liệu"
        else:
            odom_text = (
                f"{odom_age:.2f} giây trước"
            )

        detail = (
            f"/odomfromSTM32: "
            f"{'OK' if odom_ok else 'MẤT'}\n"

            f"Lần cuối nhận: "
            f"{odom_text}\n\n"

            f"Service Agent: {service_state}\n"

            f"micro-ROS process: "
            f"{'OK' if agent_process_ok else 'DOWN'}\n"

            f"UDP :8888: "
            f"{'OK' if agent_port_ok else 'DOWN'}\n"

            f"Ethernet {STM32_IFACE}: "
            f"{'LINK UP' if ethernet_carrier_ok else 'LINK DOWN'}\n"

            f"STM32 {STM32_IP}: "
            f"{'REACHABLE' if stm32_ping_ok else 'UNREACHABLE'}\n"

            f"stm32_odom_node: "
            f"{'CÓ' if stm32_node_ok else 'KHÔNG'}\n"

            f"Publisher /odomfromSTM32: "
            f"{publisher_count}\n\n"

            f"CHẨN ĐOÁN:\n"
            f"{diagnosis}"
        )

        QMessageBox.information(
            self,
            "Chẩn đoán kết nối STM32",
            detail
        )

    def start_micro_ros(self):

        # Kiểm tra xem UDP port 8888 đã có process sử dụng chưa
        try:
            result = subprocess.run(
                [
                    "ss",
                    "-lun",
                ],
                capture_output=True,
                text=True,
                timeout=2.0
            )

            if ":8888" in result.stdout:
                self.log(
                    "micro-ROS agent đã chạy trên UDP port 8888"
                )
                return

        except Exception as e:
            self.log(
                f"[WARN] Không kiểm tra được port 8888: {e}"
            )

        # Port chưa được dùng -> khởi động agent
        self.process_mgr.launch_terminal(
            shell_source_workspace(
                'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; exec bash'
            ),
            'micro-ROS agent'
        )

        self.log(
            "Đã khởi động micro-ROS agent trên UDP port 8888"
        )

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

    # ================================================================
    # VOICE NAVIGATION
    # ================================================================

    def _load_waypoints(self):
        try:
            data, _ = load_waypoint_file(self._waypoints_file)

            self.log(
                f"Đã tải {len(data)} waypoint cho Bé Son"
            )

            return data

        except Exception as e:
            self.log(
                f"Lỗi đọc waypoints.json: {e}"
            )
            return {}

    def update_language_ui(self):

        language = get_language()

        text = get_ui_text(
            language
        )

        print(
            f"[LANGUAGE UI] Cập nhật giao diện: "
            f"{language}"
        )

        self.btn_waypoints.setText(
            text["waypoints"]
        )

        self.btn_docking.setText(
            text["docking"]
        )

        self.btn_load_map.setText(
            text["load_map"]
        )

        self.btn_new_map.setText(
            text["new_map"]
        )

        self.btn_tracking.setText(
            text["tracking"]
        )

        self.btn_reestimate.setText(
            text["reestimate"]
        )

        self.btn_nav2.setText(
            text["nav2"]
        )

        self.btn_language.setText(
            text["language"]
        )

        self.btn_dev.setText(
            text["developer"]
        )

    def mode_changed(self, mode):
        self.log(f"Đã chuyển sang chế độ {mode}")
        if mode == "Tracking":
            subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/tracking_mode_layout.py'])
            self.close()
        elif mode == "Waypoints":
            self.open_destination_dialog()
        elif mode == "Nav2":
            self.start_nav2()

    def start_nav2(self):
        # Kiểm tra thực tế xem tiến trình Nav2 launch có đang thực sự chạy ngầm không
        nav2_is_running = False
        try:
            result = subprocess.run(
                ["pgrep", "-f", "NAV2_BRINGUP.launch.py"],
                capture_output=True,
                text=True,
                timeout=1.0
            )
            nav2_is_running = (result.returncode == 0 and bool(result.stdout.strip()))
        except Exception:
            nav2_is_running = False

        # Nếu thực tế nó vẫn đang chạy thì không làm gì cả
        if nav2_is_running:
            self.log("Nav2 đã được khởi động và đang chạy")
            return

        # Nếu thực tế nó đã tắt (hoặc chưa từng chạy), reset lại cờ trạng thái
        self._nav2_started = False
        self._allow_pose_save = False
        self._nav2_started = True

        try:
            self.process_mgr.launch_terminal(
                shell_source_workspace(
                    f'ros2 launch {shlex.quote(os.path.join(SOURCE_PATH, "src/view_robot/launch/NAV2_BRINGUP.launch.py"))}; exec bash'
                ),
                'Nav2'
            )

            self.log(
                "Đã khởi động hệ thống điều hướng Nav2"
            )

            # Chờ AMCL xuất hiện rồi restore pose cũ
            QTimer.singleShot(
                1000,
                self.restore_last_robot_pose
            )

        except Exception as e:
            self._nav2_started = False
            self.log(
                f"Lỗi khởi động Nav2: {e}"
            )

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
                self.cancel_voice_navigation()
                if update_map_files(map_name, self.log):
                    self._latest_pose = None
                    self._latest_amcl_msg = None
                    self._waypoints = self._load_waypoints()
                    self._refresh_map_window()

    def open_destination_dialog(self):
        if getattr(self, '_destination_dialog', None) is None:
            self._destination_dialog = DestinationDialog(self)
        self._destination_dialog.show()
        self._destination_dialog.raise_()
        self._destination_dialog.activateWindow()

    def open_waypoint_picker(self):
        self._waypoints = self._load_waypoints()
        dialog = WaypointPickerDialog(self._waypoints, get_current_map_name(), self)
        if dialog.exec() and dialog.get_selected_key():
            self._run_waypoint_sequence([dialog.get_selected_key()])

    def open_path_manager(self):
        path = os.path.join(SOURCE_PATH, 'robot_ui', 'multi_waypoints.json')
        while True:
            current_map = get_current_map_name()
            dialog = PathManagerDialog(path, current_map, self)
            dialog.run_path_requested.connect(self._run_waypoint_sequence)
            if dialog.exec() != 2:
                break
            self._waypoints = self._load_waypoints()
            NewPathDialog(self._waypoints, current_map, path, self).exec()

    def _run_waypoint_sequence(self, sequence):
        self._waypoints = self._load_waypoints()
        current_map = get_current_map_name()
        if (not isinstance(sequence, list) or not sequence
                or any(resolve_waypoint(self._waypoints, key, current_map) is None
                       for key in sequence)):
            QMessageBox.warning(self, 'Không thể điều hướng',
                                'Lộ trình trống hoặc có địa điểm không thuộc bản đồ hiện tại.')
            return
        # Keep keys as a list: waypoint names may contain commas.
        self.cancel_voice_navigation()
        self._voice_nav_queue = list(sequence)
        self._send_next_voice_goal()

    def open_new_waypoint_dialog(self):
        if self._latest_pose is None:
            QMessageBox.warning(self, 'Chưa có vị trí',
                                'Robot chưa có pose AMCL. Hãy định vị robot trước khi tạo địa điểm.')
            return
        current_map = get_current_map_name()
        dialog = NewWaypointDialog(self)
        if not dialog.exec():
            return
        if self._latest_pose is None or current_map != get_current_map_name():
            QMessageBox.warning(self, 'Không thể lưu địa điểm',
                                'Bản đồ hoặc pose đã thay đổi. Hãy định vị và thử lại.')
            return
        name = dialog.get_name()
        try:
            data, snapshot = load_waypoint_file(self._waypoints_file)
            if (not name or name.startswith('__')
                    or any(name.casefold() == key.casefold() for key in data)):
                raise ValueError('Tên trống, trùng tên hoặc bắt đầu bằng __. Hãy chọn tên khác.')
            pose = self._latest_pose
            data[name] = {
                'id': str(uuid.uuid4()), 'display_name': name,
                'deletable': dialog.get_deletable(), 'map_name': current_map,
                'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
                'qx': pose.orientation.x, 'qy': pose.orientation.y,
                'qz': pose.orientation.z, 'qw': pose.orientation.w,
            }
            self._waypoints, _ = save_waypoint_file(self._waypoints_file, data, snapshot)
        except (OSError, ValueError, UnicodeError) as exc:
            QMessageBox.warning(self, 'Không thể lưu địa điểm', str(exc))
            return
        self._refresh_map_window()
        self.log(f'Đã lưu địa điểm mới: {name}')

    def toggle_map_window(self):
        window = getattr(self, '_map_window', None)
        if window is not None and window.isVisible():
            window.close()
            return
        if window is None:
            self._map_window = QDialog(self)
            self._map_window.setWindowTitle('Bản đồ')
            self._map_window.resize(800, 600)
            self._map_window.setStyleSheet(DIALOG_STYLE)
            QVBoxLayout(self._map_window)
        if self._refresh_map_window():
            self._map_window.show()
            self._map_window.raise_()
            self._map_window.activateWindow()

    def _refresh_map_window(self):
        window = getattr(self, '_map_window', None)
        if window is None:
            return True
        try:
            path = get_current_map_path()
            metadata = load_map_yaml(path)
            widget = MapWidget(os.path.join(os.path.dirname(path), metadata['image']), metadata)
            if widget.map_image.isNull():
                widget.deleteLater()
                raise ValueError('Không đọc được ảnh bản đồ.')
        except (OSError, KeyError, TypeError, ValueError, SyntaxError) as exc:
            window.hide()
            QMessageBox.warning(self, 'Không thể mở bản đồ', str(exc))
            return False
        old = getattr(self, '_map_view', None)
        if old is not None:
            window.layout().removeWidget(old)
            old.deleteLater()
        self._map_view = widget
        self._waypoints = self._load_waypoints()
        widget.set_waypoints({k: v for k, v in self._waypoints.items()
                              if v['map_name'] == get_current_map_name()})
        widget.set_robot_pose(self._latest_amcl_msg)
        window.layout().addWidget(widget)
        return True

    def open_language_dialog(self):

        dialog = LanguageDialog(self)

        if dialog.exec():

            language = (
                dialog.get_selected_language()
            )

            if language == "vi":

                language_name = (
                    "Tiếng Việt"
                )

            elif language == "en":

                language_name = (
                    "English"
                )

            else:

                language_name = str(
                    language
                )

            self.update_language_ui()

            self.chat_panel.set_language(language)

            self.log(
                f"Đã chọn ngôn ngữ: "
                f"{language_name}"
            )

    def log(self, message):
        print(f"[LOG] {message}")

    def open_developer_mode(self):
        self.log("Đang mở chế độ Developer")
        self.process_mgr.launch_terminal(
            f'cd {shlex.quote(SOURCE_PATH)} && source install/setup.bash && claude; exec bash',
            'Developer Mode'
        )

    def closeEvent(self, event):
        self._nav2_started = False

        if self.localization_worker:
            self.localization_worker.stop()

        # Cancel Nav2 goal before destroying ROS node
        if self._nav_goal_handle is not None:

            try:
                self._voice_nav_queue.clear()

                cancel_future = (
                    self._nav_goal_handle.cancel_goal_async()
                )

                rclpy.spin_until_future_complete(
                    self._ros_node,
                    cancel_future,
                    timeout_sec=1.0
                )

                self.log(
                    "[Bé Son] Đã yêu cầu hủy navigation khi đóng UI"
                )

            except Exception as e:
                self.log(
                    f"[Bé Son] Lỗi cancel khi đóng UI: {e}"
                )

        self._ros_spin_timer.stop()

        self._ros_node.destroy_node()

        if not self._switching_layout:
            self.chat_panel.cleanup()

        self.process_mgr.cleanup_all()

        event.accept()

    def _get_voice_waypoints(self):
        self._waypoints = self._load_waypoints()
        result = []

        for key, data in self._waypoints.items():
            if data['map_name'] != get_current_map_name():
                continue
            result.append({
                "key": key,
                "aliases": [data.get("display_name", key), *data.get("aliases", [])],
                "x": data.get("x"),
                "y": data.get("y"),
            })

        return result

    def voice_navigate_to_waypoint(self, command):

        self.log(
            f"[Bé Son] Lệnh điều hướng: {command}"
        )

        names = [
            item.strip()
            for item in command.split(",")
            if item.strip()
        ]

        if not names:
            self.log(
                "Không có waypoint hợp lệ"
            )
            return

        # Cho phép nhiều điểm:
        # X5.7,X5.11,phong co Tam
        self.cancel_voice_navigation()
        self._voice_nav_queue = names

        self._send_next_voice_goal()

    def _send_next_voice_goal(self, generation=None):
        if generation is not None and generation != self._navigation_generation:
            return
        generation = self._navigation_generation

        if not self._voice_nav_queue:
            self.log(
                "[Bé Son] Đã hoàn thành hành trình"
            )
            return

        target = self._voice_nav_queue.pop(0)

        # ============================================================
        # Trường hợp quay lại vị trí lúc ra lệnh
        # ============================================================

        if target.startswith("__return_here__:"):

            try:
                values = target.split(":", 1)[1]
                x, y, qz, qw = map(
                    float,
                    values.split(";")
                )

                waypoint = {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": qz,
                    "qw": qw,
                }

                display_name = "vị trí ban đầu"

            except Exception as e:
                self.log(
                    f"Lỗi đọc __return_here__: {e}"
                )
                return

        # ============================================================
        # Waypoint bình thường
        # ============================================================

        else:

            self._waypoints = self._load_waypoints()
            key = resolve_waypoint(self._waypoints, target, get_current_map_name())
            waypoint = self._waypoints.get(key)

            if waypoint is None:
                self.log(
                    f"Không tìm thấy waypoint: {target}"
                )
                return

            display_name = waypoint.get('display_name', target)

        # ============================================================
        # Kiểm tra Nav2
        # ============================================================

        if not self._nav_client.wait_for_server(
            timeout_sec=1.0
        ):
            self.log(
                "Nav2 chưa sẵn sàng: "
                "/navigate_to_pose không tồn tại"
            )
            return

        # ============================================================
        # Tạo goal
        # ============================================================

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"

        goal.pose.header.stamp = (
            self._ros_node
            .get_clock()
            .now()
            .to_msg()
        )

        # Position
        goal.pose.pose.position.x = float(
            waypoint["x"]
        )

        goal.pose.pose.position.y = float(
            waypoint["y"]
        )

        goal.pose.pose.position.z = float(
            waypoint.get("z", 0.0)
        )

        # Orientation
        goal.pose.pose.orientation.x = float(
            waypoint.get("qx", 0.0)
        )

        goal.pose.pose.orientation.y = float(
            waypoint.get("qy", 0.0)
        )

        goal.pose.pose.orientation.z = float(
            waypoint["qz"]
        )

        goal.pose.pose.orientation.w = float(
            waypoint["qw"]
        )

        self.log(
            f"[Bé Son] Đang đi tới {display_name} | "
            f"x={waypoint['x']:.2f}, "
            f"y={waypoint['y']:.2f}"
        )

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback
        )

        future.add_done_callback(
            lambda done: self._nav_goal_response_callback(done, generation)
        )

    def _nav_goal_response_callback(self, future, generation=None):
        if generation is None:
            generation = self._navigation_generation

        try:
            goal_handle = future.result()
            if generation != self._navigation_generation:
                if goal_handle.accepted:
                    goal_handle.cancel_goal_async()
                return

        except Exception as e:
            self.log(
                f"Lỗi gửi Nav2 goal: {e}"
            )
            return

        if not goal_handle.accepted:

            self.log(
                "Nav2 từ chối waypoint"
            )

            self._voice_nav_queue.clear()
            return

        self._nav_goal_handle = goal_handle

        self.log(
            "Nav2 đã nhận waypoint"
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            lambda done: self._nav_result_callback(done, generation)
        )

    def _nav_result_callback(self, future, generation=None):
        if generation is None:
            generation = self._navigation_generation
        if generation != self._navigation_generation:
            return

        try:
            wrapped_result = future.result()
            status = wrapped_result.status

        except Exception as e:
            self.log(
                f"Lỗi nhận kết quả Nav2: {e}"
            )

            self._voice_nav_queue.clear()
            return

        self._nav_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:

            self.log(
                "[Bé Son] Đã tới waypoint"
            )

            # Nếu người dùng yêu cầu nhiều waypoint
            # thì đi điểm tiếp theo.
            QTimer.singleShot(
                300,
                lambda: self._send_next_voice_goal(generation)
            )

        elif status == GoalStatus.STATUS_CANCELED:

            self.log(
                "[Bé Son] Navigation đã bị hủy"
            )

            self._voice_nav_queue.clear()

        else:

            self.log(
                f"[Bé Son] Navigation thất bại, "
                f"status={status}"
            )

            self._voice_nav_queue.clear()

    def _nav_feedback_callback(
        self,
        feedback_msg
    ):

        try:
            distance = (
                feedback_msg.feedback
                .distance_remaining
            )

            # Không log từng frame vì sẽ spam.
            # Nếu cần sau này có thể throttle.
            print(
                f"[NAV] Còn {distance:.2f} m"
            )

        except Exception:
            pass

    def cancel_voice_navigation(self):
        self._navigation_generation += 1

        self._voice_nav_queue.clear()

        if self._nav_goal_handle is None:
            self.log(
                "Không có navigation goal đang chạy"
            )
            return

        self.log(
            "[Bé Son] Đang hủy navigation"
        )

        self._nav_goal_handle.cancel_goal_async()
        self._nav_goal_handle = None

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    skip_micro_ros = '--skip-micro-ros' in sys.argv
    window = RobotUI(skip_micro_ros)
    window.showMaximized()
    sys.exit(app.exec())
