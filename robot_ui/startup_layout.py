#!/usr/bin/env python3
import sys
import subprocess
import os
import threading
import time
import math
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel, QSizePolicy)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QFontDatabase
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty
from load_map_dialog import LoadMapDialog
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH


# ── Tuning constants ──────────────────────────────────────────────────────────
ANGULAR_SPEED        = 0.314          # rad/s  (~18 °/s)
HALF_ROTATION_RAD    = math.pi        # 180 °  → time = π / ANGULAR_SPEED ≈ 10 s
HALF_ROTATION_TIME   = HALF_ROTATION_RAD / ANGULAR_SPEED   # seconds per half-spin

RAMP_STEPS           = 5              # number of steps for velocity ramp
RAMP_INTERVAL        = 0.05           # seconds between ramp steps

GOOD_COV_THRESHOLD   = 0.10           # early-exit: both x+y+yaw below this → well localised
ACCEPTABLE_COV       = 0.20           # after full spin: "good enough" to skip retry
MAX_RETRIES          = 2              # extra full rotations if covariance stays poor
SPIN_TICK            = 0.05           # seconds per spin-loop iteration
COV_LOCK_TIMEOUT     = 15.0           # seconds to wait for first AMCL message before abort
# ─────────────────────────────────────────────────────────────────────────────


class LocalizationWorker(QObject):
    log_signal      = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._stop_event   = threading.Event()
        self._cov_lock     = threading.Lock()
        self._current_cov  = float('inf')   # latest covariance reading (thread-safe)
        self._best_cov     = float('inf')   # best seen so far across the whole sequence
        self._received_pose = threading.Event()   # set on first AMCL message

    # ── Public helpers ────────────────────────────────────────────────────────

    def stop(self):
        self._stop_event.set()

    # ── ROS callback (background thread) ─────────────────────────────────────

    def _pose_callback(self, msg):
        cov = msg.pose.covariance
        total = cov[0] + cov[7] + cov[35]   # σ²_x + σ²_y + σ²_yaw
        with self._cov_lock:
            self._current_cov = total
            if total < self._best_cov:
                self._best_cov = total
        self._received_pose.set()

    # ── Velocity helpers ──────────────────────────────────────────────────────

    def _ramp_velocity(self, publisher, target_z: float):
        """Linearly ramp angular.z from 0 → target_z (or target_z → 0)."""
        twist = Twist()
        for i in range(1, RAMP_STEPS + 1):
            if self._stop_event.is_set():
                break
            twist.angular.z = target_z * (i / RAMP_STEPS)
            publisher.publish(twist)
            time.sleep(RAMP_INTERVAL)

    def _stop_robot(self, publisher):
        self._ramp_velocity(publisher, 0.0)   # ramp down before hard-zero
        twist = Twist()
        twist.angular.z = 0.0
        publisher.publish(twist)

    def _spin_arc(self, publisher, node, arc_time: float, label: str):
        """
        Rotate at ANGULAR_SPEED for *arc_time* seconds.
        Returns early if covariance drops below GOOD_COV_THRESHOLD.
        Returns (elapsed_seconds, best_covariance_seen_during_arc).
        """
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
            if i > 0 and i % int(tick_count / 5 or 1) == 0:   # log ~5 times per arc
                self.log_signal.emit(
                    f"[{label}] {progress}% — current σ²={cov:.4f}, best={arc_best:.4f}"
                )

            # ── Early exit: already well-localised ───────────────────────────
            if cov < GOOD_COV_THRESHOLD:
                self.log_signal.emit(
                    f"[{label}] Early exit at {elapsed:.1f}s — covariance {cov:.4f} "
                    f"< threshold {GOOD_COV_THRESHOLD}"
                )
                break

        return elapsed, arc_best
    
    def run(self):
        # ── ROS initialisation ────────────────────────────────────────────────
        try:
            rclpy.init()
        except Exception:
            pass

        node = Node('localization_worker')

        cmd_vel_pub       = node.create_publisher(Twist, '/cmd_vel', 10)
        _pose_sub         = node.create_subscription(          # noqa: F841
            PoseWithCovarianceStamped, '/amcl_pose',
            self._pose_callback, 10
        )
        global_loc_client = node.create_client(Empty, '/reinitialize_global_localization')
        clear_local_client= node.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')

        try:
            self._run_sequence(node, cmd_vel_pub, global_loc_client, clear_local_client)
        except Exception as exc:
            self.log_signal.emit(f"[ERROR] Localization sequence failed: {exc}")
            self._stop_robot(cmd_vel_pub)
        finally:
            node.destroy_node()
            self.finished_signal.emit()

    def _run_sequence(self, node, cmd_vel_pub, global_loc_client, clear_local_client):
        # ── Step 1: Trigger global (particle-spread) localisation ─────────────
        self.log_signal.emit("Waiting for /reinitialize_global_localization service…")
        if not global_loc_client.wait_for_service(timeout_sec=10.0):
            self.log_signal.emit("[ERROR] Reinitialize service unavailable — aborting")
            return

        self.log_signal.emit("Calling /reinitialize_global_localization")
        global_loc_client.call_async(Empty.Request())

        # Brief pause — let AMCL spread particles before we start moving 
        rclpy.spin_once(node, timeout_sec=1.0)

        # ── Step 2: Wait for first AMCL pose ──────────────────────────────────
        self.log_signal.emit(f"Waiting up to {COV_LOCK_TIMEOUT}s for first AMCL pose…")
        deadline = time.monotonic() + COV_LOCK_TIMEOUT
        while not self._received_pose.is_set() and not self._stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() > deadline:
                self.log_signal.emit("[ERROR] No AMCL pose received — check Nav2/AMCL — aborting")
                return

        if self._stop_event.is_set():
            return

        # ── Step 3: Spin-and-monitor loop (up to MAX_RETRIES full rotations) ──
        # Each "full rotation" = 2 × 180 ° arcs so we can report per-half
        # covariance (useful for half-occlusion diagnosis).

        for attempt in range(1, MAX_RETRIES + 2):   # attempt 1 = first try
            if self._stop_event.is_set():
                break

            # Check if already good before spinning
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
                f"(pre-spin σ²={pre_cov:.4f}) ──"
            )

            # First 180 °
            t1, best_1 = self._spin_arc(
                cmd_vel_pub, node, HALF_ROTATION_TIME, f"R{attempt} first 180°"
            )
            self.log_signal.emit(
                f"First 180° done in {t1:.1f}s — best σ²={best_1:.4f}"
            )

            # If early-exit already got us below threshold, stop here
            with self._cov_lock:
                mid_cov = self._current_cov
            if mid_cov < GOOD_COV_THRESHOLD:
                self.log_signal.emit(f"Well-localised after first half — stopping spin")
                break

            # Second 180 °
            t2, best_2 = self._spin_arc(
                cmd_vel_pub, node, HALF_ROTATION_TIME, f"R{attempt} second 180°"
            )
            self.log_signal.emit(
                f"Second 180° done in {t2:.1f}s — best σ²={best_2:.4f}"
            )

            # Per-half delta: useful for detecting asymmetric LiDAR occlusion
            delta = abs(best_1 - best_2)
            self.log_signal.emit(
                f"Half-covariance delta: {delta:.4f} "
                f"({'asymmetric occlusion suspected' if delta > 0.15 else 'symmetric'})"
            )

            with self._cov_lock:
                post_cov = self._best_cov
            self.log_signal.emit(
                f"Best overall σ² after attempt {attempt}: {post_cov:.4f}"
            )

            if post_cov < ACCEPTABLE_COV:
                self.log_signal.emit(f"Acceptable covariance reached — done")
                break
            elif attempt <= MAX_RETRIES:
                self.log_signal.emit(
                    f"Covariance {post_cov:.4f} still above {ACCEPTABLE_COV} "
                    f"— retrying ({attempt}/{MAX_RETRIES})…"
                )
            else:
                self.log_signal.emit(
                    f"[WARN] Max retries reached — best σ²={post_cov:.4f}. "
                    f"Manual intervention may be needed."
                )

        # ── Step 4: Stop robot ────────────────────────────────────────────────
        self._stop_robot(cmd_vel_pub)
        self.log_signal.emit("Robot stopped")

        # ── Step 5: Clear costmaps ────────────────────────────────────────────
        if clear_local_client.wait_for_service(timeout_sec=2.0):
            clear_local_client.call_async(Empty.Request())
            self.log_signal.emit("Cleared local costmap")
        else:
            self.log_signal.emit("[WARN] local_costmap clear service not available")

        rclpy.spin_once(node, timeout_sec=1.0)

        with self._cov_lock:
            final_cov = self._best_cov
        self.log_signal.emit(
            f"Relocalization complete — final best σ²={final_cov:.4f}"
        )


class RobotUI(QMainWindow):
    def __init__(self, skip_micro_ros=False):
        super().__init__()
        self.prev_stm32_status = None
        self.prev_lidar_status = None
        self.localization_worker = None
        self.localization_thread = None
        self.init_ui()
        if not skip_micro_ros:
            self.start_micro_ros()
        
    def init_ui(self):
        self.setWindowTitle("Robot Control Interface")

        STYLESHEET = """
            QMainWindow, QWidget {
                background-color: #0d0f12;
                color: #e8ecf0;
                border: none;
            }
            QWidget#left-panel {
                background-color: #141720;
                border-right: 2px solid #2a3040;
            }
            QWidget#header-bar {
                background-color: #141720;
                border-bottom: 1px solid #2a3040;
            }
            QWidget#status-card {
                background-color: #1c2030;
                border: 1px solid #2a3040;
                border-radius: 4px;
            }
            QWidget#log-panel {
                background-color: #080a0d;
                border-top: 1px solid #2a3040;
            }
            QPushButton#mode-btn {
                background-color: transparent;
                color: #6b7a99;
                border: none;
                border-left: 4px solid transparent;
                border-radius: 0px;
                padding: 20px 20px 20px 24px;
                text-align: left;
                font-size: 18px;
            }
            QPushButton#mode-btn:hover {
                background-color: #1a1f2e;
                color: #e8ecf0;
                border-left: 4px solid #3a4460;
            }
            QPushButton#mode-btn:checked {
                background-color: #1c2030;
                color: #00e5ff;
                border-left: 4px solid #00e5ff;
            }
            QPushButton#mode-btn:disabled {
                color: #3a4460;
                border-left: 4px solid transparent;
            }
            QTextEdit#log-text {
                background-color: #080a0d;
                color: #e8ecf0;
                border: none;
                font-size: 13px;
            }
            QLabel#log-title {
                color: #6b7a99;
                font-size: 11px;
                letter-spacing: 2px;
            }
            QLabel#clock {
                color: #6b7a99;
                font-size: 15px;
            }
            QLabel#mode-title {
                color: #e8ecf0;
                font-size: 15px;
            }
            QLabel#device-name {
                color: #6b7a99;
                font-size: 11px;
            }
            QLabel#status-ok {
                color: #00c853;
                font-size: 14px;
            }
            QLabel#status-error {
                color: #ff3b3b;
                font-size: 14px;
            }
            QLabel#status-checking {
                color: #6b7a99;
                font-size: 14px;
            }
        """
        self.setStyleSheet(STYLESHEET)

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

        wordmark = QLabel("ZACKON")
        wordmark.setFont(QFont("JetBrains Mono", 16, QFont.Weight.Bold))
        wordmark.setStyleSheet("color: #00e5ff; padding: 24px 24px 16px 24px;")
        left_layout.addWidget(wordmark)

        self.btn_tracking   = QPushButton("Tracking")
        self.btn_waypoints  = QPushButton("Waypoints")
        self.btn_reestimate = QPushButton("Re-estimate")
        self.btn_new_map    = QPushButton("New Map")
        self.btn_load_map   = QPushButton("Load Map")
        self.btn_docking    = QPushButton("Docking")
        self.btn_nav2       = QPushButton("Nav2")

        mono = QFont("JetBrains Mono", 18)
        for btn in [self.btn_tracking, self.btn_waypoints, self.btn_reestimate,
                    self.btn_new_map, self.btn_load_map, self.btn_docking, self.btn_nav2]:
            btn.setObjectName("mode-btn")
            btn.setFont(mono)
            btn.setMinimumHeight(72)
            btn.setCheckable(True)
            btn.setAutoExclusive(False)
            left_layout.addWidget(btn)

        left_layout.addStretch()

        self.btn_tracking.clicked.connect(lambda: self.mode_changed("Tracking"))
        self.btn_waypoints.clicked.connect(lambda: self.mode_changed("Waypoints"))
        self.btn_reestimate.clicked.connect(self.start_reestimate)
        self.btn_new_map.clicked.connect(self.start_new_map)
        self.btn_load_map.clicked.connect(self.load_map)
        self.btn_docking.clicked.connect(self.start_docking)
        self.btn_nav2.clicked.connect(lambda: self.mode_changed("Nav2"))

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
        cards_widget.setStyleSheet("background-color: #0d0f12; padding: 12px;")
        cards_layout = QHBoxLayout(cards_widget)
        cards_layout.setContentsMargins(12, 12, 12, 12)
        cards_layout.setSpacing(12)

        self.stm32_card  = self._make_status_card("STM32")
        self.lidar_card  = self._make_status_card("LiDAR")
        cards_layout.addWidget(self.stm32_card["widget"])
        cards_layout.addWidget(self.lidar_card["widget"])
        right_layout.addWidget(cards_widget)

        # Log panel
        log_panel = QWidget()
        log_panel.setObjectName("log-panel")
        log_layout = QVBoxLayout(log_panel)
        log_layout.setContentsMargins(16, 12, 16, 12)
        log_layout.setSpacing(6)

        log_header = QHBoxLayout()
        log_title = QLabel("SYSTEM LOG")
        log_title.setObjectName("log-title")
        log_title.setFont(QFont("DM Sans", 11))
        live_badge = QLabel("● LIVE")
        live_badge.setStyleSheet("color: #00c853; font-size: 11px;")
        live_badge.setFont(QFont("DM Sans", 11))
        log_header.addWidget(log_title)
        log_header.addStretch()
        log_header.addWidget(live_badge)
        log_layout.addLayout(log_header)

        self.log_text = QTextEdit()
        self.log_text.setObjectName("log-text")
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Code", 13))
        log_layout.addWidget(self.log_text)

        right_layout.addWidget(log_panel, 1)

        main_layout.addWidget(left_panel, 22)
        main_layout.addWidget(right_widget, 78)

        # Timers
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(5000)

        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()

        self._reestimate_pulse_timer = QTimer()
        self._reestimate_pulse_timer.timeout.connect(self._pulse_reestimate)
        self._pulse_state = False

        self.update_status()

    def _make_status_card(self, device_name):
        card = QWidget()
        card.setObjectName("status-card")
        card.setMinimumHeight(100)
        card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout = QHBoxLayout(card)
        layout.setContentsMargins(16, 12, 20, 12)

        dot = QLabel("●")
        dot.setStyleSheet("color: #6b7a99; font-size: 12px;")
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
        color = "#00c853" if available else "#ff3b3b"
        text  = "Available" if available else "Unavailable"
        obj   = "status-ok" if available else "status-error"
        card["dot"].setStyleSheet(f"color: {color}; font-size: 12px;")
        card["state"].setText(text)
        card["state"].setObjectName(obj)
        card["state"].setStyleSheet(f"color: {color}; font-size: 14px;")
        border_side = f"border-left: 4px solid {color};"
        card["widget"].setStyleSheet(
            f"QWidget#status-card {{ background-color: #1c2030; border: 1px solid #2a3040; border-radius: 4px; {border_side} }}"
        )

    def _update_clock(self):
        from datetime import datetime
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S"))

    def _pulse_reestimate(self):
        self._pulse_state = not self._pulse_state
        color = "#00e5ff" if self._pulse_state else "#3a4460"
        self.btn_reestimate.setStyleSheet(
            f"QPushButton#mode-btn {{ border-left: 4px solid {color}; color: #00e5ff; background-color: #1c2030; }}"
        )

        
    def start_micro_ros(self):
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'source ~/zackon_build_up/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; exec bash'
            ])
            self.log("Started micro-ROS agent in new terminal")
        except Exception as e:
            self.log(f"Failed to start micro-ROS agent: {e}")
    
    def update_status(self):
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                    capture_output=True, text=True, timeout=2)
            stm32_available = '/cmd_vel' in result.stdout or result.returncode == 0
        except:
            stm32_available = False

        self._set_card_status(self.stm32_card, stm32_available)

        if self.prev_stm32_status is not None and self.prev_stm32_status != stm32_available:
            self.log("STM32 connection lost" if not stm32_available else "STM32 connection restored")
        self.prev_stm32_status = stm32_available

        lidar_available = os.path.exists('/dev/lidar')
        self._set_card_status(self.lidar_card, lidar_available)

        if self.prev_lidar_status is not None and self.prev_lidar_status != lidar_available:
            self.log("LiDAR connection lost" if not lidar_available else "LiDAR connection restored")
        self.prev_lidar_status = lidar_available
    
    def mode_changed(self, mode):
        self.log(f"Mode changed to {mode}")
        if mode == "Tracking":
            subprocess.Popen(['python3', f'{SOURCE_PATH}/robot_ui/tracking_mode_layout.py'])
            self.close()
        elif mode == "Waypoints":
            subprocess.Popen(['python3', f'{SOURCE_PATH}/robot_ui/waypoints_mode_layout.py'])
            self.close()
        elif mode == "Nav2":
            try:
                subprocess.Popen([
                    'gnome-terminal', '--', 'bash', '-c',
                    'source ~/zackon_build_up/install/setup.bash && ros2 launch view_robot_pkg zackon_synthesis.launch.py; exec bash'
                ])
                self.log("Launched Nav2 navigation system")
            except Exception as e:
                self.log(f"Failed to launch Nav2: {e}")
    
    def start_reestimate(self):
        if self.prev_stm32_status == False:
            self.log("Cannot start localization: STM32 not connected")
            return
        if self.localization_thread and self.localization_thread.is_alive():
            self.log("Localization already running")
            return
        self.log("Starting global relocalization")
        self._reestimate_pulse_timer.start(600)

        self.localization_worker = LocalizationWorker()
        self.localization_worker.log_signal.connect(self.log)
        self.localization_worker.finished_signal.connect(self.localization_finished)
        self.localization_thread = threading.Thread(target=self.localization_worker.run, daemon=True)
        self.localization_thread.start()

    def localization_finished(self):
        self._reestimate_pulse_timer.stop()
        self.btn_reestimate.setStyleSheet("")
        self.localization_thread = None
        self.localization_worker = None
    
    def start_new_map(self):
        self.log("Switching to New Map mode")
        subprocess.Popen(['python3', f'{SOURCE_PATH}/robot_ui/new_map_layout.py'])
        self.close()
    
    def start_docking(self):
        self.log("Starting docking sequence")
        subprocess.Popen([
            'gnome-terminal', '--', 'bash', '-c',
            f'source {SOURCE_PATH}/install/setup.bash && python3 {SOURCE_PATH}/robot_ui/docking_sequence.py; exec bash'
        ])
    
    def load_map(self):
        dialog = LoadMapDialog(self)
        if dialog.exec():
            map_name = dialog.get_selected_map()
            if map_name:
                self.log(f"Loading map: {map_name}")
                self.update_map_files(map_name)
    
    def update_map_files(self, map_name):
        map_path = f'{SOURCE_PATH}/src/view_robot/maps/{map_name}.yaml'
        
        nav2_params = f'{SOURCE_PATH}/src/view_robot/config/nav2_params.yaml'
        try:
            with open(nav2_params, 'r') as f:
                content = f.read()
            
            updated = re.sub(
                r'(yaml_filename:\s*")[^"]*(")',
                rf'\1{map_path}\2',
                content
            )
            
            with open(nav2_params, 'w') as f:
                f.write(updated)
            self.log(f"✓ Updated nav2_params.yaml")
        except Exception as e:
            self.log(f"✗ Error updating nav2_params.yaml: {e}")
            return
        
        synthesis_launch = f'{SOURCE_PATH}/src/view_robot/launch/zackon_synthesis.launch.py'
        try:
            with open(synthesis_launch, 'r') as f:
                content = f.read()
            
            updated = re.sub(
                r"(map_file_path\s*=\s*PathJoinSubstitution\(\[pkg_dir,\s*'maps',\s*')[^']*('\]\))",
                rf"\1{map_name}.yaml\2",
                content
            )
            
            with open(synthesis_launch, 'w') as f:
                f.write(updated)
            self.log(f"✓ Updated zackon_synthesis.launch.py")
        except Exception as e:
            self.log(f"✗ Error updating zackon_synthesis.launch.py: {e}")
            return
        
        localization_launch = f'{SOURCE_PATH}/src/view_robot/launch/zackon_localization.launch.py'
        try:
            with open(localization_launch, 'r') as f:
                content = f.read()
            
            updated = re.sub(
                r"(default_value=os\.path\.join\(bringup_dir,\s*'maps',\s*')[^']*('\))",
                rf"\1{map_name}.yaml\2",
                content
            )
            
            with open(localization_launch, 'w') as f:
                f.write(updated)
            self.log(f"✓ Updated zackon_localization.launch.py")
        except Exception as e:
            self.log(f"✗ Error updating zackon_localization.launch.py: {e}")
            return
        
        # 4. Build and source workspace
        self.log("Building workspace...")
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'cd ~/zackon_build_up && colcon build --packages-select view_robot_pkg && source install/setup.bash && echo "Build complete. Closing in 2 seconds..." && sleep 2'
            ])
            self.log(f"✓ Map '{map_name}' loaded and workspace building")
        except Exception as e:
            self.log(f"✗ Error building workspace: {e}")
    
    def log(self, message):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        if "[ERROR]" in message:
            color = "#ff3b3b"
        elif "[WARN]" in message:
            color = "#ffb300"
        elif "✓" in message or "restored" in message or "complete" in message.lower():
            color = "#00c853"
        else:
            color = "#e8ecf0"
        self.log_text.append(
            f'<span style="color:#6b7a99">[{ts}]</span> <span style="color:{color}">{message}</span>'
        )
    
    def closeEvent(self, event):
        if self.localization_worker:
            self.localization_worker.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    skip_micro_ros = '--skip-micro-ros' in sys.argv
    window = RobotUI(skip_micro_ros=skip_micro_ros)
    window.showMaximized()
    sys.exit(app.exec())
