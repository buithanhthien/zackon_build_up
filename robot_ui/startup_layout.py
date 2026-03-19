#!/usr/bin/env python3
import sys
import subprocess
import os
import threading
import time
import math
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QFont
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
        self.setStyleSheet("background-color: white; color: black;")
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Left area - Mode selector (1/4)
        mode_widget = QWidget()
        mode_widget.setStyleSheet("background-color: #f0f0f0;")
        mode_layout = QVBoxLayout(mode_widget)
        
        self.btn_tracking = QPushButton("Tracking")
        self.btn_waypoints = QPushButton("Waypoints")
        self.btn_reestimate = QPushButton("Re-estimate")
        self.btn_new_map = QPushButton("New Map")
        self.btn_load_map = QPushButton("Load Map")
        self.btn_docking = QPushButton("Docking")
        self.btn_nav2 = QPushButton("Nav2")
        
        for btn in [self.btn_tracking, self.btn_waypoints, self.btn_reestimate, self.btn_new_map, self.btn_load_map, self.btn_docking, self.btn_nav2]:
            btn.setFont(QFont("Fira Sans", 24))
            btn.setMinimumHeight(100)
            if btn not in [self.btn_reestimate, self.btn_new_map, self.btn_load_map, self.btn_docking]:
                btn.clicked.connect(lambda checked, b=btn: self.mode_changed(b.text()))
            mode_layout.addWidget(btn)
        
        self.btn_reestimate.clicked.connect(self.start_reestimate)
        self.btn_new_map.clicked.connect(self.start_new_map)
        self.btn_load_map.clicked.connect(self.load_map)
        self.btn_docking.clicked.connect(self.start_docking)
        
        mode_layout.addStretch()
        
        # Right area (3/4) - split into status and logging
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        # Status area (top half)
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        title = QLabel("Robot Status")
        title.setFont(QFont("Fira Sans", 28, QFont.Weight.Bold))
        status_layout.addWidget(title)
        
        self.stm32_label = QLabel("STM32: Checking...")
        self.stm32_label.setFont(QFont("Fira Sans", 20))
        status_layout.addWidget(self.stm32_label)
        
        self.lidar_label = QLabel("LiDAR: Checking...")
        self.lidar_label.setFont(QFont("Fira Sans", 20))
        status_layout.addWidget(self.lidar_label)

        status_layout.addStretch()
        
        # Logging area (bottom half)
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_title = QLabel("System Log")
        log_title.setFont(QFont("Fira Sans", 20, QFont.Weight.Bold))
        log_layout.addWidget(log_title)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Sans", 14))
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(status_widget, 1)
        right_layout.addWidget(log_widget, 1)
        
        main_layout.addWidget(mode_widget, 1)
        main_layout.addWidget(right_widget, 3)
         
        # Timer for periodic updates (5 seconds)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(5000)
        
        self.update_status()
        
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
        # Check STM32 (micro-ROS agent) via topic list
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            stm32_available = '/cmd_vel' in result.stdout or result.returncode == 0
        except:
            stm32_available = False
            
        if stm32_available:
            self.stm32_label.setText("STM32: Available")
            self.stm32_label.setStyleSheet("color: green;")
        else:
            self.stm32_label.setText("STM32: Unavailable")
            self.stm32_label.setStyleSheet("color: red;")
        
        if self.prev_stm32_status is not None and self.prev_stm32_status != stm32_available:
            if not stm32_available:
                self.log("STM32 connection lost")
            else:
                self.log("STM32 connection restored")
        self.prev_stm32_status = stm32_available
        
        # Check LiDAR
        lidar_available = os.path.exists('/dev/lidar')
        if lidar_available:
            self.lidar_label.setText("LiDAR: Available")
            self.lidar_label.setStyleSheet("color: green;")
        else:
            self.lidar_label.setText("LiDAR: Unavailable")
            self.lidar_label.setStyleSheet("color: red;")
        
        if self.prev_lidar_status is not None and self.prev_lidar_status != lidar_available:
            if not lidar_available:
                self.log("LiDAR connection lost")
            else:
                self.log("LiDAR connection restored")
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
        
        self.localization_worker = LocalizationWorker()
        self.localization_worker.log_signal.connect(self.log)
        self.localization_worker.finished_signal.connect(self.localization_finished)
        
        self.localization_thread = threading.Thread(
            target=self.localization_worker.run, daemon=True
        )
        self.localization_thread.start()
        
    def localization_finished(self):
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
        self.log_text.append(message)
    
    def closeEvent(self, event):
        if self.localization_worker:
            self.localization_worker.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    skip_micro_ros = '--skip-micro-ros' in sys.argv
    window = RobotUI(skip_micro_ros=skip_micro_ros)
    window.show()
    window.setWindowState(Qt.WindowState.WindowMaximized)
    sys.exit(app.exec())
