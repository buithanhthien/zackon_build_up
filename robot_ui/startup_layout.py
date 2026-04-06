#!/usr/bin/env python3
import sys
import subprocess
import os
import threading
import time
import math
import re
import json
import urllib.request
import urllib.error
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QTextEdit, QLabel,
                             QSizePolicy, QLineEdit, QScrollArea, QFrame)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont, QFontDatabase, QKeyEvent
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty
from load_map_dialog import LoadMapDialog
from voice_engine import VoiceEngine, VoiceState
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH


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

# ── Groq API config ───────────────────────────────────────────────────────────
def _load_env():
    env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '.env')
    if os.path.exists(env_path):
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    k, v = line.split('=', 1)
                    os.environ.setdefault(k.strip(), v.strip())

_load_env()
GROQ_API_KEY  = os.environ.get("GROQ_API_KEY", "")
GROQ_MODEL    = "llama-3.1-8b-instant"
SYSTEM_PROMPT = (
    "Bạn là ZACKON, AI đồng hành được tích hợp trực tiếp vào robot ROS 2 tự hành. "
    "Bạn LUÔN trả lời bằng tiếng Việt, ngắn gọn và rõ ràng. "

    "Bạn hỗ trợ các chức năng chính của robot gồm: "
    "điều hướng (Nav2), định vị (AMCL), docking sạc, kiểm tra trạng thái, và hỗ trợ vận hành. "

    "Khi người vận hành hỏi, hãy ưu tiên: "
    "1. Hiểu ý định điều khiển robot "
    "2. Đưa ra phản hồi ngắn gọn, thực tế "
    "3. Gợi ý hành động nếu cần "

    "Bạn có thể trò chuyện tự nhiên, nhưng luôn ưu tiên hỗ trợ vận hành robot. "

    "Nếu robot có dấu hiệu lỗi hoặc bất thường, hãy chủ động đề xuất giải pháp. "
    "Ví dụ: mất định vị, kẹt đường, không tìm thấy dock, lỗi navigation. "

    "Bạn có thể thêm một thẻ hành động ở cuối phản hồi khi phù hợp: "
    "<STATUS_CHECK> kiểm tra trạng thái robot "
    "<HELP> trợ giúp "
    "<NAVIGATE> điều hướng "
    "<DOCK> về dock sạc "
    "<LOCALIZE> định vị lại "

    "Chỉ thêm thẻ khi cần thực thi hành động. "
    "Không thêm nếu chỉ trò chuyện. "

    "Ví dụ: "
    "'Robot có vẻ mất định vị. Bạn muốn tôi định vị lại không? <LOCALIZE>' "
    "'Pin thấp, nên quay về trạm sạc. <DOCK>'"
)

# ─────────────────────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════════════════════
#  AI Chat Worker  (runs Groq API call in background thread)
# ══════════════════════════════════════════════════════════════════════════════

class AIChatWorker(QObject):
    response_ready  = pyqtSignal(str)   # emits the reply text
    error_occurred  = pyqtSignal(str)   # emits error message
    finished        = pyqtSignal()

    def __init__(self, history: list):
        super().__init__()
        self.history = history          # full conversation history (list of dicts)

    def run(self):
        try:
            payload = json.dumps({
                "model": GROQ_MODEL,
                "messages": self.history,
                "max_tokens": 512,
                "temperature": 0.7,
            }).encode("utf-8")

            req = urllib.request.Request(
                "https://api.groq.com/openai/v1/chat/completions",
                data=payload,
                headers={
                    "Content-Type": "application/json",
                    "Authorization": f"Bearer {GROQ_API_KEY}",
                    "User-Agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 Chrome/120.0.0.0 Safari/537.36",
                },
                method="POST",
            )

            with urllib.request.urlopen(req, timeout=15) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                reply = data["choices"][0]["message"]["content"].strip()
                self.response_ready.emit(reply)

        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", errors="replace")
            self.error_occurred.emit(f"HTTP {e.code}: {body[:200]}")
        except Exception as exc:
            self.error_occurred.emit(str(exc))
        finally:
            self.finished.emit()


# ══════════════════════════════════════════════════════════════════════════════
#  Chat bubble widget
# ══════════════════════════════════════════════════════════════════════════════

class ChatBubble(QWidget):
    """A single chat message bubble — user (right) or assistant (left)."""

    def __init__(self, text: str, role: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)

        label = QLabel(text)
        label.setWordWrap(True)
        label.setFont(QFont("Fira Code", 12))
        label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
        label.setMaximumWidth(560)

        if role == "user":
            label.setStyleSheet("""
                background-color: #00e5ff18;
                color: #e8ecf0;
                border: 1px solid #00e5ff44;
                border-radius: 8px;
                padding: 10px 14px;
            """)
            layout.addStretch()
            layout.addWidget(label)
        else:
            label.setStyleSheet("""
                background-color: #1c2030;
                color: #b0c4d8;
                border: 1px solid #2a3040;
                border-radius: 8px;
                padding: 10px 14px;
            """)
            layout.addWidget(label)
            layout.addStretch()


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
        self.prev_stm32_status    = None
        self.prev_lidar_status    = None
        self.localization_worker  = None
        self.localization_thread  = None

        # ── AI chat state ─────────────────────────────────────────────────────
        self._chat_history = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._ai_thread    = None
        self._ai_worker    = None

        # ── Silence / proactive re-engagement timer ───────────────────────────
        # Fires 20 s after the last AI response if the user hasn't typed anything
        self._silence_timer = QTimer()
        self._silence_timer.setSingleShot(True)
        self._silence_timer.timeout.connect(self._on_silence_timeout)

        # ── Voice Engine ──────────────────────────────────────────────────────
        self._voice_engine = VoiceEngine()
        # Connect voice engine signals
        self._voice_engine.state_changed.connect(self._on_voice_state_changed)
        self._voice_engine.transcript_ready.connect(self._on_voice_transcript)
        # Auto‑start will be performed after UI is built

        self.init_ui()
        # After UI is ready, enable and start the voice engine automatically
        self._voice_enabled = True
        self.voice_btn.setChecked(True)
        self.voice_status_label.show()
        self._voice_engine.start()
        if not skip_micro_ros:
            self.start_micro_ros()

    # ══════════════════════════════════════════════════════════════════════════
    #  UI construction
    # ══════════════════════════════════════════════════════════════════════════

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
            QWidget#chat-panel {
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

            /* ── Panel tab buttons ── */
            QPushButton#panel-tab {
                background-color: transparent;
                color: #6b7a99;
                border: none;
                border-bottom: 2px solid transparent;
                border-radius: 0px;
                padding: 6px 16px;
                font-size: 11px;
                letter-spacing: 2px;
            }
            QPushButton#panel-tab:checked {
                color: #00e5ff;
                border-bottom: 2px solid #00e5ff;
            }
            QPushButton#panel-tab:hover {
                color: #e8ecf0;
            }

            /* ── Chat input ── */
            QLineEdit#chat-input {
                background-color: #141720;
                color: #e8ecf0;
                border: 1px solid #2a3040;
                border-radius: 6px;
                padding: 10px 14px;
                font-size: 13px;
                selection-background-color: #00e5ff44;
            }
            QLineEdit#chat-input:focus {
                border: 1px solid #00e5ff66;
            }

            /* ── Send button ── */
            QPushButton#send-btn {
                background-color: #00e5ff18;
                color: #00e5ff;
                border: 1px solid #00e5ff44;
                border-radius: 6px;
                padding: 10px 20px;
                font-size: 13px;
            }
            QPushButton#send-btn:hover {
                background-color: #00e5ff30;
                border: 1px solid #00e5ff88;
            }
            QPushButton#send-btn:pressed {
                background-color: #00e5ff44;
            }
            QPushButton#send-btn:disabled {
                color: #3a4460;
                background-color: transparent;
                border: 1px solid #2a3040;
            }

            /* ── Voice toggle button ── */
            QPushButton#voice-btn {
                background-color: transparent;
                color: #6b7a99;
                border: 1px solid #2a3040;
                border-radius: 6px;
                font-size: 14px;
                padding: 10px;
            }
            QPushButton#voice-btn:checked {
                background-color: #ff3b3b18;
                color: #ff3b3b;
                border: 1px solid #ff3b3b44;
            }

            /* ── Clear chat button ── */
            QPushButton#clear-btn {
                background-color: transparent;
                color: #3a4460;
                border: none;
                font-size: 11px;
                padding: 4px 8px;
            }
            QPushButton#clear-btn:hover {
                color: #6b7a99;
            }

            /* ── Scroll area ── */
            QScrollArea {
                background-color: #080a0d;
                border: none;
            }
            QScrollBar:vertical {
                background-color: #0d0f12;
                width: 6px;
                border-radius: 3px;
            }
            QScrollBar::handle:vertical {
                background-color: #2a3040;
                border-radius: 3px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #3a4460;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
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

        self.stm32_card = self._make_status_card("STM32")
        self.lidar_card = self._make_status_card("LiDAR")
        cards_layout.addWidget(self.stm32_card["widget"])
        cards_layout.addWidget(self.lidar_card["widget"])
        right_layout.addWidget(cards_widget)

        # ── Panel switcher (LOG / AI CHAT) ────────────────────────────────────
        tab_bar = QWidget()
        tab_bar.setFixedHeight(36)
        tab_bar.setStyleSheet("background-color: #0d0f12; border-top: 1px solid #2a3040;")
        tab_layout = QHBoxLayout(tab_bar)
        tab_layout.setContentsMargins(12, 0, 12, 0)
        tab_layout.setSpacing(0)

        self.tab_log  = QPushButton("SYSTEM LOG")
        self.tab_chat = QPushButton("AI CHAT")
        for tab in [self.tab_log, self.tab_chat]:
            tab.setObjectName("panel-tab")
            tab.setFont(QFont("DM Sans", 11))
            tab.setCheckable(True)
            tab.setAutoExclusive(True)
            tab_layout.addWidget(tab)

        tab_layout.addStretch()

        # Live badge (only visible on log tab)
        self.live_badge = QLabel("● LIVE")
        self.live_badge.setStyleSheet("color: #00c853; font-size: 11px; padding-right: 4px;")
        self.live_badge.setFont(QFont("DM Sans", 11))
        tab_layout.addWidget(self.live_badge)

        self.tab_log.setChecked(True)
        self.tab_log.clicked.connect(self._show_log_panel)
        self.tab_chat.clicked.connect(self._show_chat_panel)

        right_layout.addWidget(tab_bar)

        # ── LOG panel ─────────────────────────────────────────────────────────
        self.log_panel = QWidget()
        self.log_panel.setObjectName("log-panel")
        log_layout = QVBoxLayout(self.log_panel)
        log_layout.setContentsMargins(16, 12, 16, 12)
        log_layout.setSpacing(6)

        self.log_text = QTextEdit()
        self.log_text.setObjectName("log-text")
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Code", 13))
        log_layout.addWidget(self.log_text)

        # ── AI CHAT panel ─────────────────────────────────────────────────────
        self.chat_panel = QWidget()
        self.chat_panel.setObjectName("chat-panel")
        self.chat_panel.hide()
        chat_layout = QVBoxLayout(self.chat_panel)
        chat_layout.setContentsMargins(0, 0, 0, 0)
        chat_layout.setSpacing(0)

        # Scrollable message area
        self.chat_scroll = QScrollArea()
        self.chat_scroll.setWidgetResizable(True)
        self.chat_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.chat_messages_widget = QWidget()
        self.chat_messages_widget.setStyleSheet("background-color: #080a0d;")
        self.chat_messages_layout = QVBoxLayout(self.chat_messages_widget)
        self.chat_messages_layout.setContentsMargins(16, 16, 16, 16)
        self.chat_messages_layout.setSpacing(8)
        self.chat_messages_layout.addStretch()   # pushes messages to bottom

        self.chat_scroll.setWidget(self.chat_messages_widget)
        chat_layout.addWidget(self.chat_scroll, 1)

        # Typing indicator label
        self.typing_label = QLabel("ZACKON is thinking...")
        self.typing_label.setFont(QFont("Fira Code", 11))
        self.typing_label.setStyleSheet("color: #3a4460; padding: 4px 20px; background-color: #080a0d;")
        self.typing_label.hide()
        chat_layout.addWidget(self.typing_label)

        # Voice status indicator (e.g. 🔴 LISTENING)
        self.voice_status_label = QLabel(VoiceState.IDLE)
        self.voice_status_label.setFont(QFont("Fira Code", 11, QFont.Weight.Bold))
        self.voice_status_label.setStyleSheet("color: #6b7a99; padding: 0 20px; background-color: #080a0d;")
        self.voice_status_label.hide()
        chat_layout.addWidget(self.voice_status_label)

        # Input row
        input_row = QWidget()
        input_row.setStyleSheet("background-color: #0d0f12; border-top: 1px solid #2a3040;")
        input_layout = QHBoxLayout(input_row)
        input_layout.setContentsMargins(16, 10, 16, 10)
        input_layout.setSpacing(10)

        self.chat_input = QLineEdit()
        self.chat_input.setObjectName("chat-input")
        self.chat_input.setFont(QFont("Fira Code", 13))
        self.chat_input.setPlaceholderText("Ask ZACKON anything about the robot...")
        self.chat_input.returnPressed.connect(self.send_chat_message)

        self.voice_btn = QPushButton("🎤")
        self.voice_btn.setObjectName("voice-btn")
        self.voice_btn.setCheckable(True)
        self.voice_btn.setFixedSize(44, 44)
        self.voice_btn.clicked.connect(self._toggle_voice)

        self.send_btn = QPushButton("SEND")
        self.send_btn.setObjectName("send-btn")
        self.send_btn.setFont(QFont("JetBrains Mono", 12, QFont.Weight.Bold))
        self.send_btn.setFixedWidth(80)
        self.send_btn.clicked.connect(self.send_chat_message)

        self.clear_btn = QPushButton("CLEAR")
        self.clear_btn.setObjectName("clear-btn")
        self.clear_btn.setFont(QFont("JetBrains Mono", 11))
        self.clear_btn.clicked.connect(self.clear_chat)

        input_layout.addWidget(self.voice_btn)
        input_layout.addWidget(self.chat_input)
        input_layout.addWidget(self.send_btn)
        input_layout.addWidget(self.clear_btn)
        chat_layout.addWidget(input_row)

        # Add welcome message
        self._add_chat_bubble(
            "Nghe này, em yêu",
            "assistant"
        )

        right_layout.addWidget(self.log_panel, 1)
        right_layout.addWidget(self.chat_panel, 1)

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

        # Typing animation timer
        self._typing_dots  = 0
        self._typing_timer = QTimer()
        self._typing_timer.timeout.connect(self._animate_typing)

        self.update_status()

    # ══════════════════════════════════════════════════════════════════════════
    #  Panel switching
    # ══════════════════════════════════════════════════════════════════════════

    def _show_log_panel(self):
        self.log_panel.show()
        self.chat_panel.hide()
        self.live_badge.show()

    def _show_chat_panel(self):
        self.log_panel.hide()
        self.chat_panel.show()
        self.live_badge.hide()
        self.chat_input.setFocus()
        QTimer.singleShot(50, self._scroll_chat_to_bottom)

    # ══════════════════════════════════════════════════════════════════════════
    #  AI Chat logic
    # ══════════════════════════════════════════════════════════════════════════

    def _add_chat_bubble(self, text: str, role: str):
        """Insert a chat bubble into the scroll area."""
        bubble = ChatBubble(text, role)
        # Insert before the trailing stretch (last item)
        count = self.chat_messages_layout.count()
        self.chat_messages_layout.insertWidget(count - 1, bubble)
        QTimer.singleShot(30, self._scroll_chat_to_bottom)

    def _scroll_chat_to_bottom(self):
        sb = self.chat_scroll.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _animate_typing(self):
        dots = "." * (self._typing_dots % 4)
        self.typing_label.setText(f"ZACKON is thinking{dots}")
        self._typing_dots += 1

    def send_chat_message(self):
        text = self.chat_input.text().strip()
        if not text:
            return
        if self._ai_thread and self._ai_thread.isRunning():
            return   # already waiting for a response

        # Reset silence timer whenever the user sends a message
        self._silence_timer.stop()

        self.chat_input.clear()
        self._add_chat_bubble(text, "user")
        self._chat_history.append({"role": "user", "content": text})

        # Show typing indicator
        self.typing_label.show()
        self._typing_dots = 0
        self._typing_timer.start(400)
        self.send_btn.setEnabled(False)
        self.chat_input.setEnabled(False)

        # Spin up background worker
        self._ai_worker = AIChatWorker(list(self._chat_history))
        self._ai_thread = QThread()
        self._ai_worker.moveToThread(self._ai_thread)

        self._ai_thread.started.connect(self._ai_worker.run)
        self._ai_worker.response_ready.connect(self._on_ai_response)
        self._ai_worker.error_occurred.connect(self._on_ai_error)
        self._ai_worker.finished.connect(self._ai_thread.quit)
        self._ai_worker.finished.connect(self._on_ai_done)

        self._ai_thread.start()

    # ── Action tag parsing ────────────────────────────────────────────────────
    @staticmethod
    def _parse_and_strip_tags(reply: str):
        """Strip <TAG> action tags from LLM reply.

        Returns (clean_text, list_of_tag_names).  Tags are recognised anywhere
        in the response but are most meaningful at the end.
        """
        pattern = r'<(STATUS_CHECK|HELP|NAVIGATE|DOCK|LOCALIZE)>'
        tags    = re.findall(pattern, reply)
        clean   = re.sub(pattern, '', reply).strip()
        return clean, tags

    def _on_ai_response(self, reply: str):
        clean, tags = self._parse_and_strip_tags(reply)
        self._chat_history.append({"role": "assistant", "content": clean})
        self._add_chat_bubble(clean, "assistant")
        
        # Output speech if voice is enabled
        if self._voice_enabled:
            self._voice_engine.speak(clean)
            
        # Log any action tags to the system log for future ROS hooks
        for tag in tags:
            self.log(f"[AI-ACTION] {tag}")

    def _on_ai_error(self, error: str):
        self._add_chat_bubble(f"[ERROR] {error}", "assistant")

    def _on_ai_done(self):
        self.typing_label.hide()
        self._typing_timer.stop()
        self.send_btn.setEnabled(True)
        self.chat_input.setEnabled(True)
        self.chat_input.setFocus()
        # Start 20-second silence countdown after every AI reply
        self._silence_timer.start(20_000)

    # ── Voice Engine Slots ────────────────────────────────────────────────────
    
    def _toggle_voice(self):
        self._voice_enabled = self.voice_btn.isChecked()
        if self._voice_enabled:
            self.voice_status_label.show()
            self._voice_engine.start()
        else:
            self.voice_status_label.hide()
            self._voice_engine.stop()

    def _on_voice_state_changed(self, state: str):
        self.voice_status_label.setText(state)
        if "LISTENING" in state:
            self.voice_status_label.setStyleSheet("color: #00e5ff; padding: 0 20px; background-color: #080a0d;")
        elif "SPEAKING" in state:
            self.voice_status_label.setStyleSheet("color: #00c853; padding: 0 20px; background-color: #080a0d;")
        else:
            self.voice_status_label.setStyleSheet("color: #6b7a99; padding: 0 20px; background-color: #080a0d;")

    def _on_voice_transcript(self, text: str):
        # Programmatically paste the transcribed text and send it
        text = text.strip()
        if not text:
            return
        
        # Add 'Hey Zackon' context purely for visual, if desired, 
        # or just send the text directly since the engine triggers on the wake word.
        # Capitalize first letter:
        text = text[0].upper() + text[1:]
        
        self.chat_input.setText(text)
        self.send_chat_message()

    def _on_silence_timeout(self):
        """Proactively re-engage the operator after 20 s of idle time."""
        if not self.chat_panel.isVisible():
            return
        if self._ai_thread and self._ai_thread.isRunning():
            return

        # Ask the LLM to generate a context-aware follow-up question
        probe_history = list(self._chat_history) + [{
            "role": "user",
            "content": (
                "[SYSTEM: The operator has been idle for 20 seconds. "
                "Ask a single short follow-up question related to the last topic "
                "discussed. Do not explain why you are asking.]"
            )
        }]
        self._ai_worker = AIChatWorker(probe_history)
        self._ai_thread = QThread()
        self._ai_worker.moveToThread(self._ai_thread)
        self._ai_thread.started.connect(self._ai_worker.run)
        self._ai_worker.response_ready.connect(self._on_ai_response)
        self._ai_worker.error_occurred.connect(self._on_ai_error)
        self._ai_worker.finished.connect(self._ai_thread.quit)
        self._ai_worker.finished.connect(self._on_ai_done)
        self._ai_thread.start()

    def clear_chat(self):
        """Remove all bubbles and reset history."""
        self._silence_timer.stop()
        # Remove all widgets except the trailing stretch
        while self.chat_messages_layout.count() > 1:
            item = self.chat_messages_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._chat_history = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._add_chat_bubble(
            "Chat cleared. How can I help you?",
            "assistant"
        )

    # ══════════════════════════════════════════════════════════════════════════
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
        color  = "#00c853" if available else "#ff3b3b"
        text   = "Available" if available else "Unavailable"
        obj    = "status-ok" if available else "status-error"
        card["dot"].setStyleSheet(f"color: {color}; font-size: 12px;")
        card["state"].setText(text)
        card["state"].setObjectName(obj)
        card["state"].setStyleSheet(f"color: {color}; font-size: 14px;")
        border_side = f"border-left: 4px solid {color};"
        card["widget"].setStyleSheet(
            f"QWidget#status-card {{ background-color: #1c2030; border: 1px solid #2a3040; "
            f"border-radius: 4px; {border_side} }}"
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
                'source ~/zackon_build_up/install/setup.bash && '
                'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; exec bash'
            ])
            self.log("Started micro-ROS agent in new terminal")
        except Exception as e:
            self.log(f"Failed to start micro-ROS agent: {e}")

    def update_status(self):
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                    capture_output=True, text=True, timeout=2)
            stm32_available = '/cmd_vel' in result.stdout or result.returncode == 0
        except Exception:
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
                    f'source {SOURCE_PATH}/install/setup.bash && ros2 launch {SOURCE_PATH}/src/view_robot/launch/NAV2_BRINGUP.launch.py; exec bash'
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
        self.log("Switching to New Map mode")
        subprocess.Popen(['python3', f'{SOURCE_PATH}/robot_ui/new_map_layout.py'])
        self.close()

    def start_docking(self):
        self.log("Starting docking sequence")
        subprocess.Popen([
            'gnome-terminal', '--', 'bash', '-c',
            f'source {SOURCE_PATH}/install/setup.bash && '
            f'python3 {SOURCE_PATH}/robot_ui/docking_sequence.py; exec bash'
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
                r'(yaml_filename:\s*")[^"]*(")', rf'\1{map_path}\2', content
            )
            with open(nav2_params, 'w') as f:
                f.write(updated)
            self.log("✓ Updated nav2_params.yaml")
        except Exception as e:
            self.log(f"✗ Error updating nav2_params.yaml: {e}")
            return
        synthesis_launch = f'{SOURCE_PATH}/src/view_robot/launch/NAV2_BRINGUP.launch.py'
        try:
            with open(synthesis_launch, 'r') as f:
                content = f.read()
            updated = re.sub(
                r"(map_file_path\s*=\s*PathJoinSubstitution\(\[pkg_dir,\s*'maps',\s*')[^']*('\]\))",
                rf"\1{map_name}.yaml\2", content
            )
            with open(synthesis_launch, 'w') as f:
                f.write(updated)
            self.log(f"✓ Updated NAV2_BRINGUP.launch.py")
        except Exception as e:
            self.log(f"✗ Error updating NAV2_BRINGUP.launch.py: {e}")
            return

        localization_launch = f'{SOURCE_PATH}/src/view_robot/launch/zackon_localization.launch.py'
        try:
            with open(localization_launch, 'r') as f:
                content = f.read()
            updated = re.sub(
                r"(default_value=os\.path\.join\(bringup_dir,\s*'maps',\s*')[^']*('\))",
                rf"\1{map_name}.yaml\2", content
            )
            with open(localization_launch, 'w') as f:
                f.write(updated)
            self.log("✓ Updated zackon_localization.launch.py")
        except Exception as e:
            self.log(f"✗ Error updating zackon_localization.launch.py: {e}")
            return

        self.log("Building workspace...")
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'cd ~/zackon_build_up && colcon build --packages-select view_robot_pkg '
                '&& source install/setup.bash '
                '&& echo "Build complete. Closing in 2 seconds..." && sleep 2'
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
            f'<span style="color:#6b7a99">[{ts}]</span> '
            f'<span style="color:{color}">{message}</span>'
        )

    def closeEvent(self, event):
        if self.localization_worker:
            self.localization_worker.stop()
        if self._ai_thread and self._ai_thread.isRunning():
            self._ai_thread.quit()
            self._ai_thread.wait(2000)
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    skip_micro_ros = '--skip-micro-ros' in sys.argv
    window = RobotUI(skip_micro_ros)
    window.showMaximized()
    sys.exit(app.exec())