#!/usr/bin/env python3
import os
import re
import json
import urllib.request
import urllib.error
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QLineEdit, QScrollArea, QSizePolicy)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from voice_engine import VoiceEngine, VoiceState


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
GROQ_API_KEY = os.environ.get("GROQ_API_KEY", "")
GROQ_MODEL   = "llama-3.1-8b-instant"
SYSTEM_PROMPT = (
    "Bạn là ZACKON, AI trợ lý tích hợp trong robot ROS 2. "
    "Luôn trả lời bằng tiếng Việt, ngắn gọn (tối đa 2 câu mỗi lần). "

    "Khi hướng dẫn người dùng mới, hãy hướng dẫn TỪNG BƯỚC MỘT. "
    "Sau mỗi bước, hỏi 'Bạn muốn hỏi gì nữa không?' và chờ xác nhận. "
    "Không liệt kê nhiều bước cùng lúc. "

    "Danh sách các bước hướng dẫn theo thứ tự (chỉ dùng khi người dùng cần hướng dẫn):\n"
    "1. Kiểm tra STM32 và LiDAR ở góc trên — cả hai phải hiện 'Available'.\n"
    "2. Nhấn 'Load Map' để chọn bản đồ phù hợp với khu vực hiện tại.\n"
    "   - Nếu chưa có bản đồ: KHÔNG nhấn 'Load Map'. Thay vào đó, nhấn tab 'New Map' ở màn hình chính (đây là tab riêng biệt, không phải bên trong Load Map).\n"
    "     Trong tab New Map: nhấn 'Start Mapping' để bắt đầu SLAM, lái robot khám phá khu vực,\n"
    "     sau đó nhập tên bản đồ vào ô 'Enter map name...' và nhấn 'Apply' để lưu.\n"
    "     Nhấn 'Back' để quay lại màn hình chính và dùng 'Load Map' để tải bản đồ vừa tạo.\n"
    "3. Nhấn 'Re-estimate' để robot định vị lại vị trí trên bản đồ.\n"
    "4. Chọn chế độ vận hành: Tracking (theo người), Waypoints (đi theo điểm), hoặc Nav2 (điều hướng tự do).\n"

    "Ngoài hướng dẫn, bạn hỗ trợ: điều hướng, định vị, docking, kiểm tra trạng thái. "
    "Nếu phát hiện lỗi, đề xuất giải pháp ngay. "

    "Thẻ hành động (chỉ thêm khi cần thực thi, không dùng khi trò chuyện): "
    "<STATUS_CHECK> <HELP> <NAVIGATE> <DOCK> <LOCALIZE>"
)


class _AIChatWorker(QObject):
    response_ready = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    finished       = pyqtSignal()

    def __init__(self, history):
        super().__init__()
        self.history = history

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
                    "User-Agent": "Mozilla/5.0",
                },
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=15) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                self.response_ready.emit(data["choices"][0]["message"]["content"].strip())
        except urllib.error.HTTPError as e:
            self.error_occurred.emit(f"HTTP {e.code}: {e.read().decode('utf-8', errors='replace')[:200]}")
        finally:
            self.finished.emit()


class _ChatBubble(QWidget):
    def __init__(self, text, role, parent=None):
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
            label.setStyleSheet("background-color:#00e5ff18;color:#e8ecf0;border:1px solid #00e5ff44;border-radius:8px;padding:10px 14px;")
            layout.addStretch()
            layout.addWidget(label)
        else:
            label.setStyleSheet("background-color:#1c2030;color:#b0c4d8;border:1px solid #2a3040;border-radius:8px;padding:10px 14px;")
            layout.addWidget(label)
            layout.addStretch()


class ChatPanel(QWidget):
    action_tag = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._chat_history = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._ai_worker = None
        self._ai_thread = None
        self._voice_enabled = False
        self._voice_engine = VoiceEngine()
        self._voice_engine.state_changed.connect(self._on_voice_state_changed)
        self._voice_engine.transcript_ready.connect(self._on_voice_transcript)

        self._build_ui()
        self._add_bubble(
            "Xin chào! Tôi là ZACKON, trợ lý robot của bạn. Hãy hỏi tôi về trạng thái robot, "
            "định vị, điều hướng, hoặc bất cứ điều gì khác.",
            "assistant"
        )

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.chat_scroll = QScrollArea()
        self.chat_scroll.setWidgetResizable(True)
        self.chat_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self._messages_widget = QWidget()
        self._messages_widget.setStyleSheet("background-color:#080a0d;")
        self._messages_layout = QVBoxLayout(self._messages_widget)
        self._messages_layout.setContentsMargins(16, 16, 16, 16)
        self._messages_layout.setSpacing(8)
        self._messages_layout.addStretch()

        self.chat_scroll.setWidget(self._messages_widget)
        layout.addWidget(self.chat_scroll, 1)

        self.typing_label = QLabel("ZACKON is thinking...")
        self.typing_label.setFont(QFont("Fira Code", 11))
        self.typing_label.setStyleSheet("color:#3a4460;padding:4px 20px;background-color:#080a0d;")
        self.typing_label.hide()
        layout.addWidget(self.typing_label)

        self.voice_status_label = QLabel(VoiceState.IDLE)
        self.voice_status_label.setFont(QFont("Fira Code", 11, QFont.Weight.Bold))
        self.voice_status_label.setStyleSheet("color:#6b7a99;padding:0 20px;background-color:#080a0d;")
        self.voice_status_label.hide()
        layout.addWidget(self.voice_status_label)

        input_row = QWidget()
        input_row.setStyleSheet("background-color:#0d0f12;border-top:1px solid #2a3040;")
        input_layout = QHBoxLayout(input_row)
        input_layout.setContentsMargins(16, 10, 16, 10)
        input_layout.setSpacing(10)

        self.voice_btn = QPushButton("[MIC]")
        self.voice_btn.setObjectName("voice-btn")
        self.voice_btn.setCheckable(True)
        self.voice_btn.setFixedSize(44, 44)
        self.voice_btn.clicked.connect(self._toggle_voice)

        self.chat_input = QLineEdit()
        self.chat_input.setObjectName("chat-input")
        self.chat_input.setFont(QFont("Fira Code", 13))
        self.chat_input.setPlaceholderText("Ask ZACKON anything about the robot...")
        self.chat_input.returnPressed.connect(self.send_message)

        self.send_btn = QPushButton("SEND")
        self.send_btn.setObjectName("send-btn")
        self.send_btn.setFont(QFont("JetBrains Mono", 12, QFont.Weight.Bold))
        self.send_btn.setFixedWidth(80)
        self.send_btn.clicked.connect(self.send_message)

        self.clear_btn = QPushButton("CLEAR")
        self.clear_btn.setObjectName("clear-btn")
        self.clear_btn.setFont(QFont("JetBrains Mono", 11))
        self.clear_btn.clicked.connect(self.clear_chat)

        input_layout.addWidget(self.voice_btn)
        input_layout.addWidget(self.chat_input)
        input_layout.addWidget(self.send_btn)
        input_layout.addWidget(self.clear_btn)
        layout.addWidget(input_row)

        self._typing_dots = 0
        self._typing_timer = QTimer()
        self._typing_timer.timeout.connect(self._animate_typing)

    def _add_bubble(self, text, role):
        bubble = _ChatBubble(text, role)
        count = self._messages_layout.count()
        self._messages_layout.insertWidget(count - 1, bubble)
        QTimer.singleShot(30, self._scroll_to_bottom)

    def _scroll_to_bottom(self):
        sb = self.chat_scroll.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _animate_typing(self):
        dots = "." * (self._typing_dots % 4)
        self.typing_label.setText(f"ZACKON is thinking{dots}")
        self._typing_dots += 1

    def send_message(self):
        text = self.chat_input.text().strip()
        if not text or (self._ai_thread and self._ai_thread.isRunning()):
            return
        self.chat_input.clear()
        self._add_bubble(text, "user")
        self._chat_history.append({"role": "user", "content": text})

        self.typing_label.show()
        self._typing_dots = 0
        self._typing_timer.start(400)
        self.send_btn.setEnabled(False)
        self.chat_input.setEnabled(False)

        self._ai_worker = _AIChatWorker(list(self._chat_history))
        self._ai_thread = QThread()
        self._ai_worker.moveToThread(self._ai_thread)
        self._ai_thread.started.connect(self._ai_worker.run)
        self._ai_worker.response_ready.connect(self._on_response)
        self._ai_worker.error_occurred.connect(self._on_error)
        self._ai_worker.finished.connect(self._ai_thread.quit)
        self._ai_worker.finished.connect(self._on_done)
        self._ai_thread.start()

    @staticmethod
    def _strip_tags(reply):
        pattern = r'<(STATUS_CHECK|HELP|NAVIGATE|DOCK|LOCALIZE)>'
        tags = re.findall(pattern, reply)
        clean = re.sub(pattern, '', reply).strip()
        return clean, tags

    def _on_response(self, reply):
        clean, tags = self._strip_tags(reply)
        self._chat_history.append({"role": "assistant", "content": clean})
        self._add_bubble(clean, "assistant")
        if self._voice_enabled:
            self._voice_engine.speak(clean)
        for tag in tags:
            self.action_tag.emit(tag)

    def _on_error(self, error):
        self._add_bubble(f"[ERROR] {error}", "assistant")

    def _on_done(self):
        self.typing_label.hide()
        self._typing_timer.stop()
        self.send_btn.setEnabled(True)
        self.chat_input.setEnabled(True)
        self.chat_input.setFocus()

    def _toggle_voice(self):
        self._voice_enabled = self.voice_btn.isChecked()
        if self._voice_enabled:
            self.voice_status_label.show()
            self._voice_engine.start()
        else:
            self.voice_status_label.hide()
            self._voice_engine.stop()

    def _on_voice_state_changed(self, state):
        self.voice_status_label.setText(state)
        if "LISTENING" in state:
            self.voice_status_label.setStyleSheet("color:#00e5ff;padding:0 20px;background-color:#080a0d;")
        elif "SPEAKING" in state:
            self.voice_status_label.setStyleSheet("color:#00c853;padding:0 20px;background-color:#080a0d;")
        else:
            self.voice_status_label.setStyleSheet("color:#6b7a99;padding:0 20px;background-color:#080a0d;")

    def _on_voice_transcript(self, text):
        text = text.strip()
        if not text:
            return
        text = text[0].upper() + text[1:]
        self.chat_input.setText(text)
        self.send_message()

    def clear_chat(self):
        while self._messages_layout.count() > 1:
            item = self._messages_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._chat_history = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._add_bubble("Chat cleared. How can I help you?", "assistant")

    def focus_input(self):
        self.chat_input.setFocus()
        QTimer.singleShot(50, self._scroll_to_bottom)

    def cleanup(self):
        if self._voice_enabled:
            self._voice_engine.stop()
