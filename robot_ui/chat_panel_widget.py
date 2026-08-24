#!/usr/bin/env python3
import os
from openai import OpenAI
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from voice_engine import VoiceEngine

_DIR = os.path.dirname(os.path.abspath(__file__))


def _load_env():
    env_path = os.path.join(os.path.dirname(_DIR), '.env')
    if os.path.exists(env_path):
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    k, v = line.split('=', 1)
                    os.environ.setdefault(k.strip(), v.strip())

_load_env()
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
OPENAI_MODEL   = "gpt-5.4-mini"

SYSTEM_PROMPT = (
    "Bạn là ZACKON.\n"
    "Luôn trả lời bằng tiếng Việt, rõ ràng, ngắn gọn, hài hước và bựa.\n"
    "Không sử dụng dấu ngoặc kép (\") trong câu trả lời.\n"
    "Hãy trả lời ngắn gọn trong 2 đến 3 câu.\n"
    "Viết nội dung này theo giọng của một trợ lý hướng dẫn, nói chuyện với khách hàng. \n"
    "Mục tiêu: giúp khách hàng hiểu hơn về khoa công nghệ điện trường đại học công nghiệp Thành phố Hồ Chí Minh.\n"
    "Tình huống / bối cảnh: Là một trợ lý AI trong môi trường giáo dục, hay đi dọc các hành lang và có người vô tình bắt gặp nói chuyện.\n"
    "Dùng ngôn ngữ đời thường.\n"
    "Viết như đang giải thích cho một người, không phải cho đám đông.\n"
    "Không dùng câu chữ kịch tính. Không giọng marketing.\n"
    "Chỉ nói những gì thật sự cần nói, theo trình tự tự nhiên.\n"
    "Nội dung: Tra cứu và trả lời câu hỏi của khách hàng.\n"
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
            client = OpenAI(api_key=OPENAI_API_KEY)
            stream = client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=self.history,
                max_completion_tokens=1000,
                temperature=0.7,
                stream=True,
            )
            full = []
            for chunk in stream:
                token = (chunk.choices[0].delta.content or "") if chunk.choices else ""
                if token:
                    full.append(token)
            self.response_ready.emit("".join(full).strip())
        except Exception as e:
            self.error_occurred.emit(str(e)[:200])
        finally:
            self.finished.emit()


class ChatPanel(QWidget):
    # Emits log messages so startup_layout can display them in the system log
    log_signal       = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._chat_history  = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._ai_worker     = None
        self._ai_thread     = None
        self._voice_enabled = False
        self._pending_reply = None
        self._did_speak     = False

        self._voice_engine = VoiceEngine()
        self._voice_engine.state_changed.connect(self._on_voice_state_changed)
        self._voice_engine.transcript_ready.connect(self._on_voice_transcript)

        self._build_ui()

    # ------------------------------------------------------------------ UI --
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Voice status label (LISTENING / THINKING / SPEAKING)
        self.voice_status_label = QLabel("")
        self.voice_status_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.voice_status_label.setFont(QFont("Fira Code", 11, QFont.Weight.Bold))
        self.voice_status_label.setStyleSheet("color:#5a7abf; background-color:transparent;")
        self.voice_status_label.hide()
        layout.addWidget(self.voice_status_label)

        # Mic button — the only interactive element
        self.voice_btn = QPushButton("CLICK\n TO SPEAK")
        self.voice_btn.setObjectName("voice-btn")
        self.voice_btn.setCheckable(True)
        self.voice_btn.clicked.connect(self._on_listen_btn_clicked)

        # Stop-speaking button
        self.interrupt_btn = QPushButton("■ Dừng")
        self.interrupt_btn.setObjectName("interrupt-btn")
        self.interrupt_btn.setFont(QFont("JetBrains Mono", 11))
        self.interrupt_btn.setFixedHeight(40)
        self.interrupt_btn.setToolTip("Dừng phát âm")
        self.interrupt_btn.clicked.connect(self._voice_engine.stop_speaking)

        layout.addWidget(self.voice_btn, 0, Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(self.interrupt_btn, 0, Qt.AlignmentFlag.AlignHCenter)

        self._typing_dots = 0
        self._typing_timer = QTimer()
        self._typing_timer.timeout.connect(self._animate_status)

    def _animate_status(self):
        dots = "." * (self._typing_dots % 4)
        self.voice_status_label.setText(f"ZACKON đang suy nghĩ{dots}")
        self._typing_dots += 1

    # ----------------------------------------------------------- AI call ---
    def _ask_ai(self, text: str):
        if self._ai_thread and self._ai_thread.isRunning():
            return

        self._chat_history.append({"role": "user", "content": text})
        self.log_signal.emit(f"[Bạn] {text}")

        # Show thinking animation
        self.voice_status_label.show()
        self._typing_dots = 0
        self._typing_timer.start(400)

        self._ai_worker = _AIChatWorker(list(self._chat_history))
        self._ai_thread = QThread()
        self._ai_worker.moveToThread(self._ai_thread)
        self._ai_thread.started.connect(self._ai_worker.run)
        self._ai_worker.response_ready.connect(self._on_response)
        self._ai_worker.error_occurred.connect(self._on_error)
        self._ai_worker.finished.connect(self._ai_thread.quit)
        self._ai_thread.start()

    def _on_response(self, reply):
        self._typing_timer.stop()
        self._chat_history.append({"role": "assistant", "content": reply})
        self.log_signal.emit(f"[ZACKON] {reply}")
        print(f"[CHAT] Assistant: {reply}")

        # Always speak the reply — voice was active when the user spoke
        self._pending_reply = reply
        self._voice_engine.speak(reply)

    def _on_error(self, error):
        self._typing_timer.stop()
        self.voice_status_label.hide()
        self.log_signal.emit(f"[LỖI] {error}")

    def _finish_turn(self):
        self.voice_status_label.hide()

    # ----------------------------------------------------------- voice -----
    def _on_listen_btn_clicked(self):
        if not self.voice_btn.isChecked():
            self._voice_enabled = False
            self._voice_engine.stop_speaking()
            self.voice_status_label.hide()
            return
        self._voice_enabled = True
        self._voice_engine.listen_once()

    def _on_voice_state_changed(self, state):
        if "SPEAKING" in state and self._pending_reply:
            self._pending_reply = None
            self._did_speak = True
            self._finish_turn()

        if not state:
            # Reset button on any end-of-turn: STT failure, timeout, or after TTS
            self._voice_enabled = False
            self.voice_btn.setChecked(False)
            self.voice_status_label.hide()
            self._did_speak = False
            return

        self.voice_status_label.show()
        if "LISTENING" in state:
            self._typing_timer.stop()
            self.voice_status_label.setText(state)
            self.voice_status_label.setStyleSheet("color:#214196; background-color:transparent;")
        elif "SPEAKING" in state:
            self._typing_timer.stop()
            self.voice_status_label.setText(state)
            self.voice_status_label.setStyleSheet("color:#22c55e; background-color:transparent;")
        else:
            self.voice_status_label.setStyleSheet("color:#5a7abf; background-color:transparent;")

    def _on_voice_transcript(self, text):
        text = text.strip().capitalize()
        if not text:
            return
        print(f"[CHAT] User: {text}")
        self._ask_ai(text)

    # ----------------------------------------------------------- helpers ---

    def cleanup(self):
        if self._voice_enabled:
            self._voice_engine.stop_speaking()
