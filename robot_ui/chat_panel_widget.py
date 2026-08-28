#!/usr/bin/env python3
import os
import json
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

# ── Intent classification (voice navigation) ──────────────────────────────
INTENT_SYSTEM_PROMPT_TEMPLATE = (
    "Bạn là bộ phân loại ý định cho một robot điều hướng trong tòa nhà.\n"
    "Danh sách địa điểm hợp lệ hiện tại, định dạng \"TÊN_CHÍNH_XÁC: cách gọi khác 1, cách gọi khác 2, ...\":\n"
    "{waypoint_list}\n"
    "Người dùng có thể gọi một địa điểm bằng bất kỳ cách gọi nào ở trên (kể cả TÊN_CHÍNH_XÁC hoặc bất kỳ "
    "cách gọi khác nào được liệt kê sau dấu hai chấm). Dù người dùng nói theo cách nào, khi trả kết quả bạn "
    "LUÔN LUÔN phải dùng đúng TÊN_CHÍNH_XÁC (phần đứng trước dấu hai chấm), TUYỆT ĐỐI không trả về cách gọi khác.\n"
    "Nhiệm vụ: đọc câu nói của người dùng và xác định:\n"
    "- Nếu người dùng muốn ĐIỀU HƯỚNG robot đến một hoặc nhiều địa điểm nằm trong danh sách hợp lệ "
    "(theo đúng thứ tự họ nói ra), hoặc muốn quay lại/ở lại vị trí hiện tại của robot vào một thời điểm "
    "nào đó trong hành trình (dùng đúng chuỗi placeholder __return_here__ cho ý đó), "
    "hãy trả về intent \"navigate\" và mảng waypoints là danh sách TÊN_CHÍNH_XÁC tương ứng "
    "(hoặc __return_here__), theo đúng thứ tự người dùng muốn đi.\n"
    "- Nếu câu nói KHÔNG phải lệnh điều hướng (hỏi thông tin, trò chuyện, chào hỏi, không rõ ràng, "
    "hoặc nhắc đến địa điểm không khớp với bất kỳ mục nào ở trên), hãy trả về intent \"chat\" và waypoints là mảng rỗng.\n"
    "CHỈ được trả về đúng một object JSON hợp lệ, không thêm bất kỳ văn bản, giải thích, hay markdown nào khác. "
    "Định dạng bắt buộc:\n"
    '{{"intent": "navigate", "waypoints": ["TEN_CHINH_XAC_1", "TEN_CHINH_XAC_2"]}}\n'
    "hoặc\n"
    '{{"intent": "chat", "waypoints": []}}'
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


class _IntentWorker(QObject):
    """Classifies a voice transcript as a navigation command or plain chat,
    using a dedicated (non-streaming, temperature=0) OpenAI call that must
    return strict JSON: {"intent": "navigate"|"chat", "waypoints": [...]}.

    `waypoints` accepts either a list of plain key strings, or a list of
    dicts like {"key": "X5.7", "aliases": ["phong x5.7", ...]} — the latter
    lets the classifier match whatever phrasing the user actually said back
    to the correct canonical key."""
    intent_ready   = pyqtSignal(dict)
    error_occurred = pyqtSignal(str)
    finished       = pyqtSignal()

    def __init__(self, text, waypoints):
        super().__init__()
        self.text = text
        self.waypoints = waypoints

    def _format_waypoint_list(self):
        if not self.waypoints:
            return "(không có địa điểm nào được lưu)"
        lines = []
        for wp in self.waypoints:
            if isinstance(wp, dict):
                key = wp.get("key", "")
                aliases = wp.get("aliases") or []
            else:
                key = str(wp)
                aliases = []
            if not key:
                continue
            if aliases:
                lines.append(f"{key}: {', '.join(aliases)}")
            else:
                lines.append(f"{key}: (không có cách gọi khác)")
        return "\n".join(lines) if lines else "(không có địa điểm nào được lưu)"

    def run(self):
        try:
            client = OpenAI(api_key=OPENAI_API_KEY)
            wp_list_str = self._format_waypoint_list()
            system_prompt = INTENT_SYSTEM_PROMPT_TEMPLATE.format(waypoint_list=wp_list_str)
            resp = client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": self.text},
                ],
                max_completion_tokens=200,
                temperature=0,
            )
            raw = (resp.choices[0].message.content or "").strip()
            # Defensively strip markdown code fences in case the model adds them
            if raw.startswith("```"):
                raw = raw.strip("`").strip()
                if raw.lower().startswith("json"):
                    raw = raw[4:].strip()
            data = json.loads(raw)
            if not isinstance(data, dict) or "intent" not in data:
                raise ValueError("Phản hồi phân loại ý định không đúng định dạng JSON mong đợi")
            self.intent_ready.emit(data)
        except Exception as e:
            self.error_occurred.emit(str(e)[:200])
        finally:
            self.finished.emit()


class ChatPanel(QWidget):
    # Emits log messages so startup_layout can display them in the system log
    log_signal       = pyqtSignal(str)
    # Emits a comma-separated waypoint slot/name list (matching the format
    # expected by WaypointsModeLayout.voice_navigate_to_waypoint), whenever
    # the AI intent classifier decides the voice command was a navigation
    # request rather than a general chat message.
    waypoint_command = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._chat_history  = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._ai_worker     = None
        self._ai_thread     = None
        self._intent_worker = None
        self._intent_thread = None
        self._voice_enabled = False
        self._pending_reply = None
        self._did_speak     = False

        # Optional callbacks wired in by the host layout (e.g. WaypointsModeLayout)
        # to enable voice-driven navigation. When _waypoints_provider is None,
        # ChatPanel behaves exactly as before: every transcript goes to the
        # general-purpose AI chat.
        self._pose_provider      = None
        self._waypoints_provider = None

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

    # ------------------------------------------------------- host wiring ---
    def set_pose_provider(self, provider):
        """provider: zero-arg callable returning the current robot Pose (or
        None if unavailable). Used to freeze the robot's current position at
        the moment a 'return here' voice command is issued, before it starts
        moving toward any other waypoints in the same command."""
        self._pose_provider = provider

    def set_waypoints_provider(self, provider):
        """provider: zero-arg callable returning a list of waypoint
        descriptors for the current map, either plain key strings or dicts
        like {"key": "X5.7", "aliases": [...]}. Passing aliases lets the
        intent classifier recognize whatever phrasing the user says (e.g.
        an alias from waypoints.json) and still resolve it to the correct
        canonical key. Setting this enables voice intent classification
        (navigate vs chat); leaving it unset preserves the original
        behavior of always treating speech as general chat."""
        self._waypoints_provider = provider

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

    # ------------------------------------------------- intent classification
    def _classify_intent(self, text: str):
        if self._intent_thread and self._intent_thread.isRunning():
            return

        waypoints = []
        try:
            waypoints = list(self._waypoints_provider() or [])
        except Exception:
            waypoints = []

        self.voice_status_label.show()
        self._typing_dots = 0
        self._typing_timer.start(400)

        self._intent_worker = _IntentWorker(text, waypoints)
        self._intent_thread = QThread()
        self._intent_worker.moveToThread(self._intent_thread)
        self._intent_thread.started.connect(self._intent_worker.run)
        self._intent_worker.intent_ready.connect(lambda data: self._on_intent_ready(data, text))
        self._intent_worker.error_occurred.connect(lambda err: self._on_intent_error(err, text))
        self._intent_worker.finished.connect(self._intent_thread.quit)
        self._intent_thread.start()

    def _on_intent_ready(self, data: dict, original_text: str):
        self._typing_timer.stop()
        intent    = data.get("intent")
        waypoints = data.get("waypoints") or []

        if intent == "navigate" and waypoints:
            resolved = []
            for w in waypoints:
                if w == "__return_here__":
                    pose = None
                    if self._pose_provider is not None:
                        try:
                            pose = self._pose_provider()
                        except Exception:
                            pose = None
                    if pose is not None:
                        # Freeze the current pose now, before the robot moves
                        resolved.append(
                            f"__return_here__:{pose.position.x};{pose.position.y};"
                            f"{pose.orientation.z};{pose.orientation.w}"
                        )
                    else:
                        # No pose available yet — downstream will try to
                        # capture it live when it's this waypoint's turn.
                        resolved.append("__return_here__")
                else:
                    resolved.append(w)

            self.voice_status_label.hide()
            self.log_signal.emit(f"[Bạn] {original_text}")
            self.waypoint_command.emit(",".join(resolved))
        else:
            # Classifier says this isn't a navigation command — treat as
            # a normal chat message instead.
            self.voice_status_label.hide()
            self._ask_ai(original_text)

    def _on_intent_error(self, error: str, original_text: str):
        self._typing_timer.stop()
        self.voice_status_label.hide()
        print(f"[Intent] classification failed, falling back to chat: {error}")
        self._ask_ai(original_text)

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
        if self._waypoints_provider is not None:
            self._classify_intent(text)
        else:
            self._ask_ai(text)

    # ----------------------------------------------------------- helpers ---

    def cleanup(self):
        if self._voice_enabled:
            self._voice_engine.stop_speaking()