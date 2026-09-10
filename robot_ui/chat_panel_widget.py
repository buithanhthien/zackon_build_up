#!/usr/bin/env python3
import json
import os
import unicodedata
import re
import sys

from openai import OpenAI
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from voice_engine import VoiceEngine

_DIR = os.path.dirname(os.path.abspath(__file__))

IUH_DATABASE_PATH = os.path.join(
    _DIR,
    "iuh_database.json"
)

def _remove_vietnamese_accents(text: str) -> str:
    text = text.lower().replace("đ", "d")
    text = unicodedata.normalize("NFD", text)

    return "".join(
        c for c in text
        if unicodedata.category(c) != "Mn"
    )

def _vietnamese_number_to_int(words: str):
    """
    Chuyển một số cách STT tiếng Việt thường trả về thành số.
    Dùng cho số phòng: 1 -> 99.
    """

    words = words.strip()

    if words.isdigit():
        return int(words)

    unit = {
        "khong": 0,
        "mot": 1,
        "mốt": 1,
        "hai": 2,
        "ba": 3,
        "bon": 4,
        "tu": 4,
        "nam": 5,
        "lam": 5,
        "sau": 6,
        "bay": 7,
        "tam": 8,
        "chin": 9,
    }

    tokens = words.split()

    if len(tokens) == 1:
        return unit.get(tokens[0])

    # Ví dụ:
    # muoi -> 10
    # muoi mot -> 11
    # muoi bon -> 14
    # hai muoi -> 20
    # hai muoi mot -> 21

    if tokens[0] == "muoi":
        if len(tokens) == 1:
            return 10

        last = unit.get(tokens[1])
        if last is not None:
            return 10 + last

    if len(tokens) >= 2 and tokens[1] == "muoi":
        tens = unit.get(tokens[0])

        if tens is None:
            return None

        value = tens * 10

        if len(tokens) >= 3:
            last = unit.get(tokens[2])

            if last is None:
                return None

            value += last

        return value

    return None

def _normalize_room_names(text: str) -> str:
    """
    Chuẩn hóa tên phòng do STT nhận dạng.

    Ví dụ:
    X NĂM CHẤM BỐN
        -> x5.4

    NĂM CHẤM BỐN
        -> x5.4

    X NĂM CHẤM BA CHẤM BA
        -> x5.3.3

    NĂM CHẤM BA CHẤM BA
        -> x5.3.3

    NĂM CHẤM HAI CHẤM BỐN
        -> x5.2.4
    """

    original = text

    normalized = _remove_vietnamese_accents(text)

    normalized = re.sub(
        r"\s+",
        " ",
        normalized
    ).strip()

    number_pattern = (
        r"(?:"
        r"\d+|"
        r"khong|mot|hai|ba|bon|tu|nam|lam|sau|bay|tam|chin|"
        r"muoi(?:\s+(?:mot|hai|ba|bon|tu|nam|lam|sau|bay|tam|chin))?|"
        r"(?:hai|ba|bon|nam|sau|bay|tam|chin)\s+muoi"
        r"(?:\s+(?:mot|hai|ba|bon|tu|nam|lam|sau|bay|tam|chin))?"
        r")"
    )

    # ============================================================
    # 1. Dạng X5.3-3
    #
    # STT có thể nghe:
    #
    # X NĂM CHẤM BA CHẤM BA
    # NĂM CHẤM BA CHẤM BA
    #
    # -> x5.3-3
    # ============================================================

    three_level_pattern = re.compile(
        rf"\b(?:x\s*)?"
        rf"({number_pattern})"
        rf"\s*(?:cham|\.)\s*"
        rf"({number_pattern})"
        rf"\s*(?:cham|\.|-)\s*"
        rf"({number_pattern})\b",
        re.IGNORECASE
    )

    def replace_three_level(match):

        first = _vietnamese_number_to_int(
            match.group(1)
        )

        second = _vietnamese_number_to_int(
            match.group(2)
        )

        third = _vietnamese_number_to_int(
            match.group(3)
        )

        if (
            first is None
            or second is None
            or third is None
        ):
            return match.group(0)

        # Các waypoint của hệ thống đang thuộc nhà X5.
        # Chỉ tự thêm X khi số đầu là 5.
        if first != 5:
            return match.group(0)

        return f"x{first}.{second}.{third}"

    normalized = three_level_pattern.sub(
        replace_three_level,
        normalized
    )

    # ============================================================
    # 2. Dạng X5.4
    #
    # X NĂM CHẤM BỐN
    # NĂM CHẤM BỐN
    #
    # -> x5.4
    # ============================================================

    two_level_pattern = re.compile(
        rf"\b(?:x\s*)?"
        rf"({number_pattern})"
        rf"\s*(?:cham|\.)\s*"
        rf"({number_pattern})\b",
        re.IGNORECASE
    )

    def replace_two_level(match):

        first = _vietnamese_number_to_int(
            match.group(1)
        )

        second = _vietnamese_number_to_int(
            match.group(2)
        )

        if (
            first is None
            or second is None
        ):
            return match.group(0)

        # Chỉ tự suy luận chữ X khi đang nói về X5
        if first != 5:
            return match.group(0)

        return f"x{first}.{second}"

    normalized = two_level_pattern.sub(
        replace_two_level,
        normalized
    )

    print(
        f"[VOICE NORMALIZE] "
        f"{original} -> {normalized}"
    )

    return normalized

def _load_iuh_database():
    try:
        with open(
            IUH_DATABASE_PATH,
            "r",
            encoding="utf-8"
        ) as f:
            data = json.load(f)

        print(
            f"[CHAT] Loaded IUH database: "
            f"{IUH_DATABASE_PATH}"
        )

        return data

    except Exception as e:
        print(
            f"[CHAT] Failed to load IUH database: {e}"
        )
        return {}


IUH_DATABASE = _load_iuh_database()

IUH_DATABASE_TEXT = json.dumps(
    IUH_DATABASE,
    ensure_ascii=False,
    indent=2
)


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
    "Bạn là Bé Son.\n"

    "Luôn trả lời bằng tiếng Việt, rõ ràng, ngắn gọn, "
    "hài hước và tự nhiên.\n"

    "Bạn là robot trợ lý tại Đại học Công nghiệp "
    "Thành phố Hồ Chí Minh (IUH).\n"

    "Khi người dùng hỏi về IUH, Khoa Công nghệ Điện, "
    "giảng viên, chuyên ngành, cơ sở, địa chỉ, liên hệ, "
    "lịch sử hoặc các thông tin có trong CƠ SỞ DỮ LIỆU IUH "
    "bên dưới, hãy ưu tiên sử dụng dữ liệu này.\n"

    "Không được tự bịa thông tin không tồn tại trong database.\n"

    "Nếu database không chứa câu trả lời, hãy nói ngắn gọn "
    "rằng bạn chưa có thông tin đó thay vì đoán.\n"

    "Không sử dụng dấu ngoặc kép trong câu trả lời.\n"
    "Hãy trả lời ngắn gọn trong 2 đến 3 câu.\n"
    "Dùng ngôn ngữ đời thường.\n"

    "\n"
    "===== CƠ SỞ DỮ LIỆU IUH =====\n"
    + IUH_DATABASE_TEXT +
    "\n===== KẾT THÚC CƠ SỞ DỮ LIỆU IUH =====\n"
)

# ── Intent classification (voice navigation) ──────────────────────────────
INTENT_SYSTEM_PROMPT_TEMPLATE = (
    "Bạn là bộ phân loại ý định cho robot Bé Son trong một tòa nhà.\n"

    "Danh sách địa điểm hợp lệ hiện tại, định dạng "
    "\"TÊN_CHÍNH_XÁC: cách gọi khác 1, cách gọi khác 2, ...\":\n"
    "{waypoint_list}\n"

    "Người dùng có thể gọi một địa điểm bằng TÊN_CHÍNH_XÁC hoặc bất kỳ "
    "cách gọi khác nào trong danh sách. Khi trả kết quả, luôn sử dụng "
    "đúng TÊN_CHÍNH_XÁC.\n"

    "QUY TẮC PHÂN LOẠI:\n"

    "1. Chỉ trả intent \"navigate\" khi người dùng thực sự RA LỆNH "
    "cho robot di chuyển đến một hoặc nhiều địa điểm.\n"

    "2. Việc chỉ nhắc đến tên địa điểm KHÔNG phải lệnh điều hướng.\n"

    "3. Các câu hỏi về địa điểm, kể chuyện, mô tả, chào hỏi hoặc nói "
    "đang ở một địa điểm phải là intent \"chat\".\n"

    "Ví dụ CHAT:\n"
    "- Bé Son, phòng X5.7 ở đâu?\n"
    "- Bé Son, hôm nay tôi học ở X5.7.\n"
    "- Bé Son, phòng SCADA có mở cửa không?\n"
    "- Bé Son, cô Tâm có ở phòng không?\n"
    "- Bé Son, tôi vừa đi ngang X5.11.\n"

    "4. Các câu thể hiện rõ yêu cầu di chuyển mới là navigate.\n"

    "Ví dụ NAVIGATE:\n"
    "- Bé Son, đi tới X5.7.\n"
    "- Bé Son, đưa tôi đến phòng SCADA.\n"
    "- Bé Son, dẫn tôi tới gặp cô Tâm.\n"
    "- Bé Son, hãy tới phòng X5.11.\n"

    "5. Nếu câu có ý phủ định việc di chuyển như "
    "\"đừng đi\", \"không đi\", \"không cần tới\", "
    "\"đừng đến\" thì KHÔNG được trả navigate.\n"

    "6. Nếu người dùng yêu cầu đi qua nhiều địa điểm, trả các waypoint "
    "theo đúng thứ tự được yêu cầu.\n"

    "7. Nếu người dùng muốn quay lại vị trí hiện tại sau đó, sử dụng "
    "đúng placeholder __return_here__ tại vị trí tương ứng trong danh sách.\n"

    "Nếu là lệnh điều hướng, trả:\n"
    '{{"intent": "navigate", "waypoints": ["TEN_CHINH_XAC_1", "TEN_CHINH_XAC_2"]}}\n'

    "Nếu không phải lệnh điều hướng, trả:\n"
    '{{"intent": "chat", "waypoints": []}}\n'

    "CHỈ trả về một object JSON hợp lệ. "
    "Không giải thích, không markdown, không thêm văn bản khác."
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

    navigation_stop = pyqtSignal()

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
        self.voice_status_label.setText(f"Bé Son đang suy nghĩ{dots}")
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
        self.log_signal.emit(f"[Bé Son] {reply}")
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

        text = text.strip()

        if not text:
            return

        print(f"[CHAT] User: {text}")

        normalized = _normalize_room_names(text)

        # ============================================================
        # 1. LỆNH DỪNG
        #
        # Vì đây là lệnh an toàn nên KHÔNG bắt buộc phải nói "Bé Son".
        # ============================================================

        stop_commands = [
            "dừng lại",
            "dừng robot",
            "dừng xe",
            "hủy lệnh",
            "hủy hành trình",
            "dừng hành trình",
            "stop",
        ]

        if any(
            cmd in normalized
            for cmd in stop_commands
        ):

            self.log_signal.emit(
                "[VOICE] Phát hiện lệnh dừng"
            )

            # Dừng Bé Son nói
            self._voice_engine.stop_speaking()

            # Dừng navigation
            self.navigation_stop.emit()

            return

        # ============================================================
        # 2. WAKE WORD
        # ============================================================

        wake_words = [
            "bé son",
            "be son",
            "bé sơn",
            "be sơn",
        ]

        has_wake_word = any(
            wake in normalized
            for wake in wake_words
        )

        # ============================================================
        # 3. KHÔNG GỌI "BÉ SON"
        #
        # Không bao giờ được gửi sang navigation classifier.
        # Chỉ coi là trò chuyện bình thường.
        # ============================================================

        if not has_wake_word:

            self.log_signal.emit(
                "[VOICE] Không có wake word Bé Son -> không navigation"
            )

            self._ask_ai(text)

            return

        # ============================================================
        # 4. CÓ GỌI "BÉ SON"
        #
        # Lúc này mới kiểm tra xem là:
        # - navigate
        # - hay chat
        # ============================================================

        if self._waypoints_provider is not None:

            self._classify_intent(normalized)

        else:

            self._ask_ai(text)

    # ----------------------------------------------------------- helpers ---

    def cleanup(self):
        if self._voice_enabled:
            self._voice_engine.stop_speaking()