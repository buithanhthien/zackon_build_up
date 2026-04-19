#!/usr/bin/env python3
import os
import re
import json
import math
import unicodedata
from openai import OpenAI
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QLineEdit, QScrollArea, QSizePolicy)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from voice_engine import VoiceEngine, VoiceState

_DIR = os.path.dirname(os.path.abspath(__file__))

def _strip_diacritics(s: str) -> str:
    return ''.join(c for c in unicodedata.normalize('NFD', s) if unicodedata.category(c) != 'Mn')

_LOCATION_PATTERNS = re.compile(
    r'(tôi đang ở đâu|tôi đang ở dâu|robot đang ở đâu|vị trí (hiện tại|của tôi|robot))',
    re.IGNORECASE
)
_ACTION_TAG_RE = re.compile(r'<(STATUS_CHECK|HELP|DOCK|LOCALIZE)>')
_NAVIGATE_TAG_RE = re.compile(r'<NAVIGATE:([^>]+)>')
_TOUR_TAG_RE = re.compile(r'<TOUR:([^>]+)>')
_TOUR_PATTERN = re.compile(
    r'(tham quan|tour|dẫn.*tất cả|đi.*tất cả|các phòng trọng điểm|phòng trọng điểm)',
    re.IGNORECASE
)
_FULL_TOUR_PATTERN = re.compile(
    r'(một vòng|một vòng quanh|đi một vòng|đi hết một vòng|tham quan toàn bộ|tham quan tất cả|tham quan hết|tham quan cả khu|đi khắp nơi|đi xung quanh khu vực|dẫn tôi đi một vòng|dẫn đi tham quan toàn bộ)',
    re.IGNORECASE
)
_MULTI_NAV_PATTERN = re.compile(
    r'(sau đó|sau đấy|rồi|rồi thì|tiếp theo|kế tiếp|tiếp tục|tiếp đến|xong rồi|bước tiếp theo|và sau đó)',
    re.IGNORECASE
)
_REVERSE_NAV_PATTERN = re.compile(
    r'(sau khi|trước khi|khi đã|khi xong|làm xong thì|xong thì|sau lúc)',
    re.IGNORECASE
)
_NO_EXEC_PATTERN = re.compile(
    r'\b(không|chỉ|just|only|answer|trả lời|cho biết|là gì|có không|bao nhiêu|mấy|ở đâu|khi nào|tại sao|vì sao|có phải|có đúng|được không|đúng không|hay không)\b',
    re.IGNORECASE
)
_TOUR_WAYPOINTS = ['x5.4', 'x5.10', 'X5.11', 'x5.12']
_RETURN_HERE_PATTERN = re.compile(
    r'(quay trở về đây|quay trở về|quay trở lại đây|quay trở lại|quay lại đây|quay lại|quay về đây|quay về|trở về đây|trở về|về đây|về chỗ cũ|về chỗ ban đầu|về vị trí cũ|về vị trí ban đầu|rồi quay lại|sau đó quay lại|rồi trở về|xong thì quay về)',
    re.IGNORECASE
)
_RETURN_HERE_WAYPOINT_KEY = '__return_here__'

_WAYPOINTS = None

def _load_waypoints():
    global _WAYPOINTS
    if _WAYPOINTS is None:
        try:
            with open(os.path.join(_DIR, 'waypoints.json'), encoding='utf-8') as f:
                _WAYPOINTS = json.load(f)
        except Exception:
            _WAYPOINTS = {}
    return _WAYPOINTS


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


def _load_iuh_database() -> str:
    try:
        with open(os.path.join(_DIR, 'iuh_database.json'), encoding='utf-8') as f:
            db = json.load(f)
    except Exception:
        return "(Không tải được dữ liệu IUH)"

    t = db.get('truong', {})
    lines = [
        f"Tên trường: {t.get('ten_day_du', '')} ({t.get('viet_tat', '')})",
        f"Tiếng Anh: {t.get('ten_tieng_anh', '')}",
        f"Website: {t.get('website', '')}  |  Email: {t.get('email_chinh', '')}  |  ĐT: {t.get('dien_thoai', '')}",
        f"Lịch sử: {t.get('lich_su', '')}",
        f"Tầm nhìn: {t.get('tam_nhin', '')}",
        f"Sứ mạng: {t.get('su_mang', '')}",
        f"Giá trị cốt lõi: {t.get('gia_tri_cot_loi', '')}",
        f"Cơ sở vật chất: {t.get('co_so_vat_chat', '')}",
        "Thành tựu nổi bật: " + "; ".join(t.get('thanh_tuu', [])),
        "",
        "=== Cơ sở & Phân hiệu ===",
    ]
    for cs in db.get('co_so', {}).values():
        lines.append(f"  - {cs.get('dia_chi', '')}")
    
    lh = db.get('lien_he', {})
    lines.extend([
        "",
        "=== Liên hệ ===",
        f"  Phòng Đào tạo: {lh.get('phong_dao_tao', '')}",
        f"  Tuyển sinh: {lh.get('tuyen_sinh', '')}",
        f"  Portal SV: {lh.get('portal_sinh_vien', '')}",
        "",
    ])

    feet = db.get('khoa_cong_nghe_dien', {})
    lines.append(f"=== {feet.get('ten', '')} ===")
    lines.append(f"  Website: {feet.get('website', '')}")
    bl = feet.get('ban_lanh_dao', {})
    lines.append(f"  Trưởng khoa: {bl.get('truong_khoa', '')}")
    lines.append("  Phó trưởng khoa: " + ", ".join(bl.get('pho_truong_khoa', [])))
    lines.append("  Chương trình đào tạo: " + ", ".join(feet.get('chuong_trinh_dao_tao', [])))
    for bm in feet.get('bo_mon', []):
        truong = f" | Trưởng Bộ môn: {bm['truong_bo_mon']}" if bm.get('truong_bo_mon') else ""
        lines.append(f"  Bộ môn {bm['ten']}{truong} | GV: {', '.join(bm.get('giang_vien', []))}")

    zk = db.get('robot_zackon', {})
    lines.extend([
        "",
        "=== Robot ZACKON ===",
        f"  {zk.get('mo_ta', '')}",
        "  Công nghệ: " + ", ".join(zk.get('cong_nghe', [])),
        f"  {zk.get('lien_quan_den_khoa', '')}",
    ])
    return "\n".join(lines)


def _load_waypoint_keys() -> str:
    wps = _load_waypoints()
    if not wps:
        return "(Không tải được danh sách địa điểm)"
    lines = []
    for key, data in wps.items():
        aliases = data.get('aliases', []) if isinstance(data, dict) else []
        alias_str = f" (aliases: {', '.join(aliases)})" if aliases else ""
        lines.append(f"  - {key}{alias_str}")
    return "\n".join(lines)

OPENAI_MODEL = "gpt-5.4-mini"
SYSTEM_PROMPT = (
    "Bạn là ZACKON, AI trợ lý tích hợp trong robot ROS 2 của hệ thống Zackon.\n"
    "Luôn trả lời bằng tiếng Việt, rõ ràng, ngắn gọn và thân thiện.\n"
    "Không sử dụng dấu ngoặc kép (\") trong câu trả lời.\n\n"
    "Hãy trả lời ngắn gọn trong 2 đến 3 câu"
    "Các phòng trọng điểm của khoa điện bao gòm: X5.4 - phòng thí nghiệm robot và đièu khiển thông minh, X5.10 - phòng thực hành plc mitsu, X5.11 - phòng SCADA, X5.12 - phòng lập trình plc siemen\n"

    "## Kiến trúc hệ thống\n"
    "Giao diện chính (startup_layout) có thanh bên trái với các nút:\n"
    "- 'Tải bản đồ': tải file bản đồ (.pgm/.yaml) đã có\n"
    "- 'Bản đồ mới': vào chế độ lập bản đồ SLAM\n"
    "- 'Định vị lại': chạy AMCL để robot tự xác định vị trí trên bản đồ\n"
    "- 'Theo dõi' (Tracking Mode): robot theo dõi người dùng qua camera\n"
    "- 'Điểm đến' (Waypoints Mode): robot tự điều hướng đến các waypoint đã lưu\n"
    "- 'Về trạm sạc' (Docking): robot tự về trạm sạc qua Nav2 + DockRobot action\n"
    "- 'Nav2': chế độ điều hướng thủ công qua Nav2\n"
    "Góc trên hiển thị 2 thẻ trạng thái: STM32 (vi điều khiển) và LiDAR — phải Available trước khi dùng.\n\n"

    "## Quy trình khởi động chuẩn (hướng dẫn TỪNG BƯỚC MỘT, chờ xác nhận)\n"
    "Bước 1: Kiểm tra STM32 và LiDAR ở góc trên — cả hai phải hiển thị Available\n"
    "Bước 2: Nhấn 'Tải bản đồ' để chọn bản đồ\n"
    "  → Nếu chưa có bản đồ: nhấn 'Bản đồ mới' → nhấn 'Bắt đầu lập bản đồ' → lái robot khám phá khu vực → nhập tên bản đồ → nhấn 'Áp dụng' → quay lại màn hình chính → tải bản đồ vừa tạo\n"
    "Bước 3: Nhấn 'Định vị lại' — robot sẽ tự xoay để xác định vị trí\n"
    "Bước 4: Chọn chế độ vận hành phù hợp\n\n"

    "## Chế độ Waypoints\n"
    "- Địa điểm (waypoint) là các vị trí đã lưu trên bản đồ (lưu trong waypoints.json)\n"
    "- Khi người dùng yêu cầu đi đến một địa điểm: NGAY LẬP TỨC thực hiện, KHÔNG hỏi thêm bất kỳ điều gì\n"
    "- Hệ thống tự động chuyển sang Waypoints Mode nếu cần — người dùng không cần làm gì thêm\n"
    "- Lệnh điều hướng hợp lệ: 'Đi tới <tên>', 'Tới <tên>', 'Dẫn tôi đến <tên>'\n"
    "  Ví dụ: 'Đi tới Cua phong X5.4', 'Tới phòng hội thảo', 'Đi đến X5.11'\n"
    "- Có thể dùng tên đầy đủ hoặc tên viết tắt (aliases) của địa điểm\n"
    "- Khi nói về các vị trí đã lưu, LUÔN dùng từ 'địa điểm' thay vì 'waypoint'\n\n"

    "## Chế độ Tracking\n"
    "- Robot dùng camera + YOLOv8 để phát hiện và theo dõi người\n"
    "- Không liên quan đến waypoint hay điều hướng bản đồ\n\n"

    "## Quy tắc hội thoại\n"
    "- Khi hướng dẫn thao tác: chỉ hướng dẫn TỪNG BƯỚC MỘT, chờ xác nhận rồi mới tiếp\n"
    "- Không liệt kê nhiều bước cùng lúc\n"
    "- Khi người dùng muốn đi đến một địa điểm: THỰC HIỆN NGAY, không hỏi xác nhận, không hỏi về chế độ hiện tại\n\n"

    "## Thẻ hành động\n"
    "Chỉ dùng khi cần thực thi trực tiếp trên robot. Tối đa một thẻ mỗi phản hồi. Không dùng khi trò chuyện thông thường.\n"
    "- <STATUS_CHECK>: kiểm tra trạng thái STM32 và LiDAR\n"
    "- <LOCALIZE>: chạy lại định vị (Định vị lại)\n"
    "- <DOCK>: gửi robot về trạm sạc\n"
    "- <NAVIGATE:key>: điều hướng đến MỘT địa điểm. key là tên chính xác trong waypoints.json. Ví dụ: 'Đang dẫn bạn tới X5.4! <NAVIGATE:x5.4>'\n"
    "- <TOUR:key1,key2,key3,...>: điều hướng lần lượt qua NHIỀU địa điểm theo thứ tự. Dùng khi người dùng muốn đi nhiều nơi. Ví dụ: 'Bắt đầu tham quan! <TOUR:x5.11,x5.4,x5.12>'\n"
    "- <HELP>: hiển thị hướng dẫn sử dụng\n"

    "## Danh sách địa điểm đã lưu (dùng key chính xác trong <NAVIGATE:key>)\n"
    + _load_waypoint_keys() + "\n\n"

    "## Thông tin về trường IUH (từ cơ sở dữ liệu)\n"
    + _load_iuh_database() + "\n"
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
            label.setStyleSheet("background-color:#e8f0ff;color:#1a2a5e;border:1px solid #c8d4f0;border-radius:8px;padding:10px 14px;")
            layout.addStretch()
            layout.addWidget(label)
        else:
            label.setStyleSheet("background-color:#ffffff;color:#1a2a5e;border:1px solid #c8d4f0;border-radius:8px;padding:10px 14px;")
            layout.addWidget(label)
            layout.addStretch()


class ChatPanel(QWidget):
    action_tag = pyqtSignal(str)
    waypoint_command = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._chat_history = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._ai_worker = None
        self._ai_thread = None
        self._voice_enabled = False
        self._voice_engine = VoiceEngine()
        self._voice_engine.state_changed.connect(self._on_voice_state_changed)
        self._voice_engine.transcript_ready.connect(self._on_voice_transcript)
        self._pose_provider = None
        self._pending_response = None
        self._return_here_pose = None

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
        self._messages_widget.setStyleSheet("background-color:#f8faff;")
        self._messages_layout = QVBoxLayout(self._messages_widget)
        self._messages_layout.setContentsMargins(16, 16, 16, 16)
        self._messages_layout.setSpacing(8)
        self._messages_layout.addStretch()

        self.chat_scroll.setWidget(self._messages_widget)
        layout.addWidget(self.chat_scroll, 1)

        self.typing_label = QLabel("ZACKON is thinking...")
        self.typing_label.setFont(QFont("Fira Code", 11))
        self.typing_label.setStyleSheet("color:#8fa3cc;padding:4px 20px;background-color:#f8faff;")
        self.typing_label.hide()
        layout.addWidget(self.typing_label)

        self.voice_status_label = QLabel("")
        self.voice_status_label.setFont(QFont("Fira Code", 11, QFont.Weight.Bold))
        self.voice_status_label.setStyleSheet("color:#5a7abf;padding:0 20px;background-color:#f8faff;")
        self.voice_status_label.hide()
        layout.addWidget(self.voice_status_label)

        input_row = QWidget()
        input_row.setStyleSheet("background-color:#ffffff;border-top:1px solid #c8d4f0;")
        input_layout = QHBoxLayout(input_row)
        input_layout.setContentsMargins(16, 10, 16, 10)
        input_layout.setSpacing(10)

        self.voice_btn = QPushButton("🎤")
        self.voice_btn.setObjectName("voice-btn")
        self.voice_btn.setCheckable(True)
        self.voice_btn.setFixedSize(64, 64)
        self.voice_btn.setToolTip("Click to toggle voice")
        self.voice_btn.clicked.connect(self._on_listen_btn_clicked)

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

        self.interrupt_btn = QPushButton("■")
        self.interrupt_btn.setObjectName("interrupt-btn")
        self.interrupt_btn.setFont(QFont("JetBrains Mono", 11))
        self.interrupt_btn.setFixedWidth(44)
        self.interrupt_btn.setToolTip("Stop speaking")
        self.interrupt_btn.clicked.connect(self._voice_engine.stop_speaking)

        input_layout.addWidget(self.chat_input)
        input_layout.addWidget(self.send_btn)
        input_layout.addWidget(self.clear_btn)
        input_layout.addWidget(self.interrupt_btn)
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

    def set_pose_provider(self, provider):
        self._pose_provider = provider

    def _pose_context(self) -> str | None:
        if not self._pose_provider:
            return None
        pose = self._pose_provider()
        if pose is None:
            return None
        x, y = pose.position.x, pose.position.y

        waypoints = _load_waypoints()
        nearest_id, nearest_dist = None, float('inf')
        for wp_id, wp in waypoints.items():
            if 'x' in wp and 'y' in wp:
                d = math.hypot(x - wp['x'], y - wp['y'])
                if d < nearest_dist:
                    nearest_dist, nearest_id = d, wp_id

        if nearest_id and nearest_dist <= 2.0:
            return f"Robot đang ở gần vị trí số {nearest_id} (cách {nearest_dist:.1f} m)."
        elif nearest_id:
            return f"Robot đang ở x={x:.2f} m, y={y:.2f} m. Điểm đến gần nhất là vị trí số {nearest_id} nhưng còn cách khá xa ({nearest_dist:.1f} m)."
        return f"Vị trí hiện tại của robot: x={x:.2f} m, y={y:.2f} m. Chưa có waypoint nào được lưu."

    def send_message(self):
        text = self.chat_input.text().strip()
        if not text or (self._ai_thread and self._ai_thread.isRunning()):
            return
        self.chat_input.clear()
        self._add_bubble(text, "user")
        print(f"[CHAT] User: {text}")

        if _FULL_TOUR_PATTERN.search(text) and not _NO_EXEC_PATTERN.search(text):
            wps = _load_waypoints()
            all_keys = [k for k in wps if wps[k].get('x', 0) != 0 or wps[k].get('y', 0) != 0]
            # put x5.4 at the end as return point
            home = next((k for k in all_keys if k.lower() == 'x5.4'), None)
            if home:
                all_keys = [k for k in all_keys if k.lower() != 'x5.4'] + [home]
            reply = f"Đang dẫn bạn tham quan toàn bộ khoa điện, sau đó quay về X5.4!"
            self._chat_history.append({"role": "user", "content": text})
            self._chat_history.append({"role": "assistant", "content": reply})
            if self._voice_enabled:
                self._voice_engine.speak(reply)
            self._show_response(reply, [], all_keys)
            return

        if _TOUR_PATTERN.search(text) and not _NO_EXEC_PATTERN.search(text):
            reply = "Đang dẫn bạn tham quan lần lượt các phòng trọng điểm của khoa điện!"
            self._chat_history.append({"role": "user", "content": text})
            self._chat_history.append({"role": "assistant", "content": reply})
            if self._voice_enabled:
                self._voice_engine.speak(reply)
            self._show_response(reply, [], _TOUR_WAYPOINTS)
            return

        if (_MULTI_NAV_PATTERN.search(text) or _REVERSE_NAV_PATTERN.search(text) or _RETURN_HERE_PATTERN.search(text)) and not _NO_EXEC_PATTERN.search(text):
            wps = _load_waypoints()
            # Match waypoints by checking key and aliases, preserving order of appearance
            found = []
            text_lower = text.lower()
            # Normalize STT artifacts: "x 5.12" → "x5.12", "x 5 12" → "x5.12", "5,12" → "5.12"
            text_lower = re.sub(r'(\d),(\d)', r'\1.\2', text_lower)
            text_lower = re.sub(r'\bx\s+(\d)', r'x\1', text_lower)
            text_lower = re.sub(r'(\d)\s+(\d)', r'\1.\2', text_lower)
            text_plain = _strip_diacritics(text_lower)
            for k, data in wps.items():
                aliases = data.get('aliases', []) if isinstance(data, dict) else []
                candidates = [k] + aliases
                for c in candidates:
                    # Use word boundary for numeric keys to avoid substring matches
                    if c.isdigit():
                        pattern = r'\b' + re.escape(c) + r'\b'
                        if re.search(pattern, text_lower, re.IGNORECASE):
                            if k not in found:
                                found.append(k)
                            break
                    else:
                        # Try exact match first, then diacritic-insensitive fallback
                        if re.search(re.escape(c), text_lower, re.IGNORECASE) or \
                           re.search(re.escape(_strip_diacritics(c.lower())), text_plain, re.IGNORECASE):
                            if k not in found:
                                found.append(k)
                            break

            # Snapshot current pose as return point if user said "quay lại đây"
            return_here = bool(_RETURN_HERE_PATTERN.search(text))
            if return_here:
                pose = self._pose_provider() if self._pose_provider else None
                if pose:
                    self._return_here_pose = (pose.position.x, pose.position.y)
                    o = pose.orientation
                    found.append(f'{_RETURN_HERE_WAYPOINT_KEY}:{pose.position.x:.4f};{pose.position.y:.4f};{o.z:.4f};{o.w:.4f}')
                else:
                    # No pose available — use a sentinel so we still intercept the request
                    found.append(_RETURN_HERE_WAYPOINT_KEY)

            if len(found) >= 2 or (return_here and len(found) >= 1):
                if _REVERSE_NAV_PATTERN.search(text):
                    found = list(reversed(found))
                display = [k for k in found if not k.startswith(_RETURN_HERE_WAYPOINT_KEY)]
                suffix = " rồi quay lại vị trí hiện tại của bạn" if return_here else ""
                reply = f"Đang dẫn bạn lần lượt tới {', '.join(display)}{suffix}!"
                self._chat_history.append({"role": "user", "content": text})
                self._chat_history.append({"role": "assistant", "content": reply})
                if self._voice_enabled:
                    self._voice_engine.speak(reply)
                self._show_response(reply, [], found)
                return

        self._chat_history.append({"role": "user", "content": text})

        history = list(self._chat_history)
        if _LOCATION_PATTERNS.search(text):
            pose_ctx = self._pose_context()
            if pose_ctx:
                history.append({"role": "system", "content": pose_ctx})
            else:
                history.append({"role": "system", "content": "Chưa nhận được dữ liệu vị trí từ AMCL. Robot có thể chưa được định vị."})

        self.typing_label.show()
        self._typing_dots = 0
        self._typing_timer.start(400)
        self.send_btn.setEnabled(False)
        self.chat_input.setEnabled(False)

        self._ai_worker = _AIChatWorker(history)
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
        tags = _ACTION_TAG_RE.findall(reply)
        nav_keys = _NAVIGATE_TAG_RE.findall(reply)
        tour_match = _TOUR_TAG_RE.search(reply)
        tour_keys = [k.strip() for k in tour_match.group(1).split(',')] if tour_match else []
        clean = _ACTION_TAG_RE.sub('', reply)
        clean = _NAVIGATE_TAG_RE.sub('', clean)
        clean = _TOUR_TAG_RE.sub('', clean).strip()
        return clean, tags, nav_keys, tour_keys

    def _fallback_nav_key(self, text: str) -> list:
        """If AI forgot the <NAVIGATE:key> tag, try to find a waypoint key in the text.
        Skips numeric-only keys. Matches longest candidate first to avoid substring collisions."""
        wps = _load_waypoints()
        text_lower = text.lower()
        # Build (candidate_str, key) pairs, skip pure-numeric keys, sort longest first
        pairs = []
        for key, data in wps.items():
            if key.isdigit():
                continue
            aliases = data.get('aliases', []) if isinstance(data, dict) else []
            for c in [key] + aliases:
                pairs.append((c, key))
        pairs.sort(key=lambda p: len(p[0]), reverse=True)
        text_plain = _strip_diacritics(text_lower)
        for c, key in pairs:
            c_lower = c.lower()
            if c_lower in text_lower or _strip_diacritics(c_lower) in text_plain:
                return [key]
        return []

    def _on_response(self, reply):
        clean, tags, nav_keys, tour_keys = self._strip_tags(reply)

        # Fallback: AI said it's navigating but forgot the tag
        if not nav_keys and not tour_keys:
            _NAV_INTENT = re.compile(
                r'(đang dẫn|dẫn bạn|đi tới|điều hướng|navigate|going to)',
                re.IGNORECASE
            )
            if _NAV_INTENT.search(clean):
                nav_keys = self._fallback_nav_key(clean)
                if nav_keys:
                    print(f"[CHAT] Fallback nav key resolved: {nav_keys}")

        self._chat_history.append({"role": "assistant", "content": clean})
        print(f"[CHAT] Assistant: {clean}")
        if nav_keys:
            print(f"[CHAT] Navigation triggered: {nav_keys}")
        if tour_keys:
            print(f"[CHAT] Tour triggered: {tour_keys}")
        if self._voice_enabled:
            self._pending_response = (clean, tags, nav_keys, tour_keys)
            self._voice_engine.speak(clean)
        else:
            self._show_response(clean, tags, nav_keys, tour_keys)

    def _show_response(self, clean, tags, nav_keys, tour_keys=None):
        self._add_bubble(clean, "assistant")
        all_keys = tour_keys if tour_keys else nav_keys
        if all_keys and self._voice_enabled:
            QTimer.singleShot(100, lambda: self._emit_waypoint_after_speech(all_keys))
        elif all_keys:
            self.waypoint_command.emit(",".join(k.strip() for k in all_keys))
        for tag in tags:
            self.action_tag.emit(tag)
        self.typing_label.hide()
        self._typing_timer.stop()
        self.send_btn.setEnabled(True)
        self.chat_input.setEnabled(True)
        self.chat_input.setFocus()

    def _emit_waypoint_after_speech(self, keys):
        if self._voice_engine._tts_queue.empty():
            self.waypoint_command.emit(",".join(k.strip() for k in keys))
        else:
            QTimer.singleShot(500, lambda: self._emit_waypoint_after_speech(keys))

    def _on_error(self, error):
        self._add_bubble(f"[ERROR] {error}", "assistant")
        self.typing_label.hide()
        self._typing_timer.stop()
        self.send_btn.setEnabled(True)
        self.chat_input.setEnabled(True)

    def _on_done(self):
        pass

    def _on_listen_btn_clicked(self):
        if not self.voice_btn.isChecked():
            self._voice_enabled = False
            self._voice_engine.stop_speaking()
            self.voice_status_label.hide()
            return
        self._voice_enabled = True
        self.voice_status_label.show()
        self._voice_engine.listen_once()

    def _on_voice_state_changed(self, state):
        # Show response when TTS starts speaking
        if "SPEAKING" in state and self._pending_response:
            clean, tags, nav_keys, tour_keys = self._pending_response
            self._pending_response = None
            self._show_response(clean, tags, nav_keys, tour_keys)

        if not state:
            self._voice_enabled = False
            self.voice_btn.setChecked(False)
            self.voice_status_label.hide()
            return

        self.voice_status_label.show()
        self.voice_status_label.setText(state)
        if "LISTENING" in state:
            self.voice_status_label.setStyleSheet("color:#214196;padding:0 20px;background-color:#f8faff;")
        elif "SPEAKING" in state:
            self.voice_status_label.setStyleSheet("color:#22c55e;padding:0 20px;background-color:#f8faff;")
        else:
            self.voice_status_label.setStyleSheet("color:#5a7abf;padding:0 20px;background-color:#f8faff;")

    def _on_voice_transcript(self, text):
        text = text.strip().capitalize()
        if not text:
            return
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
            self._voice_engine.stop_speaking()
