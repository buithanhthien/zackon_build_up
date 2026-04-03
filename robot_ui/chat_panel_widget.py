#!/usr/bin/env python3
import os
import re
import json
from openai import OpenAI
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QLineEdit, QScrollArea, QSizePolicy)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from voice_engine import VoiceEngine, VoiceState


def _load_env():
    env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '.env')
    if not os.path.exists(env_path):
        return
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                k, v = line.split('=', 1)
                os.environ.setdefault(k.strip(), v.strip())

_load_env()
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")


def _load_iuh_database() -> str:
    """Load iuh_database.json and convert to a concise text block for the system prompt."""
    db_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'iuh_database.json')
    try:
        with open(db_path, encoding='utf-8') as f:
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
    ]

    # Campus locations
    lines.append("=== Cơ sở & Phân hiệu ===")
    for key, cs in db.get('co_so', {}).items():
        lines.append(f"  - {cs.get('dia_chi', '')}")
    lines.append("")

    # Contact
    lh = db.get('lien_he', {})
    lines.append("=== Liên hệ ===")
    lines.append(f"  Phòng Đào tạo: {lh.get('phong_dao_tao', '')}")
    lines.append(f"  Tuyển sinh: {lh.get('tuyen_sinh', '')}")
    lines.append(f"  Portal SV: {lh.get('portal_sinh_vien', '')}")
    lines.append("")

    # FEET
    feet = db.get('khoa_cong_nghe_dien', {})
    lines.append(f"=== {feet.get('ten', '')} ===")
    lines.append(f"  Website: {feet.get('website', '')}")
    bl_feet = feet.get('ban_lanh_dao', {})
    lines.append(f"  Trưởng khoa: {bl_feet.get('truong_khoa', '')}")
    lines.append("  Phó trưởng khoa: " + ", ".join(bl_feet.get('pho_truong_khoa', [])))
    lines.append("  Chương trình đào tạo: " + ", ".join(feet.get('chuong_trinh_dao_tao', [])))
    for bm in feet.get('bo_mon', []):
        gv_list = ", ".join(bm.get('giang_vien', []))
        truong = bm.get('truong_bo_mon', '')
        truong_str = f" | Trưởng Bộ môn: {truong}" if truong else ""
        lines.append(f"  Bộ môn {bm['ten']}{truong_str} | GV: {gv_list}")
    lines.append("")

    # ZACKON
    zk = db.get('robot_zackon', {})
    lines.append("=== Robot ZACKON ===")
    lines.append(f"  {zk.get('mo_ta', '')}")
    lines.append("  Công nghệ: " + ", ".join(zk.get('cong_nghe', [])))
    lines.append(f"  {zk.get('lien_quan_den_khoa', '')}")

    return "\n".join(lines)


_IUH_DATABASE_TEXT = _load_iuh_database()
GROQ_MODEL     = "gpt-5.4-nano"
SYSTEM_PROMPT = (
    "Bạn là ZACKON, AI trợ lý tích hợp trong robot ROS 2 của hệ thống Zackon.\n"
    "Luôn trả lời bằng tiếng Việt, rõ ràng, ngắn gọn và thân thiện.\n\n"

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
    "- Waypoint là các vị trí đã lưu trên bản đồ (lưu trong waypoints.json)\n"
    "- Để điều hướng: nhấn nút waypoint trên giao diện HOẶC dùng lệnh giọng nói\n"
    "- Lệnh giọng nói hợp lệ: 'Đi tới <số>', 'Tới vị trí <số>', 'Đi đến số <số>'\n"
    "  Ví dụ: 'Đi tới 3', 'Tới vị trí 5', 'Đi đến số hai'\n"
    "- Chỉ hoạt động khi đang ở Waypoints Mode\n\n"

    "## Chế độ Tracking\n"
    "- Robot dùng camera + YOLOv8 để phát hiện và theo dõi người\n"
    "- Không liên quan đến waypoint hay điều hướng bản đồ\n\n"

    "## Quy tắc hội thoại\n"
    "- Khi hướng dẫn thao tác: chỉ hướng dẫn TỪNG BƯỚC MỘT, chờ xác nhận rồi mới tiếp\n"
    "- Không liệt kê nhiều bước cùng lúc\n"
    "- Nếu người dùng hỏi về waypoint/điểm đến: luôn nhắc họ phải đang ở chế độ 'Điểm đến'\n\n"

    "## Thẻ hành động\n"
    "Chỉ dùng khi cần thực thi trực tiếp trên robot. Tối đa một thẻ mỗi phản hồi. Không dùng khi trò chuyện thông thường.\n"
    "- <STATUS_CHECK>: kiểm tra trạng thái STM32 và LiDAR\n"
    "- <LOCALIZE>: chạy lại định vị (Định vị lại)\n"
    "- <DOCK>: gửi robot về trạm sạc\n"
    "- <NAVIGATE>: bắt đầu điều hướng (dùng kèm Waypoints Mode)\n"
    "- <HELP>: hiển thị hướng dẫn sử dụng\n"

    "## Thông tin về trường IUH (từ cơ sở dữ liệu)\n"
    + _IUH_DATABASE_TEXT + "\n"
    
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
                model=GROQ_MODEL,
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
        self.waypoint_command = self._voice_engine.waypoint_command  # expose for parent

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

        self.interrupt_btn = QPushButton("■")
        self.interrupt_btn.setObjectName("interrupt-btn")
        self.interrupt_btn.setFont(QFont("JetBrains Mono", 11))
        self.interrupt_btn.setFixedWidth(44)
        self.interrupt_btn.setToolTip("Stop speaking")
        self.interrupt_btn.clicked.connect(self._voice_engine.stop_speaking)

        input_layout.addWidget(self.voice_btn)
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
            self._voice_engine.stop()
