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
from language_config import get_language

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
    # 1. Dạng X5.3.3
    #
    # STT có thể nghe:
    #
    # X NĂM CHẤM BA CHẤM BA
    # NĂM CHẤM BA CHẤM BA
    #
    # -> x5.3.3
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

def get_system_prompt(language):

    # ============================================================
    # English
    # ============================================================

    if language == "en":

        return (
            "You are Be Son.\n"
            "You are an assistant robot at the "
            "Industrial University of Ho Chi Minh City (IUH).\n"
            "Always answer in English clearly, naturally, "
            "briefly, and in a friendly manner.\n"
            "When the user asks about IUH, the Faculty of "
            "Electrical Engineering Technology, lecturers, "
            "academic programs, campuses, addresses, contacts, "
            "history, or information contained in the IUH DATABASE "
            "below, prioritize this database.\n"
            "Do not invent IUH information that is not contained "
            "in the database.\n"
            "If the database does not contain enough information, "
            "the system may use Web Search before answering.\n"
            "Do not use quotation marks unnecessarily.\n"
            "Keep answers concise, normally 2 to 4 sentences.\n"
            "Use natural spoken language suitable for a robot.\n"
            "\n"
            "===== IUH DATABASE =====\n"
            + IUH_DATABASE_TEXT +
            "\n===== END IUH DATABASE =====\n"
        )

    # ============================================================
    # Tiếng Việt
    # ============================================================

    return (
        "Bạn là Bé Son.\n"
        "Bạn là robot trợ lý tại Đại học Công nghiệp "
        "Thành phố Hồ Chí Minh (IUH).\n"
        "Luôn trả lời bằng tiếng Việt, rõ ràng, ngắn gọn, "
        "hài hước và tự nhiên.\n"
        "Khi người dùng hỏi về IUH, Khoa Công nghệ Điện, "
        "giảng viên, chuyên ngành, cơ sở, địa chỉ, liên hệ, "
        "lịch sử hoặc các thông tin có trong CƠ SỞ DỮ LIỆU IUH "
        "bên dưới, hãy ưu tiên sử dụng dữ liệu này.\n"
        "Không được tự bịa thông tin về IUH không tồn tại "
        "trong database.\n"
        "Nếu câu hỏi không có đủ thông tin trong database, "
        "hệ thống có thể sử dụng Web Search trước khi trả lời.\n"
        "Không sử dụng dấu ngoặc kép không cần thiết.\n"
        "Hãy trả lời ngắn gọn khoảng 2 đến 4 câu.\n"
        "Dùng ngôn ngữ đời thường, phù hợp để robot nói.\n"
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

    def __init__(self, history, language):

        super().__init__()

        self.history = history
        self.language = language

    def _response_language_instruction(self):

        if self.language == "en":

            return (
                "Answer ONLY in English. "
                "Do NOT answer in Vietnamese, even if previous "
                "conversation messages are in Vietnamese."
            )

        return (
            "Chỉ trả lời bằng tiếng Việt. "
            "Không trả lời bằng tiếng Anh, kể cả khi lịch sử "
            "hội thoại trước đó có tiếng Anh."
        )

    # ============================================================
    # Lấy câu hỏi gần nhất của người dùng
    # ============================================================

    def _get_latest_user_question(self):

        for message in reversed(self.history):

            if message.get("role") == "user":

                return str(
                    message.get("content", "")
                ).strip()

        return ""

     # ============================================================
    # Lấy ngữ cảnh hội thoại gần đây
    # Không lấy system prompt/database vì quá dài
    # ============================================================

    def _get_recent_context(self):

        lines = []

        for message in self.history[-8:]:

            role = message.get("role")

            if role == "system":
                continue

            content = str(
                message.get("content", "")
            ).strip()

            if not content:
                continue

            if role == "user":
                prefix = "Người dùng"

            elif role == "assistant":
                prefix = "Bé Son"

            else:
                continue

            lines.append(
                f"{prefix}: {content}"
            )

        return "\n".join(lines)

    # ============================================================
    # BƯỚC 1
    #
    # Kiểm tra xem IUH database có đủ dữ liệu hay không.
    #
    # Nếu có:
    # {
    #     "source": "database",
    #     "answer": "..."
    # }
    #
    # Nếu không:
    # {
    #     "source": "web",
    #     "answer": ""
    # }
    # ============================================================

    def _check_database(
        self,
        client,
        question,
        context
    ):
        language_instruction = (
            self._response_language_instruction()
        )

        prompt = f"""
            Bạn là bộ kiểm tra dữ liệu cho robot Bé Son.

            Nhiệm vụ:

            Kiểm tra xem CƠ SỞ DỮ LIỆU IUH bên dưới có đủ thông tin
            để trả lời câu hỏi của người dùng hay không.

            QUY TẮC RẤT QUAN TRỌNG:

            1. Chỉ sử dụng dữ liệu có trong CƠ SỞ DỮ LIỆU IUH.

            2. Không được dùng kiến thức riêng của mô hình để giả vờ
            rằng thông tin có trong database.

            3. Nếu database có đủ thông tin:
            trả về:

            {{
                "source": "database",
                "answer": "câu trả lời"
            }}

            4. Nếu database không có hoặc không đủ thông tin:
            trả về:

            {{
                "source": "web",
                "answer": ""
            }}

            5. Nếu câu hỏi là kiến thức chung không liên quan tới IUH,
            ví dụ:
            - danh lam thắng cảnh Việt Nam
            - lịch sử thế giới
            - khoa học
            - công nghệ
            - thể thao

            thì database IUH không chứa dữ liệu phù hợp,
            vì vậy phải chọn "web".

            6. Nếu câu hỏi yêu cầu thông tin hiện tại như:
            - hôm nay
            - mới nhất
            - hiện nay
            - thời tiết
            - tin tức
            - sự kiện
            - giá cả

            và database không chứa dữ liệu cập nhật đó,
            phải chọn "web".

            7. Nếu trả lời từ database:

            {language_instruction}

            - Bắt buộc tuân thủ đúng ngôn ngữ trên.
            - Trả lời ngắn gọn.
            - Trả lời tự nhiên.
            - Không bịa thêm thông tin.

            8. Chỉ trả về JSON.
            Không markdown.
            Không giải thích bên ngoài JSON.


            ===== CƠ SỞ DỮ LIỆU IUH =====

            {IUH_DATABASE_TEXT}

            ===== KẾT THÚC DATABASE =====


            Ngữ cảnh hội thoại gần đây:

            {context}


            Câu hỏi hiện tại:

            {question}
        """

        response = client.chat.completions.create(
            model=OPENAI_MODEL,

            messages=[
                {
                    "role": "user",
                    "content": prompt,
                }
            ],

            max_completion_tokens=500,
            temperature=0,
        )

        raw = (
            response
            .choices[0]
            .message
            .content
            or ""
        ).strip()

        # Nếu model lỡ bọc JSON bằng ```json
        if raw.startswith("```"):

            raw = raw.strip("`").strip()

            if raw.lower().startswith("json"):
                raw = raw[4:].strip()

        data = json.loads(raw)

        if not isinstance(data, dict):
            raise ValueError(
                "Database router không trả JSON object"
            )

        source = data.get("source")

        if source not in (
            "database",
            "web"
        ):
            raise ValueError(
                f"Database router trả source không hợp lệ: {source}"
            )

        return data

    # ============================================================
    # BƯỚC 2
    #
    # Database không đủ -> Web Search
    # ============================================================

    def _search_web(
        self,
        client,
        question,
        context
    ):

        # print("[CHAT WEB] Searching Internet...")

        language_instruction = (self._response_language_instruction())

        instructions = f"""
            Bạn là Bé Son, robot trợ lý tại
            Đại học Công nghiệp Thành phố Hồ Chí Minh.

            QUY TẮC NGÔN NGỮ BẮT BUỘC:

            {language_instruction}

            QUY TẮC TRẢ LỜI:

            - Phải tuân thủ đúng ngôn ngữ được yêu cầu ở trên.
            - Trả lời rõ ràng, tự nhiên và thân thiện.
            - Ưu tiên nguồn đáng tin cậy.
            - Không tự bịa thông tin.
            - Với câu hỏi có yếu tố thời gian,
            ưu tiên thông tin mới nhất tìm được.
            - KHÔNG hiển thị URL.
            - KHÔNG hiển thị markdown link.
            - KHÔNG hiển thị citation.
            - KHÔNG hiển thị tên miền nguồn.
            - KHÔNG thêm phần "Nguồn", "Tham khảo" hoặc "Link".
            - Chỉ trả về nội dung câu trả lời cuối cùng.
            - Không nói rằng đã tìm kiếm trên Internet.
            - Trả lời ngắn gọn khoảng 2 đến 4 câu.
            - Câu trả lời sẽ được robot đọc bằng TTS.
        """

        web_input = f"""
    Ngữ cảnh hội thoại gần đây:

    {context}

    Câu hỏi hiện tại:

    {question}

    Hãy tìm kiếm Internet để trả lời câu hỏi trên.

    Ngôn ngữ hiện tại của hệ thống:
    {self.language}
    """

        response = client.responses.create(
            model=OPENAI_MODEL,

            tools=[
                {
                    "type": "web_search",
                    "search_context_size": "low",
                }
            ],

            tool_choice="required",

            instructions=instructions,

            input=web_input,
        )

        answer = (
            response.output_text
            or ""
        ).strip()

        if not answer:

            raise RuntimeError(
                "Web Search không trả về nội dung"
            )

        return answer

    # ============================================================
    # MAIN
    # ============================================================


    def run(self):
        try:
            client = OpenAI(api_key=OPENAI_API_KEY)
            question = (self._get_latest_user_question())
            if not question:

                raise ValueError(
                    "Không tìm thấy câu hỏi người dùng"
                )

            context = (
                self._get_recent_context()
            )

            # ----------------------------------------------------
            # Bước 1: hỏi database
            # ----------------------------------------------------

            route = self._check_database(
                client,
                question,
                context
            )

            source = route.get("source")

            #print(f"[CHAT ROUTE] source={source}")

            # ----------------------------------------------------
            # Database có dữ liệu
            # ----------------------------------------------------

            if source == "database":

                answer = str(
                    route.get("answer", "")
                ).strip()

                if not answer:

                    raise ValueError(
                        "Database có source=database "
                        "nhưng answer rỗng"
                    )

                print(
                    "[CHAT ROUTE] "
                    "Using IUH database"
                )

            # ----------------------------------------------------
            # Database không có -> Internet
            # ----------------------------------------------------

            else:

                print(
                    "[CHAT ROUTE] "
                    "Database insufficient -> Web Search"
                )

                answer = self._search_web(
                    client,
                    question,
                    context
                )

            self.response_ready.emit(
                answer
            )

        except Exception as e:

            print(
                f"[CHAT ERROR] {e}"
            )

            self.error_occurred.emit(
                str(e)[:300]
            )

        finally:

            self.finished.emit()

class _IntentWorker(QObject):
    """
        Phân loại câu nói của người dùng thành:
        - lệnh điều hướng cho robot;
        - hoặc hội thoại thông thường.

        Bộ phân loại sử dụng một lời gọi OpenAI riêng,
        không sử dụng streaming và đặt temperature = 0.

        Kết quả bắt buộc phải là JSON đúng định dạng:

        {
            "intent": "navigate" | "chat",
            "waypoints": [...]
        }

        Trường "waypoints" có thể nhận:

        1. Danh sách tên waypoint dạng chuỗi:

        ["X5.7", "X5.11"]

        2. Hoặc danh sách dictionary có dạng:

        {
            "key": "X5.7",
            "aliases": [
                "phong x5.7",
                "cua x5.7"
            ]
        }

        Việc sử dụng aliases giúp bộ phân loại nhận ra
        nhiều cách gọi khác nhau của cùng một địa điểm
        và ánh xạ chúng về đúng tên waypoint chính.
    """
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
    log_signal       = pyqtSignal(str)

    waypoint_command = pyqtSignal(str)

    navigation_stop = pyqtSignal()

    def __init__(self, parent=None):

        super().__init__(parent)

        self._language = get_language()

        self._chat_history = [
            {
                "role": "system",
                "content": get_system_prompt(
                    self._language
                )
            }
        ]

        self._ai_worker     = None
        self._ai_thread     = None
        self._intent_worker = None
        self._intent_thread = None
        self._voice_enabled = False
        self._pending_reply = None
        self._did_speak     = False

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
    def set_language(self, language):

        if language not in (
            "vi",
            "en"
        ):
            return

        if language == self._language:
            return

        self._language = language

        # ============================================================
        # Đổi ngôn ngữ TTS
        # ============================================================
        self._voice_engine.set_language(language)

        # ============================================================
        # Cập nhật system prompt
        # ============================================================

        new_system_prompt = (
            get_system_prompt(language)
        )

        if (
            self._chat_history
            and
            self._chat_history[0].get("role")
            == "system"
        ):

            self._chat_history[0] = {
                "role": "system",
                "content": new_system_prompt
            }

        else:

            self._chat_history.insert(
                0,
                {
                    "role": "system",
                    "content": new_system_prompt
                }
            )

        print(
            f"[CHAT LANGUAGE] "
            f"Đã chuyển ChatPanel sang: "
            f"{language}"
        )

    def set_pose_provider(self, provider):
        """
            provider: một hàm không nhận tham số, dùng để trả về
            vị trí hiện tại của robot dưới dạng Pose.

            Nếu chưa lấy được vị trí hiện tại thì provider có thể trả về None.

            Hàm này được dùng để ghi lại vị trí hiện tại của robot
            tại thời điểm người dùng ra lệnh kiểu "quay lại đây".

            Vị trí đó sẽ được lưu lại trước khi robot bắt đầu di chuyển
            đến các waypoint khác trong cùng một lệnh.
        """
        self._pose_provider = provider

    def set_waypoints_provider(self, provider):
        """
            provider: một hàm không nhận tham số, dùng để trả về
            danh sách waypoint của bản đồ hiện tại.

            Mỗi waypoint có thể là:

            - một chuỗi chứa tên chính, ví dụ:
            "X5.7"

            - hoặc một dictionary có dạng:
            {
                "key": "X5.7",
                "aliases": [...]
            }

            Việc truyền thêm aliases giúp bộ phân loại ý định
            nhận ra nhiều cách gọi khác nhau của cùng một địa điểm,
            ví dụ các cách gọi được khai báo trong waypoints.json,
            sau đó ánh xạ chúng về đúng tên waypoint chính.

            Khi thiết lập provider này, hệ thống sẽ bật chức năng
            phân loại câu nói bằng giọng nói thành:
            - lệnh điều hướng;
            - hoặc hội thoại thông thường.

            Nếu không thiết lập provider, hệ thống sẽ giữ cách hoạt động cũ:
            mọi câu nói đều được xử lý như hội thoại thông thường.
        """
        self._waypoints_provider = provider

    # ----------------------------------------------------------- AI call ---
    def _ask_ai(self, text: str):

        if (
            self._ai_thread
            and self._ai_thread.isRunning()
        ):
            return

        self._chat_history.append(
            {
                "role": "user",
                "content": text
            }
        )

        self.log_signal.emit(
            f"[Bạn] {text}"
        )

        self.voice_status_label.show()

        self._typing_dots = 0
        self._typing_timer.start(400)

        # ============================================================
        # Khóa ngôn ngữ tại thời điểm gửi request
        # ============================================================

        self._ai_request_language = (
            self._language
        )

        print(
            "[CHAT] Request language:",
            self._ai_request_language
        )

        self._ai_worker = _AIChatWorker(
            list(self._chat_history),
            self._ai_request_language
        )

        self._ai_thread = QThread()

        self._ai_worker.moveToThread(
            self._ai_thread
        )

        self._ai_thread.started.connect(
            self._ai_worker.run
        )

        self._ai_worker.response_ready.connect(
            self._on_response
        )

        self._ai_worker.error_occurred.connect(
            self._on_error
        )

        self._ai_worker.finished.connect(
            self._ai_thread.quit
        )

        self._ai_thread.start()

    def _on_response(self, reply):

        self._typing_timer.stop()

        self._chat_history.append(
            {
                "role": "assistant",
                "content": reply
            }
        )

        self.log_signal.emit(
            f"[Bé Son] {reply}"
        )

        print(
            f"[CHAT] Assistant: {reply}"
        )

        self._pending_reply = reply

        response_language = getattr(
            self,
            "_ai_request_language",
            self._language
        )

        self._voice_engine.speak_in_language(
            reply,
            response_language
        )

    def _on_error(self, error):
        self._typing_timer.stop()
        self.voice_status_label.hide()
        self.log_signal.emit(f"[LỖI] {error}")

    def _finish_turn(self):
        self.voice_status_label.hide()

    # ------------------------------------------------- intent classification
    def _classify_intent(
        self,
        text: str,
        original_text: str = None
    ):

        if self._intent_thread and self._intent_thread.isRunning():
            return

        if original_text is None:
            original_text = text

        waypoints = []

        try:
            waypoints = list(
                self._waypoints_provider() or []
            )
        except Exception:
            waypoints = []

        self.voice_status_label.show()
        self._typing_dots = 0
        self._typing_timer.start(400)

        self._intent_worker = _IntentWorker(
            text,
            waypoints
        )

        self._intent_thread = QThread()

        self._intent_worker.moveToThread(
            self._intent_thread
        )

        self._intent_thread.started.connect(
            self._intent_worker.run
        )

        self._intent_worker.intent_ready.connect(
            lambda data:
            self._on_intent_ready(
                data,
                original_text
            )
        )

        self._intent_worker.error_occurred.connect(
            lambda err:
            self._on_intent_error(
                err,
                original_text
            )
        )

        self._intent_worker.finished.connect(
            self._intent_thread.quit
        )

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

    def _answer_current_location(self):

        # ============================================================
        # 1. Lấy Pose hiện tại từ AMCL
        # ============================================================

        pose = None

        if self._pose_provider is not None:
            try:
                pose = self._pose_provider()
            except Exception as e:
                print(
                    f"[LOCATION] Không lấy được pose: {e}"
                )

        if pose is None:

            reply = (
                "Bé Son chưa xác định được vị trí hiện tại. "
                "Bạn hãy chờ hệ thống định vị AMCL ổn định."
            )

            self.log_signal.emit(
                f"[Bé Son] {reply}"
            )

            self._pending_reply = reply
            self._voice_engine.speak(reply)

            return

        robot_x = pose.position.x
        robot_y = pose.position.y

        print(
            f"[LOCATION] Robot pose: "
            f"x={robot_x:.3f}, y={robot_y:.3f}"
        )

        # ============================================================
        # 2. Lấy waypoint
        # ============================================================

        waypoints = []

        if self._waypoints_provider is not None:
            try:
                waypoints = list(
                    self._waypoints_provider() or []
                )
            except Exception as e:
                print(
                    f"[LOCATION] Không lấy được waypoints: {e}"
                )

        # ============================================================
        # 3. Tìm waypoint gần nhất
        # ============================================================

        nearest = None
        nearest_distance = float("inf")

        for wp in waypoints:

            if not isinstance(wp, dict):
                continue

            x = wp.get("x")
            y = wp.get("y")

            if x is None or y is None:
                continue

            dx = robot_x - float(x)
            dy = robot_y - float(y)

            distance = (
                dx * dx
                + dy * dy
            ) ** 0.5

            if distance < nearest_distance:

                nearest_distance = distance
                nearest = wp

        # ============================================================
        # 4. Trả lời
        # ============================================================

        if nearest is not None:

            room = nearest.get(
                "key",
                "một khu vực trên tầng 5"
            )

            reply = (
                f"Bạn đang ở tầng 5, tòa X, "
                f"gần khu vực {room}. "
                f"Đây là cơ sở 12 Nguyễn Văn Bảo của IUH."
            )

            print(
                f"[LOCATION] Nearest waypoint: "
                f"{room}, "
                f"distance={nearest_distance:.2f} m"
            )

        else:

            reply = (
                "Bạn đang ở tầng 5, tòa X, "
                "tại cơ sở 12 Nguyễn Văn Bảo của IUH. "
                "Bé Son chưa xác định được phòng gần nhất."
            )

        self.log_signal.emit(
            f"[Bé Son] {reply}"
        )

        self._pending_reply = reply

        self._voice_engine.speak(
            reply
        )

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

        stop_commands = {
            "vi": [
            "dung lai",
            "dung robot",
            "huy lenh",
            "huy hanh trinh",
            ],

            "en": [
                "stop",
                "stop robot",
                "stop moving",
                "cancel",
                "cancel navigation",
            ],
        }

        location_commands = [
            "toi dang o dau",
            "toi dang o cho nao",
            "day la dau",
            "day la cho nao",
            "vi tri hien tai",
            "vi tri hien tai cua toi",
            "be son toi dang o dau",
            "be son cho toi biet toi dang o dau",
        ]

        active_stop_commands = (
            stop_commands.get(
                self._language,
                stop_commands["vi"]
            )
        )

        if any(
            cmd in normalized
            for cmd in active_stop_commands
        ):
            self.log_signal.emit(
                "[VOICE] Phát hiện lệnh dừng"
            )

            # Dừng Bé Son nói
            self._voice_engine.stop_speaking()

            # Dừng navigation
            self.navigation_stop.emit()

            return

        if any(
            cmd in normalized
            for cmd in location_commands
        ):
            self._answer_current_location()
            return

        # ============================================================
        # 2. WAKE WORD
        # ============================================================

        wake_words = [
            "bé son",
            "be son",
            "bson",
            "haha",
            "ha ha" 
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

            self._classify_intent(
                normalized,
                text
            )

        else:

            self._ask_ai(
                text
            )

    # ----------------------------------------------------------- helpers ---

    def cleanup(self):
        if self._voice_enabled:
            self._voice_engine.stop_speaking()