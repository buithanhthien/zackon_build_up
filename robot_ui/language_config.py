#!/usr/bin/env python3

import json
import os


_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

LANGUAGE_SETTINGS_FILE = os.path.join(
    _DIR,
    "language_settings.json"
)


# ============================================================
# Các ngôn ngữ Bé Son hỗ trợ
# ============================================================

LANGUAGES = {
    "vi": {
        "name": "Tiếng Việt",
        "stt_model": "gipformer",
        "tts_language": "vi",
    },

    "en": {
        "name": "English",
        "stt_model": "zipformer_en",
        "tts_language": "en",
    },
}

# ============================================================
# Nội dung giao diện theo ngôn ngữ
# ============================================================

UI_TEXT = {
    "vi": {
        "waypoints": "Điểm đến",
        "docking": "Về trạm sạc",
        "load_map": "Tải bản đồ",
        "new_map": "Bản đồ mới",
        "tracking": "Theo dõi",
        "reestimate": "Định vị lại",
        "nav2": "Nav2",
        "language": "Ngôn Ngữ",
        "developer": "⚙ Developer",

        "language_window": "Ngôn Ngữ",
        "select_language": "CHỌN NGÔN NGỮ",
        "select": "Chọn",
        "cancel": "Hủy",
    },

    "en": {
        "waypoints": "Destinations",
        "docking": "Docking",
        "load_map": "Load Map",
        "new_map": "New Map",
        "tracking": "Tracking",
        "reestimate": "Relocalize",
        "nav2": "Nav2",
        "language": "Language",
        "developer": "⚙ Developer",

        "language_window": "Language",
        "select_language": "SELECT LANGUAGE",
        "select": "Select",
        "cancel": "Cancel",
    },
}


DEFAULT_LANGUAGE = "vi"


# ============================================================
# Đọc ngôn ngữ hiện tại
# ============================================================

def get_language():

    try:

        if not os.path.exists(
            LANGUAGE_SETTINGS_FILE
        ):
            return DEFAULT_LANGUAGE

        with open(
            LANGUAGE_SETTINGS_FILE,
            "r",
            encoding="utf-8"
        ) as f:

            data = json.load(f)

        language = data.get(
            "language",
            DEFAULT_LANGUAGE
        )

        if language not in LANGUAGES:
            return DEFAULT_LANGUAGE

        return language

    except Exception as e:

        print(
            f"[LANGUAGE] Lỗi đọc cấu hình: {e}"
        )

        return DEFAULT_LANGUAGE


# ============================================================
# Đổi ngôn ngữ
# ============================================================

def set_language(language):

    if language not in LANGUAGES:

        print(
            f"[LANGUAGE] Ngôn ngữ không hợp lệ: "
            f"{language}"
        )

        return False

    try:

        with open(
            LANGUAGE_SETTINGS_FILE,
            "w",
            encoding="utf-8"
        ) as f:

            json.dump(
                {
                    "language": language
                },
                f,
                ensure_ascii=False,
                indent=2
            )

        print(
            f"[LANGUAGE] Đã đổi sang: "
            f"{LANGUAGES[language]['name']}"
        )

        return True

    except Exception as e:

        print(
            f"[LANGUAGE] Lỗi lưu cấu hình: {e}"
        )

        return False


# ============================================================
# Lấy toàn bộ cấu hình của ngôn ngữ hiện tại
# ============================================================

def get_language_config():

    language = get_language()

    return LANGUAGES[language]

# ============================================================
# Lấy text giao diện theo ngôn ngữ hiện tại
# ============================================================

def get_ui_text(language=None):

    if language is None:
        language = get_language()

    if language not in UI_TEXT:
        language = DEFAULT_LANGUAGE

    return UI_TEXT[language]