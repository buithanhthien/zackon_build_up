"""Keep assistant display, history and speech free of pictographic decoration."""
import re


_EMOJI = re.compile(
    r"[0-9#*]\ufe0f?\u20e3|"
    r"[\U0001f000-\U0001faff\u2600-\u27bf]|"
    r"[\u2300-\u23ff\u2190-\u21ff\u2b00-\u2bff\u00a9\u00ae\u2122]\ufe0f|"
    r"[\u200d\ufe0e\ufe0f\u20e3\U000e0020-\U000e007f]"
)


def plain_chat_text(text):
    """Remove emoji sequences while retaining accents, numbers and math text."""
    return _EMOJI.sub("", text).strip()
