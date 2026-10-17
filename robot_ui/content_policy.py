"""Safety checks for requests that would open searchable web content."""

import re
import unicodedata


_ADULT_MARKERS = (
    "18+",
    "adult content",
    "adult site",
    "adult sites",
    "adult website",
    "adult websites",
    "adult video",
    "adult videos",
    "adult film",
    "adult films",
    "adult actress",
    "adult actor",
    "adult performer",
    "adult entertainment",
    "porn actor",
    "porn actress",
    "pornographic film",
    "pornographic films",
    "av actress",
    "jav actress",
    "sex industry",
    "porn",
    "pornography",
    "xxx",
    "hentai",
    "nsfw",
    "erotic",
    "sexual content",
    "explicit sexual",
    "nude",
    "nudes",
    "missav",
    "jav",
    "onlyfans",
    "xvideos",
    "xnxx",
    "pornhub",
    "redtube",
    "rule34",
    "sex video",
    "sex site",
    "phim nguoi lon",
    "web nguoi lon",
    "trang web 18 tuoi",
    "noi dung 18",
    "khieu dam",
    "phim sex",
    "anh nude",
)
_FILMOGRAPHY_MARKERS = (
    "filmography",
    "list of films",
    "list of movies",
    "titles she acted in",
    "movies she acted in",
    "films she acted in",
    "movies she starred in",
    "films she starred in",
    "what movies did she",
    "what films did she",
    "film titles",
    "movie titles",
    "danh sach phim",
    "liet ke phim",
    "ten phim",
    "nhung bo phim",
    "cac bo phim",
)
_ACCESS_CUES = (
    "open", "visit", "watch", "view", "play", "search", "find", "stream",
    "download", "browse", "mo", "truy cap", "vao", "xem", "tim", "phat",
    "tai xuong", "nghe",
)


def _normalize(value: str) -> str:
    decomposed = unicodedata.normalize(
        "NFD", value.casefold().replace("đ", "d")
    )
    return "".join(
        character for character in decomposed
        if unicodedata.category(character) != "Mn"
    )


def contains_adult_content_marker(value: str) -> bool:
    """Return whether text contains a known adult-content term or site name."""
    normalized = _normalize(value)
    for marker in _ADULT_MARKERS:
        escaped = re.escape(_normalize(marker)).replace(r"\+", r"\s*\+")
        if re.search(rf"(?<!\w){escaped}(?!\w)", normalized):
            return True
    return False


def is_blocked_adult_access_request(value: str) -> bool:
    """Identify direct access/search requests for adult content or websites."""
    normalized = _normalize(value)
    has_access_cue = any(
        re.search(rf"(?<!\w){re.escape(_normalize(cue))}(?!\w)", normalized)
        for cue in _ACCESS_CUES
    )
    return has_access_cue and contains_adult_content_marker(normalized)


def is_blocked_adult_filmography_request(
    question: str,
    conversation_context: str = "",
) -> bool:
    """Block filmographies when the current subject is identified as adult."""
    normalized_question = _normalize(question)
    filmography_request = any(
        re.search(
            rf"(?<!\w){re.escape(_normalize(marker))}(?!\w)",
            normalized_question,
        )
        for marker in _FILMOGRAPHY_MARKERS
    ) or bool(
        re.search(
            r"(?<!\w)(?:bo phim|phim).{0,50}"
            r"(?:dong|tham gia|dien|co ay|co co ay)(?!\w)",
            normalized_question,
        )
        or re.search(
            r"(?<!\w)(?:dong|tham gia|dien).{0,50}"
            r"(?:bo phim|phim)(?!\w)",
            normalized_question,
        )
    )
    if not filmography_request:
        return False
    return (
        contains_adult_content_marker(question)
        or contains_adult_content_marker(conversation_context)
    )
