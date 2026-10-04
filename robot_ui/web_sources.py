"""Extract actual web tool sources independently of generated answer prose."""

from urllib.parse import urlsplit
import re
import unicodedata


def format_answer_sources(answer, question):
    """Hide unsolicited citation markup before displaying or speaking an answer."""
    normalized = unicodedata.normalize("NFD", question.lower().replace("đ", "d"))
    normalized = "".join(c for c in normalized if not unicodedata.combining(c))
    requested = re.search(
        r"\b(?:nguon|link|url|website|trang web|duong dan|trich dan|"
        r"sources?|citations?|references?)\b", normalized)
    declined = re.search(
        r"\b(?:khong|dung|bo qua|no|without|do not)\b.{0,35}"
        r"\b(?:nguon|link|url|trich dan|sources?|citations?|references?)\b", normalized)
    if requested and not declined:
        return answer

    # Remove parenthesized citations in the exact form returned by web search.
    link = r"\[[^\]\n]*\]\(https?://(?:[^\s()]|\([^\s()]*\))*\)"
    answer = re.sub(r"\(\s*" + link + r"(?:\s*[,;]\s*" + link + r")*\s*\)", "", answer)

    def plain_label(match):
        label = match.group(1)
        return "" if re.fullmatch(r"(?:https?://)?[\w.-]+\.[a-z]{2,}(?:/\S*)?", label) else label

    answer = re.sub(r"\[([^\]\n]*)\]\(https?://(?:[^\s()]|\([^\s()]*\))*\)", plain_label, answer)
    answer = re.sub(r"https?://[^\s<>]+", "", answer)
    answer = re.sub(r"[^]*", "", answer)
    answer = re.sub(r"(?m)^\s*(?:\*\*)?(?:Nguồn|Nguồn tham khảo|Sources|References)(?:\*\*)?\s*:.*$", "", answer)
    answer = re.sub(r"[ \t]+\n", "\n", answer)
    answer = re.sub(r"[ \t]{2,}", " ", answer)
    return answer.strip()


def extract_web_sources(output):
    sources = []
    seen = set()

    def add(source):
        url = source.get("url", "")
        if not isinstance(url, str):
            return
        try:
            parsed = urlsplit(url)
        except ValueError:
            return
        if parsed.scheme not in {"http", "https"} or not parsed.netloc or url in seen:
            return
        seen.add(url)
        sources.append({"url": url, "title": source.get("title") or parsed.netloc})

    for item in output:
        if item.get("type") == "web_search_call":
            for source in (item.get("action") or {}).get("sources") or []:
                add(source)
        elif item.get("type") == "message":
            for content in item.get("content", []):
                for annotation in content.get("annotations", []):
                    if annotation.get("type") == "url_citation":
                        add(annotation)
    return sources
