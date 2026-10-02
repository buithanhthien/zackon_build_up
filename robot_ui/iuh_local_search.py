"""Deterministic local retrieval for iuh_database.json.

This module does not call an LLM, Hindsight, Qt, ROS, or the web.  It returns
verifiable evidence with the exact JSON path that produced each match.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import json
from pathlib import Path
import re
import unicodedata
from typing import Any, Iterable


_DIR = Path(__file__).resolve().parent
DEFAULT_DATABASE_PATH = _DIR / "iuh_database.json"


STOP_WORDS = {
    "a", "ai", "anh", "ban", "biet", "cai", "cho", "co", "cua", "duoc",
    "gi", "hay", "hoi", "la", "minh", "mot", "nao", "noi", "o", "robot",
    "thong", "tin", "toi", "ve", "xin",
}

# Query phrases that identify an attribute even when the JSON field name differs.
ATTRIBUTE_ALIASES = {
    "dia_chi": (
        "dia chi", "o dau", "nam o dau", "tai dau", "tru so o dau",
    ),
    "dien_thoai": (
        "dien thoai", "so dien thoai", "sdt", "phone", "hotline",
    ),
    "email": (
        "email", "e mail", "thu dien tu",
    ),
    "website": (
        "website", "trang web", "web site",
    ),
    "truong_khoa": (
        "truong khoa",
    ),
    "pho_truong_khoa": (
        "pho truong khoa",
    ),
    "truong_bo_mon": (
        "truong bo mon",
    ),
}

COUNT_PHRASES = (
    "bao nhieu", "so luong", "co may", "tong so",
)

PLURAL_PHRASES = (
    "cac ", "nhung ", "danh sach", "gom nhung", "gom cac",
)

FRESHNESS_PHRASES = (
    "hom nay", "bay gio", "hien tai", "hien nay", "moi nhat", "moi day",
    "thoi tiet", "tin tuc", "gia ca", "ty gia",
    "today", "right now", "current", "currently", "latest", "newest",
    "weather", "news", "price", "exchange rate",
)

_DESCRIPTOR_KEYS = {"ten", "ten_day_du", "ten_tieng_anh", "viet_tat"}


@dataclass(frozen=True)
class Evidence:
    path: str
    field: str
    value: Any
    score: float
    reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["reasons"] = list(self.reasons)
        return data


@dataclass(frozen=True)
class SearchResult:
    status: str  # sufficient | insufficient | ambiguous
    evidence: tuple[Evidence, ...]
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "reason": self.reason,
            "evidence": [item.to_dict() for item in self.evidence],
        }


@dataclass(frozen=True)
class _Record:
    path: str
    field: str
    value: Any
    search_text: str
    tokens: frozenset[str]
    structural_tokens: frozenset[str]
    value_type: str


def normalize_text(value: Any) -> str:
    text = str(value).lower().replace("đ", "d")
    text = unicodedata.normalize("NFD", text)
    text = "".join(ch for ch in text if unicodedata.category(ch) != "Mn")
    text = re.sub(r"[^a-z0-9@._+/-]+", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def _label_from_key(key: str) -> str:
    return normalize_text(key.replace("_", " "))


def _tokenize(text: str) -> set[str]:
    return {
        token for token in normalize_text(text).split()
        if len(token) > 1 and token not in STOP_WORDS
    }


def _value_type(value: Any) -> str:
    if isinstance(value, list):
        return "list"
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, (int, float)):
        return "number"
    text = str(value).strip()
    if re.fullmatch(r"[^\s@]+@[^\s@]+\.[^\s@]+", text):
        return "email"
    if re.match(r"https?://", text, re.IGNORECASE):
        return "website"
    digits = re.sub(r"\D", "", text)
    if len(digits) >= 8 and any(mark in text for mark in ("(", ")", "+", "-")):
        return "dien_thoai"
    return "text"


def _descriptors(node: dict[str, Any]) -> list[str]:
    result = []
    for key in _DESCRIPTOR_KEYS:
        value = node.get(key)
        if isinstance(value, (str, int, float)):
            result.append(str(value))
    return result


def _flatten(
    node: Any,
    path: tuple[str, ...] = (),
    inherited_descriptors: tuple[str, ...] = (),
) -> Iterable[_Record]:
    if isinstance(node, dict):
        here = inherited_descriptors + tuple(_descriptors(node))
        for key, value in node.items():
            yield from _flatten(value, path + (key,), here)
        return

    if isinstance(node, list):
        if path:
            path_labels = [_label_from_key(part) for part in path if not part.isdigit()]
            scalar_preview = []
            for value in node:
                if isinstance(value, (str, int, float)):
                    scalar_preview.append(str(value))
                elif isinstance(value, dict):
                    scalar_preview.extend(_descriptors(value))
            structural_text = " ".join(path_labels + list(inherited_descriptors))
            search_text = " ".join(
                path_labels + list(inherited_descriptors) + scalar_preview
            )
            yield _Record(
                path=".".join(path),
                field=path[-1],
                value=node,
                search_text=normalize_text(search_text),
                tokens=frozenset(_tokenize(search_text)),
                structural_tokens=frozenset(_tokenize(structural_text)),
                value_type="list",
            )
        for index, value in enumerate(node):
            yield from _flatten(value, path + (str(index),), inherited_descriptors)
        return

    if not path:
        return

    path_labels = [_label_from_key(part) for part in path if not part.isdigit()]
    structural_text = " ".join(path_labels + list(inherited_descriptors))
    search_text = " ".join(path_labels + list(inherited_descriptors) + [str(node)])
    yield _Record(
        path=".".join(path),
        field=path[-1],
        value=node,
        search_text=normalize_text(search_text),
        tokens=frozenset(_tokenize(search_text)),
        structural_tokens=frozenset(_tokenize(structural_text)),
        value_type=_value_type(node),
    )


def requires_web_for_freshness(question: str) -> bool:
    question_norm = normalize_text(question)
    return any(phrase in question_norm for phrase in FRESHNESS_PHRASES)


class IuhLocalSearch:
    """Search structured IUH JSON without asking an LLM whether it is sufficient."""

    def __init__(self, database_path: str | Path = DEFAULT_DATABASE_PATH):
        self.database_path = Path(database_path)
        self.database = {}
        self.records = ()
        self._loaded_mtime_ns = None
        self._reload()

    def _reload(self):
        with self.database_path.open("r", encoding="utf-8") as handle:
            database = json.load(handle)
        records = tuple(_flatten(database))
        stat = self.database_path.stat()
        self.database = database
        self.records = records
        self._loaded_mtime_ns = stat.st_mtime_ns

    def reload_if_changed(self):
        stat = self.database_path.stat()
        if self._loaded_mtime_ns != stat.st_mtime_ns:
            self._reload()

    @staticmethod
    def _attribute_intent(question_norm: str) -> str | None:
        for attribute, phrases in ATTRIBUTE_ALIASES.items():
            if any(phrase in question_norm for phrase in phrases):
                return attribute
        return None

    @staticmethod
    def _is_count_question(question_norm: str) -> bool:
        return any(phrase in question_norm for phrase in COUNT_PHRASES)

    @staticmethod
    def _is_plural_question(question_norm: str) -> bool:
        return any(phrase in question_norm for phrase in PLURAL_PHRASES)

    @staticmethod
    def _field_matches_attribute(record: _Record, attribute: str) -> bool:
        field = normalize_text(record.field.replace("_", " "))
        if attribute == "email":
            return record.value_type == "email" or "email" in field
        if attribute == "website":
            return record.value_type == "website" or "website" in field or "portal" in field
        if attribute == "dien_thoai":
            return record.value_type == "dien_thoai" or "dien thoai" in field
        return normalize_text(attribute.replace("_", " ")) == field

    def search(self, question: str, limit: int = 5) -> SearchResult:
        self.reload_if_changed()
        question_norm = normalize_text(question)
        if not question_norm:
            return SearchResult("insufficient", (), "empty_question")

        attribute = self._attribute_intent(question_norm)
        is_count = self._is_count_question(question_norm)
        is_plural = self._is_plural_question(question_norm)

        query_tokens = _tokenize(question_norm)
        intent_tokens: set[str] = set()
        if attribute:
            intent_tokens |= _tokenize(attribute.replace("_", " "))
            for phrase in ATTRIBUTE_ALIASES.get(attribute, ()):
                if phrase in question_norm:
                    intent_tokens |= _tokenize(phrase)
        if is_count:
            for phrase in COUNT_PHRASES:
                if phrase in question_norm:
                    intent_tokens |= _tokenize(phrase)
        if is_plural:
            for phrase in PLURAL_PHRASES:
                if phrase.strip() and phrase.strip() in question_norm:
                    intent_tokens |= _tokenize(phrase)
        entity_tokens = query_tokens - intent_tokens

        ranked: list[Evidence] = []
        for record in self.records:
            reasons: list[str] = []
            score = 0.0

            attr_match = bool(attribute and self._field_matches_attribute(record, attribute))
            if attr_match:
                score += 6.0
                reasons.append(f"attribute:{attribute}")
            elif attribute:
                # Explicit attribute questions should not be answered by another field.
                continue

            token_hits = sorted(entity_tokens & set(record.tokens))
            structural_hits = sorted(entity_tokens & set(record.structural_tokens))
            value_only_hits = sorted(set(token_hits) - set(structural_hits))
            if structural_hits:
                score += 2.0 * len(structural_hits)
                reasons.append("structure:" + ",".join(structural_hits))
            if value_only_hits:
                score += 0.5 * len(value_only_hits)
                reasons.append("value_tokens:" + ",".join(value_only_hits))

            # Strong bonus when a human-readable JSON path segment appears as a phrase.
            for part in record.path.split("."):
                if part.isdigit():
                    continue
                label = _label_from_key(part)
                if len(label) >= 4 and label in question_norm:
                    score += 3.0
                    reasons.append("path_phrase:" + part)

            evidence_value = record.value
            if is_count:
                if record.value_type == "list":
                    # Count only a structurally matched collection. A list item merely
                    # mentioning the query words is not proof that it represents them.
                    if entity_tokens:
                        min_hits = max(2, (len(entity_tokens) + 1) // 2)
                        if len(structural_hits) < min_hits:
                            continue
                    evidence_value = len(record.value)
                    score += 4.0
                    reasons.append("derived_count_from_list")
                elif record.value_type == "number":
                    score += 3.0
                    reasons.append("numeric_value")
                else:
                    # Text containing unrelated digits is not evidence of a requested count.
                    continue

            if is_plural and record.value_type == "list":
                score += 2.0
                reasons.append("list_value")

            # Without an explicit attribute, require some semantic overlap.
            if not attribute and not token_hits:
                continue

            # For an attribute-only question such as "địa chỉ là gì?", keep all
            # attribute matches so ambiguity can be detected rather than guessed.
            if score >= 4.5:
                ranked.append(Evidence(
                    path=record.path,
                    field=record.field,
                    value=evidence_value,
                    score=round(score, 3),
                    reasons=tuple(reasons),
                ))

        if not ranked:
            return SearchResult("insufficient", (), "no_matching_local_evidence")

        ranked.sort(key=lambda item: (-item.score, item.path))
        top = ranked[0]
        near_top = [item for item in ranked if top.score - item.score <= 1.0]

        # A plural/list request may legitimately need several equally relevant leaves.
        if is_plural and len(near_top) > 1:
            return SearchResult(
                "sufficient",
                tuple(near_top[:limit]),
                "multiple_local_evidence_for_plural_question",
            )

        if len(near_top) > 1:
            return SearchResult(
                "ambiguous",
                tuple(near_top[:limit]),
                "multiple_local_candidates_with_similar_score",
            )

        return SearchResult("sufficient", (top,), "single_strong_local_evidence")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("question")
    parser.add_argument("--database", default=str(DEFAULT_DATABASE_PATH))
    args = parser.parse_args()

    result = IuhLocalSearch(args.database).search(args.question)
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
