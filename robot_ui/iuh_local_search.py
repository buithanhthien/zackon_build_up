"""Deterministic local retrieval for iuh_database.json.

This module does not call an LLM, Hindsight, Qt, ROS, or the web.  It returns
verifiable evidence with the exact JSON path that produced each match.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict, replace
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

PERSON_TITLE_TOKENS = {
    "tien", "si", "thac", "cu", "nhan", "pho", "giao", "su",
    "ts", "pgs", "gs",
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
    # Questions that inherently require current information.
    "hom nay", "bay gio", "hien tai", "hien nay", "moi nhat", "moi day",
    "thoi tiet", "tin tuc", "gia ca", "ty gia",
    "today", "right now", "current", "currently", "latest", "newest",
    "weather", "news", "price", "exchange rate",

    # Explicit requests from the user to use the Internet.
    "tren mang",
    "tim tren mang",
    "tra tren mang",
    "tra cuu tren mang",
    "tim tren internet",
    "tra cuu tren internet",
    "search the web",
    "search online",
    "look it up online",
)

_DESCRIPTOR_KEYS = {"ten", "ten_day_du", "ten_tieng_anh", "viet_tat"}


@dataclass(frozen=True)
class Evidence:
    path: str
    field: str
    value: Any
    score: float
    reasons: tuple[str, ...]
    # Human-readable structure for list members.  These fields deliberately
    # duplicate parent information so an answer model never has to infer a
    # department name from a zero-based JSON array index.
    parent_department_name: str | None = None
    parent_department_path: str | None = None
    parent_department_head: str | None = None
    ordinal_position: int | None = None

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["reasons"] = list(self.reasons)
        return {key: value for key, value in data.items() if value is not None}


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

    def _count_matches(self, record: _Record, question_norm: str) -> bool:
        """Require the counted field and its scope, not just shared school words."""
        label = _label_from_key(record.field)
        label = re.sub(r"^(?:so luong|tong so|so) ", "", label)
        if not label or record.field.isdigit():
            return False
        markers = r"(?:bao nhieu|co may|so luong|tong so)"
        if not re.search(r"\b" + markers + r"\s+" + re.escape(label) + r"\b",
                         question_norm):
            return False
        # No certified complete university-wide unit list exists in this schema.
        if record.value_type == "list" and label in {"khoa", "vien", "phong ban"}:
            return False
        if record.path.startswith("khoa_cong_nghe_dien."):
            department = self._mentioned_bo_mon(question_norm)
            if ".bo_mon." in record.path:
                if department is None:
                    return False
                prefix = f"khoa_cong_nghe_dien.bo_mon.{department['index']}."
                return record.path.startswith(prefix)
            if not re.search(r"\bkhoa cong nghe dien\b(?! tu\b)", question_norm):
                return False
        return True

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

    @staticmethod
    def _requested_bo_mon_ordinal(question_norm: str) -> int | None:
        """Return the human 1-based department ordinal explicitly requested."""
        if "bo mon" not in question_norm:
            return None

        ordinal_words = {
            "nhat": 1, "mot": 1, "hai": 2, "ba": 3,
            "tu": 4, "bon": 4, "nam": 5, "sau": 6,
            "bay": 7, "tam": 8, "chin": 9, "muoi": 10,
        }
        match = re.search(
            r"\bbo mon(?:\s+(?:thu|so))?\s+"
            r"(\d+|nhat|mot|hai|ba|tu|bon|nam|sau|bay|tam|chin|muoi)\b",
            question_norm,
        )
        if not match:
            return None
        token = match.group(1)
        if token.isdigit():
            return int(token)
        return ordinal_words.get(token)

    def _bo_mon_entries(self):
        """Return known departments from the current structured IUH JSON."""
        khoa = self.database.get("khoa_cong_nghe_dien", {})

        if not isinstance(khoa, dict):
            return []

        bo_mon = khoa.get("bo_mon", [])

        if not isinstance(bo_mon, list):
            return []

        result = []

        for index, item in enumerate(bo_mon):
            if not isinstance(item, dict):
                continue

            name = item.get("ten")

            if isinstance(name, str) and name.strip():
                result.append((index, name.strip()))

        return result


    def _bo_mon_details(self, index: int) -> dict[str, Any] | None:
        khoa = self.database.get("khoa_cong_nghe_dien", {})
        bo_mon = khoa.get("bo_mon", []) if isinstance(khoa, dict) else []
        if not isinstance(bo_mon, list) or not (0 <= index < len(bo_mon)):
            return None
        item = bo_mon[index]
        if not isinstance(item, dict):
            return None
        name = item.get("ten")
        if not isinstance(name, str) or not name.strip():
            return None
        head = item.get("truong_bo_mon")
        if not isinstance(head, str) or not head.strip():
            head = None
        return {
            "index": index,
            "ordinal_position": index + 1,
            "name": name.strip(),
            "path": f"khoa_cong_nghe_dien.bo_mon.{index}.ten",
            "head": head.strip() if head else None,
        }

    def _enrich_department_context(self, evidence: Evidence) -> Evidence:
        index = self._lecturer_bo_mon_index(evidence.path)
        if index is None:
            return evidence
        details = self._bo_mon_details(index)
        if details is None:
            return evidence
        return replace(
            evidence,
            parent_department_name=details["name"],
            parent_department_path=details["path"],
            parent_department_head=details["head"],
        )


    def _mentioned_bo_mon(self, question_norm):
        """
        Detect a department explicitly mentioned by the user.

        Example:
        "bo mon cung cap"
        -> Bộ môn Cung cấp & Hệ thống điện
        """
        if "bo mon" not in question_norm:
            return None

        ordinal = self._requested_bo_mon_ordinal(question_norm)
        if ordinal is not None:
            details = self._bo_mon_details(ordinal - 1)
            if details is not None:
                return {"index": details["index"], "name": details["name"]}

        query_tokens = _tokenize(question_norm)

        candidates = []

        for index, name in self._bo_mon_entries():
            name_tokens = _tokenize(name)

            # "bo" and "mon" identify the object type but do not
            # distinguish one department from another.
            distinctive_tokens = name_tokens - {"bo", "mon"}

            hits = distinctive_tokens & query_tokens

            if len(hits) >= 2:
                candidates.append(
                    (len(hits), index, name)
                )

        if not candidates:
            return None

        candidates.sort(
            key=lambda item: (-item[0], item[1])
        )

        best = candidates[0]

        # Two departments matching equally well -> do not guess.
        if (
            len(candidates) > 1
            and candidates[1][0] == best[0]
        ):
            return None

        return {
            "index": best[1],
            "name": best[2],
        }


    @staticmethod
    def _lecturer_bo_mon_index(path):
        """
        Extract department index from a lecturer JSON path.

        Example:
        khoa_cong_nghe_dien.bo_mon.0.giang_vien.14
        -> 0
        """
        parts = path.split(".")

        try:
            bo_mon_pos = parts.index("bo_mon")
            giang_vien_pos = parts.index("giang_vien")
        except ValueError:
            return None

        if giang_vien_pos <= bo_mon_pos:
            return None

        index_pos = bo_mon_pos + 1

        if index_pos >= len(parts):
            return None

        try:
            return int(parts[index_pos])
        except ValueError:
            return None

    def search(self, question: str, limit: int = 5) -> SearchResult:
        self.reload_if_changed()
        question_norm = normalize_text(question)
        if not question_norm:
            return SearchResult("insufficient", (), "empty_question")

        attribute = self._attribute_intent(question_norm)
        is_count = self._is_count_question(question_norm)
        is_plural = self._is_plural_question(question_norm)

        electrical = bool(re.search(r"\bkhoa cong nghe dien\b(?! tu\b)", question_norm))
        electronics = "khoa cong nghe dien tu" in question_norm
        if electrical and attribute is None and not is_count and (
            re.search(r"(?:gioi thieu|tong quan) (?:ve )?khoa cong nghe dien\b", question_norm)
            or re.search(r"khoa cong nghe dien.*ban co biet", question_norm)
        ):
            # A faculty introduction needs descriptive evidence, not a highly
            # ranked contact leaf sharing words with the university's name.
            paths = {"khoa_cong_nghe_dien." + field for field in
                     ("ten", "chuong_trinh_dao_tao", "chuyen_nganh")}
            evidence = tuple(Evidence(r.path, r.field, r.value, 20.0,
                                      ("faculty_overview",))
                             for r in self.records if r.path in paths)
            if evidence:
                return SearchResult("sufficient", evidence, "faculty_overview")

        if attribute == "dia_chi" and (
            "truong dai hoc cong nghiep" in question_norm or "iuh" in question_norm.split()
        ) and not re.search(r"\b(?:khoa|phong|co so|phan hieu)\b", question_norm):
            for record in self.records:
                if record.path == "co_so.tru_so_chinh.dia_chi":
                    return SearchResult("sufficient", (Evidence(
                        record.path, record.field, record.value, 20.0,
                        ("university_main_address",)),), "university_main_address")

        # Human ordinals are 1-based.  Resolve them before generic lexical
        # ranking so JSON index 3 can never be presented as "bộ môn thứ ba".
        ordinal = self._requested_bo_mon_ordinal(question_norm)
        if ordinal is not None and attribute is None and not is_count:
            details = self._bo_mon_details(ordinal - 1)
            if details is None:
                return SearchResult(
                    "insufficient", (), "department_ordinal_out_of_range"
                )
            return SearchResult(
                "sufficient",
                (Evidence(
                    path=details["path"],
                    field="ten",
                    value=details["name"],
                    score=12.0,
                    reasons=("human_ordinal_department",),
                    parent_department_name=details["name"],
                    parent_department_path=details["path"],
                    parent_department_head=details["head"],
                    ordinal_position=details["ordinal_position"],
                ),),
                "department_selected_by_human_ordinal",
            )

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
            if electrical and not record.path.startswith("khoa_cong_nghe_dien."):
                continue
            if electronics and record.path.startswith("khoa_cong_nghe_dien."):
                continue
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
            if (
                record.value_type == "text"
                and isinstance(record.value, str)
                and "giang_vien" in record.path
            ):
                scalar_tokens = _tokenize(record.value) - PERSON_TITLE_TOKENS

                if len(scalar_tokens) >= 2 and scalar_tokens <= query_tokens:
                    score += 5.0
                    reasons.append("exact_person_scalar_tokens")
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
                if not self._count_matches(record, question_norm):
                    continue
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
                ranked.append(self._enrich_department_context(Evidence(
                    path=record.path,
                    field=record.field,
                    value=evidence_value,
                    score=round(score, 3),
                    reasons=tuple(reasons),
                )))

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

        # If every candidate matched only because of the requested
        # attribute (email / phone / address / ...), but none matched
        # the actual subject/entity in the question, local data is not
        # sufficient. Do not ask the user to choose between unrelated
        # records.
        if near_top and all(
            evidence.reasons
            and all(
                reason.startswith("attribute:")
                for reason in evidence.reasons
            )
            for evidence in near_top
        ):
            return SearchResult(
                "insufficient",
                (),
                "attribute_match_without_subject_match",
            )

        if len(near_top) > 1:
            return SearchResult(
                "ambiguous",
                tuple(near_top[:limit]),
                "multiple_local_candidates_with_similar_score",
            )

        # ------------------------------------------------------------
        # Person / department consistency check
        # ------------------------------------------------------------

        actual_bo_mon_index = self._lecturer_bo_mon_index(
            top.path
        )

        mentioned_bo_mon = self._mentioned_bo_mon(
            question_norm
        )

        if (
            actual_bo_mon_index is not None
            and mentioned_bo_mon is not None
            and actual_bo_mon_index
                != mentioned_bo_mon["index"]
        ):
            actual_entries = dict(
                self._bo_mon_entries()
            )

            actual_name = actual_entries.get(
                actual_bo_mon_index,
                "",
            )

            department_path = (
                "khoa_cong_nghe_dien."
                f"bo_mon.{actual_bo_mon_index}.ten"
            )

            department_evidence = Evidence(
                path=department_path,
                field="ten",
                value=actual_name,
                score=top.score,
                reasons=("actual_parent_department",),
            )

            return SearchResult(
                "ambiguous",
                (
                    top,
                    department_evidence,
                ),
                "person_department_conflict",
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
