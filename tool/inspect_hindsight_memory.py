#!/usr/bin/env python3
"""Export pending SQLite rows and, optionally, Hindsight documents as readable JSON.

Read-only: never retains facts, creates banks, retries pending writes or deletes data.
"""

import argparse
import json
import os
from pathlib import Path
import sqlite3
import sys
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_ui.correction_memory import CorrectionMemory, MemoryUnavailable


def read_pending(path):
    if not path.exists():
        return []
    connection = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
    try:
        return [{"namespace": namespace, "document_id": document_id,
                 "record": json.loads(record)}
                for namespace, document_id, record in connection.execute(
                    "SELECT namespace, document_id, record FROM pending ORDER BY namespace, document_id")]
    finally:
        connection.close()


def read_documents(memory):
    documents = []
    offset = 0
    while True:
        page = memory._request("GET", f"/documents?limit=100&offset={offset}")
        items = page["items"]
        for item in items:
            detail = memory._request("GET", "/documents/" + quote(item["id"], safe=""))
            params = item.get("retain_params") or {}
            if isinstance(params, str):
                params = json.loads(params)
            documents.append({"document_id": item["id"],
                              "original_text": detail.get("original_text"),
                              "created_at": detail.get("created_at"),
                              "updated_at": detail.get("updated_at"),
                              "memory_unit_count": detail.get("memory_unit_count"),
                              "metadata": params.get("metadata", {}),
                              "retain_params": params})
        offset += len(items)
        if not items or offset >= page.get("total", offset):
            return documents


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pending", type=Path)
    parser.add_argument("--hindsight", action="store_true", help="Also read source documents from the configured bank")
    parser.add_argument("--query", action="append", default=[], help="Read-only recall and local scope comparison (requires --hindsight)")
    parser.add_argument("--output", type=Path, help="Save UTF-8 JSON; otherwise print it")
    args = parser.parse_args()
    if args.query and not args.hindsight:
        parser.error("--query requires --hindsight")
    # Match the UI's .env precedence, without importing Qt/ROS or printing secrets.
    env_path = ROOT / ".env"
    if env_path.exists():
        for line in env_path.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, value = line.split("=", 1)
                if key.strip().startswith("HINDSIGHT_"):
                    os.environ.setdefault(key.strip(), value.strip())
    pending_path = args.pending or Path(os.getenv("HINDSIGHT_PENDING_PATH", str(ROOT / ".local/hindsight-pending.sqlite3")))
    result = {"pending_explanation": "Only unconfirmed writes; confirmed documents live in Hindsight.",
              "pending_file": str(pending_path), "pending": [], "documents": [], "errors": []}
    try:
        result["pending"] = read_pending(pending_path)
    except (OSError, sqlite3.Error, ValueError):
        result["errors"].append("Cannot read pending SQLite records")
    if args.hindsight:
        result["bank"] = os.getenv("HINDSIGHT_BANK_ID", "beson-corrections")
        try:
            memory = CorrectionMemory(storage_path=False)
            result["documents"] = read_documents(memory)
            metadata = {d["document_id"]: d["metadata"] for d in result["documents"]}
            result["recall_checks"] = []
            if args.query:
                from robot_ui.iuh_local_search import IuhLocalSearch
                local_search = IuhLocalSearch()
            for query in args.query:
                records = memory._request("POST", "/memories/recall", {
                    "query": query, "budget": "mid", "max_tokens": 2048,
                    "types": ["world", "experience"],
                })["results"]
                for record in records:
                    if not record.get("metadata"):
                        record["metadata"] = metadata.get(record.get("document_id"), {})
                local = local_search.search(query)
                evidence = local.evidence if local.status == "sufficient" else None
                filtered = memory._filter_for_local_scope(records, evidence)
                supplements = memory._supplemental_memories(records, evidence)
                result["recall_checks"].append({
                    "query": query, "local_status": local.status,
                    "local_evidence": memory._local_evidence_items(evidence),
                    "raw_records": records, "after_scope_filter": filtered,
                    "supplemental_memories": supplements,
                })
        except MemoryUnavailable as exc:
            result["errors"].append("Hindsight: " + exc.category)
        except (ValueError, KeyError, TypeError):
            result["errors"].append("Invalid Hindsight document response")
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(encoded)
        print(f"Saved {len(result['pending'])} pending rows and {len(result['documents'])} documents to {args.output}")
        for error in result["errors"]:
            print(error, file=sys.stderr)
    else:
        print(encoded, end="")
    return 1 if result["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
