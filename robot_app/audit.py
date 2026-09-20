"""Yibu-compatible usage ledger. Unknown usage stays null, never zero."""
import argparse
import csv
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4


def record(event, model, key, endpoint, started, ok=True, error=None):
    usage = event.get("response", {}).get("usage") or event.get("usage") or {}
    def count(*names):
        for name in names:
            value = usage.get(name)
            if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
                return value
        return None
    incoming = count("input_tokens", "prompt_tokens")
    outgoing = count("output_tokens", "completion_tokens")
    total = count("total_tokens")
    derived = total is None and incoming is not None and outgoing is not None
    if derived:
        total = incoming + outgoing
    row = dict(schema_version="yibu_call_audit_v1", call_id=uuid4().hex,
               timestamp_utc=datetime.now(timezone.utc).isoformat(), provider="yibuapi",
               model=model, key_suffix="..." + key[-4:], purpose="robot_realtime",
               endpoint=endpoint, transport="websocket", ok=ok,
               latency_s=round(time.monotonic() - started, 4),
               input_tokens=incoming, output_tokens=outgoing, total_tokens=total,
               total_tokens_derived=derived, usage_reported=bool(usage), usage_raw=usage)
    if error:
        row["error"] = error  # Only internally generated error codes, never upstream bodies.
    path = Path(os.getenv("YIBU_AUDIT_LOG", "artifacts/yibu_api_calls.jsonl"))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row) + "\n")
    return row


def summarize(path, destination):
    groups, seen = {}, {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        previous = seen.get(row["call_id"])
        if previous is not None:
            if previous != row:
                raise ValueError("Conflicting duplicate call ID")
            continue
        seen[row["call_id"]] = row
        key = (row["model"], row["key_suffix"], row["purpose"])
        group = groups.setdefault(key, dict(model=key[0], key_suffix=key[1], purpose=key[2],
                                           calls=0, failures=0, input_tokens=0, output_tokens=0,
                                           total_tokens=0, missing_input_tokens=0,
                                           missing_output_tokens=0, missing_total_tokens=0))
        group["calls"] += 1
        group["failures"] += not row["ok"]
        for field in ("input_tokens", "output_tokens", "total_tokens"):
            if row.get(field) is None:
                group["missing_" + field] += 1
            else:
                group[field] += row[field]
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    rows = list(groups.values())
    (destination / "usage_summary.json").write_text(json.dumps(
        {"calls": len(seen), "groups": rows, "accounting": "One record per response.done; interrupted sessions may have unknown usage"},
        indent=2), encoding="utf-8")
    fields = list(rows[0]) if rows else ["model", "key_suffix", "purpose", "calls"]
    with (destination / "usage_by_model_key_purpose.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", default=os.getenv("YIBU_AUDIT_LOG", "artifacts/yibu_api_calls.jsonl"))
    parser.add_argument("--out-dir", default="artifacts/summary")
    args = parser.parse_args()
    summarize(args.log, args.out_dir)
