#!/usr/bin/env python3
"""Read-only metadata inspection for ROS1 bags and ROS2 sqlite bags."""

from __future__ import annotations

import argparse
import json
import sqlite3
from collections import Counter
from pathlib import Path
from typing import Any


def inspect_ros1(path: Path) -> dict[str, Any]:
    try:
        from rosbags.rosbag1 import Reader  # type: ignore
    except ImportError as exc:
        return {
            "format": "ros1_bag",
            "path": str(path),
            "status": "missing_dependency",
            "error": "install the rosbags package to inspect ROS1 bags",
            "detail": str(exc),
        }

    topics: Counter[str] = Counter()
    types: dict[str, str] = {}
    first_ns: int | None = None
    last_ns: int | None = None
    messages = 0
    with Reader(path) as reader:
        for connection in reader.connections:
            types[connection.topic] = connection.msgtype
        for connection, timestamp, _rawdata in reader.messages():
            topics[connection.topic] += 1
            first_ns = timestamp if first_ns is None else min(first_ns, timestamp)
            last_ns = timestamp if last_ns is None else max(last_ns, timestamp)
            messages += 1
    return {
        "format": "ros1_bag",
        "path": str(path),
        "status": "ok",
        "size_bytes": path.stat().st_size,
        "message_count": messages,
        "start_ns": first_ns,
        "end_ns": last_ns,
        "duration_sec": None if first_ns is None else (last_ns - first_ns) / 1e9,
        "topics": [
            {"topic": topic, "message_type": types.get(topic), "message_count": count}
            for topic, count in sorted(topics.items())
        ],
    }


def inspect_ros2(path: Path) -> dict[str, Any]:
    db_path = path
    if path.is_dir():
        candidates = sorted(path.glob("*.db3"))
        if not candidates:
            return {"format": "ros2_sqlite", "path": str(path), "status": "no_db3"}
        db_path = candidates[0]
    conn = sqlite3.connect(str(db_path))
    try:
        topic_rows = conn.execute(
            "SELECT id, name, type FROM topics ORDER BY name"
        ).fetchall()
        counts = dict(
            conn.execute("SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id")
            .fetchall()
        )
        bounds = conn.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
        return {
            "format": "ros2_sqlite",
            "path": str(db_path),
            "status": "ok",
            "size_bytes": db_path.stat().st_size,
            "message_count": sum(counts.values()),
            "start_ns": bounds[0],
            "end_ns": bounds[1],
            "duration_sec": None if bounds[0] is None else (bounds[1] - bounds[0]) / 1e9,
            "topics": [
                {"topic": name, "message_type": msg_type, "message_count": counts.get(topic_id, 0)}
                for topic_id, name, msg_type in topic_rows
            ],
        }
    finally:
        conn.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not args.input.exists():
        parser.error(f"input does not exist: {args.input}")
    suffix = args.input.suffix.lower()
    if suffix == ".bag":
        result = inspect_ros1(args.input)
    elif suffix == ".db3" or args.input.is_dir():
        result = inspect_ros2(args.input)
    else:
        result = {"path": str(args.input), "status": "unsupported_suffix"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=True))
    return 0 if result.get("status") == "ok" else 2


if __name__ == "__main__":
    raise SystemExit(main())
