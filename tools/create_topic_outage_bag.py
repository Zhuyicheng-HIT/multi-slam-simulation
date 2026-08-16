#!/usr/bin/env python3
"""Copy a rosbag2 while dropping one topic during a /clock-relative interval."""

import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
import yaml


def within_outage(elapsed_s, start_s, duration_s):
    elapsed_s = float(elapsed_s)
    start_s = float(start_s)
    duration_s = float(duration_s)
    if not all(math.isfinite(value) for value in (elapsed_s, start_s, duration_s)):
        raise ValueError("outage times must be finite")
    if start_s < 0.0 or duration_s <= 0.0:
        raise ValueError("outage start must be nonnegative and duration positive")
    return start_s <= elapsed_s < start_s + duration_s


def clock_nanoseconds(message):
    return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)


def bag_compression_mode(input_path):
    """Return the rosbag2 compression mode declared by metadata.yaml."""
    metadata_path = Path(input_path) / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    information = metadata.get("rosbag2_bagfile_information", {})
    return str(information.get("compression_mode", "")).strip().upper()


def make_reader(input_path):
    """Select the rosbag2 reader that matches compressed or plain storage."""
    compression_mode = bag_compression_mode(input_path)
    if compression_mode and compression_mode != "NONE":
        return rosbag2_py.SequentialCompressionReader()
    return rosbag2_py.SequentialReader()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--start-s", required=True, type=float)
    parser.add_argument("--duration-s", required=True, type=float)
    parser.add_argument("--report")
    args = parser.parse_args()
    within_outage(args.start_s, args.start_s, args.duration_s)

    input_path = Path(args.input)
    output_path = Path(args.output)
    if output_path.exists():
        raise SystemExit(f"output already exists: {output_path}")

    reader = make_reader(input_path)
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = reader.get_all_topics_and_types()
    available = {item.name for item in topics}
    if "/clock" not in available:
        raise RuntimeError("input bag must contain /clock")
    if args.topic not in available:
        raise RuntimeError(f"outage topic missing: {args.topic}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for metadata in topics:
        writer.create_topic(metadata)

    first_clock_ns = None
    current_clock_ns = None
    copied = 0
    dropped = 0
    topic_seen = 0
    first_dropped_clock_s = None
    last_dropped_clock_s = None
    while reader.has_next():
        topic, payload, storage_ns = reader.read_next()
        if topic == "/clock":
            current_clock_ns = clock_nanoseconds(
                deserialize_message(payload, Clock)
            )
            if first_clock_ns is None:
                first_clock_ns = current_clock_ns
        elapsed_s = None
        if current_clock_ns is not None and first_clock_ns is not None:
            elapsed_s = (current_clock_ns - first_clock_ns) * 1.0e-9
        drop = False
        if topic == args.topic:
            topic_seen += 1
            drop = elapsed_s is not None and within_outage(
                elapsed_s, args.start_s, args.duration_s
            )
        if drop:
            dropped += 1
            first_dropped_clock_s = (
                elapsed_s if first_dropped_clock_s is None
                else first_dropped_clock_s
            )
            last_dropped_clock_s = elapsed_s
            continue
        writer.write(topic, payload, int(storage_ns))
        copied += 1

    report = {
        "schema_version": 1,
        "input": str(input_path.resolve()),
        "output": str(output_path.resolve()),
        "topic": args.topic,
        "outage_start_s": args.start_s,
        "outage_duration_s": args.duration_s,
        "topic_messages_seen": topic_seen,
        "topic_messages_dropped": dropped,
        "messages_copied": copied,
        "first_dropped_clock_s": first_dropped_clock_s,
        "last_dropped_clock_s": last_dropped_clock_s,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.report:
        Path(args.report).write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
