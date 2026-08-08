#!/usr/bin/env python3
"""Create a frozen bag whose playback schedule follows its recorded /clock.

Only rosbag storage timestamps are changed.  Sensor/header timestamps and CDR
payloads remain byte-for-byte unchanged.  This removes the original WSL/Gazebo
software-rendering RTF from functional estimator experiments while retaining
the recorded causal ordering.
"""

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise SystemExit(f"output already exists: {output}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.input, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = reader.get_all_topics_and_types()
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for topic in topics:
        writer.create_topic(topic)
    first_clock_ns = None
    current_clock_ns = None
    first_storage_ns = None
    last_output_ns = 999_999_999
    count = 0
    while reader.has_next():
        topic, data, storage_ns = reader.read_next()
        if first_storage_ns is None:
            first_storage_ns = int(storage_ns)
        if topic == "/clock":
            clock = deserialize_message(data, Clock).clock
            current_clock_ns = int(clock.sec) * 1_000_000_000 + int(clock.nanosec)
            if first_clock_ns is None:
                first_clock_ns = current_clock_ns
        if current_clock_ns is not None and first_clock_ns is not None:
            desired_ns = 1_000_000_000 + current_clock_ns - first_clock_ns
        else:
            desired_ns = 1_000_000_000 + int(storage_ns) - first_storage_ns
        output_ns = max(last_output_ns + 1, desired_ns)
        writer.write(topic, data, output_ns)
        last_output_ns = output_ns
        count += 1
    duration_s = (last_output_ns - 1_000_000_000) * 1.0e-9
    print(f"messages={count} normalized_duration_s={duration_s:.6f} output={output}")


if __name__ == "__main__":
    main()
