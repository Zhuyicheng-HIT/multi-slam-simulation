#!/usr/bin/env python3
"""Measure a ROS 2 topic rate without parsing the long-running CLI tool."""

import argparse
from collections import deque
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message


def arrival_rate(arrivals):
    if len(arrivals) < 2:
        return 0.0
    span_s = float(arrivals[-1]) - float(arrivals[0])
    return (len(arrivals) - 1) / span_s if span_s > 0.0 else 0.0


def source_stamp_s(message):
    try:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return value if value > 0.0 else None
    except (AttributeError, TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--minimum-hz", type=float, required=True)
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("--window", type=int, default=20)
    parser.add_argument("--minimum-wall-source-ratio", type=float, default=0.0)
    args = parser.parse_args()
    if (
        args.minimum_hz <= 0.0
        or args.timeout <= 0.0
        or args.window < 2
        or args.minimum_wall_source_ratio < 0.0
    ):
        parser.error("rate, timeout, and window must be positive; ratio must be non-negative")

    rclpy.init()
    node = Node("topic_rate_probe")
    arrivals = deque(maxlen=args.window)
    source_stamps = deque(maxlen=args.window)
    subscription = None
    deadline = time.monotonic() + args.timeout
    wall_source_ratio = 0.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if subscription is None:
                topic_types = dict(node.get_topic_names_and_types()).get(args.topic, ())
                message_type = get_message(topic_types[0]) if topic_types else None
                if message_type is not None:
                    qos = QoSProfile(
                        history=HistoryPolicy.KEEP_LAST,
                        depth=max(10, args.window),
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                    )
                    def sample(message):
                        arrivals.append(time.monotonic())
                        stamp_s = source_stamp_s(message)
                        if stamp_s is not None:
                            source_stamps.append(stamp_s)

                    subscription = node.create_subscription(
                        message_type,
                        args.topic,
                        sample,
                        qos,
                    )
                else:
                    time.sleep(0.1)
                    continue
            rclpy.spin_once(node, timeout_sec=0.1)
            rate_hz = arrival_rate(source_stamps)
            wall_rate_hz = arrival_rate(arrivals)
            wall_source_ratio = (
                wall_rate_hz / rate_hz if rate_hz > 0.0 else 0.0
            )
            observation_span_s = (
                source_stamps[-1] - source_stamps[0]
                if len(source_stamps) >= 2 else 0.0
            )
            required_samples = max(3, min(args.window, int(math.ceil(args.minimum_hz * 2.0)) + 1))
            # A bounded window of N samples spans N-1 periods. Requiring a
            # full two seconds at exactly 10 Hz with the default N=20 is
            # impossible (the span is 1.9 s), so derive the gate from the
            # actual sample count instead of the nominal two-second target.
            required_span_s = (required_samples - 1) / max(rate_hz, args.minimum_hz, 1.0e-9)
            if (
                len(source_stamps) >= required_samples
                and observation_span_s >= min(2.0, required_span_s)
                and rate_hz >= args.minimum_hz
                and wall_source_ratio >= args.minimum_wall_source_ratio
            ):
                print(
                    f"ready: {args.topic} {rate_hz:.3f} Hz source_stamp "
                    f"({wall_rate_hz:.3f} Hz wall arrival, "
                    f"wall/source={wall_source_ratio:.3f})"
                )
                return 0
        print(
            f"timeout: {args.topic} measured {arrival_rate(source_stamps):.3f} Hz "
            f"from {len(source_stamps)} source stamps "
            f"({arrival_rate(arrivals):.3f} Hz wall arrival, "
            f"wall/source={wall_source_ratio:.3f})",
        )
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
