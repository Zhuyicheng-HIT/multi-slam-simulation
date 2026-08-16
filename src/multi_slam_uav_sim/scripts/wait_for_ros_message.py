#!/usr/bin/env python3
"""Wait for one ROS 2 message without repeatedly starting the ROS CLI."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument(
        "--reliability", choices=("best_effort", "reliable"),
        default="best_effort",
    )
    parser.add_argument(
        "--field",
        help="Optional dot-separated message field that must match --equals.",
    )
    parser.add_argument(
        "--equals",
        help="Case-insensitive expected value for --field.",
    )
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if bool(args.field) != bool(args.equals):
        parser.error("--field and --equals must be provided together")

    rclpy.init(args=None)
    node = Node("wait_for_ros_message")
    received = False
    subscription = None
    deadline = time.monotonic() + args.timeout
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
        reliability=(
            ReliabilityPolicy.BEST_EFFORT
            if args.reliability == "best_effort"
            else ReliabilityPolicy.RELIABLE
        ),
    )

    def callback(message):
        nonlocal received
        value = message
        if args.field:
            try:
                for component in args.field.split("."):
                    value = getattr(value, component)
            except AttributeError:
                return
            if str(value).strip().lower() != args.equals.strip().lower():
                return
        received = True

    try:
        while rclpy.ok() and time.monotonic() < deadline and not received:
            if subscription is None:
                topics = dict(node.get_topic_names_and_types())
                type_names = topics.get(args.topic, ())
                if type_names:
                    message_type = get_message(type_names[0])
                    subscription = node.create_subscription(
                        message_type, args.topic, callback, qos
                    )
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if received:
        print(f"received: {args.topic}")
        return 0
    print(f"timed out waiting for {args.topic}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
