#!/usr/bin/env python3
"""Publish a short, reliable JSON event burst for rosbag evidence."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", required=True)
    parser.add_argument("--event", required=True)
    parser.add_argument("--detail", default="")
    parser.add_argument("--repeat", type=int, default=20)
    parser.add_argument("--period", type=float, default=0.05)
    args = parser.parse_args()
    if args.repeat < 1 or not 0.0 < args.period <= 1.0:
        parser.error("repeat must be positive and period must be in (0, 1]")

    rclpy.init()
    node = Node("competition_event_marker")
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(String, "/competition/event", qos)
    try:
        deadline = time.monotonic() + 1.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        payload = {
            "schema": "competition_event_v1",
            "mode": args.mode,
            "event": args.event,
            "detail": args.detail,
            "ros_time_ns": int(node.get_clock().now().nanoseconds),
            "wall_time_ns": time.time_ns(),
        }
        message = String(data=json.dumps(payload, sort_keys=True, separators=(",", ":")))
        for _ in range(args.repeat):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(args.period)
        print(message.data)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
