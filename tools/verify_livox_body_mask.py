#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from livox_ros_driver2.msg import CustomMsg


def parse_args():
    parser = argparse.ArgumentParser(
        description="Verify that a Livox CustomMsg contains no aircraft self returns."
    )
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--pitch-deg", type=float, default=10.0)
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=6,
        default=(-0.45, 0.45, -0.45, 0.45, -0.35, 0.15),
        metavar=("MIN_X", "MAX_X", "MIN_Y", "MAX_Y", "MIN_Z", "MAX_Z"),
    )
    parser.add_argument(
        "--translation",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
    )
    return parser.parse_args()


class CloudReceiver(Node):
    def __init__(self, topic):
        super().__init__("verify_livox_body_mask")
        self.message = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CustomMsg, topic, self._callback, qos)

    def _callback(self, message):
        self.message = message


def main():
    args = parse_args()
    rclpy.init()
    node = CloudReceiver(args.topic)
    deadline = time.monotonic() + args.timeout
    try:
        while node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            print(json.dumps({"topic": args.topic, "error": "timeout"}))
            return 2

        angle = math.radians(args.pitch_deg)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        min_x, max_x, min_y, max_y, min_z, max_z = args.bounds
        tx, ty, tz = args.translation
        inside = 0
        minimum_range = math.inf
        for point in node.message.points:
            body_x = cosine * point.x + sine * point.z + tx
            body_y = point.y + ty
            body_z = -sine * point.x + cosine * point.z + tz
            if (
                min_x <= body_x <= max_x
                and min_y <= body_y <= max_y
                and min_z <= body_z <= max_z
            ):
                inside += 1
            minimum_range = min(
                minimum_range,
                math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z),
            )

        report = {
            "topic": args.topic,
            "points": len(node.message.points),
            "inside_body_box": inside,
            "minimum_range_m": minimum_range if math.isfinite(minimum_range) else None,
            "passed": inside == 0,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if inside == 0 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
