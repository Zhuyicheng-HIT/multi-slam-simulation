#!/usr/bin/env python3

import argparse
from collections import deque
import json
from pathlib import Path
import statistics
import time

from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class FastLioLatencyProbe(Node):
    def __init__(self, lidar_topic, odom_topic):
        super().__init__("fast_lio_latency_probe")
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.source = deque(maxlen=256)
        self.latencies_ms = []
        self.matched_source_ends = set()
        self.source_count = 0
        self.odom_count = 0
        self.create_subscription(CustomMsg, lidar_topic, self.on_lidar, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, sensor_qos)

    def on_lidar(self, message):
        start = int(message.timebase) or stamp_ns(message.header.stamp)
        end = start + max((int(point.offset_time) for point in message.points), default=0)
        self.source.append((end, time.monotonic_ns()))
        self.source_count += 1

    def on_odom(self, message):
        self.odom_count += 1
        if not self.source:
            return
        output_stamp = stamp_ns(message.header.stamp)
        source_end, source_wall = min(
            self.source, key=lambda item: abs(item[0] - output_stamp)
        )
        if abs(source_end - output_stamp) > 25_000_000:
            return
        if source_end in self.matched_source_ends:
            return
        latency = (time.monotonic_ns() - source_wall) * 1.0e-6
        if latency < 0.0:
            return
        self.matched_source_ends.add(source_end)
        self.latencies_ms.append(latency)


def percentile(values, quantile):
    return float(np.percentile(values, quantile)) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-topic", required=True)
    parser.add_argument("--odom-topic", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = FastLioLatencyProbe(args.lidar_topic, args.odom_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        result = {
            "contract": "wall time from scan-end callback to matching odometry callback",
            "source_count": node.source_count,
            "odom_count": node.odom_count,
            "matched_count": len(node.latencies_ms),
            "p50_ms": percentile(node.latencies_ms, 50.0),
            "p95_ms": percentile(node.latencies_ms, 95.0),
            "p99_ms": percentile(node.latencies_ms, 99.0),
            "mean_ms": (
                statistics.fmean(node.latencies_ms) if node.latencies_ms else None
            ),
        }
        Path(args.output).write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
