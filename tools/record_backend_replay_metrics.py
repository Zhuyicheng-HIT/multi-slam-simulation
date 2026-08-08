#!/usr/bin/env python3
"""Record per-diagnostic backend timings during a frozen ROS replay."""

import argparse
import json
import math
import signal
import statistics
import time
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node


def stats(values):
    values = [float(value) for value in values]
    if not values:
        return {}
    mean = statistics.fmean(values)
    return {
        "count": len(values),
        "mean": mean,
        "median": statistics.median(values),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "std": statistics.pstdev(values),
        "cv": statistics.pstdev(values) / mean if mean else 0.0,
        "min": min(values),
        "max": max(values),
    }


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        if math.isnan(value):
            return "nan"
        return "inf" if value > 0.0 else "-inf"
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


class Recorder(Node):
    def __init__(self):
        super().__init__("backend_replay_metrics_recorder")
        self.samples = []
        self.odom_count = 0
        self.create_subscription(
            DiagnosticArray,
            "/fusion/unified/diagnostics",
            self._diagnostics,
            50,
        )
        self.create_subscription(
            Odometry, "/fusion/unified/odom", self._odom, 50
        )

    def _diagnostics(self, message):
        for status in message.status:
            if status.name != "unified_backend_fusion":
                continue
            values = {}
            for item in status.values:
                try:
                    values[item.key] = float(item.value)
                except ValueError:
                    values[item.key] = item.value
            self.samples.append({
                "wall_monotonic_s": time.monotonic(),
                "ros_time_s": self.get_clock().now().nanoseconds * 1.0e-9,
                "values": values,
            })

    def _odom(self, _message):
        self.odom_count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--wall-timeout", type=float, default=900.0)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder()
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    deadline = time.monotonic() + args.wall_timeout
    try:
        while not stopping and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        solve = [
            sample["values"].get("backend_solve_ms")
            for sample in node.samples
            if isinstance(sample["values"].get("backend_solve_ms"), (int, float))
        ]
        callback = [
            sample["values"].get("callback_ms")
            for sample in node.samples
            if isinstance(sample["values"].get("callback_ms"), (int, float))
        ]
        last = node.samples[-1]["values"] if node.samples else {}
        report = {
            "schema_version": 1,
            "diagnostic_samples": len(node.samples),
            "odom_count": node.odom_count,
            "solver_ms": stats(solve),
            "callback_ms": stats(callback),
            "last_values": last,
            "samples": node.samples,
        }
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(
                json_safe(report), indent=2, sort_keys=True, allow_nan=False
            ) + "\n",
            encoding="utf-8",
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
