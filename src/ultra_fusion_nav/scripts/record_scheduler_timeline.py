#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from uf_interfaces.msg import SchedulerState


class SchedulerRecorder(Node):
    def __init__(self):
        super().__init__("scheduler_timeline_recorder")
        self.started = time.monotonic()
        self.rows = []
        self.create_subscription(
            SchedulerState,
            "/reliability/scheduler_state",
            self._state,
            20,
        )

    def _state(self, msg):
        count = min(
            len(msg.modality_names),
            len(msg.degradation_scores),
            len(msg.reliability_weights),
            len(msg.covariance_inflation),
            len(msg.factor_enabled),
            len(msg.reasons),
        )
        elapsed = time.monotonic() - self.started
        for index in range(count):
            self.rows.append({
                "elapsed_s": elapsed,
                "health_state": msg.health_state,
                "modality": msg.modality_names[index],
                "degradation_score": float(msg.degradation_scores[index]),
                "reliability_weight": float(msg.reliability_weights[index]),
                "covariance_inflation": float(msg.covariance_inflation[index]),
                "factor_enabled": int(msg.factor_enabled[index]),
                "reasons": msg.reasons[index],
                "relocalization_requested": int(msg.relocalization_requested),
            })


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=145.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = SchedulerRecorder()
    while rclpy.ok() and time.monotonic() - node.started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "elapsed_s", "health_state", "modality", "degradation_score",
        "reliability_weight", "covariance_inflation", "factor_enabled",
        "reasons", "relocalization_requested",
    ]
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(node.rows)
    print(f"scheduler timeline rows={len(node.rows)} output={output}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
