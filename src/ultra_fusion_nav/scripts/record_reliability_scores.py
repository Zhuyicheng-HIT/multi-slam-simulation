#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from uf_interfaces.msg import ReliabilityScore


MODALITIES = ("lidar", "gnss", "imu", "optical_flow", "vision")


class ScoreRecorder(Node):
    def __init__(self):
        super().__init__("reliability_score_recorder")
        self.started = time.monotonic()
        self.rows = []
        for modality in MODALITIES:
            self.create_subscription(
                ReliabilityScore,
                f"/reliability/{modality}_score",
                lambda msg, key=modality: self._score(key, msg),
                20,
            )

    def _score(self, modality, msg):
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        evidence = dict(zip(msg.evidence_names, msg.evidence_values))
        self.rows.append({
            "elapsed_s": time.monotonic() - self.started,
            "stamp_s": stamp,
            "modality": modality,
            "degradation_score": float(msg.degradation_score),
            "reliability_weight": float(msg.reliability_weight),
            "valid": int(msg.valid),
            "reasons": "|".join(msg.reasons),
            "evidence_json": json.dumps(evidence, separators=(",", ":")),
        })


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=145.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = ScoreRecorder()
    while rclpy.ok() and time.monotonic() - node.started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "elapsed_s", "stamp_s", "modality", "degradation_score",
        "reliability_weight", "valid", "reasons", "evidence_json",
    ]
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(node.rows)
    counts = {key: sum(row["modality"] == key for row in node.rows) for key in MODALITIES}
    print(json.dumps({"output": str(output), "counts": counts}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(counts[key] > 0 for key in MODALITIES) else 1


if __name__ == "__main__":
    raise SystemExit(main())
