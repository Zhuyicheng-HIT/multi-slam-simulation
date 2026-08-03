#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from uf_interfaces.msg import ReliabilityScore


MODALITIES = ("lidar", "gnss", "imu", "optical_flow", "vision")


class ScoreRecorder(Node):
    def __init__(self):
        super().__init__(
            "reliability_score_recorder",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.started_wall = time.monotonic()
        self.first_source_stamp_s = None
        self.invalid_header_stamp_counts = {modality: 0 for modality in MODALITIES}
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
        if stamp <= 0.0:
            self.invalid_header_stamp_counts[modality] += 1
            return
        if self.first_source_stamp_s is None:
            self.first_source_stamp_s = stamp
        elif stamp < self.first_source_stamp_s:
            self.first_source_stamp_s = stamp
            for row in self.rows:
                elapsed = row["stamp_s"] - stamp
                row["elapsed_s"] = elapsed
                row["elapsed_ros_s"] = elapsed
        elapsed_ros_s = stamp - self.first_source_stamp_s
        evidence = dict(zip(msg.evidence_names, msg.evidence_values))
        self.rows.append({
            "elapsed_s": elapsed_ros_s,
            "elapsed_ros_s": elapsed_ros_s,
            "arrival_elapsed_wall_s": time.monotonic() - self.started_wall,
            "stamp_s": stamp,
            "modality": modality,
            "degradation_score": float(msg.degradation_score),
            "reliability_weight": float(msg.reliability_weight),
            "valid": int(msg.valid),
            "observation_count": int(msg.observation_count),
            "minimum_observation_count": int(msg.minimum_observation_count),
            "observation_ready": int(
                msg.observation_count >= msg.minimum_observation_count
            ),
            "reasons": "|".join(msg.reasons),
            "evidence_json": json.dumps(evidence, separators=(",", ":")),
        })


def record_for_ros_duration(node, duration_s, wall_timeout_s):
    wall_started = time.monotonic()
    last_progress_wall = wall_started
    ros_started_ns = None
    last_ros_ns = None
    elapsed_ros_s = 0.0
    elapsed_wall_s = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        now_ros_ns = node.get_clock().now().nanoseconds
        now_wall = time.monotonic()
        if now_ros_ns > 0 and ros_started_ns is None:
            ros_started_ns = now_ros_ns
        if last_ros_ns is not None and now_ros_ns < last_ros_ns:
            raise RuntimeError("ROS clock moved backwards while recording reliability scores")
        if last_ros_ns is None or now_ros_ns > last_ros_ns:
            last_progress_wall = now_wall
        last_ros_ns = now_ros_ns
        elapsed_ros_s = (
            (now_ros_ns - ros_started_ns) * 1.0e-9
            if ros_started_ns is not None else 0.0
        )
        elapsed_wall_s = now_wall - wall_started
        if elapsed_ros_s >= duration_s:
            return elapsed_ros_s, elapsed_wall_s
        stalled_wall_s = now_wall - last_progress_wall
        if stalled_wall_s >= wall_timeout_s:
            raise RuntimeError(
                f"ROS clock stalled for {stalled_wall_s:.1f}s "
                f"after advancing {elapsed_ros_s:.1f}s"
            )
    return elapsed_ros_s, time.monotonic() - wall_started


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=145.0)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=0.0,
        help="wall seconds without ROS-clock progress; 0 selects a conservative limit",
    )
    parser.add_argument(
        "--allow-missing",
        action="store_true",
        help="return success when a deliberately disabled modality has no rows",
    )
    args = parser.parse_args()
    rclpy.init()
    node = ScoreRecorder()
    wall_timeout_s = (
        args.wall_timeout if args.wall_timeout > 0.0
        else max(args.duration * 10.0, args.duration + 60.0)
    )
    duration_ros_s, duration_wall_s = record_for_ros_duration(
        node, args.duration, wall_timeout_s
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "elapsed_s", "elapsed_ros_s", "stamp_s", "arrival_elapsed_wall_s",
        "modality", "degradation_score",
        "reliability_weight", "valid", "observation_count",
        "minimum_observation_count", "observation_ready", "reasons",
        "evidence_json",
    ]
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sorted(
            node.rows,
            key=lambda row: (row["stamp_s"], row["modality"]),
        ))
    counts = {key: sum(row["modality"] == key for row in node.rows) for key in MODALITIES}
    print(json.dumps({
        "output": str(output),
        "counts": counts,
        "invalid_header_stamp_counts": dict(node.invalid_header_stamp_counts),
        "duration_s": duration_ros_s,
        "duration_ros_s": duration_ros_s,
        "duration_wall_s": duration_wall_s,
        "wall_stall_timeout_s": wall_timeout_s,
    }, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    complete = all(counts[key] > 0 for key in MODALITIES)
    return 0 if complete or args.allow_missing else 1


if __name__ == "__main__":
    raise SystemExit(main())
