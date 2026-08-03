#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from uf_interfaces.msg import SchedulerState


class SchedulerRecorder(Node):
    def __init__(self):
        super().__init__(
            "scheduler_timeline_recorder",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.started_wall = time.monotonic()
        self.first_source_stamp_s = None
        self.invalid_header_stamp_count = 0
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
        source_stamp_s = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )
        if source_stamp_s <= 0.0:
            self.invalid_header_stamp_count += 1
            return
        if self.first_source_stamp_s is None:
            self.first_source_stamp_s = source_stamp_s
        elif source_stamp_s < self.first_source_stamp_s:
            self.first_source_stamp_s = source_stamp_s
            for row in self.rows:
                elapsed = row["source_stamp_s"] - source_stamp_s
                row["elapsed_s"] = elapsed
                row["elapsed_ros_s"] = elapsed
        elapsed_ros_s = source_stamp_s - self.first_source_stamp_s
        arrival_elapsed_wall_s = time.monotonic() - self.started_wall
        for index in range(count):
            self.rows.append({
                "elapsed_s": elapsed_ros_s,
                "elapsed_ros_s": elapsed_ros_s,
                "source_stamp_s": source_stamp_s,
                "arrival_elapsed_wall_s": arrival_elapsed_wall_s,
                "health_state": msg.health_state,
                "modality": msg.modality_names[index],
                "degradation_score": float(msg.degradation_scores[index]),
                "reliability_weight": float(msg.reliability_weights[index]),
                "covariance_inflation": float(msg.covariance_inflation[index]),
                "factor_enabled": int(msg.factor_enabled[index]),
                "reasons": msg.reasons[index],
                "relocalization_requested": int(msg.relocalization_requested),
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
            raise RuntimeError("ROS clock moved backwards while recording scheduler state")
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
    args = parser.parse_args()
    rclpy.init()
    node = SchedulerRecorder()
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
        "elapsed_s", "elapsed_ros_s", "source_stamp_s",
        "arrival_elapsed_wall_s", "health_state", "modality", "degradation_score",
        "reliability_weight", "covariance_inflation", "factor_enabled",
        "reasons", "relocalization_requested",
    ]
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sorted(
            node.rows,
            key=lambda row: (row["source_stamp_s"], row["modality"]),
        ))
    print(
        f"scheduler timeline rows={len(node.rows)} "
        f"invalid_header_stamps={node.invalid_header_stamp_count} "
        f"duration_ros_s={duration_ros_s:.3f} duration_wall_s={duration_wall_s:.3f} "
        f"output={output}"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
