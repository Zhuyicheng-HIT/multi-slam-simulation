#!/usr/bin/env python3
import argparse
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def tum_row(msg):
    stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
    if stamp <= 0.0:
        return None
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class TrajectoryRecorder(Node):
    def __init__(self, estimate_topic, truth_topic):
        super().__init__(
            "uf_trajectory_recorder",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.estimate = []
        self.truth = []
        self.invalid_header_stamp_counts = {"estimate": 0, "truth": 0}
        self.create_subscription(
            Odometry,
            estimate_topic,
            lambda msg: self._record("estimate", self.estimate, msg),
            20,
        )
        self.create_subscription(
            Odometry,
            truth_topic,
            lambda msg: self._record("truth", self.truth, msg),
            qos_profile_sensor_data,
        )

    def _record(self, stream, target, msg):
        row = tum_row(msg)
        if row is None:
            self.invalid_header_stamp_counts[stream] += 1
            return
        target.append(row)


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
            raise RuntimeError("ROS clock moved backwards while recording trajectories")
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


def write_tum(path, rows):
    with path.open("w", encoding="ascii") as handle:
        for row in sorted(rows, key=lambda value: value[0]):
            handle.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Record evaluator-only LIO and truth trajectories in TUM format")
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--estimate-topic", default="/lio/odom")
    parser.add_argument("--truth-topic", default="/sim/mid360/ground_truth_odom")
    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=0.0,
        help="wall seconds without ROS-clock progress; 0 selects a conservative limit",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryRecorder(args.estimate_topic, args.truth_topic)
    wall_timeout_s = (
        args.wall_timeout if args.wall_timeout > 0.0
        else max(args.duration * 10.0, args.duration + 60.0)
    )
    duration_ros_s, duration_wall_s = record_for_ros_duration(
        node, args.duration, wall_timeout_s
    )
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    write_tum(output / "estimate.tum", node.estimate)
    write_tum(output / "ground_truth.tum", node.truth)
    print(
        f"estimate_samples={len(node.estimate)} truth_samples={len(node.truth)} "
        f"invalid_header_stamps={node.invalid_header_stamp_counts} "
        f"duration_ros_s={duration_ros_s:.3f} duration_wall_s={duration_wall_s:.3f} "
        f"output={output}"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if len(node.estimate) >= 10 and len(node.truth) >= 10 else 1


if __name__ == "__main__":
    raise SystemExit(main())
