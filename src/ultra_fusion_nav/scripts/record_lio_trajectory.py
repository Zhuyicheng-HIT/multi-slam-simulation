#!/usr/bin/env python3
import argparse
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def tum_row(msg):
    stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class TrajectoryRecorder(Node):
    def __init__(self, estimate_topic, truth_topic):
        super().__init__("uf_trajectory_recorder")
        self.estimate = []
        self.truth = []
        self.create_subscription(Odometry, estimate_topic, lambda msg: self.estimate.append(tum_row(msg)), 20)
        self.create_subscription(Odometry, truth_topic, lambda msg: self.truth.append(tum_row(msg)), qos_profile_sensor_data)


def write_tum(path, rows):
    with path.open("w", encoding="ascii") as handle:
        for row in rows:
            handle.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Record evaluator-only LIO and truth trajectories in TUM format")
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--estimate-topic", default="/lio/odom")
    parser.add_argument("--truth-topic", default="/sim/mid360/ground_truth_odom")
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryRecorder(args.estimate_topic, args.truth_topic)
    started = time.monotonic()
    while rclpy.ok() and time.monotonic() - started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    write_tum(output / "estimate.tum", node.estimate)
    write_tum(output / "ground_truth.tum", node.truth)
    print(f"estimate_samples={len(node.estimate)} truth_samples={len(node.truth)} output={output}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if len(node.estimate) >= 10 and len(node.truth) >= 10 else 1


if __name__ == "__main__":
    raise SystemExit(main())
