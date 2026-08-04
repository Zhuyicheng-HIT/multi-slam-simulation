#!/usr/bin/env python3
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def stamp_seconds(header):
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_odom(samples, stamp, max_gap_s=0.5):
    times = [sample[0] for sample in samples]
    index = bisect.bisect_left(times, stamp)
    if index == 0 or index >= len(samples):
        return None
    before = samples[index - 1]
    after = samples[index]
    dt = after[0] - before[0]
    if dt <= 0.0 or dt > max_gap_s:
        return None
    ratio = (stamp - before[0]) / dt
    if ratio < 0.0 or ratio > 1.0:
        return None
    yaw_delta = wrap_angle(after[3] - before[3])
    return np.asarray([
        before[1] + ratio * (after[1] - before[1]),
        before[2] + ratio * (after[2] - before[2]),
        wrap_angle(before[3] + ratio * yaw_delta),
    ])


def build_pairs(odom_samples, flow_samples, min_motion_m=0.002, max_gap_s=0.5):
    odom_samples = sorted(odom_samples)
    pairs = []
    for sample in flow_samples:
        (stamp, integration_s, output_interval_s, flow_x, flow_y,
         quality, distance, gyro_x, gyro_y, gyro_z) = sample
        if output_interval_s <= 1.0e-4 or quality <= 0 or distance <= 0.0:
            continue
        current = interpolate_odom(odom_samples, stamp, max_gap_s)
        previous = interpolate_odom(odom_samples, stamp - output_interval_s, max_gap_s)
        if current is None or previous is None:
            continue
        world_delta = current[:2] - previous[:2]
        yaw = current[2]
        body_delta = np.asarray([
            math.cos(yaw) * world_delta[0] + math.sin(yaw) * world_delta[1],
            -math.sin(yaw) * world_delta[0] + math.cos(yaw) * world_delta[1],
        ])
        if float(np.linalg.norm(body_delta)) < min_motion_m:
            continue
        flow_delta = np.asarray([
            (flow_x - gyro_x) * distance,
            (flow_y - gyro_y) * distance,
        ])
        yaw_rate_rad_s = abs(gyro_z) / integration_s
        pairs.append({
            "stamp_s": stamp,
            "source_integration_s": integration_s,
            "output_interval_s": output_interval_s,
            "output_to_integration_ratio": output_interval_s / integration_s,
            "quality": quality,
            "distance_m": distance,
            "yaw_rate_rad_s": yaw_rate_rad_s,
            "integrated_xgyro_rad": gyro_x,
            "integrated_ygyro_rad": gyro_y,
            "integrated_zgyro_rad": gyro_z,
            "flow_x_m_raw": float(flow_delta[0]),
            "flow_y_m_raw": float(flow_delta[1]),
            "body_dx_m": float(body_delta[0]),
            "body_dy_m": float(body_delta[1]),
        })
    return pairs


def correlation(a, b):
    if len(a) < 3 or float(np.std(a)) < 1.0e-9 or float(np.std(b)) < 1.0e-9:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def fit_candidates(pairs):
    if not pairs:
        return []
    flow = np.asarray([[row["flow_x_m_raw"], row["flow_y_m_raw"]] for row in pairs])
    body = np.asarray([[row["body_dx_m"], row["body_dy_m"]] for row in pairs])
    body_scale = max(1.0e-9, float(np.median(np.linalg.norm(body, axis=1))))
    candidates = []
    for swap in (False, True):
        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                mapped = flow[:, ::-1] if swap else flow.copy()
                mapped = mapped * np.asarray([sign_x, sign_y])
                denominator = float(np.sum(mapped * mapped))
                if denominator <= 1.0e-12:
                    continue
                scale = max(0.0, float(np.sum(mapped * body) / denominator))
                predicted = scale * mapped
                error = predicted - body
                rmse = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))
                candidates.append({
                    "swap_xy": swap,
                    "sign_x": sign_x,
                    "sign_y": sign_y,
                    "scale": scale,
                    "rmse_m": rmse,
                    "normalized_rmse": rmse / body_scale,
                    "correlation_flat": correlation(predicted.ravel(), body.ravel()),
                    "correlation_x": correlation(predicted[:, 0], body[:, 0]),
                    "correlation_y": correlation(predicted[:, 1], body[:, 1]),
                })
    return sorted(candidates, key=lambda value: (value["normalized_rmse"], -value["correlation_flat"]))


def sample_range(samples):
    if not samples:
        return None
    values = [sample[0] for sample in samples]
    return {"min_s": min(values), "max_s": max(values), "span_s": max(values) - min(values)}


class FlowLioRecorder(Node):
    def __init__(self):
        super().__init__(
            "flow_lio_calibration",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.odom_samples = []
        self.flow_samples = []
        self.invalid_header_stamp_counts = {"odom": 0, "flow": 0}
        self.last_flow_stamp = None
        self.create_subscription(Odometry, "/lio/odom", self._odom, 50)
        self.create_subscription(
            OpticalFlowRad, "/sensors/optical_flow/rad", self._flow, qos_profile_sensor_data
        )

    def _odom(self, msg):
        stamp = stamp_seconds(msg.header)
        if stamp <= 0.0:
            self.invalid_header_stamp_counts["odom"] += 1
            return
        pose = msg.pose.pose
        self.odom_samples.append((
            stamp,
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        ))

    def _flow(self, msg):
        stamp = stamp_seconds(msg.header)
        if stamp <= 0.0:
            self.invalid_header_stamp_counts["flow"] += 1
            return
        output_interval = 0.0 if self.last_flow_stamp is None else stamp - self.last_flow_stamp
        self.last_flow_stamp = stamp
        self.flow_samples.append((
            stamp,
            float(msg.integration_time_us) * 1.0e-6,
            output_interval,
            float(msg.integrated_x),
            float(msg.integrated_y),
            int(msg.quality),
            float(msg.distance),
            float(msg.integrated_xgyro),
            float(msg.integrated_ygyro),
            float(msg.integrated_zgyro),
        ))


def record_for_ros_duration(node, duration_s, wall_timeout_s):
    wall_started = time.monotonic()
    last_progress_wall = wall_started
    ros_started_ns = None
    last_ros_ns = None
    elapsed_ros_s = 0.0
    elapsed_wall_s = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now_ros_ns = node.get_clock().now().nanoseconds
            now_wall = time.monotonic()
            if now_ros_ns > 0 and ros_started_ns is None:
                ros_started_ns = now_ros_ns
            if last_ros_ns is not None and now_ros_ns < last_ros_ns:
                raise RuntimeError("ROS clock moved backwards during optical-flow calibration")
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    return elapsed_ros_s, time.monotonic() - wall_started


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--min-quality", type=int, default=80)
    parser.add_argument("--min-distance", type=float, default=0.6)
    parser.add_argument("--max-distance", type=float, default=12.0)
    parser.add_argument("--max-yaw-rate", type=float, default=0.10)
    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=0.0,
        help="wall seconds without ROS-clock progress; 0 selects a conservative limit",
    )
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = FlowLioRecorder()
    wall_timeout_s = (
        args.wall_timeout if args.wall_timeout > 0.0
        else max(args.duration * 10.0, args.duration + 60.0)
    )
    duration_ros_s, duration_wall_s = record_for_ros_duration(
        node, args.duration, wall_timeout_s
    )

    filtered_flows = [
        sample for sample in node.flow_samples
        if sample[5] >= args.min_quality
        and args.min_distance <= sample[6] <= args.max_distance
        and abs(sample[9]) / max(1.0e-6, sample[1]) <= args.max_yaw_rate
    ]
    pairs = build_pairs(node.odom_samples, filtered_flows)
    candidates = fit_candidates(pairs)
    best = candidates[0] if candidates else None
    timing_ratios = [row["output_to_integration_ratio"] for row in pairs]
    timing_ratio_median = float(np.median(timing_ratios)) if timing_ratios else float("nan")
    timing_consistent = bool(
        timing_ratios
        and math.isfinite(timing_ratio_median)
        and 0.75 <= timing_ratio_median <= 1.25
    )
    passed = bool(
        best and len(pairs) >= 80
        and best["correlation_flat"] >= 0.50
        and best["normalized_rmse"] <= 0.75
        and 0.70 <= best["scale"] <= 1.30
    )
    result = {
        "duration_s": duration_ros_s,
        "duration_ros_s": duration_ros_s,
        "duration_wall_s": duration_wall_s,
        "requested_duration_ros_s": args.duration,
        "wall_timeout_s": wall_timeout_s,
        "wall_stall_timeout_s": wall_timeout_s,
        "invalid_header_stamp_counts": dict(node.invalid_header_stamp_counts),
        "odom_samples": len(node.odom_samples),
        "flow_samples": len(node.flow_samples),
        "filtered_flow_samples": len(filtered_flows),
        "max_yaw_rate_rad_s": args.max_yaw_rate,
        "matched_pairs": len(pairs),
        "timing": {
            "clock_model": "ros_source_header_stamp",
            "association_basis": "source_header_stamp",
            "output_to_integration_ratio_median": timing_ratio_median,
            "consistent": timing_consistent,
            "note": (
                "Output intervals and integration intervals are both measured in "
                "the ROS/source time domain; wall arrival timing is excluded."
            ),
        },
        "stamp_ranges": {
            "odom": sample_range(node.odom_samples),
            "flow": sample_range(node.flow_samples),
        },
        "best": best,
        "candidates": candidates,
        "acceptance": {
            "min_pairs": 80,
            "min_correlation_flat": 0.50,
            "max_normalized_rmse": 0.75,
            "scale_range": [0.70, 1.30],
        },
        "passed": passed,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = list(pairs[0].keys()) if pairs else (
            "stamp_s", "source_integration_s", "output_interval_s", "quality", "distance_m",
            "yaw_rate_rad_s", "integrated_xgyro_rad", "integrated_ygyro_rad",
            "integrated_zgyro_rad", "output_to_integration_ratio",
            "flow_x_m_raw", "flow_y_m_raw", "body_dx_m", "body_dy_m",
        )
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(pairs)
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed or not args.require_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
