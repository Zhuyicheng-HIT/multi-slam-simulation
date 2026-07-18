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
        super().__init__("flow_lio_calibration")
        self.odom_samples = []
        self.flow_samples = []
        self.last_flow_stamp = None
        self.create_subscription(Odometry, "/lio/odom", self._odom, 50)
        self.create_subscription(
            OpticalFlowRad, "/sensors/optical_flow/rad", self._flow, qos_profile_sensor_data
        )

    def _odom(self, msg):
        pose = msg.pose.pose
        self.odom_samples.append((
            stamp_seconds(msg.header),
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        ))

    def _flow(self, msg):
        stamp = stamp_seconds(msg.header)
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--min-quality", type=int, default=80)
    parser.add_argument("--min-distance", type=float, default=0.6)
    parser.add_argument("--max-distance", type=float, default=12.0)
    parser.add_argument("--max-yaw-rate", type=float, default=0.10)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = FlowLioRecorder()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

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
    timing_consistent = bool(timing_ratios and math.isfinite(timing_ratio_median))
    passed = bool(
        best and len(pairs) >= 80
        and best["correlation_flat"] >= 0.50
        and best["normalized_rmse"] <= 0.75
        and 0.70 <= best["scale"] <= 1.30
    )
    result = {
        "duration_s": args.duration,
        "odom_samples": len(node.odom_samples),
        "flow_samples": len(node.flow_samples),
        "filtered_flow_samples": len(filtered_flows),
        "max_yaw_rate_rad_s": args.max_yaw_rate,
        "matched_pairs": len(pairs),
        "timing": {
            "clock_model": "gazebo_integration_with_ros_wall_output_stamp",
            "output_to_integration_ratio_median": timing_ratio_median,
            "consistent": timing_consistent,
            "note": "Ratio follows Gazebo real-time factor and is diagnostic, not an acceptance gate.",
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
