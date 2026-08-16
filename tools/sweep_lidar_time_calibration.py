#!/usr/bin/env python3
"""Replay LiDAR/IMU calibration in-process over a frozen rosbag."""

import argparse
from bisect import bisect_left, bisect_right
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from uf_backend_fusion.online_backend import (
    lidar_calibration_motion_from_message,
)
from uf_backend_fusion.spatiotemporal_calibration import (
    OnlineSpatiotemporalCalibrator,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def read_inputs(bag_path, motion_topic, imu_topic):
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    for topic in (motion_topic, imu_topic):
        if topic not in topic_types:
            raise RuntimeError(f"required topic missing: {topic}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (motion_topic, imu_topic)
    }
    motions = []
    imu = []
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic == motion_topic:
            motions.append(deserialize_message(payload, message_types[topic]))
        elif topic == imu_topic:
            message = deserialize_message(payload, message_types[topic])
            imu.append(SimpleNamespace(
                stamp_s=stamp_seconds(message.header.stamp),
                angular_velocity=np.array([
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ], dtype=float),
            ))
    motions.sort(key=lambda item: stamp_seconds(item.header.stamp))
    imu.sort(key=lambda item: item.stamp_s)
    return motions, imu


def run_sweep(motions, imu, solve_period_s):
    calibrator = OnlineSpatiotemporalCalibrator(
        window_s=12.0,
        minimum_pairs=8,
        time_offset_range_s=0.10,
        time_offset_step_s=0.005,
        minimum_correlation=0.70,
        minimum_correlation_margin=0.002,
        minimum_time_peak_separation_s=0.020,
        minimum_time_accumulated_rotation_rad=0.25,
        minimum_excitation_eigenvalue=1.0e-4,
        minimum_excitation_ratio=0.05,
        minimum_accumulated_rotation_rad=0.25,
        minimum_rotation_inlier_ratio=0.70,
        maximum_rotation_residual_rad=0.08,
        sharp_turn_rate_radps=1.5,
        solve_period_s=float(solve_period_s),
    )
    calibrator.set_initial_rotation(np.array([
        [0.984807753, 0.0, 0.173648178],
        [0.0, 1.0, 0.0],
        [-0.173648178, 0.0, 0.984807753],
    ]))
    imu_stamps = [sample.stamp_s for sample in imu]
    converted = 0
    rejected = 0
    updates = 0
    candidate_admissions = 0
    first_lock_s = None
    lock_events = []
    for message in motions:
        try:
            motion = lidar_calibration_motion_from_message(message)
        except ValueError:
            rejected += 1
            continue
        converted += 1
        left = bisect_left(imu_stamps, motion.end_s - 12.25)
        right = bisect_right(imu_stamps, motion.end_s + 0.25)
        history_before = len(calibrator.time_offset_history)
        update = calibrator.update(motion, imu[left:right])
        if update.reason != "update_throttled":
            updates += 1
        if len(calibrator.time_offset_history) > history_before:
            candidate_admissions += 1
        if calibrator.time_locked and first_lock_s is None:
            first_lock_s = motion.end_s
            lock_events.append({
                "stamp_s": motion.end_s,
                "offset_s": calibrator.time_offset_s,
                "correlation": calibrator.last_time_candidate.correlation,
                "margin": calibrator.last_time_candidate.margin,
            })
    candidate = calibrator.last_time_candidate
    return {
        "solve_period_s": float(solve_period_s),
        "motion_messages": len(motions),
        "converted": converted,
        "rejected": rejected,
        "solver_updates": updates,
        "time_candidate_admissions": candidate_admissions,
        "time_locked": bool(calibrator.time_locked),
        "first_lock_stamp_s": first_lock_s,
        "final_time_offset_s": float(calibrator.time_offset_s),
        "final_candidate": {
            "valid": bool(candidate.valid),
            "offset_s": float(candidate.offset_s),
            "correlation": float(candidate.correlation),
            "margin": float(candidate.margin),
            "reason": str(candidate.reason),
        },
        "lock_events": lock_events,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument(
        "--motion-topic", default="/calibration/lidar_relative_motion"
    )
    parser.add_argument("--imu-topic", default="/sensors/imu")
    parser.add_argument(
        "--solve-periods", default="1.0,0.5,0.3,0.2"
    )
    parser.add_argument("--output")
    args = parser.parse_args()
    periods = [float(value) for value in args.solve_periods.split(",")]
    if any(value < 0.0 for value in periods):
        raise ValueError("solve periods must be non-negative")
    motions, imu = read_inputs(Path(args.bag), args.motion_topic, args.imu_topic)
    report = {
        "bag": str(Path(args.bag).resolve()),
        "results": [run_sweep(motions, imu, value) for value in periods],
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
