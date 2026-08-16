#!/usr/bin/env python3
"""Replay visual/IMU time calibration in-process over a frozen rosbag."""

import argparse
from collections import Counter
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from uf_backend_fusion.visual_initialization import OnlineVisualTimeCalibrator
from uf_backend_fusion.manifold import so3_log
from uf_backend_fusion.spatiotemporal_calibration import (
    _integrate_prepared_gyro_vector,
    _prepare_gyro_interpolation,
    _prepare_gyro_vector_integral,
    estimate_interval_time_offset,
)


DEFAULT_ROTATION_BODY_CAMERA = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=float)


def stamp_seconds(stamp):
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def read_inputs(bag_path, visual_topic, imu_topic, imu_stamp_offset_s):
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    for topic in (visual_topic, imu_topic):
        if topic not in topic_types:
            raise RuntimeError(f"required topic missing: {topic}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (visual_topic, imu_topic)
    }
    visual = []
    imu = []
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic == visual_topic:
            visual.append(deserialize_message(payload, message_types[topic]))
        elif topic == imu_topic:
            message = deserialize_message(payload, message_types[topic])
            imu.append(SimpleNamespace(
                stamp_s=(
                    stamp_seconds(message.header.stamp)
                    + float(imu_stamp_offset_s)
                ),
                angular_velocity=np.array([
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ], dtype=float),
            ))
    visual.sort(key=lambda item: stamp_seconds(item.header.stamp))
    imu.sort(key=lambda item: item.stamp_s)
    return visual, imu


def _summary(values):
    array = np.asarray(values, dtype=float)
    if not array.size:
        return {"count": 0}
    return {
        "count": int(array.size),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95.0)),
        "maximum": float(np.max(array)),
    }


def rotation_convention_diagnostics(visual, imu, offset_range_s=0.120):
    gyro_stamps, gyro_values = _prepare_gyro_interpolation(imu)
    gyro_integrals, gyro_segments = _prepare_gyro_vector_integral(
        gyro_stamps, gyro_values
    )
    rotation = DEFAULT_ROTATION_BODY_CAMERA
    conventions = {
        "camera_to_body_physical": lambda value: (
            rotation @ value.T @ rotation.T
        ),
        "camera_to_body_pnp_direct": lambda value: (
            rotation @ value @ rotation.T
        ),
        "body_to_camera_physical": lambda value: (
            rotation.T @ value.T @ rotation
        ),
        "body_to_camera_pnp_direct": lambda value: (
            rotation.T @ value @ rotation
        ),
    }
    intervals = {name: [] for name in conventions}
    imu_vectors = []
    visual_vectors = {name: [] for name in conventions}
    for message in visual:
        if not bool(message.pnp_valid):
            continue
        previous = stamp_seconds(message.previous_stamp)
        current = stamp_seconds(message.header.stamp)
        if not 0.03 <= current - previous <= 0.50:
            continue
        pnp_rotation = np.asarray(
            message.pnp_rotation_previous_to_current, dtype=float
        ).reshape(3, 3)
        imu_vector = _integrate_prepared_gyro_vector(
            gyro_stamps,
            gyro_values,
            gyro_integrals,
            gyro_segments,
            previous,
            current,
        )
        if imu_vector is None:
            continue
        imu_vectors.append(imu_vector)
        for name, transform in conventions.items():
            vector = so3_log(transform(pnp_rotation))
            visual_vectors[name].append(vector)
            intervals[name].append((previous, current, vector, 1.0))
    imu_array = np.asarray(imu_vectors, dtype=float)
    report = {
        "paired_intervals": int(len(imu_vectors)),
        "imu_rotation_norm_rad": _summary(
            np.linalg.norm(imu_array, axis=1) if imu_array.size else []
        ),
        "conventions": {},
    }
    offsets = np.arange(
        -float(offset_range_s), float(offset_range_s) + 0.001, 0.002
    )
    for name in conventions:
        visual_array = np.asarray(visual_vectors[name], dtype=float)
        if not visual_array.size or not imu_array.size:
            report["conventions"][name] = {"count": 0}
            continue
        visual_centered = visual_array - np.mean(visual_array, axis=0)
        imu_centered = imu_array - np.mean(imu_array, axis=0)
        denominator = np.linalg.norm(visual_centered) * np.linalg.norm(
            imu_centered
        )
        correlation = (
            float(np.sum(visual_centered * imu_centered) / denominator)
            if denominator > 1.0e-12 else -1.0
        )
        denominator_scale = float(np.sum(imu_array * imu_array))
        scale = (
            float(np.sum(visual_array * imu_array) / denominator_scale)
            if denominator_scale > 1.0e-12 else 0.0
        )
        candidate = estimate_interval_time_offset(
            intervals[name],
            imu,
            offsets,
            minimum_pairs=8,
            minimum_peak_separation_s=0.010,
        )
        report["conventions"][name] = {
            "zero_offset_centered_correlation": correlation,
            "signed_least_squares_scale": scale,
            "vector_rmse_rad": float(np.sqrt(np.mean(
                np.sum((visual_array - imu_array) ** 2, axis=1)
            ))),
            "visual_rotation_norm_rad": _summary(
                np.linalg.norm(visual_array, axis=1)
            ),
            "best_offset_candidate": {
                "valid": bool(candidate.valid),
                "offset_s": float(candidate.offset_s),
                "correlation": float(candidate.correlation),
                "margin": float(candidate.margin),
                "pair_count": int(candidate.pair_count),
                "reason": str(candidate.reason),
            },
        }
    return report


def run_replay(
        visual, imu, expected_offset_s=None, offset_range_s=0.120):
    calibrator = OnlineVisualTimeCalibrator(
        initial_offset_s=0.0,
        window_s=12.0,
        minimum_pairs=8,
        offset_range_s=float(offset_range_s),
        offset_step_s=0.002,
        minimum_correlation=0.65,
        minimum_correlation_margin=0.002,
        minimum_peak_separation_s=0.010,
        reject_boundary_candidates=True,
        history_length=4,
        lock_count=3,
        stability_tolerance_s=0.006,
        minimum_accumulated_rotation_rad=0.10,
        minimum_interval_rotation_rad=0.001,
        minimum_lock_candidate_separation_s=1.0,
        minimum_interval_s=0.03,
        maximum_interval_s=0.50,
    )
    pnp_valid = 0
    invalid_rotation = 0
    reasons = Counter()
    accepted_candidates = []
    first_lock = None
    for message in visual:
        if not bool(message.pnp_valid):
            continue
        pnp_valid += 1
        try:
            rotation = np.asarray(
                message.pnp_rotation_previous_to_current, dtype=float
            ).reshape(3, 3)
            update = calibrator.update(
                stamp_seconds(message.previous_stamp),
                stamp_seconds(message.header.stamp),
                rotation,
                imu,
                DEFAULT_ROTATION_BODY_CAMERA,
            )
        except (ValueError, np.linalg.LinAlgError):
            invalid_rotation += 1
            continue
        reasons[update.reason] += 1
        if update.accepted:
            accepted_candidates.append({
                "stamp_s": stamp_seconds(message.header.stamp),
                "offset_s": float(update.candidate_offset_s),
                "locked_offset_s": float(update.time_offset_s),
                "correlation": float(update.correlation),
                "margin": float(update.margin),
                "pair_count": int(update.pair_count),
                "reason": str(update.reason),
            })
        if update.locked and first_lock is None:
            first_lock = {
                "stamp_s": stamp_seconds(message.header.stamp),
                "offset_s": float(update.time_offset_s),
                "correlation": float(update.correlation),
                "margin": float(update.margin),
                "pair_count": int(update.pair_count),
            }
    final = calibrator.last_update
    error_s = None
    if final.locked and expected_offset_s is not None:
        error_s = float(final.time_offset_s) - float(expected_offset_s)
    return {
        "visual_messages": len(visual),
        "pnp_valid_messages": pnp_valid,
        "invalid_rotations": invalid_rotation,
        "imu_samples": len(imu),
        "time_locked": bool(final.locked),
        "expected_offset_s": (
            float(expected_offset_s)
            if expected_offset_s is not None else None
        ),
        "estimated_offset_s": float(final.time_offset_s),
        "locked_error_s": error_s,
        "first_lock": first_lock,
        "accepted_candidates": accepted_candidates,
        "reason_counts": dict(sorted(reasons.items())),
        "final_update": {
            "accepted": bool(final.accepted),
            "locked": bool(final.locked),
            "time_offset_s": float(final.time_offset_s),
            "correlation": float(final.correlation),
            "margin": float(final.margin),
            "pair_count": int(final.pair_count),
            "reason": str(final.reason),
        },
        "rotation_convention_diagnostics": rotation_convention_diagnostics(
            visual, imu, offset_range_s
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument(
        "--visual-topic", default="/vision/feature_tracks"
    )
    parser.add_argument("--imu-topic", default="/sensors/imu")
    parser.add_argument(
        "--imu-stamp-offset-s",
        type=float,
        default=0.0,
        help=(
            "Synthetic shift added to every IMU stamp. This is an injected "
            "delta, not the absolute expected camera-to-IMU offset."
        ),
    )
    parser.add_argument(
        "--expected-offset-s",
        type=float,
        help=(
            "Optional absolute expected offset after injection, using "
            "t_imu=t_camera+td_C. Omit when only checking relative shifts."
        ),
    )
    parser.add_argument(
        "--offset-range-s",
        type=float,
        default=0.120,
        help="Symmetric time-offset search range; mirrors the online backend.",
    )
    parser.add_argument("--output")
    args = parser.parse_args()
    visual, imu = read_inputs(
        Path(args.bag),
        args.visual_topic,
        args.imu_topic,
        args.imu_stamp_offset_s,
    )
    report = {
        "bag": str(Path(args.bag).resolve()),
        "synthetic_imu_stamp_shift_s": float(args.imu_stamp_offset_s),
        "result": run_replay(
            visual, imu, args.expected_offset_s, args.offset_range_s
        ),
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
