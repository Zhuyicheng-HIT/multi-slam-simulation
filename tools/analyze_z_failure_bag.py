#!/usr/bin/env python3
"""Capture factor and reliability state at causal vertical-error crossings."""

import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DIAGNOSTIC_KEYS = (
    "gnss_admission_reason",
    "gnss_prefit_residual_xyz_m",
    "gnss_prefit_xy_nis",
    "gnss_prefit_z_nis",
    "gnss_xy_information_scale",
    "gnss_z_information_scale",
    "gnss_z_admitted",
    "gnss_z_rejected_nis",
    "gnss_z_robust_downweighted",
    "native_lidar_observability_degradation_xyz",
    "native_lidar_axis_information_scale_xyz",
    "native_lidar_axis_relative_support_xyz",
    "native_lidar_weakest_translation_direction_xyz",
    "native_lidar_matches",
    "visual_factors",
    "visual_solver_accepted",
    "visual_solver_rejected",
    "visual_state_consistency_rejected",
    "flow_factors",
    "barometer_factors",
    "optimization_rollbacks",
    "native_worker_queue_overflow",
    "output_position_variance_m2",
    "output_source_age_s",
)


def stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def nearest(samples, stamp_s):
    if not samples:
        return None
    stamps = [sample[0] for sample in samples]
    index = bisect_left(stamps, stamp_s)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda sample: abs(sample[0] - stamp_s))


def threshold_crossings(samples_path, thresholds):
    with samples_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    result = []
    for threshold in thresholds:
        for row in rows:
            if abs(float(row["error_z_m"])) < threshold:
                continue
            result.append(
                {
                    "threshold_m": threshold,
                    "stamp_s": float(row["stamp_s"]),
                    "error_xyz_m": [
                        float(row["error_x_m"]),
                        float(row["error_y_m"]),
                        float(row["error_z_m"]),
                    ],
                    "error_3d_m": float(row["error_3d_m"]),
                }
            )
            break
    return result


def read_evidence(bag_path):
    topics = {
        "/fusion/unified/diagnostics",
        "/reliability/scheduler_state",
        "/reliability/lidar_score",
        "/reliability/imu_score",
        "/reliability/gnss_score",
        "/reliability/optical_flow_score",
        "/reliability/vision_score",
        "/reliability/vision_factor_score",
    }
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    available = topics & set(types)
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(available)))
    message_types = {topic: get_message(types[topic]) for topic in available}
    evidence = {topic: [] for topic in available}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, message_types[topic])
        stamp_s = stamp_seconds(message)
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            continue
        if topic == "/fusion/unified/diagnostics":
            values = {}
            for status in message.status:
                for entry in status.values:
                    if entry.key in DIAGNOSTIC_KEYS:
                        values[entry.key] = entry.value
            evidence[topic].append((stamp_s, values))
        elif topic == "/reliability/scheduler_state":
            modalities = {}
            for index, name in enumerate(message.modality_names):
                modalities[name] = {
                    "degradation": float(message.degradation_scores[index]),
                    "weight": float(message.reliability_weights[index]),
                    "inflation": float(message.covariance_inflation[index]),
                    "enabled": bool(message.factor_enabled[index]),
                }
            evidence[topic].append(
                (
                    stamp_s,
                    {
                        "health_state": message.health_state,
                        "estimator_support": float(message.estimator_support),
                        "modalities": modalities,
                    },
                )
            )
        else:
            evidence[topic].append(
                (
                    stamp_s,
                    {
                        "modality": message.modality,
                        "degradation": float(message.degradation_score),
                        "weight": float(message.reliability_weight),
                        "valid": bool(message.valid),
                        "reasons": list(message.reasons),
                    },
                )
            )
    for samples in evidence.values():
        samples.sort(key=lambda item: item[0])
    return evidence


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    crossings = threshold_crossings(args.samples, [0.2, 0.5, 1.0, 2.0, 4.0])
    evidence = read_evidence(args.bag)
    for crossing in crossings:
        crossing["evidence"] = {}
        for topic, samples in evidence.items():
            sample = nearest(samples, crossing["stamp_s"])
            if sample is None:
                continue
            crossing["evidence"][topic] = {
                "stamp_s": sample[0],
                "age_s": crossing["stamp_s"] - sample[0],
                "values": sample[1],
            }
    result = {
        "bag": str(args.bag.resolve()),
        "crossings": crossings,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
