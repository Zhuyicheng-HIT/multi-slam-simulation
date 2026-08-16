#!/usr/bin/env python3
"""Compare raw LiDAR calibration motions with simulation truth.

Gazebo truth is evaluation-only. This tool never publishes data and is not
part of the estimator graph.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_seconds(stamp):
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def quaternion_array(quaternion):
    values = np.array([
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    ], dtype=float)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("invalid quaternion")
    return values / norm


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion_array(quaternion)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def quaternion_slerp(left, right, ratio):
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    dot = float(left @ right)
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        value = left + float(ratio) * (right - left)
        return value / np.linalg.norm(value)
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - float(ratio)) * angle) / sine * left
        + math.sin(float(ratio) * angle) / sine * right
    )


def array_matrix(quaternion):
    class Quaternion:
        pass

    message = Quaternion()
    message.x, message.y, message.z, message.w = quaternion
    return quaternion_matrix(message)


def rotation_vector(rotation):
    trace = float(np.trace(rotation))
    angle = math.acos(float(np.clip(0.5 * (trace - 1.0), -1.0, 1.0)))
    if angle <= 1.0e-9:
        return np.zeros(3)
    skew = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    if abs(math.sin(angle)) <= 1.0e-8:
        eigenvalues, eigenvectors = np.linalg.eigh(rotation)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        return angle * axis / max(1.0e-12, np.linalg.norm(axis))
    return angle * skew / (2.0 * math.sin(angle))


def interpolate_truth(stamps, quaternions, stamp_s):
    index = int(np.searchsorted(stamps, float(stamp_s)))
    if index == 0 or index >= len(stamps):
        return None
    left_stamp = float(stamps[index - 1])
    right_stamp = float(stamps[index])
    if right_stamp <= left_stamp or right_stamp - left_stamp > 0.25:
        return None
    ratio = (float(stamp_s) - left_stamp) / (right_stamp - left_stamp)
    return array_matrix(quaternion_slerp(
        quaternions[index - 1], quaternions[index], ratio
    ))


def percentile(values, percentage):
    return float(np.percentile(np.asarray(values, dtype=float), percentage))


def summarize(records):
    accepted = [record for record in records if record["accepted"]]
    compared = [record for record in accepted if record["truth_angle_rad"] is not None]
    if not compared:
        raise RuntimeError("no accepted calibration motions overlap truth")
    estimated = np.asarray([record["estimated_angle_rad"] for record in compared])
    truth = np.asarray([record["truth_angle_rad"] for record in compared])
    nonzero = truth > 1.0e-4
    ratios = estimated[nonzero] / truth[nonzero]
    vector_errors = np.asarray([
        record["rotation_vector_error_rad"] for record in compared
    ])
    return {
        "messages": len(records),
        "accepted": len(accepted),
        "compared": len(compared),
        "truth_frame_id": records[0]["truth_frame_id"],
        "truth_child_frame_id": records[0]["truth_child_frame_id"],
        "estimated_angle_rad": {
            "median": float(np.median(estimated)),
            "p95": percentile(estimated, 95.0),
            "sum": float(np.sum(estimated)),
        },
        "truth_angle_rad": {
            "median": float(np.median(truth)),
            "p95": percentile(truth, 95.0),
            "sum": float(np.sum(truth)),
        },
        "estimated_to_truth_angle_ratio": {
            "median": float(np.median(ratios)),
            "p05": percentile(ratios, 5.0),
            "p95": percentile(ratios, 95.0),
        },
        "rotation_vector_error_rad": {
            "rmse": float(np.sqrt(np.mean(vector_errors ** 2))),
            "p95": percentile(vector_errors, 95.0),
            "max": float(np.max(vector_errors)),
        },
        "records": records,
    }


def read_bag(bag_path, motion_topic, truth_topic):
    # The validation bags use rosbag2 file compression (zstd). The compression
    # reader also accepts ordinary sqlite3 bags, so one path covers both.
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    for topic in (motion_topic, truth_topic):
        if topic not in topic_types:
            raise RuntimeError(f"required topic missing: {topic}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (motion_topic, truth_topic)
    }
    motions = []
    truth = []
    truth_frames = ("", "")
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic == truth_topic:
            message = deserialize_message(payload, message_types[topic])
            truth.append((
                stamp_seconds(message.header.stamp),
                quaternion_array(message.pose.pose.orientation),
            ))
            truth_frames = (message.header.frame_id, message.child_frame_id)
        elif topic == motion_topic:
            message = deserialize_message(payload, message_types[topic])
            motions.append(message)
    truth.sort(key=lambda item: item[0])
    stamps = np.asarray([item[0] for item in truth], dtype=float)
    quaternions = [item[1] for item in truth]
    records = []
    for message in motions:
        start_s = stamp_seconds(message.start_stamp)
        end_s = stamp_seconds(message.header.stamp)
        estimated_rotation = quaternion_matrix(message.relative_rotation)
        start_rotation = interpolate_truth(stamps, quaternions, start_s)
        end_rotation = interpolate_truth(stamps, quaternions, end_s)
        truth_angle = None
        vector_error = None
        if start_rotation is not None and end_rotation is not None:
            truth_vector = rotation_vector(start_rotation.T @ end_rotation)
            truth_angle = float(np.linalg.norm(truth_vector))
            vector_error = float(np.linalg.norm(
                rotation_vector(estimated_rotation) - truth_vector
            ))
        records.append({
            "start_s": start_s,
            "end_s": end_s,
            "duration_s": end_s - start_s,
            "accepted": bool(message.accepted),
            "reason": str(message.reason),
            "estimated_angle_rad": float(np.linalg.norm(
                rotation_vector(estimated_rotation)
            )),
            "truth_angle_rad": truth_angle,
            "rotation_vector_error_rad": vector_error,
            "fitness_score": float(message.fitness_score),
            "inlier_ratio": float(message.inlier_ratio),
            "truth_frame_id": truth_frames[0],
            "truth_child_frame_id": truth_frames[1],
        })
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument(
        "--motion-topic", default="/calibration/lidar_relative_motion"
    )
    parser.add_argument(
        "--truth-topic", default="/sim/mid360/ground_truth_odom"
    )
    parser.add_argument("--output")
    args = parser.parse_args()
    report = summarize(read_bag(
        Path(args.bag), args.motion_topic, args.truth_topic
    ))
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    console_report = dict(report)
    console_report.pop("records", None)
    print(json.dumps(console_report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
