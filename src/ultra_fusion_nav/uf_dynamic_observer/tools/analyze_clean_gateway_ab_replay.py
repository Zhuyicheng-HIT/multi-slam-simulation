#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path
import statistics

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


def message_stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def percentile(values, fraction):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), 100.0 * fraction))


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    messages = {name: [] for name in types}
    while reader.has_next():
        topic, data, receipt_ns = reader.read_next()
        messages[topic].append(
            (receipt_ns, deserialize_message(data, get_message(types[topic])))
        )
    return messages


def topic_with_suffix(messages, suffix):
    matches = [name for name in messages if name.endswith(suffix)]
    return matches[0] if len(matches) == 1 else None


def quaternion_yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def align_positions(estimated, truth):
    estimated = np.asarray(estimated, dtype=float)
    truth = np.asarray(truth, dtype=float)
    estimated_center = estimated.mean(axis=0)
    truth_center = truth.mean(axis=0)
    # These frozen low-altitude trajectories do not excite roll/pitch enough for
    # a full 3-D rotational alignment to be observable. A free SO(3) Umeyama fit
    # can therefore report a spurious yaw error. The evaluator contract uses an
    # SE(2) alignment plus an independent Z translation, preserving every
    # vertical error and all relative motion.
    covariance = (
        (estimated[:, :2] - estimated_center[:2]).T
        @ (truth[:, :2] - truth_center[:2])
    )
    u_matrix, _, v_transpose = np.linalg.svd(covariance)
    rotation_2d = v_transpose.T @ u_matrix.T
    if np.linalg.det(rotation_2d) < 0.0:
        v_transpose[-1, :] *= -1.0
        rotation_2d = v_transpose.T @ u_matrix.T
    rotation = np.eye(3)
    rotation[:2, :2] = rotation_2d
    translation = truth_center - rotation @ estimated_center
    return (rotation @ estimated.T).T + translation, rotation


def trajectory_metrics(odometry_messages, truth_scans):
    if not odometry_messages:
        return {"odom_count": 0, "lost": True, "reset_count": 0}
    truth_stamps = np.asarray([scan["stamp_ns"] for scan in truth_scans], dtype=np.int64)
    estimated = []
    truth = []
    estimated_yaw = []
    truth_yaw = []
    source_stamps = []
    for _, message in odometry_messages:
        stamp_ns = message_stamp_ns(message)
        truth_index = int(np.argmin(np.abs(truth_stamps - stamp_ns)))
        pose = truth_scans[truth_index]["pose_xyzyaw"]
        estimated.append(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ]
        )
        truth.append(pose[:3])
        estimated_yaw.append(quaternion_yaw(message.pose.pose.orientation))
        truth_yaw.append(pose[3])
        source_stamps.append(stamp_ns)
    aligned, rotation = align_positions(estimated, truth)
    truth_array = np.asarray(truth)
    errors = np.linalg.norm(aligned - truth_array, axis=1)
    rotation_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    yaw_errors = [
        abs(wrap_angle(value + rotation_yaw - reference))
        for value, reference in zip(estimated_yaw, truth_yaw)
    ]
    rpe_translation = []
    rpe_yaw = []
    for index in range(1, len(aligned)):
        estimated_delta = aligned[index] - aligned[index - 1]
        truth_delta = truth_array[index] - truth_array[index - 1]
        rpe_translation.append(float(np.linalg.norm(estimated_delta - truth_delta)))
        estimated_delta_yaw = wrap_angle(estimated_yaw[index] - estimated_yaw[index - 1])
        truth_delta_yaw = wrap_angle(truth_yaw[index] - truth_yaw[index - 1])
        rpe_yaw.append(abs(wrap_angle(estimated_delta_yaw - truth_delta_yaw)))
    gaps = [
        (source_stamps[index] - source_stamps[index - 1]) * 1.0e-9
        for index in range(1, len(source_stamps))
    ]
    jumps = [
        np.linalg.norm(np.asarray(estimated[index]) - np.asarray(estimated[index - 1]))
        for index in range(1, len(estimated))
    ]
    return {
        "odom_count": len(odometry_messages),
        "trajectory_completeness": len(odometry_messages) / len(truth_scans),
        "ate_rmse_m": float(math.sqrt(np.mean(errors * errors))),
        "ate_p95_m": percentile(errors.tolist(), 0.95),
        "rpe_translation_rmse_m": (
            float(math.sqrt(np.mean(np.square(rpe_translation))))
            if rpe_translation
            else None
        ),
        "rpe_yaw_rmse_deg": (
            math.degrees(math.sqrt(np.mean(np.square(rpe_yaw))))
            if rpe_yaw
            else None
        ),
        "yaw_rmse_deg": math.degrees(math.sqrt(np.mean(np.square(yaw_errors)))),
        "endpoint_error_m": float(errors[-1]),
        "max_odom_gap_s": max(gaps) if gaps else None,
        "lost": len(odometry_messages) < max(1, len(truth_scans) - 10) or any(
            gap > 0.35 for gap in gaps
        ),
        "reset_count": int(sum(jump > 1.0 for jump in jumps)),
    }


def native_factor_metrics(messages):
    residual_rms = []
    matched = []
    translation_information = []
    valid_count = 0
    for _, message in messages:
        if not message.correspondences_valid or message.matched_points == 0:
            continue
        valid_count += 1
        matched.append(int(message.matched_points))
        residuals = np.asarray(message.residuals, dtype=float)
        if residuals.size:
            residual_rms.append(float(math.sqrt(np.mean(residuals * residuals))))
        hessian = np.asarray(message.state_hessian, dtype=float).reshape(12, 12)
        variance = max(float(message.measurement_variance), 1.0e-12)
        translation_information.append(hessian[:3, :3] / variance)
    if not translation_information:
        return {"packet_count": len(messages), "effective_factor_count": 0}
    aggregate = np.mean(np.stack(translation_information), axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh(aggregate)
    weakest = eigenvectors[:, 0]
    diagonal = np.diag(aggregate)
    return {
        "packet_count": len(messages),
        "effective_factor_count": valid_count,
        "matched_points_median": float(statistics.median(matched)),
        "residual_rms_median_m": float(statistics.median(residual_rms)),
        "translation_information_xyz": diagonal.tolist(),
        "translation_information_min_eigenvalue": float(eigenvalues[0]),
        "translation_information_condition": float(
            eigenvalues[-1] / max(eigenvalues[0], 1.0e-12)
        ),
        "weakest_translation_direction_xyz": weakest.tolist(),
    }


def map_metrics(messages):
    if not messages:
        return {"map_messages": 0, "final_map_points": 0}
    cloud = messages[-1][1]
    xyz = point_cloud2.read_points_numpy(cloud, field_names=["x", "y", "z"])
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    finite = np.isfinite(xyz).all(axis=1)
    finite_xyz = xyz[finite]
    voxels = {
        tuple(np.floor(point / 0.25).astype(np.int64)) for point in finite_xyz
    }
    return {
        "map_messages": len(messages),
        "final_map_points": int(xyz.shape[0]),
        "finite_map_ratio": float(np.mean(finite)) if xyz.size else 1.0,
        "occupied_voxels_0_25m": len(voxels),
        "z_span_m": (
            float(finite_xyz[:, 2].max() - finite_xyz[:, 2].min())
            if finite_xyz.size
            else 0.0
        ),
    }


def expand_voxels(voxels, radius=1):
    expanded = set()
    for x_value, y_value, z_value in voxels:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    expanded.add((x_value + dx, y_value + dy, z_value + dz))
    return expanded


def map_truth_metrics(map_messages, raw_messages, odometry_messages, truth_scans):
    """Evaluate the final FAST-LIO map against evaluator-only frozen truth.

    The map is aligned with the same low-altitude SE(2)+Z contract used for ATE.
    A one-voxel neighborhood absorbs voxel-boundary and centimetre-scale pose
    error. Dynamic voxels that overlap any static support are excluded so a
    moving object beside a wall cannot make the wall count as contamination.
    """
    if not map_messages or not raw_messages or not odometry_messages:
        return {"applicable": False}
    truth_stamps = np.asarray(
        [scan["stamp_ns"] for scan in truth_scans], dtype=np.int64
    )
    estimated = []
    truth_positions = []
    for _, message in odometry_messages:
        stamp_ns = message_stamp_ns(message)
        truth_index = int(np.argmin(np.abs(truth_stamps - stamp_ns)))
        estimated.append(
            [message.pose.pose.position.x, message.pose.pose.position.y,
             message.pose.pose.position.z]
        )
        truth_positions.append(truth_scans[truth_index]["pose_xyzyaw"][:3])
    _, rotation = align_positions(estimated, truth_positions)
    estimated_center = np.asarray(estimated, dtype=float).mean(axis=0)
    truth_center = np.asarray(truth_positions, dtype=float).mean(axis=0)
    translation = truth_center - rotation @ estimated_center

    final_cloud = map_messages[-1][1]
    map_xyz = np.asarray(
        point_cloud2.read_points_numpy(final_cloud, field_names=["x", "y", "z"]),
        dtype=float,
    ).reshape(-1, 3)
    map_xyz = map_xyz[np.isfinite(map_xyz).all(axis=1)]
    aligned_map = (rotation @ map_xyz.T).T + translation
    voxel_size = 0.25
    map_voxels = {
        tuple(np.floor(point / voxel_size).astype(np.int64))
        for point in aligned_map
    }

    truth_by_stamp = {scan["stamp_ns"]: scan for scan in truth_scans}
    static_voxels = set()
    dynamic_voxels = set()
    for _, message in raw_messages:
        stamp_ns = message_stamp_ns(message)
        scan = truth_by_stamp.get(stamp_ns)
        if scan is None:
            continue
        px, py, pz, yaw = scan["pose_xyzyaw"]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        dynamic_offsets = set(scan["dynamic_offsets"])
        for point in message.points:
            world = np.asarray([
                px + cosine * point.x - sine * point.y,
                py + sine * point.x + cosine * point.y,
                pz + point.z,
            ])
            voxel = tuple(np.floor(world / voxel_size).astype(np.int64))
            if point.offset_time in dynamic_offsets:
                dynamic_voxels.add(voxel)
            else:
                static_voxels.add(voxel)

    expanded_map = expand_voxels(map_voxels)
    expanded_static = expand_voxels(static_voxels)
    dynamic_exclusive = dynamic_voxels - expanded_static
    expanded_dynamic = expand_voxels(dynamic_exclusive)
    static_covered = sum(voxel in expanded_map for voxel in static_voxels)
    dynamic_retained = sum(voxel in expanded_map for voxel in dynamic_exclusive)
    contaminated_map = sum(
        voxel in expanded_dynamic and voxel not in expanded_static
        for voxel in map_voxels
    )
    return {
        "applicable": True,
        "static_truth_voxels": len(static_voxels),
        "dynamic_exclusive_truth_voxels": len(dynamic_exclusive),
        "static_map_completeness": (
            static_covered / len(static_voxels) if static_voxels else None
        ),
        "dynamic_trace_retention": (
            dynamic_retained / len(dynamic_exclusive) if dynamic_exclusive else None
        ),
        "map_contamination_ratio": (
            contaminated_map / len(map_voxels) if map_voxels else None
        ),
        "contaminated_map_voxels": contaminated_map,
    }
def gateway_metrics(clean_messages, status_messages, raw_input, truth_scans):
    raw_by_stamp = {message_stamp_ns(message): message for _, message in raw_input}
    clean_by_stamp = {message_stamp_ns(message): message for _, message in clean_messages}
    truth_by_stamp = {scan["stamp_ns"]: scan for scan in truth_scans}
    tp = fp = fn = tn = 0
    missing = 0
    for stamp_ns, raw in raw_by_stamp.items():
        truth = truth_by_stamp[stamp_ns]
        dynamic_offsets = set(truth["dynamic_offsets"])
        clean = clean_by_stamp.get(stamp_ns)
        if clean is None:
            missing += 1
            retained_offsets = {point.offset_time for point in raw.points}
        else:
            retained_offsets = {point.offset_time for point in clean.points}
        for point in raw.points:
            dynamic = point.offset_time in dynamic_offsets
            removed = point.offset_time not in retained_offsets
            if dynamic and removed:
                tp += 1
            elif dynamic:
                fn += 1
            elif removed:
                fp += 1
            else:
                tn += 1
    statuses = [json.loads(message.data) for _, message in status_messages]
    dynamic_count = tp + fn
    if dynamic_count == 0:
        # Pure-static scenes have no positive class. Dynamic P/R/F1 are N/A;
        # false classifications remain visible in the static metrics below.
        precision = recall = f1 = None
    else:
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / dynamic_count
        f1 = (
            2.0 * precision * recall / (precision + recall)
            if precision + recall > 0.0
            else 0.0
        )
    processing = [value["processing_ms"] for value in statuses]
    unknown = sum(value.get("unknown_points", 0) for value in statuses)
    raw_points = sum(value.get("raw_points", 0) for value in statuses)
    return {
        "dynamic_precision": precision,
        "dynamic_recall": recall,
        "dynamic_f1": f1,
        "static_preservation_rate": tn / (tn + fp) if tn + fp else None,
        "false_dynamic_ratio": fp / (tn + fp) if tn + fp else None,
        "static_map_contamination": fn / (tp + fn) if tp + fn else None,
        "map_completeness": tn / (tn + fp) if tn + fp else None,
        "unknown_ratio": unknown / raw_points if raw_points else None,
        "missing_clean_scans": missing,
        "clean_scan_count": len(clean_messages),
        "status_count": len(statuses),
        "fail_open_count": sum(value.get("fail_open", False) for value in statuses),
        "queue_overflow": max(
            [value.get("queue_overflow", 0) for value in statuses] or [0]
        ),
        "pose_timeout": max([value.get("pose_timeout", 0) for value in statuses] or [0]),
        "deskew_reject": max([value.get("deskew_reject", 0) for value in statuses] or [0]),
        "latency_p50_ms": percentile(processing, 0.50),
        "latency_p95_ms": percentile(processing, 0.95),
        "latency_p99_ms": percentile(processing, 0.99),
        "counts": {"tp": tp, "fp": fp, "fn": fn, "tn": tn},
    }


def turnaround_latency(raw_messages, odometry_messages):
    if not raw_messages or not odometry_messages:
        return {"p50_ms": None, "p95_ms": None, "p99_ms": None}
    raw = [(message_stamp_ns(message), receipt) for receipt, message in raw_messages]
    values = []
    for receipt, message in odometry_messages:
        stamp_ns = message_stamp_ns(message)
        source_stamp, source_receipt = min(raw, key=lambda item: abs(item[0] - stamp_ns))
        if abs(source_stamp - stamp_ns) <= 150_000_000 and receipt >= source_receipt:
            values.append((receipt - source_receipt) * 1.0e-6)
    return {
        "p50_ms": percentile(values, 0.50),
        "p95_ms": percentile(values, 0.95),
        "p99_ms": percentile(values, 0.99),
        "samples": len(values),
    }


def raw_dynamic_metrics(raw_input, truth_scans):
    dynamic = sum(len(scan["dynamic_offsets"]) for scan in truth_scans)
    static = sum(scan["point_count"] - len(scan["dynamic_offsets"]) for scan in truth_scans)
    return {
        "dynamic_precision": None if dynamic == 0 else 0.0,
        "dynamic_recall": None if dynamic == 0 else 0.0,
        "dynamic_f1": None if dynamic == 0 else 0.0,
        "static_preservation_rate": 1.0,
        "false_dynamic_ratio": 0.0,
        "static_map_contamination": None if dynamic == 0 else 1.0,
        "map_completeness": 1.0,
        "unknown_ratio": 0.0,
        "counts": {"tp": 0, "fp": 0, "fn": dynamic, "tn": static},
        "raw_scan_count": len(raw_input),
    }


def analyze_run(root, frozen_root, scenario, branch):
    run_dir = root / scenario / branch
    messages = read_bag(run_dir / "output")
    truth = json.loads((frozen_root / scenario / "truth.json").read_text(encoding="utf-8"))
    odom_topic = topic_with_suffix(messages, "/odom")
    factor_topic = topic_with_suffix(messages, "/native_lidar_factor")
    map_topic = topic_with_suffix(messages, "/map")
    raw_topic = topic_with_suffix(messages, "/raw")
    raw_messages = messages.get(raw_topic, [])
    if not raw_messages:
        frozen_messages = read_bag(frozen_root / scenario / "input")
        frozen_topic = topic_with_suffix(frozen_messages, "/lidar")
        raw_messages = frozen_messages[frozen_topic]
    odometry_messages = messages.get(odom_topic, [])
    map_messages = messages.get(map_topic, [])
    runtime = json.loads((run_dir / "runtime.json").read_text(encoding="utf-8"))
    result = {
        "scenario": scenario,
        "branch": branch,
        "trajectory": trajectory_metrics(odometry_messages, truth["scans"]),
        "native_lidar_factor": native_factor_metrics(messages.get(factor_topic, [])),
        "map": map_metrics(map_messages),
        "map_truth": map_truth_metrics(
            map_messages, raw_messages, odometry_messages, truth["scans"]
        ),
        "bag_recorder_turnaround_diagnostic": turnaround_latency(
            messages.get(raw_topic, []), odometry_messages
        ),
        "fast_lio_callback_latency_ms": runtime.get(
            "fast_lio_callback_latency_ms"
        ),
        "runtime": runtime,
    }
    if branch == "clean":
        clean_topic = topic_with_suffix(messages, "/clean")
        status_topic = topic_with_suffix(messages, "/gateway_status")
        result["dynamic_map"] = gateway_metrics(
            messages.get(clean_topic, []), messages.get(status_topic, []),
            raw_messages, truth["scans"]
        )
    else:
        result["dynamic_map"] = raw_dynamic_metrics(raw_messages, truth["scans"])
    return result


def aggregate(runs, branch):
    selected = [run for run in runs if run["branch"] == branch]
    counts = {key: 0 for key in ["tp", "fp", "fn", "tn"]}
    for run in selected:
        for key in counts:
            counts[key] += run["dynamic_map"]["counts"][key]
    dynamic = counts["tp"] + counts["fn"]
    precision = counts["tp"] / (counts["tp"] + counts["fp"]) if counts["tp"] + counts["fp"] else 0.0
    recall = counts["tp"] / dynamic if dynamic else None
    f1 = 2.0 * precision * recall / (precision + recall) if recall is not None and precision + recall else 0.0
    dynamic_runs = [
        run for run in selected
        if run["dynamic_map"]["dynamic_recall"] is not None
    ]
    map_truth_runs = [
        run["map_truth"] for run in selected if run["map_truth"].get("applicable")
    ]
    dynamic_map_truth_runs = [
        value for value in map_truth_runs
        if value.get("dynamic_trace_retention") is not None
    ]
    return {
        "scenario_count": len(selected),
        "dynamic_micro_precision": precision,
        "dynamic_micro_recall": recall,
        "dynamic_micro_f1": f1,
        "dynamic_macro_precision": statistics.mean(
            run["dynamic_map"]["dynamic_precision"] for run in dynamic_runs
        ),
        "dynamic_macro_recall": statistics.mean(
            run["dynamic_map"]["dynamic_recall"] for run in dynamic_runs
        ),
        "dynamic_macro_f1": statistics.mean(
            run["dynamic_map"]["dynamic_f1"] for run in dynamic_runs
        ),
        "dynamic_macro_scenario_count": len(dynamic_runs),
        "static_preservation_rate": counts["tn"] / (counts["tn"] + counts["fp"]),
        "false_dynamic_ratio": counts["fp"] / (counts["tn"] + counts["fp"]),
        "static_map_contamination": counts["fn"] / dynamic if dynamic else None,
        "ate_rmse_median_m": statistics.median(
            run["trajectory"]["ate_rmse_m"] for run in selected
        ),
        "rpe_translation_rmse_median_m": statistics.median(
            run["trajectory"]["rpe_translation_rmse_m"] for run in selected
        ),
        "native_effective_factors": sum(
            run["native_lidar_factor"]["effective_factor_count"] for run in selected
        ),
        "actual_map_contamination_mean": statistics.mean(
            value["map_contamination_ratio"] for value in map_truth_runs
        ),
        "actual_map_contamination_median": statistics.median(
            value["map_contamination_ratio"] for value in map_truth_runs
        ),
        "actual_dynamic_trace_retention_mean": statistics.mean(
            value["dynamic_trace_retention"] for value in dynamic_map_truth_runs
        ),
        "actual_static_map_completeness_mean": statistics.mean(
            value["static_map_completeness"] for value in map_truth_runs
        ),
        "counts": counts,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_root")
    parser.add_argument("frozen_manifest")
    parser.add_argument("output")
    args = parser.parse_args()
    root = Path(args.run_root).resolve()
    frozen_manifest = Path(args.frozen_manifest).resolve()
    frozen_root = frozen_manifest.parent
    manifest = json.loads(frozen_manifest.read_text(encoding="utf-8"))
    scenarios = [entry["scenario"] for entry in manifest["scenarios"]]
    runs = []
    for scenario in scenarios:
        for branch in ["raw", "clean"]:
            runs.append(analyze_run(root, frozen_root, scenario, branch))
    report = {
        "schema": "clean_gateway_fastlio_ab_v1",
        "truth_role": "evaluator_only",
        "aggregation_contract": {
            "micro": "pooled TP/FP/FN including pure-static false positives",
            "macro": "unweighted dynamic-bearing scenarios; pure-static dynamic metrics N/A",
        },
        "raw": aggregate(runs, "raw"),
        "clean": aggregate(runs, "clean"),
        "runs": runs,
    }
    output = Path(args.output).resolve()
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
