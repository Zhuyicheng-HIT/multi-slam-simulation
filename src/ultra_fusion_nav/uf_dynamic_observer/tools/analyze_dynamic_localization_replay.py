#!/usr/bin/env python3

"""Evaluate DYN-LOC-007 Raw/Clean localization in event phases."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import statistics

import numpy as np


PRIMARY_SCENARIOS = [
    "person_crossing",
    "multiple_targets",
    "small_fast_target",
    "slow_target",
    "opening_closing_door",
    "large_dynamic_occlusion",
    "radial_motion",
    "moving_then_stops",
    "near_wall_motion",
    "occlusion_appear",
]
OCCLUSION_SCENARIOS = [
    "c1_persistent_occlusion",
    "c2_same_view_reobservation",
    "c3_natural_multiview_reobservation",
]
PHASES = ["BEFORE", "DURING", "AFTER"]


def load_base_analyzer():
    source = Path(__file__).with_name("analyze_clean_gateway_ab_replay.py")
    spec = importlib.util.spec_from_file_location("dyn_loc_ab_base", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BASE = load_base_analyzer()


def rmse(values):
    return float(math.sqrt(np.mean(np.square(values)))) if values else None


def recovery_time_s(times_s, errors_m, threshold_m, consecutive=3):
    """Return first bounded recovery with consecutive healthy estimates."""
    if not times_s or len(times_s) != len(errors_m) or consecutive < 1:
        return None
    for start in range(0, len(errors_m) - consecutive + 1):
        if all(value <= threshold_m for value in errors_m[start : start + consecutive]):
            return float(times_s[start] - times_s[0])
    return None


def phase_trajectory_metrics(odometry_messages, truth):
    scans = truth["scans"]
    if not odometry_messages:
        return {name: {"odom_count": 0, "lost": True} for name in PHASES}
    truth_stamps = np.asarray([scan["stamp_ns"] for scan in scans], dtype=np.int64)
    estimated = []
    reference = []
    estimated_yaw = []
    reference_yaw = []
    source_stamps = []
    frame_indices = []
    for _, message in odometry_messages:
        stamp_ns = BASE.message_stamp_ns(message)
        index = int(np.argmin(np.abs(truth_stamps - stamp_ns)))
        pose = scans[index]["pose_xyzyaw"]
        estimated.append(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ]
        )
        reference.append(pose[:3])
        estimated_yaw.append(BASE.quaternion_yaw(message.pose.pose.orientation))
        reference_yaw.append(pose[3])
        source_stamps.append(stamp_ns)
        frame_indices.append(index)

    aligned, rotation = BASE.align_positions(estimated, reference)
    reference_array = np.asarray(reference, dtype=float)
    estimated_array = np.asarray(estimated, dtype=float)
    errors = np.linalg.norm(aligned - reference_array, axis=1)
    z_errors = np.abs(aligned[:, 2] - reference_array[:, 2])
    rotation_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    yaw_errors = np.asarray(
        [
            abs(BASE.wrap_angle(value + rotation_yaw - target))
            for value, target in zip(estimated_yaw, reference_yaw)
        ],
        dtype=float,
    )
    phase_by_frame = truth["phase_by_frame"]
    output = {}
    for phase_name in PHASES:
        selected = [
            index
            for index, frame in enumerate(frame_indices)
            if phase_by_frame[frame] == phase_name
        ]
        selected_set = set(selected)
        rpe_translation = []
        rpe_yaw = []
        pose_jump = []
        odom_gaps = []
        for index in selected:
            if index == 0 or index - 1 not in selected_set:
                continue
            estimated_delta = aligned[index] - aligned[index - 1]
            reference_delta = reference_array[index] - reference_array[index - 1]
            delta_error = float(np.linalg.norm(estimated_delta - reference_delta))
            rpe_translation.append(delta_error)
            pose_jump.append(delta_error)
            rpe_yaw.append(
                abs(
                    BASE.wrap_angle(
                        (estimated_yaw[index] - estimated_yaw[index - 1])
                        - (reference_yaw[index] - reference_yaw[index - 1])
                    )
                )
            )
            odom_gaps.append((source_stamps[index] - source_stamps[index - 1]) * 1.0e-9)
        phase_errors = [float(errors[index]) for index in selected]
        phase_z = [float(z_errors[index]) for index in selected]
        phase_yaw = [float(yaw_errors[index]) for index in selected]
        output[phase_name] = {
            "odom_count": len(selected),
            "ate_rmse_m": rmse(phase_errors),
            "ate_p95_m": BASE.percentile(phase_errors, 0.95),
            "rpe_translation_rmse_m": rmse(rpe_translation),
            "rpe_yaw_rmse_deg": (
                math.degrees(rmse(rpe_yaw)) if rpe_yaw else None
            ),
            "z_rmse_m": rmse(phase_z),
            "z_p95_m": BASE.percentile(phase_z, 0.95),
            "yaw_rmse_deg": math.degrees(rmse(phase_yaw)) if phase_yaw else None,
            "pose_jump_p95_m": BASE.percentile(pose_jump, 0.95),
            "pose_jump_max_m": max(pose_jump) if pose_jump else None,
            "max_odom_gap_s": max(odom_gaps) if odom_gaps else None,
        }

    before_errors = [
        float(errors[index])
        for index, frame in enumerate(frame_indices)
        if phase_by_frame[frame] == "BEFORE"
    ]
    after_indices = [
        index
        for index, frame in enumerate(frame_indices)
        if phase_by_frame[frame] == "AFTER"
    ]
    threshold = max(0.03, 1.5 * (BASE.percentile(before_errors, 0.95) or 0.0))
    output["AFTER"]["recovery_threshold_m"] = threshold
    output["AFTER"]["recovery_time_s"] = recovery_time_s(
        [source_stamps[index] * 1.0e-9 for index in after_indices],
        [float(errors[index]) for index in after_indices],
        threshold,
    )
    output["full"] = BASE.trajectory_metrics(odometry_messages, scans)
    return output


def phase_native_metrics(messages, truth):
    scans = truth["scans"]
    truth_stamps = np.asarray([scan["stamp_ns"] for scan in scans], dtype=np.int64)
    grouped = {name: [] for name in PHASES}
    for item in messages:
        stamp_ns = BASE.message_stamp_ns(item[1])
        frame = int(np.argmin(np.abs(truth_stamps - stamp_ns)))
        grouped[truth["phase_by_frame"][frame]].append(item)
    return {name: BASE.native_factor_metrics(grouped[name]) for name in PHASES}


def analyze_run(run_root, frozen_root, scenario, branch):
    run_dir = run_root / scenario / branch
    messages = BASE.read_bag(run_dir / "output")
    truth = json.loads((frozen_root / scenario / "truth.json").read_text(encoding="utf-8"))
    odom_topic = BASE.topic_with_suffix(messages, "/odom")
    factor_topic = BASE.topic_with_suffix(messages, "/native_lidar_factor")
    map_topic = BASE.topic_with_suffix(messages, "/map")
    raw_topic = BASE.topic_with_suffix(messages, "/raw")
    raw_messages = messages.get(raw_topic, [])
    if not raw_messages:
        frozen_messages = BASE.read_bag(frozen_root / scenario / "input")
        frozen_topic = BASE.topic_with_suffix(frozen_messages, "/lidar")
        raw_messages = frozen_messages[frozen_topic]
    odometry = messages.get(odom_topic, [])
    factors = messages.get(factor_topic, [])
    map_messages = messages.get(map_topic, [])
    runtime = json.loads((run_dir / "runtime.json").read_text(encoding="utf-8"))
    result = {
        "scenario": scenario,
        "branch": branch,
        "trajectory": phase_trajectory_metrics(odometry, truth),
        "native_lidar_factor": phase_native_metrics(factors, truth),
        "map_truth": BASE.map_truth_metrics(
            map_messages, raw_messages, odometry, truth["scans"]
        ),
        "runtime": runtime,
    }
    if branch == "clean":
        clean_topic = BASE.topic_with_suffix(messages, "/clean")
        status_topic = BASE.topic_with_suffix(messages, "/gateway_status")
        result["gateway"] = BASE.gateway_metrics(
            messages.get(clean_topic, []),
            messages.get(status_topic, []),
            raw_messages,
            truth["scans"],
        )
    return result


def finite_values(runs, branch, phase_name, section, metric):
    output = []
    for run in runs:
        if run["branch"] != branch or run["scenario"] not in PRIMARY_SCENARIOS:
            continue
        value = run[section][phase_name].get(metric)
        if value is not None and math.isfinite(value):
            output.append(float(value))
    return output


def median_or_none(values):
    return float(statistics.median(values)) if values else None


def aggregate(runs, branch):
    output = {"scenario_count": len(PRIMARY_SCENARIOS), "phases": {}}
    for phase_name in PHASES:
        output["phases"][phase_name] = {
            metric: median_or_none(
                finite_values(runs, branch, phase_name, "trajectory", metric)
            )
            for metric in [
                "ate_rmse_m",
                "rpe_translation_rmse_m",
                "rpe_yaw_rmse_deg",
                "z_rmse_m",
                "yaw_rmse_deg",
                "pose_jump_p95_m",
                "pose_jump_max_m",
                "max_odom_gap_s",
                "recovery_time_s",
            ]
        }
        output["phases"][phase_name]["native_residual_rms_median_m"] = median_or_none(
            finite_values(
                runs, branch, phase_name, "native_lidar_factor", "residual_rms_median_m"
            )
        )
        output["phases"][phase_name]["native_effective_factors"] = sum(
            run["native_lidar_factor"][phase_name].get("effective_factor_count", 0)
            for run in runs
            if run["branch"] == branch and run["scenario"] in PRIMARY_SCENARIOS
        )

    selected = [
        run for run in runs if run["branch"] == branch and run["scenario"] in PRIMARY_SCENARIOS
    ]
    output["runtime"] = {
        "fast_lio_cpu_percent_median": median_or_none(
            [run["runtime"]["cpu_percent"]["fast_lio"]["mean"] for run in selected]
        ),
        "fast_lio_rss_mib_median": median_or_none(
            [run["runtime"]["rss_mib"]["fast_lio"]["max"] for run in selected]
        ),
        "fast_lio_callback_p95_ms_median": median_or_none(
            [
                run["runtime"]["fast_lio_callback_latency_ms"]["p95_ms"]
                for run in selected
                if run["runtime"].get("fast_lio_callback_latency_ms")
            ]
        ),
        "lost_runs": sum(run["trajectory"]["full"].get("lost", False) for run in selected),
        "reset_count": sum(run["trajectory"]["full"].get("reset_count", 0) for run in selected),
    }
    if branch == "clean":
        output["runtime"].update(
            {
                "gateway_cpu_percent_median": median_or_none(
                    [run["runtime"]["cpu_percent"]["gateway"]["mean"] for run in selected]
                ),
                "gateway_rss_mib_median": median_or_none(
                    [run["runtime"]["rss_mib"]["gateway"]["max"] for run in selected]
                ),
                "gateway_p50_ms_median": median_or_none(
                    [run["gateway"]["latency_p50_ms"] for run in selected]
                ),
                "gateway_p95_ms_median": median_or_none(
                    [run["gateway"]["latency_p95_ms"] for run in selected]
                ),
                "gateway_p99_ms_median": median_or_none(
                    [run["gateway"]["latency_p99_ms"] for run in selected]
                ),
                "queue_overflow": sum(run["gateway"]["queue_overflow"] for run in selected),
                "missing_clean_scans": sum(
                    run["gateway"]["missing_clean_scans"] for run in selected
                ),
                "fail_open_count": sum(run["gateway"]["fail_open_count"] for run in selected),
            }
        )
    return output


def information_change(runs):
    by_key = {(run["scenario"], run["branch"]): run for run in runs}
    output = {}
    for phase_name in PHASES:
        changes = [[], [], []]
        condition = []
        for scenario in PRIMARY_SCENARIOS:
            raw = by_key[(scenario, "raw")]["native_lidar_factor"][phase_name]
            clean = by_key[(scenario, "clean")]["native_lidar_factor"][phase_name]
            raw_xyz = raw.get("translation_information_xyz")
            clean_xyz = clean.get("translation_information_xyz")
            if not raw_xyz or not clean_xyz:
                continue
            for axis in range(3):
                changes[axis].append(100.0 * (clean_xyz[axis] / raw_xyz[axis] - 1.0))
            condition.append(
                100.0
                * (
                    clean["translation_information_condition"]
                    / raw["translation_information_condition"]
                    - 1.0
                )
            )
        output[phase_name] = {
            "clean_vs_raw_information_change_percent_xyz_median": [
                median_or_none(values) for values in changes
            ],
            "condition_change_percent_median": median_or_none(condition),
        }
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_root")
    parser.add_argument("frozen_manifest")
    parser.add_argument("output")
    args = parser.parse_args()
    run_root = Path(args.run_root).resolve()
    manifest_path = Path(args.frozen_manifest).resolve()
    frozen_root = manifest_path.parent
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenarios = [record["scenario"] for record in manifest["scenarios"]]
    runs = [
        analyze_run(run_root, frozen_root, scenario, branch)
        for scenario in scenarios
        for branch in ["raw", "clean"]
    ]
    report = {
        "schema": "dyn_loc_007_current_localization_v1",
        "truth_role": "evaluator_only",
        "detector_truth_access": False,
        "causal_previous_state_only": True,
        "primary_scenarios": PRIMARY_SCENARIOS,
        "occlusion_scenarios": OCCLUSION_SCENARIOS,
        "raw": aggregate(runs, "raw"),
        "clean": aggregate(runs, "clean"),
        "native_information_change": information_change(runs),
        "runs": runs,
    }
    output = Path(args.output).resolve()
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
