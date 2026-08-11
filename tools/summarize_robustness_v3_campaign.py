#!/usr/bin/env python3
"""Merge single, calibration, double and endurance V3 evidence."""

import argparse
import json
from pathlib import Path
import statistics


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def nested(value, *keys):
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def paired(reports):
    names = sorted({key.split(":", 1)[0] for key in reports})
    output = {}
    for name in names:
        on, off = reports.get(name + ":on"), reports.get(name + ":off")
        if not on or not off:
            continue
        on_ate = nested(on, "trajectory", "ate_rmse_m")
        off_ate = nested(off, "trajectory", "ate_rmse_m")
        output[name] = {
            "frs_on_ate_m": on_ate,
            "frs_off_ate_m": off_ate,
            "ate_benefit_m": (
                off_ate - on_ate if on_ate is not None and off_ate is not None else None
            ),
            "frs_on_completeness": on.get("trajectory_completeness"),
            "frs_off_completeness": off.get("trajectory_completeness"),
            "frs_on_errors": on.get("errors"),
            "frs_off_errors": off.get("errors"),
            "frs_on_solver_median_ms": nested(on, "solver_ms", "median"),
            "frs_off_solver_median_ms": nested(off, "solver_ms", "median"),
        }
    return output


def passed(report):
    return bool(
        report and report.get("pass_invariants")
        and (report.get("trajectory_completeness") or 0.0) >= 0.90
        and report.get("trajectory_continuous", False)
        and nested(report, "trajectory", "ate_rmse_m") is not None
        and nested(report, "trajectory", "ate_rmse_m") <= 0.10
    )


def maximum_passed(reports, prefix, ordered):
    answer = None
    for name, value in ordered:
        report = reports.get(f"{prefix}_{name}:on")
        if passed(report):
            answer = {"level": name, "value": value}
    return answer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", action="append", required=True)
    parser.add_argument("--joint-map")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    reports = {}
    matrices = []
    for path in args.matrix:
        matrix = load(path)
        matrices.append(matrix)
        reports.update(matrix.get("reports", {}))
    comparisons = paired(reports)
    replay_all_invariants_pass = all(
        item.get("pass_invariants", False) for item in reports.values()
    )
    benefits = [
        value["ate_benefit_m"] for value in comparisons.values()
        if value["ate_benefit_m"] is not None
    ]
    response_delays = {}
    recovery_delays = {}
    for key, report in reports.items():
        if not key.endswith(":on"):
            continue
        for modality, values in report.get("reliability", {}).items():
            response = values.get("frs_weight_response_delay_s")
            recovery = values.get("recovery_delay_s")
            if response is not None:
                response_delays.setdefault(modality, []).append(response)
            if recovery is not None:
                recovery_delays.setdefault(modality, []).append(recovery)
    doubles = {
        key: {
            "pass": passed(report),
            "completeness": report.get("trajectory_completeness"),
            "ate_m": nested(report, "trajectory", "ate_rmse_m"),
            "errors": report.get("errors"),
        }
        for key, report in reports.items()
        if key.startswith("dual_")
    }
    danger = []
    for name, values in comparisons.items():
        score = 0.0
        off = reports.get(name + ":off", {})
        if not passed(off):
            score += 10.0
            score += 100.0 * (1.0 - float(
                off.get("trajectory_completeness") or 0.0
            ))
        if values["ate_benefit_m"] is not None:
            score += max(0.0, values["ate_benefit_m"])
        danger.append((score, name))
    joint_map = load(args.joint_map) if args.joint_map else None
    joint_errors = (joint_map or {}).get("errors", {})
    joint_map_pass = bool(
        joint_map
        and joint_map.get("headless_status") == 0
        and joint_map.get("land_observed")
        and joint_map.get("disarm_observed")
        and ((joint_map.get("joint_map") or {}).get("voxel_count") or 0) > 0
        and all((joint_errors.get(name) or 0) == 0 for name in (
            "optimization", "optimization_not_committed", "rollback"
        ))
    )
    report = {
        "schema_version": 1,
        "matrix_count": len(matrices),
        "run_count": len(reports),
        "all_command_reports_present": all(
            not matrix.get("missing_reports") for matrix in matrices
        ),
        "replay_all_invariants_pass": replay_all_invariants_pass,
        "joint_map_pass": joint_map_pass,
        "all_invariants_pass": replay_all_invariants_pass and joint_map_pass,
        "frs_ab": comparisons,
        "frs_ate_benefit_median": statistics.median(benefits) if benefits else None,
        "frs_ate_win_count": sum(value > 0.0 for value in benefits),
        "frs_ate_loss_count": sum(value < 0.0 for value in benefits),
        "frs_response_delay_median_s": {
            name: statistics.median(values) for name, values in response_delays.items()
        },
        "recovery_delay_median_s": {
            name: statistics.median(values) for name, values in recovery_delays.items()
        },
        "most_dangerous_profile": max(danger)[1] if danger else None,
        "sensor_fault_boundaries": {
            "visual": maximum_passed(reports, "visual", [
                ("light", "20% track dropout"),
                ("medium", "55% dropout + 1.5 px bias"),
                ("heavy", "6 s outage"),
            ]),
            "lidar": maximum_passed(reports, "lidar", [
                ("light", "25% correspondence dropout"),
                ("medium", "60% correspondence dropout"),
                ("heavy", "25 s outage"),
            ]),
            "gnss_denial": maximum_passed(reports, "gnss_denial", [
                ("light", "4 s denial"), ("medium", "12 s denial"),
                ("heavy", "30 s denial"),
            ]),
            "gnss_jump": maximum_passed(reports, "gnss_jump", [
                ("light", "5 m jump"), ("medium", "15 m jump"),
                ("heavy", "35 m jump"),
            ]),
            "optical_flow": maximum_passed(reports, "flow", [
                ("light", "quality=120"), ("medium", "quality=35"),
                ("heavy", "25 s outage"),
            ]),
            "imu": maximum_passed(reports, "imu", [
                ("light", "0.01 rad/s + 0.05 m/s^2 bias"),
                ("medium", "0.05 rad/s + 0.30 m/s^2 bias"),
                ("heavy", "25 s outage"),
            ]),
        },
        "camera_time_tolerance": maximum_passed(reports, "camera_time", [
            ("light", 0.020), ("medium", 0.050), ("heavy", 0.100)
        ]),
        "lidar_time_tolerance": maximum_passed(reports, "lidar_time", [
            ("light", 0.002), ("medium", 0.005), ("heavy", 0.020)
        ]),
        "d435_rotation_tolerance": maximum_passed(reports, "d435_extrinsic_rot", [
            ("light", 1.0), ("medium", 3.0), ("heavy", 8.0)
        ]),
        "d435_translation_tolerance": maximum_passed(reports, "d435_extrinsic_trans", [
            ("light", 0.01), ("medium", 0.05), ("heavy", 0.15)
        ]),
        "mid360_rotation_tolerance": maximum_passed(reports, "mid360_extrinsic_rot", [
            ("light", 1.0), ("medium", 3.0), ("heavy", 8.0)
        ]),
        "mid360_translation_tolerance": maximum_passed(reports, "mid360_extrinsic_trans", [
            ("light", 0.01), ("medium", 0.05), ("heavy", 0.15)
        ]),
        "double_faults": doubles,
        "endurance": {
            key: report for key, report in reports.items()
            if key.startswith("long_")
        },
        "joint_map": joint_map,
        "metric_basis": {
            "replay_trajectory": "delta-ATE/RPE against frozen nominal replay",
            "continuity": ">= 90% of frozen nominal estimator span",
            "maximum_odom_gap_s": 1.0,
            "time_and_extrinsic_boundary_ate_limit_m": 0.10,
            "absolute_truth": "reported only by full-stack joint-map run",
        },
    }
    Path(args.output).write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "runs": report["run_count"],
        "all_invariants_pass": report["all_invariants_pass"],
        "most_dangerous": report["most_dangerous_profile"],
        "camera_time": report["camera_time_tolerance"],
        "lidar_time": report["lidar_time_tolerance"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
