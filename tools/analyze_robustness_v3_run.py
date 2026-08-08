#!/usr/bin/env python3
"""Summarize one frozen-input Robustness V3 replay without cherry-picking."""

import argparse
import json
import math
from pathlib import Path
import re
import statistics

import numpy as np

from uf_sensor_pipeline.fault_profiles import load_fault_profile


def read_json(path, default=None):
    path = Path(path)
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else default


def read_tum(path):
    rows = []
    for line in Path(path).read_text(encoding="ascii").splitlines():
        if line and not line.startswith("#"):
            rows.append([float(value) for value in line.split()])
    return rows


def parse_resource(path):
    text = Path(path).read_text(encoding="utf-8") if Path(path).exists() else ""
    def number(label, default=0.0):
        match = re.search(rf"{re.escape(label)}:\s*([0-9.]+)", text)
        return float(match.group(1)) if match else default
    return {
        "user_cpu_s": number("User time (seconds)"),
        "system_cpu_s": number("System time (seconds)"),
        "cpu_percent": number("Percent of CPU this job got"),
        "max_rss_kib": number("Maximum resident set size (kbytes)"),
        "minor_page_faults": number("Minor (reclaiming a frame) page faults"),
        "major_page_faults": number("Major (requiring I/O) page faults"),
        "voluntary_context_switches": number("Voluntary context switches"),
        "involuntary_context_switches": number("Involuntary context switches"),
    }


def scheduler_effects(events, profile):
    scheduler = [event for event in events if event.get("kind") == "scheduler"]
    faults = [event for event in events if event.get("kind") == "fault"]
    output = {}
    for modality in ("lidar", "imu", "gnss", "optical_flow", "vision"):
        weights = [event.get("weights", {}).get(modality) for event in scheduler]
        weights = [value for value in weights if value is not None]
        scores = [event.get("degradation_scores", {}).get(modality) for event in scheduler]
        scores = [value for value in scores if value is not None]
        enabled = [event.get("factor_enabled", {}).get(modality) for event in scheduler]
        active = [
            event for event in faults
            if event.get("modality") == modality and event.get("active")
        ]
        fault_start = min((event["stamp_s"] for event in active), default=None)
        configured = [spec for spec in profile.faults if spec.modality == modality]
        inactive_transitions = [
            event["stamp_s"] for event in faults
            if event.get("modality") == modality and not event.get("active")
            and fault_start is not None and event["stamp_s"] > fault_start
        ]
        positive_durations = [
            spec.duration_s for spec in configured if spec.duration_s > 0.0
        ]
        configured_end = (
            max(inactive_transitions) if inactive_transitions
            else (
                min((event["stamp_s"] for event in active), default=0.0)
                + max(positive_durations)
                if active and positive_durations else None
            )
        )
        switched = next((
            event["stamp_s"] for event in scheduler
            if fault_start is not None and event["stamp_s"] >= fault_start
            and not event.get("factor_enabled", {}).get(modality, True)
        ), None)
        score_floor = max((spec.score_floor for spec in configured), default=0.0)
        responded = next((
            event["stamp_s"] for event in scheduler
            if fault_start is not None and score_floor > 0.0
            and event["stamp_s"] >= fault_start
            and event.get("degradation_scores", {}).get(modality, 0.0)
            >= max(0.0, score_floor - 0.02)
        ), None)
        recovered = next((
            event["stamp_s"] for event in scheduler
            if configured_end is not None and event["stamp_s"] >= configured_end
            and event.get("factor_enabled", {}).get(modality, False)
            and event.get("weights", {}).get(modality, 0.0) >= 0.45
        ), None)
        active_scheduler = [
            event for event in scheduler
            if fault_start is not None and event["stamp_s"] >= fault_start
            and (configured_end is None or event["stamp_s"] < configured_end)
        ]
        active_weights = [
            event.get("weights", {}).get(modality)
            for event in active_scheduler
            if event.get("weights", {}).get(modality) is not None
        ]
        active_scores = [
            event.get("degradation_scores", {}).get(modality)
            for event in active_scheduler
            if event.get("degradation_scores", {}).get(modality) is not None
        ]
        output[modality] = {
            "scheduler_samples": len(weights),
            "minimum_weight": min(weights) if weights else None,
            "median_weight": statistics.median(weights) if weights else None,
            "maximum_degradation": max(scores) if scores else None,
            "active_median_weight": (
                statistics.median(active_weights) if active_weights else None
            ),
            "active_median_degradation": (
                statistics.median(active_scores) if active_scores else None
            ),
            "factor_disabled_samples": sum(value is False for value in enabled),
            "fault_start_stamp_s": fault_start,
            "frs_switch_stamp_s": switched,
            "frs_switch_delay_s": (
                switched - fault_start if switched is not None else None
            ),
            "frs_weight_response_stamp_s": responded,
            "frs_weight_response_delay_s": (
                responded - fault_start if responded is not None else None
            ),
            "configured_recovery_stamp_s": configured_end,
            "recovery_stamp_s": recovered,
            "recovery_delay_s": (
                recovered - configured_end if recovered is not None else None
            ),
        }
    return output


def factor_counts(values):
    aliases = {
        "lidar": "native_lidar_factors",
        "imu": "imu_factors",
        "gnss": "gnss_factors",
        "optical_flow": "flow_factors",
        "visual": "visual_factors",
    }
    return {name: int(float(values.get(key, 0))) for name, key in aliases.items()}


def integrity_reject_count(values):
    raw = str(values.get("optimization_integrity_counts", ""))
    rejected = 0
    for item in raw.split(","):
        if ":" not in item:
            continue
        reason, count = item.rsplit(":", 1)
        if reason.strip() in {"ok", "not_checked", "not_evaluated", "none"}:
            continue
        try:
            rejected += int(float(count))
        except ValueError:
            continue
    return rejected


def active_factor_deltas(events, profile):
    faults = [event for event in events if event.get("kind") == "fault"]
    backend = [event for event in events if event.get("kind") == "backend"]
    keys = {
        "lidar": ("native_lidar_factors", "native_lidar_hard_disabled"),
        "imu": ("imu_factors", None),
        "gnss": ("gnss_factors", "gnss_jump_rejected"),
        "optical_flow": ("flow_factors", "flow_disabled_quality"),
        "vision": ("visual_factors", "visual_rejected_time"),
    }
    result = {}
    for modality in {spec.modality for spec in profile.faults}:
        active = [
            event for event in faults
            if event.get("modality") == modality and event.get("active")
        ]
        if not active or not backend:
            continue
        start = min(event["stamp_s"] for event in active)
        inactive = [
            event["stamp_s"] for event in faults
            if event.get("modality") == modality and not event.get("active")
            and event["stamp_s"] > start
        ]
        end = max(inactive) if inactive else max(event["stamp_s"] for event in active)
        before = min(backend, key=lambda event: abs(event["stamp_s"] - start))
        after = min(backend, key=lambda event: abs(event["stamp_s"] - end))
        accepted_key, rejected_key = keys[modality]
        result[modality] = {
            "start_stamp_s": start,
            "end_stamp_s": end,
            "accepted": max(0, int(after.get(accepted_key, 0)) - int(before.get(accepted_key, 0))),
            "rejected_or_disabled": (
                max(0, int(after.get(rejected_key, 0)) - int(before.get(rejected_key, 0)))
                if rejected_key else None
            ),
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--profile-path", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--frs", choices=("on", "off"), required=True)
    parser.add_argument("--truth", required=True)
    parser.add_argument("--reference-estimate", required=True)
    parser.add_argument("--play-started", type=float, required=True)
    parser.add_argument("--play-finished", type=float, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    run_dir = Path(args.run_dir)
    profile = load_fault_profile(args.profile_path, args.profile)
    metrics = read_json(run_dir / "replay_metrics.json", {})
    timeline = read_json(run_dir / "reliability_timeline.json", {})
    timeline_events = timeline.get("events", [])
    trajectory = read_json(run_dir / "trajectory_metrics.json", {})
    last = metrics.get("last_values", {})
    estimate_rows = read_tum(run_dir / "estimate.tum") if (run_dir / "estimate.tum").exists() else []
    truth_rows = read_tum(args.truth)
    reference_rows = read_tum(args.reference_estimate)
    estimate_span = estimate_rows[-1][0] - estimate_rows[0][0] if len(estimate_rows) > 1 else 0.0
    truth_span = truth_rows[-1][0] - truth_rows[0][0] if len(truth_rows) > 1 else 0.0
    reference_span = (
        reference_rows[-1][0] - reference_rows[0][0]
        if len(reference_rows) > 1 else 0.0
    )
    estimate_intervals = [
        estimate_rows[index + 1][0] - estimate_rows[index][0]
        for index in range(len(estimate_rows) - 1)
        if estimate_rows[index + 1][0] > estimate_rows[index][0]
    ]
    maximum_odom_gap_s = max(estimate_intervals) if estimate_intervals else None
    first_large_gap = next((
        {
            "start_stamp_s": estimate_rows[index][0],
            "end_stamp_s": estimate_rows[index + 1][0],
            "duration_s": estimate_rows[index + 1][0] - estimate_rows[index][0],
        }
        for index in range(len(estimate_rows) - 1)
        if estimate_rows[index + 1][0] - estimate_rows[index][0] > 1.0
    ), None)
    visual_attempts = int(float(last.get("visual_factor_attempts", 0)))
    visual_accepted = int(float(last.get("visual_factors", 0)))
    factors = factor_counts(last)
    errors = {
        "optimization_error": int(float(last.get("optimization_errors", 0))),
        "integrity_reject": integrity_reject_count(last),
        "rollback": int(float(last.get("optimization_rollbacks", 0))),
    }
    wall = max(0.0, args.play_finished - args.play_started)
    report = {
        "schema_version": 1,
        "profile": profile.name,
        "description": profile.description,
        "frs": args.frs,
        "fault_count": len(profile.faults),
        "faults": [spec.__dict__ for spec in profile.faults],
        "calibration": dict(profile.calibration),
        "odom_count": int(metrics.get("odom_count", 0)),
        "trajectory_completeness": (
            min(1.0, estimate_span / reference_span)
            if reference_span > 0.0 else None
        ),
        "trajectory_reference_span_s": reference_span,
        "trajectory_maximum_odom_gap_s": maximum_odom_gap_s,
        "trajectory_first_gap_over_1s": first_large_gap,
        "trajectory_odom_interval_p95_s": (
            float(np.percentile(estimate_intervals, 95))
            if estimate_intervals else None
        ),
        "trajectory_continuous": (
            maximum_odom_gap_s is not None and maximum_odom_gap_s <= 1.0
        ),
        "trajectory": trajectory,
        "factor_counts": factors,
        "visual_factor_attempts": visual_attempts,
        "visual_factor_accepted": visual_accepted,
        "visual_acceptance_ratio": (
            visual_accepted / visual_attempts if visual_attempts else None
        ),
        "factor_rejections": {
            "optimization_not_committed": int(float(last.get("optimization_rejected", 0))),
            "visual_time": int(float(last.get("visual_rejected_time", 0))),
            "visual_tracks": int(float(last.get("visual_rejected_tracks", 0))),
            "gnss_jump": int(float(last.get("gnss_jump_rejected", 0))),
            "lidar_invalid": int(float(last.get("native_lidar_invalid", 0))),
            "lidar_hard_disabled": int(float(last.get("native_lidar_hard_disabled", 0))),
        },
        "reliability": scheduler_effects(timeline_events, profile),
        "active_fault_factor_deltas": active_factor_deltas(
            timeline_events, profile
        ),
        "scheduler_state_sequence": timeline.get("summary", {}).get(
            "scheduler_state_sequence", []
        ),
        "solver_ms": metrics.get("solver_ms", {}),
        "callback_ms": metrics.get("callback_ms", {}),
        "play_wall_s": wall,
        "sim_span_s": estimate_span,
        "rtf_proxy": estimate_span / wall if wall > 0.0 else None,
        "resource": parse_resource(run_dir / "backend_resource.txt"),
        "errors": errors,
        "integrity_counts": str(last.get("optimization_integrity_counts", "none")),
        "native_lidar_path": str(last.get("lidar_factor_source", "unknown")),
        "native_lidar_temporal_contract": {
            "stamp_error_ms": float(last.get("native_lidar_stamp_error_ms", -1.0)),
            "scan_requests": int(float(last.get("scan_prediction_requests", 0))),
            "scan_predictions": int(float(last.get("scan_prediction_published", 0))),
            "scan_rejected": int(float(last.get("scan_prediction_rejected", 0))),
            "scan_last_reason": str(last.get(
                "scan_prediction_last_reason", last.get("scan_last_reason", "unavailable")
            )),
            "scan_deferred": int(float(last.get("scan_prediction_deferred", 0))),
            "scan_deferred_released": int(float(
                last.get("scan_prediction_deferred_released", 0)
            )),
            "scan_duplicate_requests": int(float(
                last.get("scan_prediction_duplicate_requests", 0)
            )),
            "scan_stale_requests": int(float(last.get(
                "scan_prediction_stale_requests", 0
            ))),
            "scan_cache_hits": int(float(last.get(
                "scan_prediction_cache_hits", 0
            ))),
            "scan_cache_misses": int(float(last.get(
                "scan_prediction_cache_misses", 0
            ))),
            "scan_reuse_rejected": int(float(last.get(
                "scan_prediction_reuse_rejected", 0
            ))),
            "last_imu_reason": str(last.get("last_imu_reason", "unavailable")),
        },
        "odometry_fallbacks": int(float(last.get("native_lidar_pose_fallbacks", 0))),
        "pass_invariants": (
            int(metrics.get("odom_count", 0)) > 0
            and all(value == 0 for value in errors.values())
            and int(float(last.get("native_lidar_pose_fallbacks", 0))) == 0
        ),
    }
    output = Path(args.output)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))


if __name__ == "__main__":
    main()
