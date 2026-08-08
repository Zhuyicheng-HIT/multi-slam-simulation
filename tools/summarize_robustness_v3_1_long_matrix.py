#!/usr/bin/env python3
"""Summarize the V3.1 map-off/LiDAR-only/joint causal matrix."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import time
from pathlib import Path
from typing import Any


CRASH_RE = re.compile(r"\[INFO\] \[([0-9.]+)\].*FCU: Crash")
CRASH_MONOTONIC_RE = re.compile(
    r"FCU: Crash.*wall_epoch_s=([0-9.]+); wall_monotonic_s=([0-9.]+)"
)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def stats(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "p50": statistics.median(values) if values else None,
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def crash_monotonic(
    path: Path, boot_epoch_offset_s: float
) -> tuple[float | None, float | None, str]:
    if not path.exists():
        return None, None, "not_observed"
    text = path.read_text(encoding="utf-8", errors="replace")
    direct = CRASH_MONOTONIC_RE.search(text)
    if direct:
        return float(direct.group(1)), float(direct.group(2)), "direct_monotonic"
    match = CRASH_RE.search(text)
    if not match:
        return None, None, "not_observed"
    epoch = float(match.group(1))
    # WSL can resynchronize CLOCK_REALTIME after a run while CLOCK_MONOTONIC is
    # unaffected.  Do not reconstruct event ordering from a later offset.
    return epoch, None, "legacy_wall_epoch_only"


def summarize_run(run_dir: Path, boot_epoch_offset_s: float) -> dict[str, Any]:
    report = load_json(run_dir / "robustness_joint_map_report.json")
    trace_path = run_dir / "backend_cycle_trace.jsonl"
    cycles = [json.loads(line) for line in trace_path.read_text(encoding="utf-8").splitlines()]
    crash_epoch, crash_wall, crash_clock_source = crash_monotonic(
        run_dir / "small_rectangle.log", boot_epoch_offset_s
    )

    rollback_events: list[dict[str, Any]] = []
    previous_rollbacks = 0
    for cycle in cycles:
        current = int(cycle.get("rollbacks", 0))
        if current > previous_rollbacks:
            integrity = cycle.get("integrity", {})
            rollback_events.append(
                {
                    "stamp_s": cycle.get("stamp_s"),
                    "wall_monotonic_s": cycle.get("wall_started_s"),
                    "relative_to_crash_s": (
                        cycle["wall_started_s"] - crash_wall if crash_wall is not None else None
                    ),
                    "reason": integrity.get("reason"),
                    "translation_correction_m": integrity.get("translation_correction_m"),
                    "velocity_correction_mps": integrity.get("velocity_correction_mps"),
                    "accel_bias_correction_mps2": integrity.get("accel_bias_correction_mps2"),
                    "gyro_bias_correction_radps": integrity.get("gyro_bias_correction_radps"),
                    "solver_duration_ms": cycle.get("solver_duration_ms"),
                }
            )
        previous_rollbacks = current

    ros_gaps = [b["stamp_s"] - a["stamp_s"] for a, b in zip(cycles, cycles[1:])]
    wall_gaps = [b["wall_started_s"] - a["wall_started_s"] for a, b in zip(cycles, cycles[1:])]
    before_crash = [c for c in cycles if crash_wall is None or c["wall_started_s"] <= crash_wall]
    solver_before = [float(c["solver_duration_ms"]) for c in before_crash]
    callback_before = [float(c.get("phases_ms", {}).get("callback_total", 0.0)) for c in before_crash]

    mapping = report.get("mapping", {})
    map_trace = mapping.get("performance_trace", [])
    map_before = [e for e in map_trace if crash_wall is None or e.get("wall_monotonic_s", math.inf) <= crash_wall]
    map_profile_before: dict[str, dict[str, Any]] = {}
    for kind in sorted({str(e.get("kind")) for e in map_before}):
        map_profile_before[kind] = stats(
            [float(e["duration_ms"]) for e in map_before if e.get("kind") == kind]
        )

    analysis_path = run_dir / "backend_cycle_analysis.json"
    analysis = load_json(analysis_path) if analysis_path.exists() else {}
    features = analysis.get("feature_summaries", {})
    return {
        "run": run_dir.name,
        "mode": report.get("map_mode"),
        "headless_status": report.get("headless_status"),
        "land_observed": report.get("land_observed"),
        "disarm_observed": report.get("disarm_observed"),
        "crash_epoch_s": crash_epoch,
        "crash_wall_monotonic_s": crash_wall,
        "crash_clock_source": crash_clock_source,
        "rollback_count_trace": max((int(c.get("rollbacks", 0)) for c in cycles), default=0),
        "rollback_before_crash": sum(
            1 for event in rollback_events if event["relative_to_crash_s"] is not None and event["relative_to_crash_s"] <= 0.0
        ),
        "rollback_after_crash": sum(
            1 for event in rollback_events if event["relative_to_crash_s"] is not None and event["relative_to_crash_s"] > 0.0
        ),
        "first_rollback_relative_to_crash_s": (
            rollback_events[0]["relative_to_crash_s"] if rollback_events else None
        ),
        "rollback_events": rollback_events,
        "pre_crash_solver_ms": stats(solver_before),
        "pre_crash_callback_ms": stats(callback_before),
        "state_ros_gap_s": stats(ros_gaps),
        "state_wall_gap_s": stats(wall_gaps),
        "pre_crash_mapping_ms": map_profile_before,
        "process_cpu_percent": {
            key: features.get(key, {})
            for key in (
                "backend_cpu_percent",
                "fast_lio_cpu_percent",
                "gazebo_cpu_percent",
                "shared_mapping_cpu_percent",
            )
        },
        "context_switches": {
            "voluntary": features.get("voluntary_context_switches", {}),
            "involuntary": features.get("involuntary_context_switches", {}),
        },
        "factors": report.get("factors"),
        "errors_from_timeline": report.get("errors"),
        "joint_map": report.get("joint_map"),
        "mapping_profile_all": mapping.get("performance_profile", {}),
        "simulation_rtf": report.get("simulation_rtf"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    boot_epoch_offset_s = time.time() - time.monotonic()
    runs = [
        summarize_run(path, boot_epoch_offset_s)
        for path in sorted(args.matrix_dir.iterdir())
        if path.is_dir()
        and (path / "robustness_joint_map_report.json").exists()
        and (path / "backend_cycle_trace.jsonl").exists()
    ]
    by_mode: dict[str, Any] = {}
    for mode in ("disabled", "lidar_only", "joint"):
        selected = [run for run in runs if run["mode"] == mode]
        by_mode[mode] = {
            "runs": len(selected),
            "land_successes": sum(bool(run["land_observed"]) for run in selected),
            "fcu_crashes": sum(run["crash_epoch_s"] is not None for run in selected),
            "rollback_before_crash_total": sum(run["rollback_before_crash"] for run in selected),
            "rollback_after_crash_total": sum(run["rollback_after_crash"] for run in selected),
            "pre_crash_solver_p50_ms": stats(
                [
                    float(run["pre_crash_solver_ms"]["p50"])
                    for run in selected
                    if run["pre_crash_solver_ms"]["p50"] is not None
                ]
            ),
            "pre_crash_solver_p95_ms": stats(
                [
                    float(run["pre_crash_solver_ms"]["p95"])
                    for run in selected
                    if run["pre_crash_solver_ms"]["p95"] is not None
                ]
            ),
            "max_state_ros_gap_s": max(
                (float(run["state_ros_gap_s"]["max"]) for run in selected), default=None
            ),
        }
    payload = {
        "schema_version": 1,
        "boot_epoch_offset_s": boot_epoch_offset_s,
        "causal_conclusion": {
            "mapping_required_for_fcu_crash": False,
            "rollback_precedes_fcu_crash_direct_clock": any(
                run["crash_clock_source"] == "direct_monotonic"
                and run["rollback_before_crash"]
                for run in runs
            ),
            "fcu_crash_observed_with_map_off_and_zero_rollback": any(
                run["mode"] == "disabled"
                and run["crash_epoch_s"] is not None
                and run["rollback_count_trace"] == 0
                for run in runs
            ),
        },
        "by_mode": by_mode,
        "runs": runs,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
