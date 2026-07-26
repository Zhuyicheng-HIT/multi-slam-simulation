#!/usr/bin/env python3
"""Summarize native FAST-LIO factors before, during, and after a LiDAR fault."""

import argparse
import csv
import json
import math
from pathlib import Path


def percentile(values, q):
    values = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not values:
        return None
    position = (len(values) - 1) * float(q) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def distribution(rows, fields):
    result = {"samples": len(rows)}
    for field in fields:
        values = [row[field] for row in rows if field in row]
        result[field] = {
            "median": percentile(values, 50),
            "p05": percentile(values, 5),
            "p95": percentile(values, 95),
            "min": percentile(values, 0),
            "max": percentile(values, 100),
        }
    return result


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def summarize(run_dir):
    timeline = load_json(run_dir / "reliability_timeline.json")
    fault_events = [
        event for event in timeline["events"]
        if event["kind"] == "fault"
        and event.get("modality") == "lidar"
        and event.get("active")
    ]
    fault_stamps = [float(event["stamp_s"]) for event in fault_events]
    fault_start = min(fault_stamps) if fault_stamps else None
    fault_end = max(fault_stamps) if fault_stamps else None

    def phase(stamp):
        if fault_start is None:
            return "nominal"
        if stamp < fault_start:
            return "pre_fault"
        if stamp <= fault_end:
            return "fault_active"
        return "recovery"

    lio_rows = {name: [] for name in ("nominal", "pre_fault", "fault_active", "recovery")}
    for event in timeline["events"]:
        if event["kind"] == "lio":
            lio_rows[phase(float(event["stamp_s"]))].append(event)

    native_rows = {name: [] for name in lio_rows}
    metrics_path = run_dir / "native_factor_metrics.jsonl"
    if metrics_path.exists():
        with metrics_path.open(encoding="utf-8") as stream:
            for line in stream:
                row = json.loads(line)
                native_rows[phase(float(row["stamp_ns"]) * 1.0e-9)].append(row)

    score_rows = {name: [] for name in lio_rows}
    with (run_dir / "reliability_scores.csv").open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["modality"] != "lidar":
                continue
            converted = {
                "degradation_score": float(row["degradation_score"]),
                "reliability_weight": float(row["reliability_weight"]),
                "valid": float(row["valid"]),
            }
            score_rows[phase(float(row["stamp_s"]))].append(converted)

    scheduler_rows = {name: [] for name in lio_rows}
    for event in timeline["events"]:
        if event["kind"] != "scheduler" or "lidar" not in event.get("weights", {}):
            continue
        scheduler_rows[phase(float(event["stamp_s"]))].append({
            "reliability_weight": float(event["weights"]["lidar"]),
            "covariance_inflation": float(event["covariance_inflation"]["lidar"]),
            "factor_enabled": 1.0 if event["factor_enabled"]["lidar"] else 0.0,
        })

    report = load_json(run_dir / "report.json")
    performance = load_json(run_dir / "simulation_performance.json")
    result = {
        "run_dir": str(run_dir),
        "fault_window_stamp_s": {
            "start": fault_start,
            "end": fault_end,
            "duration": None if fault_start is None else fault_end - fault_start,
            "active_messages": len(fault_events),
        },
        "trajectory": report.get("trajectory", {}),
        "timestamp_regressions": report.get("timestamp_regressions", {}),
        "simulation": performance.get("simulation", {}),
        "phases": {},
    }
    phase_names = ("nominal",) if fault_start is None else ("pre_fault", "fault_active", "recovery")
    for name in phase_names:
        result["phases"][name] = {
            "native_factor": distribution(native_rows[name], (
                "matched_points",
                "residual_p95_m",
                "pose_hessian_min_eigenvalue",
                "pose_hessian_min_eigenvalue_per_match",
                "pose_hessian_condition_number",
            )),
            "lio_diagnostic": distribution(lio_rows[name], (
                "matched_points",
                "residual_p95_m",
                "hessian_min_eigenvalue",
                "hessian_condition",
                "normal_min_eigenvalue",
                "axial_penalty",
            )),
            "reliability": distribution(score_rows[name], (
                "degradation_score",
                "reliability_weight",
                "valid",
            )),
            "scheduler": distribution(scheduler_rows[name], (
                "reliability_weight",
                "covariance_inflation",
                "factor_enabled",
            )),
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = summarize(args.run_dir.resolve())
    payload = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")


if __name__ == "__main__":
    main()
