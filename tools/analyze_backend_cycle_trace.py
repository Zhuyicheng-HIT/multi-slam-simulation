#!/usr/bin/env python3
"""Analyze transaction-scoped backend performance traces and correlations."""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def load_jsonl(paths):
    records = []
    for path in paths:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            if line.strip():
                records.append(json.loads(line))
    return records


def numeric(values):
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


def summary(values):
    values = numeric(values)
    if not values:
        return {}
    array = np.asarray(values, dtype=float)
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)),
        "std": float(np.std(array)),
        "cv": float(np.std(array) / np.mean(array)) if np.mean(array) else None,
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def correlation(first, second):
    pairs = [
        (float(left), float(right))
        for left, right in zip(first, second)
        if left is not None and right is not None
        and math.isfinite(float(left)) and math.isfinite(float(right))
    ]
    if len(pairs) < 3:
        return None
    left = np.asarray([item[0] for item in pairs], dtype=float)
    right = np.asarray([item[1] for item in pairs], dtype=float)
    if np.std(left) <= 1.0e-12 or np.std(right) <= 1.0e-12:
        return None
    return float(np.corrcoef(left, right)[0, 1])


def nearest_compute_sample(record, samples):
    if not samples:
        return None
    target = float(record["wall_finished_s"])
    return min(samples, key=lambda sample: abs(float(sample["wall_monotonic_s"]) - target))


def process_value(sample, group, field):
    if sample is None:
        return None
    return sample.get("process_groups", {}).get(group, {}).get(field)


def active_factor_count(record, name):
    return record.get("active_factor_counts", record.get("factor_counts", {})).get(
        name
    )


def added_factor_count(record, name):
    return record.get("factor_counts_added", {}).get(name)


def analyze(records, performance=None):
    compute_samples = []
    if performance is not None:
        compute_samples = performance.get("compute", {}).get("time_series", [])
    aligned = [nearest_compute_sample(record, compute_samples) for record in records]
    solver = [record.get("solver_duration_ms") for record in records]
    marginal = [record for record in records if record.get("marginalization_happened")]
    normal = [record for record in records if not record.get("marginalization_happened")]
    stage_names = sorted({
        name
        for record in records
        for name in (
            set(record.get("phases_ms", {}))
            | set(record.get("solver_profile_ms", {}))
        )
        if name != "stage_call_counts"
    })
    stages = {}
    for name in stage_names:
        stages[name] = summary([
            record.get("phases_ms", {}).get(
                name, record.get("solver_profile_ms", {}).get(name)
            )
            for record in records
        ])
    features = {
        "window_states": [record.get("window_state_count") for record in records],
        "lidar_correspondences": [
            record.get("lidar_correspondence_count") for record in records
        ],
        "visual_factors": [
            active_factor_count(record, "visual") for record in records
        ],
        "imu_factors": [
            active_factor_count(record, "imu") for record in records
        ],
        "gnss_factors": [
            active_factor_count(record, "gnss") for record in records
        ],
        "flow_factors": [
            active_factor_count(record, "flow") for record in records
        ],
        "visual_factors_added": [
            added_factor_count(record, "visual") for record in records
        ],
        "imu_factors_added": [
            added_factor_count(record, "imu") for record in records
        ],
        "gnss_factors_added": [
            added_factor_count(record, "gnss") for record in records
        ],
        "flow_factors_added": [
            added_factor_count(record, "flow") for record in records
        ],
        "minor_faults": [
            record.get("resource_delta", {}).get("minor_faults") for record in records
        ],
        "major_faults": [
            record.get("resource_delta", {}).get("major_faults") for record in records
        ],
        "voluntary_context_switches": [
            record.get("resource_delta", {}).get("voluntary_context_switches")
            for record in records
        ],
        "involuntary_context_switches": [
            record.get("resource_delta", {}).get("involuntary_context_switches")
            for record in records
        ],
        "gc_duration_ms": [
            sum(record.get("gc", {}).get("duration_ms", [])) for record in records
        ],
        "gc_collections": [
            sum(record.get("gc", {}).get("collections", [])) for record in records
        ],
        "cpu_frequency_khz": [record.get("cpu_frequency_khz") for record in records],
        "load_1m": [
            record.get("load_average", [None])[0] for record in records
        ],
        "gazebo_cpu_percent": [
            process_value(sample, "gazebo", "cpu_percent") for sample in aligned
        ],
        "fast_lio_cpu_percent": [
            process_value(sample, "fast_lio", "cpu_percent") for sample in aligned
        ],
        "rgbd_bridge_cpu_percent": [
            process_value(sample, "rgbd_bridge", "cpu_percent") for sample in aligned
        ],
        "visual_frontend_cpu_percent": [
            process_value(sample, "visual_frontend", "cpu_percent") for sample in aligned
        ],
        "shared_mapping_cpu_percent": [
            process_value(sample, "shared_mapping", "cpu_percent") for sample in aligned
        ],
        "sitl_cpu_percent": [
            process_value(sample, "sitl", "cpu_percent") for sample in aligned
        ],
    }
    normalized = {
        "ms_per_lidar_correspondence": summary([
            duration / max(1, int(record.get("lidar_correspondence_count", 0)))
            for duration, record in zip(solver, records)
            if duration is not None
        ]),
        "ms_per_visual_factor": summary([
            duration / int(active_factor_count(record, "visual") or 0)
            for duration, record in zip(solver, records)
            if (
                duration is not None
                and int(active_factor_count(record, "visual") or 0) > 0
            )
        ]),
        "ms_per_window_state": summary([
            duration / max(1, int(record.get("window_state_count", 0)))
            for duration, record in zip(solver, records)
            if duration is not None
        ]),
    }
    factor_presence = {
        name: {
            "present": summary([
                duration
                for duration, record in zip(solver, records)
                if (
                    duration is not None
                    and int(active_factor_count(record, name) or 0) > 0
                )
            ]),
            "absent": summary([
                duration
                for duration, record in zip(solver, records)
                if (
                    duration is not None
                    and int(active_factor_count(record, name) or 0) == 0
                )
            ]),
        }
        for name in ("imu", "gnss", "flow", "visual")
    }
    return {
        "schema_version": 1,
        "cycles": len(records),
        "overall_solver_ms": summary(solver),
        "normal_solver_ms": summary([
            record.get("solver_duration_ms") for record in normal
        ]),
        "marginalization_solver_ms": summary([
            record.get("solver_duration_ms") for record in marginal
        ]),
        "marginalization_cycles": len(marginal),
        "stages_ms": stages,
        "feature_summaries": {
            name: summary(values) for name, values in features.items()
        },
        "solver_correlations": {
            name: correlation(solver, values) for name, values in features.items()
        },
        "solver_by_factor_presence_ms": factor_presence,
        "normalized_solver": normalized,
        "correctness": {
            "optimization_errors_max": max(
                (record.get("optimization_errors", 0) for record in records),
                default=0,
            ),
            "integrity_rejects_max": max(
                (record.get("integrity_rejects", 0) for record in records),
                default=0,
            ),
            "rollbacks_max": max(
                (record.get("rollbacks", 0) for record in records), default=0
            ),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", required=True, nargs="+")
    parser.add_argument("--performance")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    records = load_jsonl(args.trace)
    performance = None
    if args.performance:
        performance = json.loads(Path(args.performance).read_text(encoding="utf-8"))
    report = analyze(records, performance)
    Path(args.output).write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
