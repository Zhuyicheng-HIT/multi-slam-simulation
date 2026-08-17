#!/usr/bin/env python3
"""Correlate causal vertical error with backend Z-observability evidence."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def _nearest_trace(rows, stamps, stamp_s):
    index = int(np.argmin(np.abs(stamps - float(stamp_s))))
    return rows[index]


def _optional_float(value):
    return math.nan if value is None else float(value)


def _error_intervals(samples, threshold_m):
    intervals = []
    start = None
    for index, sample in enumerate(samples + [None]):
        exceeded = sample is not None and abs(sample["error_z_m"]) > threshold_m
        if exceeded and start is None:
            start = index
        if not exceeded and start is not None:
            chunk = samples[start:index]
            intervals.append({
                "start_s": chunk[0]["stamp_s"],
                "end_s": chunk[-1]["stamp_s"],
                "duration_s": chunk[-1]["stamp_s"] - chunk[0]["stamp_s"],
                "maximum_abs_error_m": max(abs(row["error_z_m"]) for row in chunk),
                "sample_count": len(chunk),
            })
            start = None
    return intervals


def analyze(run_dir, threshold_m):
    run_dir = Path(run_dir)
    with (run_dir / "external_nav_accuracy.samples.csv").open() as stream:
        samples = [
            {
                key: float(value)
                for key, value in row.items()
                if key != "above_threshold"
            }
            for row in csv.DictReader(stream)
        ]
    with (run_dir / "backend_cycle_trace.jsonl").open() as stream:
        traces = [json.loads(line) for line in stream if line.strip()]
    trace_stamps = np.asarray([row["stamp_s"] for row in traces], dtype=float)

    enriched = []
    for sample in samples:
        trace = _nearest_trace(traces, trace_stamps, sample["stamp_s"])
        gauge = trace.get("z_gauge", {})
        gnss = trace.get("gnss_prefit", {})
        lidar = trace.get("lidar_observability", {})
        lidar_degradation = lidar.get("combined_degradation_xyz", [math.nan] * 3)
        active_factors = trace.get("active_factor_counts", {})
        added_factors = trace.get("factor_counts_added", {})
        enriched.append({
            **sample,
            "gauge_offset_m": _optional_float(gauge.get("offset_m")),
            "gauge_target_m": _optional_float(gauge.get("target_offset_m")),
            "gauge_reason": str(gauge.get("reason", "missing")),
            "gnss_z_nis": _optional_float(gnss.get("z_nis")),
            "gnss_z_admitted": bool(gnss.get("z_admitted", False)),
            "gnss_z_routed_to_gauge": bool(
                gnss.get("z_routed_to_gauge", False)
            ),
            "lidar_z_degradation": _optional_float(lidar_degradation[2]),
            "lidar_z_support": _optional_float(
                lidar.get("isotropic_information_support_xyz", [math.nan] * 3)[2]
            ),
            "gnss_active_factors": int(active_factors.get("gnss", 0)),
            "gnss_added_factors": int(added_factors.get("gnss", 0)),
        })

    gauge_jumps = []
    previous = None
    for trace in traces:
        gauge = trace.get("z_gauge", {})
        offset = _optional_float(gauge.get("offset_m"))
        if math.isfinite(offset) and previous is not None and abs(offset - previous) > 0.20:
            gauge_jumps.append({
                "stamp_s": trace["stamp_s"],
                "previous_offset_m": previous,
                "offset_m": offset,
                "target_offset_m": gauge.get("target_offset_m"),
                "reason": gauge.get("reason"),
            })
        if math.isfinite(offset):
            previous = offset

    worst = sorted(enriched, key=lambda row: abs(row["error_z_m"]), reverse=True)[:20]
    result = {
        "sample_count": len(enriched),
        "threshold_m": threshold_m,
        "vertical_error": {
            "rmse_m": float(np.sqrt(np.mean([
                row["error_z_m"] ** 2 for row in enriched
            ]))),
            "p95_m": float(np.percentile([
                abs(row["error_z_m"]) for row in enriched
            ], 95)),
            "max_m": float(max(abs(row["error_z_m"]) for row in enriched)),
        },
        "exceedance_intervals": _error_intervals(enriched, threshold_m),
        "gauge_jumps_over_0_20_m": gauge_jumps,
        "worst_samples": worst,
        "periodic_samples": [
            min(enriched, key=lambda row: abs(row["stamp_s"] - stamp))
            for stamp in np.arange(
                samples[0]["stamp_s"], samples[-1]["stamp_s"] + 0.1, 5.0
            )
        ],
    }
    output_path = run_dir / "vertical_error_diagnosis.json"
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--threshold-m", type=float, default=0.15)
    args = parser.parse_args()
    raise SystemExit(analyze(args.run_dir, args.threshold_m))


if __name__ == "__main__":
    main()
