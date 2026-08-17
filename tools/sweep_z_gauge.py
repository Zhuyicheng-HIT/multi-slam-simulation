#!/usr/bin/env python3
"""Sweep causal Z-gauge parameters on a completed deterministic replay."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

import numpy as np

from uf_backend_fusion.z_gauge import LocalToGlobalZGauge


def _nearest_indices(reference, query):
    right = np.searchsorted(reference, query, side="left")
    right = np.clip(right, 0, len(reference) - 1)
    left = np.clip(right - 1, 0, len(reference) - 1)
    return np.where(
        np.abs(reference[right] - query) < np.abs(reference[left] - query),
        right,
        left,
    )


def _percentile(values, percentile):
    return float(np.percentile(np.asarray(values, dtype=float), percentile))


def _events(traces):
    result = []
    previous_stamp = None
    for trace in traces:
        gnss = trace.get("gnss_prefit", {})
        gauge = trace.get("z_gauge", {})
        stamp = gnss.get("stamp_s")
        target = gauge.get("raw_target_offset_m")
        if stamp is None or target is None or stamp == previous_stamp:
            continue
        previous_stamp = stamp
        reason = str(gauge.get("reason", ""))
        result.append({
            "stamp_s": float(stamp),
            "target_m": float(target),
            "lidar_z_weak": reason != "lidar_z_observable",
        })
    return result


def _simulate(events, sample_stamps, history_size, tau_s, rate_mps, step_m):
    gauge = LocalToGlobalZGauge(
        initialization_samples=3,
        initialization_max_spread_m=0.30,
        target_history_size=history_size,
        update_time_constant_s=tau_s,
        maximum_correction_rate_mps=rate_mps,
        maximum_correction_step_m=step_m,
        minimum_variance_m2=0.04,
        maximum_variance_m2=25.0,
    )
    event_stamps = []
    offsets = []
    for event in events:
        gauge.update(
            event["stamp_s"],
            0.0,
            event["target_m"],
            0.09,
            source_healthy=True,
            lidar_z_weak=event["lidar_z_weak"],
        )
        event_stamps.append(event["stamp_s"])
        offsets.append(gauge.offset_m)
    event_stamps = np.asarray(event_stamps, dtype=float)
    offsets = np.asarray(offsets, dtype=float)
    indices = np.searchsorted(event_stamps, sample_stamps, side="right") - 1
    return np.where(indices >= 0, offsets[np.maximum(indices, 0)], 0.0)


def analyze(run_dir):
    run_dir = Path(run_dir)
    with (run_dir / "external_nav_accuracy.samples.csv").open() as stream:
        samples = list(csv.DictReader(stream))
    with (run_dir / "backend_cycle_trace.jsonl").open() as stream:
        traces = [json.loads(line) for line in stream if line.strip()]
    sample_stamps = np.asarray([float(row["stamp_s"]) for row in samples])
    truth_z = np.asarray([float(row["truth_z_m"]) for row in samples])
    output_z = np.asarray([float(row["estimate_raw_z_m"]) for row in samples])
    trace_stamps = np.asarray([float(row["stamp_s"]) for row in traces])
    nearest = _nearest_indices(trace_stamps, sample_stamps)
    original_offsets = np.asarray([
        float(traces[index].get("z_gauge", {}).get("offset_m", 0.0))
        for index in nearest
    ])
    local_z = output_z - original_offsets
    events = _events(traces)

    results = []
    for history, tau_s, rate_mps, step_m in itertools.product(
        (1, 3, 5, 7),
        (0.05, 0.10, 0.20, 0.40),
        (1.0, 2.0, 3.0, 5.0),
        (0.30, 0.60, 1.00),
    ):
        candidate_offsets = _simulate(
            events, sample_stamps, history, tau_s, rate_mps, step_m
        )
        candidate_z = local_z + candidate_offsets
        initial = sample_stamps <= sample_stamps[0] + 10.0
        alignment = float(np.mean(truth_z[initial] - candidate_z[initial]))
        errors = candidate_z + alignment - truth_z
        absolute = np.abs(errors)
        results.append({
            "history_size": history,
            "tau_s": tau_s,
            "rate_mps": rate_mps,
            "step_m": step_m,
            "alignment_m": alignment,
            "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
            "p95_m": _percentile(absolute, 95),
            "max_m": float(np.max(absolute)),
            "above_0_15_ratio": float(np.mean(absolute > 0.15)),
        })
    results.sort(key=lambda row: (row["p95_m"], row["rmse_m"], row["max_m"]))
    report = {
        "run_dir": str(run_dir.resolve()),
        "event_count": len(events),
        "sample_count": len(samples),
        "best_by_p95": results[:30],
    }
    output = run_dir / "z_gauge_sweep.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    args = parser.parse_args()
    analyze(args.run_dir)


if __name__ == "__main__":
    main()
