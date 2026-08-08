#!/usr/bin/env python3
"""Aggregate frozen full-online backend replay runs without cherry-picking."""

import argparse
import json
import statistics
from pathlib import Path

import numpy as np


def stats(values):
    values = [float(value) for value in values]
    if not values:
        return {}
    mean = statistics.fmean(values)
    std = statistics.pstdev(values)
    return {
        "count": len(values),
        "mean": mean,
        "median": statistics.median(values),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "std": std,
        "cv": std / mean if mean else 0.0,
        "min": min(values),
        "max": max(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("inputs", nargs="+")
    args = parser.parse_args()
    reports = [json.loads(Path(path).read_text()) for path in args.inputs]
    per_run_median = [report["solver_ms"]["median"] for report in reports]
    pooled = [
        sample["values"]["backend_solve_ms"]
        for report in reports
        for sample in report["samples"]
        if (
            isinstance(sample["values"].get("backend_solve_ms"), (int, float))
            and sample["values"]["backend_solve_ms"] > 0.0
        )
    ]
    output = {
        "schema_version": 1,
        "label": args.label,
        "run_count": len(reports),
        "per_run_solver_median_ms": per_run_median,
        "run_median_statistics_ms": stats(per_run_median),
        "pooled_solver_ms": stats(pooled),
        "odom_counts": [report["odom_count"] for report in reports],
        "last_correctness": [
            {
                key: report["last_values"].get(key)
                for key in (
                    "optimization_errors",
                    "optimization_integrity_counts",
                    "optimization_rejected",
                    "optimization_rollbacks",
                    "native_worker_errors",
                    "visual_factors",
                )
            }
            for report in reports
        ],
    }
    Path(args.output).write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
