#!/usr/bin/env python3
"""Summarize causal trajectory errors within recorded mission phases."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def summarize(rows):
    error_x = np.asarray([row["error_x_m"] for row in rows], dtype=float)
    error_y = np.asarray([row["error_y_m"] for row in rows], dtype=float)
    error_z = np.asarray([row["error_z_m"] for row in rows], dtype=float)
    horizontal = np.hypot(error_x, error_y)
    error_3d = np.sqrt(error_x * error_x + error_y * error_y + error_z * error_z)
    stamps = np.asarray([row["stamp_s"] for row in rows], dtype=float)
    return {
        "count": len(rows),
        "duration_s": float(stamps[-1] - stamps[0]),
        "three_dimensional": {
            "rmse_m": float(math.sqrt(np.mean(error_3d * error_3d))),
            "p95_m": float(np.percentile(error_3d, 95.0)),
            "maximum_m": float(np.max(error_3d)),
        },
        "horizontal": {
            "rmse_m": float(math.sqrt(np.mean(horizontal * horizontal))),
            "p95_m": float(np.percentile(horizontal, 95.0)),
            "maximum_m": float(np.max(horizontal)),
        },
        "vertical": {
            "bias_m": float(np.mean(error_z)),
            "rmse_m": float(math.sqrt(np.mean(error_z * error_z))),
            "p95_absolute_m": float(np.percentile(np.abs(error_z), 95.0)),
            "maximum_absolute_m": float(np.max(np.abs(error_z))),
            "first_m": float(error_z[0]),
            "last_m": float(error_z[-1]),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    with args.samples.open(newline="", encoding="utf-8") as stream:
        rows = [
            {key: float(value) for key, value in row.items() if key != "above_threshold"}
            for row in csv.DictReader(stream)
        ]
    runtime = json.loads(args.runtime.read_text(encoding="utf-8"))
    timeline = runtime.get("mission_phase_timeline", [])
    result = {"samples": str(args.samples.resolve()), "phases": {}}
    for index, event in enumerate(timeline):
        start = float(event["stamp_s"])
        end = (
            float(timeline[index + 1]["stamp_s"])
            if index + 1 < len(timeline)
            else math.inf
        )
        selected = [row for row in rows if start <= row["stamp_s"] < end]
        if selected:
            result["phases"][event["phase"]] = summarize(selected)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
