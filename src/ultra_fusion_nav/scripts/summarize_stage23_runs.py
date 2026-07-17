#!/usr/bin/env python3
import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = round((len(ordered) - 1) * fraction)
    return float(ordered[index])


def score_summary(path):
    values = defaultdict(lambda: defaultdict(list))
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            modality = row["modality"]
            values[modality]["score"].append(float(row["degradation_score"]))
            values[modality]["valid"].append(float(row["valid"]))
            evidence = json.loads(row["evidence_json"])
            coverage = evidence.get("evidence_weight_coverage")
            if coverage is not None:
                values[modality]["coverage"].append(float(coverage))
    result = {}
    for modality, samples in sorted(values.items()):
        result[modality] = {
            "samples": len(samples["score"]),
            "score_p50": percentile(samples["score"], 0.50),
            "score_p95": percentile(samples["score"], 0.95),
            "valid_rate": statistics.fmean(samples["valid"]),
            "coverage_p50": percentile(samples["coverage"], 0.50),
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    runs = []
    for value in args.runs:
        directory = Path(value).resolve()
        report = json.loads((directory / "report.json").read_text(encoding="utf-8"))
        trajectory = json.loads((directory / "trajectory_metrics.json").read_text(encoding="utf-8"))
        reliability_path = directory / "reliability_scores.csv"
        runs.append({
            "run": directory.name,
            "passed": bool(report["passed"]),
            "position_rmse_m": float(report["trajectory"]["position_rmse_m"]),
            "yaw_rmse_deg": float(report["trajectory"]["yaw_rmse_deg"]),
            "final_position_error_m": float(report["trajectory"]["final_position_error_m"]),
            "yaw_gyro_correlation": float(report["trajectory"]["fast_yaw_vs_fcu_gyro_corr"]),
            "ate_rmse_m": float(trajectory["ate_rmse_m"]),
            "rpe_translation_rmse_m": float(trajectory["rpe_translation_rmse_m"]),
            "rpe_rotation_rmse_deg": float(trajectory["rpe_rotation_rmse_deg"]),
            "voxel_overlap_median": float(report["pointcloud"]["voxel_overlap_median"]),
            "centroid_jump_p95_m": float(report["pointcloud"]["centroid_jump_p95_m"]),
            "timestamp_regressions": dict(report["timestamp_regressions"]),
            "reliability": score_summary(reliability_path) if reliability_path.exists() else {},
        })

    metric_names = (
        "position_rmse_m", "yaw_rmse_deg", "final_position_error_m",
        "yaw_gyro_correlation", "ate_rmse_m", "rpe_translation_rmse_m",
        "rpe_rotation_rmse_deg", "voxel_overlap_median", "centroid_jump_p95_m",
    )
    aggregate = {}
    for name in metric_names:
        values = [run[name] for run in runs]
        aggregate[name] = {
            "min": min(values),
            "median": statistics.median(values),
            "max": max(values),
        }
    result = {
        "run_count": len(runs),
        "pass_count": sum(run["passed"] for run in runs),
        "pass_rate": statistics.fmean(float(run["passed"]) for run in runs),
        "runs": runs,
        "aggregate": aggregate,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["pass_count"] == result["run_count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
