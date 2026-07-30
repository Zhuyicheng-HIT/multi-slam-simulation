#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


ORDER = {
    name: index for index, name in enumerate((
        "t0", "stationary", "hover", "ag", "yaw_30", "yaw_90",
        "straight", "l_shape", "single_corner", "small_rectangle",
        "loop_return"))
}


def nested(data, *keys, default=None):
    value = data
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def number(value, digits=3):
    return "n/a" if value is None else f"{value:.{digits}f}"


def stage_quality(summary_path):
    latency_path = summary_path.with_name("rtab_latency.csv")
    stages = {}
    if not latency_path.is_file():
        return stages
    with latency_path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                quality = int(row["quality_inliers"])
            except (KeyError, TypeError, ValueError):
                continue
            stage = row.get("stage") or "unlabelled"
            stages.setdefault(stage, []).append(quality)
    return {
        stage: {"mean": sum(values) / len(values), "min": min(values)}
        for stage, values in stages.items()
    }


def normalize_loop_counts(summary_path, record):
    loop = record.setdefault("loop_closure", {})
    loop_path = summary_path.with_name("loop_closure.csv")
    if not loop_path.is_file():
        return
    global_events = 0
    proximity_events = 0
    with loop_path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                global_events += int(row.get("loop_closure_id") or 0) > 0
                proximity_events += int(
                    row.get("proximity_detection_id") or 0) > 0
            except ValueError:
                continue
    loop["global_accepted_events"] = global_events
    loop["proximity_accepted_events"] = proximity_events
    loop["accepted_events"] = global_events + proximity_events


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    root = Path(arguments.root).expanduser()
    output = Path(arguments.output).expanduser()
    records = []
    for summary_path in root.rglob("summary.json"):
        try:
            data = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if "profile" not in data or "bridge" not in data or "rtab" not in data:
            continue
        data["_path"] = summary_path
        data["_stage_quality"] = stage_quality(summary_path)
        normalize_loop_counts(summary_path, data)
        records.append(data)
    latest = {}
    for record in sorted(records, key=lambda item: item["_path"].stat().st_mtime):
        latest[record["profile"]] = record
    records = sorted(
        latest.values(),
        key=lambda item: (ORDER.get(item["profile"], 999), item["profile"]))

    lines = [
        "# D435i visual robustness comparison", "",
        "All rows use the unchanged 640x480 C++/16UC1/exact-sync RTAB baseline. "
        "Rates and latency are measured by the lightweight stamp-correlating "
        "side-channel profiler.", "",
        "| Profile | Grade | Bridge Hz | RTAB Hz | Process ratio | Latency mean/p95/max ms | Slope ms/s | ATE/RPE m | Quality min | Lost/reset | Loop cand/global/proximity/rej |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for record in records:
        rtab = record["rtab"]
        latency = rtab.get("latency_ms") or {}
        trajectory = record.get("trajectory") or {}
        loop = record.get("loop_closure") or {}
        lines.append(
            f"| {record['profile']} | {record['classification']} | "
            f"{number(nested(record, 'bridge', 'rate', 'mean_hz'))} | "
            f"{number(nested(rtab, 'rate', 'mean_hz'))} | "
            f"{number(rtab.get('processing_ratio'))} | "
            f"{number(latency.get('mean'))}/{number(latency.get('p95'))}/"
            f"{number(latency.get('max'))} | "
            f"{number(nested(rtab, 'latency_trend_ms_per_s', 'slope_per_s'))} | "
            f"{number(trajectory.get('ate_rmse_m'), 4)}/"
            f"{number(trajectory.get('rpe_translation_rmse_m'), 4)} | "
            f"{number(nested(rtab, 'quality', 'min'), 0)} | "
            f"{rtab.get('lost_events', 0) + rtab.get('lost_log_count', 0)}/"
            f"{rtab.get('reset_log_count', 0)} | "
            f"{loop.get('candidate_events', 0)}/"
            f"{loop.get('global_accepted_events', 0)}/"
            f"{loop.get('proximity_accepted_events', 0)}/"
            f"{loop.get('rejected_events', 0)} |")

    lines.extend(["", "## Stage quality", ""])
    for record in records:
        stages = record["_stage_quality"]
        if not stages:
            continue
        lines.append(f"### {record['profile']}")
        lines.append("")
        lines.append("| Stage | quality mean | quality minimum |")
        lines.append("|---|---:|---:|")
        for stage, values in stages.items():
            lines.append(
                f"| {stage} | {values['mean']:.2f} | {values['min']} |")
        lines.append("")

    lines.extend(["## Evidence", ""])
    for record in records:
        lines.append(f"- `{record['profile']}`: `{record['_path'].parent}`")
    lines.append("")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
