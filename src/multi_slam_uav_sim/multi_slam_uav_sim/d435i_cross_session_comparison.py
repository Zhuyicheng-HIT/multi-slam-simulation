
#!/usr/bin/env python3
"""Aggregate independent cross-session relocalization attempts."""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml


def finite(values):
    return np.asarray([
        float(value) for value in values
        if value is not None and math.isfinite(float(value))
    ], dtype=float)


def statistic(values, name):
    array = finite(values)
    if not len(array):
        return None
    if name == "mean":
        return float(np.mean(array))
    if name == "median":
        return float(np.median(array))
    if name == "p95":
        return float(np.percentile(array, 95))
    raise ValueError(name)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("matrix_dir")
    parser.add_argument("--conditions-config", required=True)
    parser.add_argument(
        "--source-matrix", action="append", default=[],
        help="Additional immutable matrix directory to include.")
    args = parser.parse_args(argv)
    matrix = Path(args.matrix_dir).resolve()
    config = yaml.safe_load(Path(args.conditions_config).read_text(
        encoding="utf-8"))
    order = list(config["conditions"])
    records = []
    sources = [matrix, *(Path(item).resolve() for item in args.source_matrix)]
    result_paths = []
    for source in sources:
        result_paths.extend(source.glob(
            "*_attempt*/result/result/relocalization_summary.json"))
        # Also accept a deliberately consolidated directory whose per-attempt
        # summaries are one level shallower.
        result_paths.extend(source.glob(
            "*_attempt*/result/relocalization_summary.json"))
    for path in sorted(set(result_paths)):
        record = json.loads(path.read_text(encoding="utf-8"))
        record["result_path"] = str(path.parent)
        record["source_matrix"] = str(next(
            (source for source in sources if source in path.parents), matrix))
        records.append(record)
    grouped = defaultdict(list)
    for record in records:
        grouped[record["condition"]].append(record)

    fields = [
        "condition", "valid_runs", "successes", "failures",
        "success_rate", "false_relocalizations", "false_relocalization_rate",
        "rejected_candidates", "candidate_latency_median_s",
        "accepted_latency_median_s", "stable_latency_median_s",
        "stable_latency_p95_s", "stable_position_error_mean_m",
        "stable_position_error_p95_m", "stable_yaw_error_mean_deg",
        "stable_yaw_error_p95_deg", "geometry_inliers_mean",
        "initial_position_error_mean_m", "initial_yaw_error_mean_deg",
        "final_position_error_mean_m", "final_yaw_error_mean_deg",
        "maximum_map_to_odom_jump_m", "lost_events", "reset_events",
        "tf_backward_jumps", "failure_reasons",
    ]
    summaries = []
    for condition in order:
        valid = [record for record in grouped[condition]
                 if record.get("validation_complete")]
        success = [record for record in valid
                   if record.get("relocalization_success")]
        false = [record for record in valid
                 if record.get("false_relocalization")]
        reasons = sorted({reason for record in valid
                          for reason in record.get("failure_reasons", [])})
        row = {
            "condition": condition, "valid_runs": len(valid),
            "successes": len(success), "failures": len(valid) - len(success),
            "success_rate": len(success) / len(valid) if valid else 0.0,
            "false_relocalizations": len(false),
            "false_relocalization_rate": len(false) / len(valid) if valid else 0.0,
            "rejected_candidates": sum(
                int(record.get("rejected_candidate_count", 0)) for record in valid),
            "candidate_latency_median_s": statistic([
                record.get("time_to_first_candidate_s") for record in valid],
                "median"),
            "accepted_latency_median_s": statistic([
                record.get("time_to_accepted_closure_s") for record in valid],
                "median"),
            "stable_latency_median_s": statistic([
                record.get("time_to_stable_alignment_s") for record in success],
                "median"),
            "stable_latency_p95_s": statistic([
                record.get("time_to_stable_alignment_s") for record in success],
                "p95"),
            "stable_position_error_mean_m": statistic([
                record.get("stable_position_error_m") for record in success],
                "mean"),
            "stable_position_error_p95_m": statistic([
                record.get("stable_position_error_m") for record in success],
                "p95"),
            "stable_yaw_error_mean_deg": statistic([
                record.get("stable_yaw_error_deg") for record in success],
                "mean"),
            "stable_yaw_error_p95_deg": statistic([
                record.get("stable_yaw_error_deg") for record in success],
                "p95"),
            "geometry_inliers_mean": statistic([
                record.get("geometry_inliers") for record in success], "mean"),
            "initial_position_error_mean_m": statistic([
                record.get("initial_position_error_m") for record in success],
                "mean"),
            "initial_yaw_error_mean_deg": statistic([
                record.get("initial_yaw_error_deg") for record in success],
                "mean"),
            "final_position_error_mean_m": statistic([
                (record.get("final_position_error_m") or {}).get("mean")
                for record in success], "mean"),
            "final_yaw_error_mean_deg": statistic([
                (record.get("final_yaw_error_deg") or {}).get("mean")
                for record in success], "mean"),
            "maximum_map_to_odom_jump_m": max((
                float(record.get("maximum_map_to_odom_jump_m") or 0.0)
                for record in valid), default=0.0),
            "lost_events": sum(int(record.get("lost_events", 0))
                               for record in valid),
            "reset_events": sum(int(record.get("reset_events", 0))
                                for record in valid),
            "tf_backward_jumps": sum(int(record.get("tf_backward_jumps", 0))
                                     for record in valid),
            "failure_reasons": "; ".join(reasons),
        }
        summaries.append(row)
    with (matrix / "cross_session_validation.csv").open(
            "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)
    run_fields = [
        "condition", "validation_complete", "relocalization_success",
        "false_relocalization", "candidate_event_count",
        "accepted_event_count", "rejected_candidate_count",
        "accepted_node_id", "accepted_map_id", "geometry_inliers",
        "visual_words", "time_to_first_candidate_s",
        "time_to_accepted_closure_s", "time_to_stable_alignment_s",
        "initial_position_error_m", "initial_yaw_error_deg",
        "stable_position_error_m", "stable_yaw_error_deg",
        "final_position_error_m", "final_yaw_error_deg",
        "maximum_map_to_odom_jump_m",
        "abnormal_post_alignment_jump_count", "lost_events", "reset_events",
        "tf_backward_jumps", "failure_reasons", "source_matrix", "result_path",
    ]
    with (matrix / "cross_session_runs.csv").open(
            "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=run_fields)
        writer.writeheader()
        for record in records:
            row = {key: record.get(key, "") for key in run_fields}
            row["failure_reasons"] = "; ".join(
                record.get("failure_reasons", []))
            writer.writerow(row)
    output = {
        "matrix_dir": str(matrix),
        "source_matrices": [str(source) for source in sources],
        "conditions": summaries, "runs": records,
        "valid_runs": sum(row["valid_runs"] for row in summaries),
        "successes": sum(row["successes"] for row in summaries),
        "false_relocalizations": sum(
            row["false_relocalizations"] for row in summaries),
    }
    (matrix / "cross_session_validation.json").write_text(
        json.dumps(output, indent=2, sort_keys=True), encoding="utf-8")
    lines = [
        "# RTAB-Map cross-session relocalization matrix", "",
        "| Condition | Valid | Success | False relocalization | "
        "Stable latency median (s) | Position mean (m) | Yaw mean (deg) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summaries:
        def show(value, digits=3):
            return "n/a" if value is None else f"{value:.{digits}f}"
        lines.append(
            f"| {row['condition']} | {row['valid_runs']} | "
            f"{row['successes']} ({row['success_rate']:.0%}) | "
            f"{row['false_relocalizations']} | "
            f"{show(row['stable_latency_median_s'])} | "
            f"{show(row['stable_position_error_mean_m'])} | "
            f"{show(row['stable_yaw_error_mean_deg'], 2)} |")
    lines.extend([
        "", f"Valid independent sessions: {output['valid_runs']}",
        f"Successful relocalizations: {output['successes']}",
        f"False relocalizations: {output['false_relocalizations']}",
    ])
    (matrix / "cross_session_validation.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")
    print(matrix / "cross_session_validation.md")


if __name__ == "__main__":
    main()

