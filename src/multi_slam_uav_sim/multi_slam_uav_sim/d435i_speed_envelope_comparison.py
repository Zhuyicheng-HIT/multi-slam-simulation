#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


FIELDS = [
    "matrix", "case", "profile", "valid_speed_experiment",
    "robustness_classification", "commanded_speed", "actual_speed_mean",
    "actual_speed_p95", "actual_speed_max", "speed_valid_duration_s",
    "commanded_yaw_rate", "actual_yaw_rate_mean", "actual_yaw_rate_p95",
    "processed_frame_translation_p95", "processed_frame_yaw_delta_p95",
    "ATE", "RPE", "quality_min", "quality_p5", "features_mean",
    "inliers_min", "inliers_p5", "lost", "reset", "GlobalClosure",
    "LocalSpaceClosure", "wrong_closure", "E2E_p95", "RTAB_Hz", "RTF",
    "classification", "result_dir",
]


def nested(mapping, *keys, default=""):
    value = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def database_summary(path):
    if not path.is_file():
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        return {row["metric"]: row["value"] for row in csv.DictReader(handle)}


def row_for(summary_path):
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    speed = summary.get("speed_envelope", {})
    profile = speed.get("profile", summary.get("profile", ""))
    result_dir = summary_path.parent
    matrix = next(
        (parent.name for parent in result_dir.parents
         if parent.name.startswith("matrix_")), "")
    database = database_summary(result_dir / "database_summary.csv")
    commanded_speed = (
        speed.get("commanded_vertical_speed_mps", 0.0)
        if profile == "vertical" else speed.get("commanded_horizontal_speed_mps", 0.0))
    actual_yaw_mean = nested(speed, "actual_yaw_rate_deg_s", "mean")
    actual_yaw_p95 = nested(speed, "actual_yaw_rate_deg_s", "p95")
    rtf = nested(summary, "resources", "rtf", "mean")
    global_closure = database.get(
        "rtabmap_info_active_links_GlobalClosure",
        nested(summary, "loop_closure", "global_accepted_events", default=0))
    local_space = database.get(
        "rtabmap_info_active_links_LocalSpaceClosure",
        nested(summary, "loop_closure", "proximity_accepted_events", default=0))
    return {
        "matrix": matrix,
        "case": result_dir.parent.name,
        "profile": profile,
        "valid_speed_experiment": speed.get(
            "valid_speed_experiment", False),
        "robustness_classification": summary.get(
            "robustness_classification", ""),
        "commanded_speed": commanded_speed,
        "actual_speed_mean": speed.get("mean", ""),
        "actual_speed_p95": speed.get("p95", ""),
        "actual_speed_max": speed.get("max", ""),
        "speed_valid_duration_s": speed.get("sustained_above_80_percent_s", ""),
        "commanded_yaw_rate": speed.get("commanded_yaw_rate_deg_s", 0.0),
        "actual_yaw_rate_mean": actual_yaw_mean,
        "actual_yaw_rate_p95": actual_yaw_p95,
        "processed_frame_translation_p95": nested(
            speed, "processed_frame_translation", "p95"),
        "processed_frame_yaw_delta_p95": nested(
            speed, "processed_frame_yaw_delta_deg", "p95"),
        "ATE": nested(summary, "trajectory", "ate_rmse_m"),
        "RPE": nested(summary, "trajectory", "rpe_translation_rmse_m"),
        "quality_min": nested(summary, "rtab", "quality", "min"),
        "quality_p5": nested(summary, "rtab", "quality", "p05"),
        "features_mean": nested(summary, "rtab", "features", "mean"),
        "inliers_min": nested(speed, "inliers", "min"),
        "inliers_p5": nested(speed, "inliers", "p05"),
        "lost": (nested(summary, "rtab", "lost_events", default=0)
                 + nested(summary, "rtab", "lost_log_count", default=0)),
        "reset": nested(summary, "rtab", "reset_log_count", default=0),
        "GlobalClosure": global_closure,
        "LocalSpaceClosure": local_space,
        "wrong_closure": nested(
            summary, "loop_closure", "wrong_loop_suspected", default=False),
        "E2E_p95": nested(summary, "rtab", "latency_ms", "p95"),
        "RTAB_Hz": nested(summary, "rtab", "rate", "mean_hz"),
        "RTF": rtf,
        "classification": summary.get("classification", ""),
        "result_dir": str(result_dir),
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("matrix_dir", type=Path)
    parser.add_argument(
        "--include-development", action="store_true",
        help="include matrix directories whose name contains 'smoke'")
    args = parser.parse_args(argv)
    matrix_dir = args.matrix_dir.expanduser().resolve()
    summary_paths = [
        path for path in sorted(matrix_dir.rglob("summary.json"))
        if path.parent.name == "result" and (
            args.include_development or not any(
                "smoke" in parent.name for parent in path.parents))]
    rows = [row_for(path) for path in summary_paths]
    output_csv = matrix_dir / "speed_envelope.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        "# D435i speed envelope comparison", "",
        f"Generated from {len(rows)} formal per-run `summary.json` files; "
        "development smoke runs are excluded. `Valid` means actual speed "
        "stayed at or above 80% of the command for at least one continuous "
        "second. A run can therefore remain visually healthy but be "
        "`NOT_EXERCISED`.", "",
        "Coarse result: H0--H4 PASS; Y0--Y3 PASS and Y4 NOT_EXERCISED; "
        "V0--V1 PASS and V2--V3 NOT_EXERCISED; C0--C3 PASS and C4 "
        "NOT_EXERCISED; R0--R1/L0--L1 PASS and higher short-route targets "
        "NOT_EXERCISED. H0, H2 and H4 each have three independent valid "
        "PASS results.", "",
        "| Matrix / case | Profile | Command | Actual mean/p95 | Frame translation/yaw p95 | ATE/RPE | Lost/reset | Global/Local | Valid | Result |",
        "|---|---|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for row in rows:
        command = (row["commanded_yaw_rate"] if row["profile"] == "yaw"
                   else row["commanded_speed"])
        lines.append(
            f"| {row['matrix']} / {row['case']} | {row['profile']} | {command} | "
            f"{row['actual_speed_mean']}/{row['actual_speed_p95']} | "
            f"{row['processed_frame_translation_p95']}/"
            f"{row['processed_frame_yaw_delta_p95']} | "
            f"{row['ATE']}/{row['RPE']} | {row['lost']}/{row['reset']} | "
            f"{row['GlobalClosure']}/{row['LocalSpaceClosure']} | "
            f"{row['valid_speed_experiment']} | "
            f"{row['classification']} |")
    lines.extend(["", f"Machine-readable table: `{output_csv.name}`", ""])
    (matrix_dir / "speed_envelope_comparison.md").write_text(
        "\n".join(lines), encoding="utf-8")
    print(output_csv)


if __name__ == "__main__":
    main()
