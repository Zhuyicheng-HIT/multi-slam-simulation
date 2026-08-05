#!/usr/bin/env python3
"""Aggregate every run; never discard failed or non-best results."""

import argparse
import csv
import json
from pathlib import Path
import statistics


METRICS = (
    "translation_error_m",
    "rotation_error_deg",
    "overlap_mean_nn_m",
    "overlap_rmse_m",
    "boundary_mean_nn_m",
    "inlier_ratio",
    "supplement_voxel_growth_ratio",
)


def collect(root):
    rows = []
    matrix_path = root / "run_matrix.tsv"
    matrix_rows = []
    if matrix_path.exists():
        with matrix_path.open(encoding="utf-8", newline="") as stream:
            matrix_rows = list(csv.DictReader(stream, delimiter="\t"))
    else:
        for result_path in sorted(root.glob("run_*/**/result.json")):
            matrix_rows.append({
                "run": result_path.parts[-3],
                "method": result_path.parts[-2],
                "exit_code": "0",
                "result": str(result_path),
            })

    for matrix_row in matrix_rows:
        result_path = Path(matrix_row["result"])
        if not result_path.is_absolute():
            result_path = root / result_path
        if result_path.exists():
            data = json.loads(result_path.read_text(encoding="utf-8"))
        else:
            data = {
                "method": matrix_row["method"],
                "converged": False,
                "failure_reason": (
                    f"process_exit_{matrix_row.get('exit_code', 'unknown')}_without_result"),
                "runtime_ms": None,
                "peak_rss_kib": None,
                "blocks": {
                    "successful": 0,
                    "failed": 0,
                    "selected_cluster_size": 0,
                },
                "metrics": {
                    **{name: None for name in METRICS},
                    "overlap_pairs": None,
                },
            }
        rows.append({
            "run": matrix_row["run"],
            "method": data["method"],
            "converged": bool(data["converged"]),
            "failure_reason": data.get("failure_reason", ""),
            "runtime_ms": data["runtime_ms"],
            "peak_rss_kib": data["peak_rss_kib"],
            "successful_blocks": data["blocks"]["successful"],
            "failed_blocks": data["blocks"]["failed"],
            "selected_cluster_size": data["blocks"]["selected_cluster_size"],
            **data["metrics"],
            "result_path": str(result_path),
        })
    return rows


def summarize(rows):
    summary = {"total_results": len(rows), "methods": {}}
    for method in sorted({row["method"] for row in rows}):
        selected = [row for row in rows if row["method"] == method]
        entry = {
            "runs": len(selected),
            "converged": sum(row["converged"] for row in selected),
            "failed": sum(not row["converged"] for row in selected),
            "metrics": {},
            "runtime_ms": {},
            "peak_rss_kib": {},
            "successful_blocks": [row["successful_blocks"] for row in selected],
            "failed_blocks": [row["failed_blocks"] for row in selected],
        }
        for key in (*METRICS, "runtime_ms", "peak_rss_kib"):
            values = [float(row[key]) for row in selected if row[key] is not None]
            stats = {
                "mean": statistics.fmean(values) if values else None,
                "min": min(values) if values else None,
                "max": max(values) if values else None,
                "stddev": statistics.pstdev(values) if len(values) > 1 else 0.0,
            }
            if key in ("runtime_ms", "peak_rss_kib"):
                entry[key] = stats
            else:
                entry["metrics"][key] = stats
        summary["methods"][method] = entry
    return summary


def write_markdown(path, summary):
    lines = [
        "# HybridFusion three-run comparison",
        "",
        "All valid and failed runs are included; no best-run selection is used.",
        "",
        "| Method | Converged | Translation error (m) | Rotation error (deg) | "
        "Overlap NN (m) | Boundary error (m) | Inlier ratio | Supplement growth | "
        "Runtime (ms) | Peak RSS (MiB) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for method, entry in summary["methods"].items():
        metric = entry["metrics"]

        def show(name):
            value = metric[name]["mean"]
            spread = metric[name]["stddev"]
            return "n/a" if value is None else f"{value:.5f} +/- {spread:.5f}"

        rss = entry["peak_rss_kib"]["mean"]
        runtime = entry["runtime_ms"]["mean"]
        runtime_text = "n/a" if runtime is None else f"{runtime:.2f}"
        rss_text = "n/a" if rss is None else f"{rss / 1024.0:.2f}"
        lines.append(
            f"| {method} | {entry['converged']}/{entry['runs']} | "
            f"{show('translation_error_m')} | {show('rotation_error_deg')} | "
            f"{show('overlap_mean_nn_m')} | {show('boundary_mean_nn_m')} | "
            f"{show('inlier_ratio')} | {show('supplement_voxel_growth_ratio')} | "
            f"{runtime_text} | {rss_text} |")
    lines.extend(("", "## Hybrid block outcomes", ""))
    hybrid = summary["methods"].get("hybrid")
    if hybrid:
        lines.append(f"- Successful blocks per run: {hybrid['successful_blocks']}")
        lines.append(f"- Failed blocks per run: {hybrid['failed_blocks']}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    arguments = parser.parse_args()
    root = arguments.root.expanduser().resolve()
    rows = collect(root)
    if not rows:
        raise SystemExit(f"no result.json files below {root}")
    fieldnames = list(rows[0])
    with (root / "comparison.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows)
    (root / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_markdown(root / "summary.md", summary)
    print(root / "summary.md")


if __name__ == "__main__":
    main()
