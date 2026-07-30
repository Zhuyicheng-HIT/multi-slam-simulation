#!/usr/bin/env python3
import argparse
import csv
import json
import re
from pathlib import Path


PROCESS_PATTERN = re.compile(
    r"^\s*(?P<pid>\d+)\s+(?P<cpu>[0-9.]+)\s+(?P<mem>[0-9.]+)\s+"
    r"(?P<rss>\d+)\s+(?P<comm>\S+)\s+(?P<args>.*)$"
)


def process_group(arguments):
    mappings = (
        ("gazebo", "gz sim"),
        ("mid360_cpp_bridge", "gz_livox_bridge"),
        ("d435_bridge", "d435i_sim_bridge"),
        ("flow_image_bridge", "gz_rgbd_latest"),
        ("flow_compute", "gazebo_optical"),
        ("mtf_bridge", "mtf01p_mavlink"),
        ("arducopter", "arducopter"),
        ("mavros", "mavros_node"),
        ("fastlio", "fastlio_mapping"),
        ("fastlio", "fast_lio"),
        ("unified_backend", "online_backend"),
        ("external_nav_gate", "external_nav_gate"),
    )
    for name, needle in mappings:
        if needle in arguments:
            return name
    return "other"


def process_means(path):
    per_sample = []
    current = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("sample="):
            if current:
                per_sample.append(current)
            current = {}
            continue
        match = PROCESS_PATTERN.match(line)
        if not match:
            continue
        group = process_group(match.group("args"))
        current[group] = current.get(group, 0.0) + float(match.group("cpu"))
    if current:
        per_sample.append(current)
    groups = {group for sample in per_sample for group in sample}
    return {
        f"cpu_{name}_mean": sum(sample.get(name, 0.0) for sample in per_sample) /
        len(per_sample)
        for name in groups
        if per_sample
    }


def topic_rate(report, name):
    return float(report.get("topics", {}).get(name, {}).get("rate_hz", 0.0))


def load_case(case_dir):
    report_path = case_dir / "performance.json"
    if not report_path.exists():
        return None
    report = json.loads(report_path.read_text(encoding="utf-8"))
    simulation = report.get("simulation", {})
    row = {
        "adapter": (case_dir / "adapter.txt").read_text(encoding="utf-8").strip(),
        "profile": (case_dir / "profile.txt").read_text(encoding="utf-8").strip(),
        "configured_rtf": 1.0,
        "rtf_median": float(simulation.get("real_time_factor_median", 0.0)),
        "rtf_p10": float(simulation.get("real_time_factor_p10", 0.0)),
        "lidar_hz": topic_rate(report, "lidar"),
        "flow_hz": topic_rate(report, "raw_flow"),
        "d435_color_hz": topic_rate(report, "d435_color"),
        "d435_depth_hz": topic_rate(report, "d435_depth"),
        "fastlio_odom_hz": topic_rate(report, "fastlio_odom"),
        "unified_odom_hz": topic_rate(report, "fusion"),
        "external_nav_hz": topic_rate(report, "external_nav"),
    }
    samples_path = case_dir / "process_samples.txt"
    if samples_path.exists():
        row.update(process_means(samples_path))
    renderer_path = case_dir / "gpu_acceleration.log"
    row["renderer"] = ""
    if renderer_path.exists():
        for line in renderer_path.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("OpenGL renderer:"):
                row["renderer"] = line.split(":", 1)[1].strip()
                break
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--json", required=True)
    args = parser.parse_args()

    root = Path(args.input)
    rows = [row for path in sorted(root.iterdir()) if path.is_dir()
            for row in [load_case(path)] if row is not None]
    fields = sorted({key for row in rows for key in row})
    with Path(args.csv).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    Path(args.json).write_text(
        json.dumps({"schema_version": 1, "cases": rows}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(rows, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
