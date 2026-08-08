#!/usr/bin/env python3
"""Summarize the long nominal joint-map stress run."""

import argparse
import json
from pathlib import Path
import re


def load(path):
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--headless-status", type=int, required=True)
    parser.add_argument("--route-x-m", type=float, default=20.0)
    parser.add_argument("--route-y-m", type=float, default=12.0)
    parser.add_argument("--route-speed-mps", type=float, default=0.7)
    parser.add_argument(
        "--map-mode", choices=("disabled", "lidar_only", "joint"),
        default="joint",
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.run_dir)
    mapping = load(root / "shared_map" / "metrics.json")
    evidence = load(root / "runtime_evidence.json")
    trajectory = load(root / "trajectory_metrics.json")
    simulation = load(root / "simulation_performance.json")
    summary = evidence.get("summary", {})
    rectangle_log = (
        (root / "small_rectangle.log").read_text(encoding="utf-8", errors="replace")
        if (root / "small_rectangle.log").exists() else ""
    )
    rgbd_total = sum(int(mapping.get(name, 0)) for name in (
        "rgbd_conflicts", "rgbd_consistent", "rgbd_supplements"
    ))
    ghosting_proxy = (
        float(mapping.get("rgbd_conflicts", 0)) / rgbd_total
        if rgbd_total else None
    )
    report = {
        "schema_version": 1,
        "headless_status": args.headless_status,
        "map_mode": args.map_mode,
        "route": {
            "type": "small_rectangle",
            "length_x_m": args.route_x_m,
            "length_y_m": args.route_y_m,
            "speed_mps": args.route_speed_mps,
        },
        "duration_ros_s": evidence.get("duration_ros_s"),
        "duration_wall_s": evidence.get("duration_wall_s"),
        "trajectory": trajectory,
        "mapping": mapping,
        "joint_map": {
            "voxel_count": mapping.get("voxel_count"),
            "lidar_voxels": mapping.get("lidar_voxels"),
            "rgbd_voxels": mapping.get("rgbd_voxels"),
            "rgb_coverage": mapping.get("color_coverage_ratio"),
            "supplementary_volume_ratio": mapping.get("volume_growth_ratio"),
            "conflict_ratio": mapping.get("conflict_ratio"),
            "ghosting_proxy": ghosting_proxy,
            "ghosting_proxy_definition": (
                "rgbd_conflicts / (consistent + supplements + conflicts); "
                "a conflict-derived proxy, not an absolute surface truth metric"
            ),
            "evictions": mapping.get("evictions"),
        },
        "factors": {
            "lidar": summary.get("backend_native_lidar_factors_max"),
            "imu": summary.get("backend_imu_factors_max"),
            "gnss": summary.get("backend_gnss_factors_max"),
            "optical_flow": summary.get("backend_flow_factors_enabled_max"),
            "visual": summary.get("backend_visual_factors_max"),
        },
        "errors": {
            "optimization": summary.get("backend_optimization_errors_max"),
            "optimization_not_committed": summary.get("backend_optimization_rejected_max"),
            "rollback": summary.get("backend_optimization_rollbacks_max"),
            "integrity_counts": summary.get("backend_optimization_integrity_counts_last"),
        },
        "simulation_rtf": simulation.get("simulation", {}).get(
            "real_time_factor_median"
        ),
        "land_observed": bool(re.search(r"LAND|land", rectangle_log)),
        "disarm_observed": bool(re.search(r"disarm", rectangle_log, re.IGNORECASE)),
    }
    Path(args.output).write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "headless_status": report["headless_status"],
        "voxel_count": report["joint_map"]["voxel_count"],
        "conflict_ratio": report["joint_map"]["conflict_ratio"],
        "rollback": report["errors"]["rollback"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
