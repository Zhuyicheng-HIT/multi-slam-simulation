#!/usr/bin/env python3
"""Summarize the deterministic A1/A2 LiDAR temporal-contract grid."""

import argparse
import json
from pathlib import Path


OFFSETS = {
    "0": 0.0,
    "p0005": 0.0005, "n0005": -0.0005,
    "p001": 0.001, "n001": -0.001,
    "p002": 0.002, "n002": -0.002,
    "p005": 0.005, "n005": -0.005,
    "p010": 0.010, "n010": -0.010,
    "p020": 0.020, "n020": -0.020,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.matrix_dir)
    rows = []
    for scope in ("a1_coherent", "a2_mismatch"):
        for suffix, offset in OFFSETS.items():
            profile = f"{scope}_{suffix}"
            path = root / profile / "robustness_report.json"
            if not path.exists():
                rows.append({"profile": profile, "offset_s": offset, "missing": True})
                continue
            report = json.loads(path.read_text(encoding="utf-8"))
            temporal = report.get("native_lidar_temporal_contract", {})
            rows.append({
                "profile": profile,
                "scope": scope,
                "offset_s": offset,
                "completeness": report.get("trajectory_completeness"),
                "continuous": report.get("trajectory_continuous"),
                "first_gap": report.get("trajectory_first_gap_over_1s"),
                "max_gap_s": report.get("trajectory_maximum_odom_gap_s"),
                "delta_ate_m": report.get("trajectory", {}).get("ate_rmse_m"),
                "delta_rpe_translation_m": report.get("trajectory", {}).get(
                    "rpe_translation_rmse_m"
                ),
                "delta_rpe_rotation_deg": report.get("trajectory", {}).get(
                    "rpe_rotation_rmse_deg"
                ),
                "native_factors": report.get("factor_counts", {}).get("lidar"),
                "native_invalid": report.get("factor_rejections", {}).get(
                    "lidar_invalid"
                ),
                "scan_prediction": temporal,
                "errors": report.get("errors"),
                "pass_invariants": report.get("pass_invariants"),
            })
    output = {
        "schema_version": 1,
        "deterministic_replay": True,
        "raw_packet_or_point_timestamps_present": False,
        "rows": rows,
    }
    Path(args.output).write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "rows": len(rows),
        "missing": sum(bool(row.get("missing")) for row in rows),
        "continuous": sum(bool(row.get("continuous")) for row in rows),
    }, sort_keys=True))


if __name__ == "__main__":
    main()
