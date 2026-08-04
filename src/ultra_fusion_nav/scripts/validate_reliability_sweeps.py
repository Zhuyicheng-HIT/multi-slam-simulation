#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from uf_reliability.scoring import (
    gnss_score, imu_score, lidar_score, optical_flow_score, vision_score,
)


MODALITIES = ("lidar", "gnss", "imu", "optical_flow", "vision")


def evaluate(severity):
    s = float(severity)
    lidar = lidar_score(
        [10.0 * (1.0 - s) ** 4 + 1.0e-8, 20.0, 30.0, 40.0, 50.0, 60.0],
        [0.1 * (1.0 - s) ** 2, 0.2, 0.7],
        s,
        1000.0 - 920.0 * s,
    )
    gnss = gnss_score(
        1.0 - s,
        0.5 + 30.0 * s,
        0.2 + 8.8 * s,
    )
    imu = imu_score(
        1.0 - s,
        0.1 + 9.9 * s,
        s >= 0.8,
    )
    optical_flow = optical_flow_score(
        0.1 + 0.9 * s,
        0.1,
        220.0 - 215.0 * s,
        3.0,
    )
    vision = vision_score(
        150.0 - 140.0 * s,
        150.0,
        1.0 - 0.9 * s,
        0.2 + 7.8 * s,
        1.0 - 0.9 * s,
    )
    return dict(zip(MODALITIES, (lidar, gnss, imu, optical_flow, vision)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--levels", type=int, default=11)
    args = parser.parse_args()

    severities = np.linspace(0.0, 1.0, max(3, args.levels))
    rows = []
    curves = {modality: [] for modality in MODALITIES}
    for severity in severities:
        for modality, (score, evidence, reasons) in evaluate(severity).items():
            row = {
                "severity": float(severity),
                "modality": modality,
                "degradation_score": float(score),
                "evidence_weight_coverage": float(evidence["evidence_weight_coverage"]),
                "score_complete": bool(evidence["score_complete"]),
                "reasons": reasons,
            }
            rows.append(row)
            curves[modality].append(float(score))

    checks = {}
    for modality, values in curves.items():
        monotonic = all(b + 1.0e-9 >= a for a, b in zip(values[:-1], values[1:]))
        span = values[-1] - values[0]
        complete = all(
            row["score_complete"] and abs(row["evidence_weight_coverage"] - 1.0) < 1.0e-9
            for row in rows if row["modality"] == modality
        )
        checks[modality] = {"monotonic": monotonic, "span": span, "complete": complete}

    passed = all(
        item["monotonic"] and item["span"] >= 0.15 and item["complete"]
        for item in checks.values()
    )
    result = {"levels": len(severities), "checks": checks, "passed": passed}

    output_json = Path(args.output_json)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    output_csv = Path(args.output_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "severity", "modality", "degradation_score",
                "evidence_weight_coverage", "score_complete", "reasons",
            ),
        )
        writer.writeheader()
        for row in rows:
            writer.writerow({**row, "reasons": ";".join(row["reasons"])})
    print(json.dumps(result, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
