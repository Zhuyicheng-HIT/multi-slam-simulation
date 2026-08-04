#!/usr/bin/env python3
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    curves = defaultdict(list)
    with open(args.input, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            curves[row["modality"]].append(
                (float(row["severity"]), float(row["degradation_score"]))
            )

    colors = {
        "lidar": "#d62728",
        "gnss": "#2ca02c",
        "imu": "#ff7f0e",
        "optical_flow": "#1f77b4",
        "vision": "#9467bd",
    }
    labels = {
        "lidar": "LiDAR",
        "gnss": "BDS/GNSS",
        "imu": "IMU",
        "optical_flow": "Optical flow",
        "vision": "RGB-D vision",
    }
    fig, axis = plt.subplots(figsize=(10, 5.5))
    for modality in labels:
        values = sorted(curves[modality])
        axis.plot(
            [value[0] for value in values],
            [value[1] for value in values],
            marker="o",
            linewidth=2.0,
            markersize=4,
            label=labels[modality],
            color=colors[modality],
        )
    axis.set_xlabel("Normalized injected severity")
    axis.set_ylabel("Degradation score D")
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.02)
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=3)
    fig.tight_layout()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
