#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    with open(args.input, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    time_s = [float(row["time_s"]) for row in rows]
    reference = [float(row["reference_x_m"]) for row in rows]
    output = [float(row["output_x_m"]) if row["output_x_m"] != "nan" else math.nan for row in rows]
    blend = [float(row["blend"]) for row in rows]

    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(time_s, reference, label="LIO continuity reference", color="#333333", linewidth=2)
    axes[0].plot(time_s, output, label="admitted GNSS local position", color="#2ca02c", linewidth=2)
    axes[0].set_ylabel("x position (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(time_s, blend, color="#1f77b4", linewidth=2)
    axes[1].fill_between(time_s, 0.0, blend, color="#1f77b4", alpha=0.15)
    axes[1].set_ylabel("GNSS blend")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylim(-0.02, 1.02)
    axes[1].grid(True, alpha=0.3)
    for axis in axes:
        axis.axvspan(5.0, 8.0, color="#d62728", alpha=0.10)
        axis.axvspan(8.0, 10.0, color="#ff7f0e", alpha=0.12)
    figure.tight_layout()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


if __name__ == "__main__":
    main()
