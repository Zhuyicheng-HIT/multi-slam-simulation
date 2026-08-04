#!/usr/bin/env python3
import argparse
import csv

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    series = {}
    with open(args.input, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            series.setdefault(row["modality"], [[], []])
            series[row["modality"]][0].append(float(row["elapsed_s"]))
            series[row["modality"]][1].append(float(row["degradation_score"]))
    figure, axis = plt.subplots(figsize=(11, 5))
    for modality, (times, scores) in series.items():
        axis.plot(times, scores, linewidth=1.2, label=modality)
    axis.set_xlim(left=0.0)
    axis.set_ylim(0.0, 1.0)
    axis.set_xlabel("experiment time (s)")
    axis.set_ylabel("degradation score")
    axis.grid(alpha=0.3)
    axis.legend(ncol=3)
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)


if __name__ == "__main__":
    main()
