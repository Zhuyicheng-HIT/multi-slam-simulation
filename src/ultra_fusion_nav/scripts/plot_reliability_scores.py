#!/usr/bin/env python3
import argparse
import json

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    report = json.load(open(args.input, encoding="utf-8"))
    modalities = list(report["healthy"])
    x = range(len(modalities))
    figure, axis = plt.subplots(figsize=(9, 4.5))
    axis.bar([value - 0.2 for value in x], [report["healthy"][key] for key in modalities], 0.4, label="healthy")
    axis.bar([value + 0.2 for value in x], [report["degraded"][key] for key in modalities], 0.4, label="degraded")
    axis.set_xticks(list(x), modalities, rotation=15)
    axis.set_ylim(0.0, 1.0)
    axis.set_ylabel("degradation score")
    axis.grid(axis="y", alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)


if __name__ == "__main__":
    main()
