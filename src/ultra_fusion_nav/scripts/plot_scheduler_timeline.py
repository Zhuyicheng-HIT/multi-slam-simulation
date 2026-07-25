#!/usr/bin/env python3
import argparse
import csv

import matplotlib.pyplot as plt


STATE_LEVEL = {
    "NORMAL": 0,
    "RECOVERED": 1,
    "DEGRADED": 2,
    "RISK": 3,
    "RELOCALIZING": 4,
    "FAILSAFE": 5,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    series = {}
    state_samples = {}
    with open(args.input, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            modality = row["modality"]
            series.setdefault(modality, [[], [], [], []])
            series[modality][0].append(float(row["elapsed_s"]))
            series[modality][1].append(float(row["reliability_weight"]))
            series[modality][2].append(float(row["covariance_inflation"]))
            series[modality][3].append(int(row["factor_enabled"]))
            state_samples[float(row["elapsed_s"])] = row["health_state"]
    figure, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for modality, (times, weights, inflation, enabled) in series.items():
        axes[0].plot(times, weights, linewidth=1.1, label=modality)
        axes[1].plot(times, inflation, linewidth=1.1, label=modality)
        axes[2].step(times, enabled, where="post", linewidth=1.0, label=modality)
    axes[0].set_ylabel("factor weight")
    axes[0].set_ylim(-0.02, 1.02)
    axes[1].set_ylabel("covariance x")
    axes[1].set_yscale("log")
    axes[2].set_ylabel("enabled")
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_xlabel("experiment time (s)")
    axes[0].legend(ncol=5, fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.25)
    if state_samples:
        labels = sorted(
            {(STATE_LEVEL.get(state, 5), state) for state in state_samples.values()}
        )
        summary = ", ".join(state for _, state in labels)
        figure.suptitle(f"Scheduler states observed: {summary}")
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)


if __name__ == "__main__":
    main()
