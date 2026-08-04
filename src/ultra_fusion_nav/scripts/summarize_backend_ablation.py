#!/usr/bin/env python3
import argparse
import csv
import json


METRICS = (
    "ate_rmse_m",
    "ate_median_m",
    "ate_max_m",
    "rpe_translation_rmse_m",
    "rpe_rotation_rmse_deg",
)


def load_metrics(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixed", required=True)
    parser.add_argument("--dynamic", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    fixed = load_metrics(args.fixed)
    dynamic = load_metrics(args.dynamic)
    with open(args.output, "w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("metric", "fixed", "dynamic", "dynamic_minus_fixed"))
        for metric in METRICS:
            fixed_value = float(fixed[metric])
            dynamic_value = float(dynamic[metric])
            writer.writerow((
                metric,
                fixed_value,
                dynamic_value,
                dynamic_value - fixed_value,
            ))


if __name__ == "__main__":
    main()
