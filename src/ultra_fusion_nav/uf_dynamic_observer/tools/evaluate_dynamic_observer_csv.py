#!/usr/bin/env python3

import argparse
import csv
import json
import math
import statistics
import sys


def ratio(numerator, denominator, fallback=0.0):
    return numerator / denominator if denominator else fallback


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = round(fraction * (len(ordered) - 1))
    return ordered[index]


def evaluate(path):
    counts = {
        "tp": 0,
        "fp": 0,
        "fn": 0,
        "tn": 0,
        "dynamic_as_static": 0,
        "static_confirmed": 0,
    }
    latencies = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            truth = row["truth_label"].strip().lower()
            prediction = row["predicted_label"].strip().lower()
            if truth not in {"static", "dynamic"}:
                continue
            if prediction not in {"static", "dynamic", "unknown"}:
                raise ValueError(f"invalid predicted_label: {prediction}")
            truth_dynamic = truth == "dynamic"
            predicted_dynamic = prediction == "dynamic"
            if truth_dynamic and predicted_dynamic:
                counts["tp"] += 1
            elif truth_dynamic:
                counts["fn"] += 1
                if prediction == "static":
                    counts["dynamic_as_static"] += 1
            elif predicted_dynamic:
                counts["fp"] += 1
            else:
                counts["tn"] += 1
                if prediction == "static":
                    counts["static_confirmed"] += 1
            latency = row.get("latency_ms", "").strip()
            if latency:
                value = float(latency)
                if math.isfinite(value):
                    latencies.append(value)

    dynamic_total = counts["tp"] + counts["fn"]
    static_total = counts["tn"] + counts["fp"]
    dynamic_applicable = dynamic_total > 0
    precision = (
        ratio(counts["tp"], counts["tp"] + counts["fp"])
        if dynamic_applicable
        else None
    )
    recall = ratio(counts["tp"], dynamic_total) if dynamic_applicable else None
    dynamic_f1 = (
        ratio(2.0 * precision * recall, precision + recall)
        if dynamic_applicable
        else None
    )
    result = {
        "dynamic_metrics_applicable": dynamic_applicable,
        "dynamic_precision": precision,
        "dynamic_recall": recall,
        "dynamic_f1": dynamic_f1,
        "static_preservation_rate": (
            ratio(counts["tn"], static_total) if static_total else None
        ),
        "false_dynamic_ratio": (
            ratio(counts["fp"], static_total) if static_total else None
        ),
        "static_map_contamination": (
            ratio(counts["dynamic_as_static"], dynamic_total)
            if dynamic_applicable
            else None
        ),
        "map_completeness": (
            ratio(counts["static_confirmed"], static_total)
            if static_total
            else None
        ),
        "latency_p50_ms": percentile(latencies, 0.50),
        "latency_p95_ms": percentile(latencies, 0.95),
        "latency_mean_ms": statistics.fmean(latencies) if latencies else None,
        "counts": counts,
        "contract": {
            "truth_is_evaluator_only": True,
            "dynamic_aggregation": {
                "micro": "sum TP/FP/FN over dynamic-bearing inputs",
                "macro": "unweighted mean over scenarios with dynamic positives",
                "pure_static_dynamic_metrics": "N/A and excluded from macro",
            },
            "required_columns": ["truth_label", "predicted_label"],
            "optional_columns": ["latency_ms", "scan_id", "point_id"],
        },
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--output")
    args = parser.parse_args()
    output = json.dumps(evaluate(args.csv_path), indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(output + "\n")
    else:
        print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
