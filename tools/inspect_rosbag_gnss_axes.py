#!/usr/bin/env python3
"""Summarize GNSS axis evidence without feeding truth to the estimator.

Gazebo truth is used only after extraction to evaluate GNSS relative motion.
The first associated GNSS/truth pair fixes the local origin, so the report does
not mistake an arbitrary geodetic altitude datum for navigation drift.
"""

import argparse
from bisect import bisect_left
from collections import Counter
import json
import math
from pathlib import Path
import statistics

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import NavSatFix

from uf_backend_fusion.rosbag_factors import LocalEnuProjector, message_stamp


GNSS_TOPIC = "/sensors/gnss/fix"
TRUTH_TOPIC = "/sim/mid360/ground_truth_odom"


def percentile(values, fraction):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    position = max(0.0, min(1.0, float(fraction))) * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    ratio = position - lower
    return values[lower] * (1.0 - ratio) + values[upper] * ratio


def error_summary(values):
    values = [float(value) for value in values]
    if not values:
        return {"samples": 0, "rmse_m": None, "p95_abs_m": None, "max_abs_m": None}
    absolute = [abs(value) for value in values]
    return {
        "samples": len(values),
        "rmse_m": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95_abs_m": percentile(absolute, 0.95),
        "max_abs_m": max(absolute),
    }


def interpolate_truth(records, stamps, stamp, maximum_gap_s):
    index = bisect_left(stamps, stamp)
    if index == 0 or index >= len(records):
        return None
    before = records[index - 1]
    after = records[index]
    span = after[0] - before[0]
    if span <= 0.0 or max(stamp - before[0], after[0] - stamp) > maximum_gap_s:
        return None
    ratio = (stamp - before[0]) / span
    return tuple(
        before[1][axis] * (1.0 - ratio) + after[1][axis] * ratio
        for axis in range(3)
    )


def read_bag(path):
    path = Path(path)
    if path.is_dir():
        sqlite_files = sorted(path.glob("*.db3"))
        if sqlite_files:
            path = sqlite_files[0]
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in (GNSS_TOPIC, TRUTH_TOPIC) if topic not in topics]
    if missing:
        raise RuntimeError(f"bag is missing required topics: {', '.join(missing)}")
    fixes = []
    truth = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == GNSS_TOPIC:
            message = deserialize_message(data, NavSatFix)
            stamp = message_stamp(message)
            if math.isfinite(stamp) and stamp > 0.0:
                fixes.append((stamp, message))
        elif topic == TRUTH_TOPIC:
            message = deserialize_message(data, Odometry)
            stamp = message_stamp(message)
            position = message.pose.pose.position
            if math.isfinite(stamp) and stamp > 0.0:
                truth.append((stamp, (position.x, position.y, position.z)))
    fixes.sort(key=lambda item: item[0])
    truth.sort(key=lambda item: item[0])
    return fixes, truth


def analyze(path, maximum_truth_gap_s=0.20):
    fixes, truth = read_bag(path)
    if not fixes or len(truth) < 2:
        raise RuntimeError("bag does not contain enough GNSS/truth samples")
    truth_stamps = [record[0] for record in truth]
    projector = LocalEnuProjector(
        fixes[0][1].latitude,
        fixes[0][1].longitude,
        fixes[0][1].altitude,
    )
    paired = []
    covariance = [[], [], []]
    covariance_types = Counter()
    status = Counter()
    for stamp, fix in fixes:
        status[int(fix.status.status)] += 1
        covariance_types[int(fix.position_covariance_type)] += 1
        diagonal = (
            float(fix.position_covariance[0]),
            float(fix.position_covariance[4]),
            float(fix.position_covariance[8]),
        )
        for axis, value in enumerate(diagonal):
            if math.isfinite(value) and value > 0.0:
                covariance[axis].append(value)
        truth_position = interpolate_truth(
            truth, truth_stamps, stamp, maximum_truth_gap_s
        )
        if truth_position is None:
            continue
        measured = projector.project(fix.latitude, fix.longitude, fix.altitude)
        paired.append((stamp, measured, truth_position))
    if not paired:
        raise RuntimeError("no GNSS fixes could be associated with truth")
    measured_origin = paired[0][1]
    truth_origin = paired[0][2]
    axis_errors = [[], [], []]
    for _, measured, truth_position in paired:
        for axis in range(3):
            measured_delta = measured[axis] - measured_origin[axis]
            truth_delta = truth_position[axis] - truth_origin[axis]
            axis_errors[axis].append(measured_delta - truth_delta)
    span = fixes[-1][0] - fixes[0][0]
    covariance_report = {}
    for axis, name in enumerate(("x", "y", "z")):
        values = covariance[axis]
        covariance_report[name] = {
            "samples": len(values),
            "median_m2": statistics.median(values) if values else None,
            "p95_m2": percentile(values, 0.95),
        }
    return {
        "bag": str(Path(path).resolve()),
        "gnss_topic": GNSS_TOPIC,
        "truth_topic_evaluation_only": TRUTH_TOPIC,
        "fix_samples": len(fixes),
        "paired_samples": len(paired),
        "span_s": span,
        "rate_hz": (len(fixes) - 1) / span if span > 0.0 else None,
        "status_counts": {str(key): value for key, value in sorted(status.items())},
        "covariance_type_counts": {
            str(key): value for key, value in sorted(covariance_types.items())
        },
        "reported_covariance": covariance_report,
        "relative_enu_error": {
            name: error_summary(axis_errors[axis])
            for axis, name in enumerate(("x", "y", "z"))
        },
        "truth_used_by_estimator": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output")
    parser.add_argument("--maximum-truth-gap-s", type=float, default=0.20)
    args = parser.parse_args()
    report = analyze(args.bag, args.maximum_truth_gap_s)
    output = json.dumps(report, indent=2, sort_keys=True)
    print(output)
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(output + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
