#!/usr/bin/env python3
"""Compare recorded pressure-derived relative height with Gazebo truth."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


GAS_CONSTANT = 8314.32
MOLECULAR_WEIGHT = 28.9644
GRAVITY = 9.80665
EARTH_RADIUS_M = 6356766.0
SEA_LEVEL_PRESSURE_PA = 101325.0
SEA_LEVEL_TEMPERATURE_K = 288.15
LAPSE_K_PER_M = 0.0065
AIR_CONSTANT = (
    GRAVITY * MOLECULAR_WEIGHT / (GAS_CONSTANT * -LAPSE_K_PER_M)
)


def stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def pressure_to_local_height_m(
    pressure_pa,
    reference_altitude_m=584.0,
    reference_pressure_pa=SEA_LEVEL_PRESSURE_PA,
):
    ratio = float(pressure_pa) / float(reference_pressure_pa)
    if not math.isfinite(ratio) or ratio <= 0.0:
        return math.nan
    temperature_k = SEA_LEVEL_TEMPERATURE_K / math.exp(
        math.log(ratio) / AIR_CONSTANT
    )
    geo_height_m = (SEA_LEVEL_TEMPERATURE_K - temperature_k) / LAPSE_K_PER_M
    height_m = EARTH_RADIUS_M * geo_height_m / (EARTH_RADIUS_M - geo_height_m)
    return float(height_m - reference_altitude_m)


def percentile(values, quantile):
    if not values:
        return math.nan
    return float(np.percentile(np.asarray(values, dtype=float), quantile))


def stream_rate_hz(stamps):
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return float((len(stamps) - 1) / (stamps[-1] - stamps[0]))


def analyze_pressure_stream(samples, truth_samples, baseline_duration_s):
    stamps = np.asarray([sample[0] for sample in samples], dtype=float)
    raw_pressures = np.asarray([sample[1] for sample in samples], dtype=float)
    pressure_scale_to_pa = 100.0 if np.median(raw_pressures) < 2000.0 else 1.0
    pressures = raw_pressures * pressure_scale_to_pa
    variances = np.asarray([sample[2] for sample in samples], dtype=float)
    heights = np.asarray(
        [pressure_to_local_height_m(value) for value in pressures], dtype=float
    )
    truth_stamps = np.asarray([sample[0] for sample in truth_samples], dtype=float)
    truth_z = np.asarray([sample[1] for sample in truth_samples], dtype=float)
    valid = (
        np.isfinite(heights)
        & (stamps >= truth_stamps[0])
        & (stamps <= truth_stamps[-1])
    )
    stamps = stamps[valid]
    pressures = pressures[valid]
    variances = variances[valid]
    heights = heights[valid]
    if stamps.size < 2:
        raise RuntimeError("insufficient pressure/truth overlap")
    aligned_truth_z = np.interp(stamps, truth_stamps, truth_z)
    baseline_end = stamps[0] + float(baseline_duration_s)
    baseline_mask = stamps <= baseline_end
    if int(np.count_nonzero(baseline_mask)) < 2:
        raise RuntimeError("insufficient barometer baseline samples")
    baro_baseline = float(np.median(heights[baseline_mask]))
    truth_baseline = float(np.median(aligned_truth_z[baseline_mask]))
    relative_height = heights - baro_baseline
    relative_truth_z = aligned_truth_z - truth_baseline
    error = relative_height - relative_truth_z
    absolute_error = np.abs(error)
    if np.std(relative_height) > 1.0e-9 and np.std(relative_truth_z) > 1.0e-9:
        correlation = float(np.corrcoef(relative_height, relative_truth_z)[0, 1])
    else:
        correlation = math.nan
    return {
        "count": int(stamps.size),
        "source_rate_hz": stream_rate_hz(stamps.tolist()),
        "input_unit_inferred": (
            "hPa" if pressure_scale_to_pa == 100.0 else "Pa"
        ),
        "pressure_scale_to_pa": pressure_scale_to_pa,
        "sensor_msgs_pressure_unit_contract_valid": pressure_scale_to_pa == 1.0,
        "raw_pressure": {
            "minimum": float(np.min(raw_pressures)),
            "maximum": float(np.max(raw_pressures)),
            "median": float(np.median(raw_pressures)),
        },
        "pressure_pa": {
            "minimum": float(np.min(pressures)),
            "maximum": float(np.max(pressures)),
            "median": float(np.median(pressures)),
        },
        "reported_variance_pa2": {
            "median": float(np.median(variances)),
            "maximum": float(np.max(variances)),
        },
        "relative_height": {
            "truth_span_m": float(np.max(relative_truth_z) - np.min(relative_truth_z)),
            "barometer_span_m": float(np.max(relative_height) - np.min(relative_height)),
            "bias_m": float(np.mean(error)),
            "rmse_m": float(math.sqrt(np.mean(error * error))),
            "p95_absolute_error_m": percentile(absolute_error.tolist(), 95.0),
            "maximum_absolute_error_m": float(np.max(absolute_error)),
            "correlation": correlation,
        },
    }


def read_bag(path, pressure_topics, truth_topic):
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    required = set(pressure_topics) | {truth_topic}
    missing = sorted(required - set(topic_types))
    if missing:
        raise RuntimeError(f"bag is missing topics: {missing}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(required)))
    message_types = {
        topic: get_message(topic_types[topic]) for topic in required
    }
    pressure_samples = {topic: [] for topic in pressure_topics}
    truth_samples = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, message_types[topic])
        stamp_s = stamp_seconds(message)
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            continue
        if topic == truth_topic:
            truth_samples.append((stamp_s, float(message.pose.pose.position.z)))
        else:
            pressure_samples[topic].append(
                (
                    stamp_s,
                    float(message.fluid_pressure),
                    float(message.variance),
                )
            )
    truth_samples.sort()
    for samples in pressure_samples.values():
        samples.sort()
    return pressure_samples, truth_samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--pressure-topic",
        action="append",
        default=[],
    )
    parser.add_argument(
        "--truth-topic",
        default="/sim/mid360/ground_truth_odom",
    )
    parser.add_argument("--baseline-duration", type=float, default=5.0)
    args = parser.parse_args()
    pressure_topics = args.pressure_topic or [
        "/sim/barometer/pressure",
        "/mavros/imu/static_pressure",
    ]
    pressure_samples, truth_samples = read_bag(
        args.bag, pressure_topics, args.truth_topic
    )
    if len(truth_samples) < 2:
        raise RuntimeError("bag has insufficient ground-truth samples")
    result = {
        "bag": str(args.bag.resolve()),
        "truth_topic": args.truth_topic,
        "reference_altitude_m": 584.0,
        "baseline_duration_s": args.baseline_duration,
        "streams": {},
    }
    for topic in pressure_topics:
        result["streams"][topic] = analyze_pressure_stream(
            pressure_samples[topic], truth_samples, args.baseline_duration
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
