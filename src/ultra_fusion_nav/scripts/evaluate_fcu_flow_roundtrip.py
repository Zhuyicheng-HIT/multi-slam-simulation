#!/usr/bin/env python3
import argparse
import bisect
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from mavros_msgs.msg import OpticalFlowRad
from multi_slam_uav_sim.mtf01p_protocol import (
    focal_length_px,
    integrated_radians_to_pixels,
    pixels_to_integrated_radians,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def stamp_seconds(header):
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9


def finite_gyro(msg):
    return all(math.isfinite(value) for value in (
        msg.integrated_xgyro, msg.integrated_ygyro, msg.integrated_zgyro
    ))


def correlation(first, second):
    if len(first) < 3 or np.std(first) < 1.0e-9 or np.std(second) < 1.0e-9:
        return float("nan")
    return float(np.corrcoef(first, second)[0, 1])


class RoundtripRecorder(Node):
    def __init__(self, direct_topic, fcu_topic):
        super().__init__("fcu_flow_roundtrip_evaluator")
        self.direct = []
        self.fcu = []
        self.direct_regressions = 0
        self.fcu_regressions = 0
        self.last_direct_stamp = None
        self.last_fcu_stamp = None
        self.create_subscription(
            OpticalFlowRad, direct_topic, self._direct, qos_profile_sensor_data
        )
        self.create_subscription(
            OpticalFlowRad, fcu_topic, self._fcu, qos_profile_sensor_data
        )

    @staticmethod
    def _sample(msg):
        integration_s = float(msg.integration_time_us) * 1.0e-6
        return {
            "stamp": stamp_seconds(msg.header),
            "arrival": time.monotonic(),
            "integration_s": integration_s,
            "integrated_x": float(msg.integrated_x),
            "integrated_y": float(msg.integrated_y),
            "rate_x": float(msg.integrated_x) / max(1.0e-6, integration_s),
            "rate_y": float(msg.integrated_y) / max(1.0e-6, integration_s),
            "quality": int(msg.quality),
            "distance": float(msg.distance),
            "gyro_valid": finite_gyro(msg),
        }

    def _direct(self, msg):
        sample = self._sample(msg)
        if self.last_direct_stamp is not None and sample["stamp"] <= self.last_direct_stamp:
            self.direct_regressions += 1
        self.last_direct_stamp = sample["stamp"]
        self.direct.append(sample)

    def _fcu(self, msg):
        sample = self._sample(msg)
        if self.last_fcu_stamp is not None and sample["stamp"] <= self.last_fcu_stamp:
            self.fcu_regressions += 1
        self.last_fcu_stamp = sample["stamp"]
        self.fcu.append(sample)


def match_samples(direct, fcu, lag_s, averaging_window_s):
    eligible_direct = [
        sample for sample in direct
        if sample["quality"] > 0 and sample["distance"] > 0.1
    ]
    times = [sample["arrival"] for sample in eligible_direct]
    pairs = []
    for returned in fcu:
        if returned["quality"] <= 0 or returned["distance"] <= 0.1 or not times:
            continue
        center = returned["arrival"] - lag_s
        start = bisect.bisect_left(times, center - 0.5 * averaging_window_s)
        end = bisect.bisect_right(times, center + 0.5 * averaging_window_s)
        candidates = eligible_direct[start:end]
        if not candidates:
            continue
        reference = min(candidates, key=lambda sample: abs(sample["arrival"] - center))
        focal = focal_length_px()
        pixel_x, pixel_y = integrated_radians_to_pixels(
            reference["integrated_x"], reference["integrated_y"], focal, focal
        )
        quantized_x, quantized_y = pixels_to_integrated_radians(
            pixel_x, pixel_y, focal, focal
        )
        integration_s = max(1.0e-6, reference["integration_s"])
        encoded_reference = {
            "rate_x": quantized_x / integration_s,
            "rate_y": quantized_y / integration_s,
            "quality": reference["quality"],
            "distance": reference["distance"],
        }
        pairs.append((encoded_reference, returned))
    return pairs


def pair_metrics(pairs):
    direct_rates = np.asarray([
        [direct["rate_x"], direct["rate_y"]] for direct, _ in pairs
    ], dtype=float)
    fcu_rates = np.asarray([
        [returned["rate_x"], returned["rate_y"]] for _, returned in pairs
    ], dtype=float)
    scale = float("nan")
    rmse = float("nan")
    normalized_rmse = float("nan")
    corr = float("nan")
    if len(pairs):
        denominator = float(np.sum(direct_rates * direct_rates))
        if denominator > 1.0e-12:
            scale = float(np.sum(direct_rates * fcu_rates) / denominator)
            residual = fcu_rates - scale * direct_rates
            rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
            reference = max(1.0e-9, float(np.median(np.linalg.norm(direct_rates, axis=1))))
            normalized_rmse = rmse / reference
            corr = correlation(direct_rates.ravel(), fcu_rates.ravel())
    distance_error = [
        abs(direct["distance"] - returned["distance"]) for direct, returned in pairs
    ]
    quality_error = [
        abs(direct["quality"] - returned["quality"]) for direct, returned in pairs
    ]
    return {
        "matched_airborne_samples": len(pairs),
        "flow_rate_scale": scale,
        "flow_rate_correlation": corr,
        "flow_rate_rmse_rad_s": rmse,
        "flow_rate_normalized_rmse": normalized_rmse,
        "distance_median_abs_error_m": (
            float(np.median(distance_error)) if distance_error else None
        ),
        "quality_median_abs_error": (
            float(np.median(quality_error)) if quality_error else None
        ),
    }


def stream_metrics(samples):
    arrivals = np.asarray([sample["arrival"] for sample in samples], dtype=float)
    stamps = np.asarray([sample["stamp"] for sample in samples], dtype=float)
    integrations = np.asarray(
        [sample["integration_s"] for sample in samples], dtype=float
    )
    arrival_periods = np.diff(arrivals)
    source_periods = np.diff(stamps)
    elapsed_s = float(arrivals[-1] - arrivals[0]) if len(arrivals) >= 2 else 0.0
    return {
        "samples": len(samples),
        "measured_rate_hz": (
            float((len(samples) - 1) / elapsed_s) if elapsed_s > 0.0 else 0.0
        ),
        "arrival_period_median_s": (
            float(np.median(arrival_periods)) if len(arrival_periods) else None
        ),
        "source_period_median_s": (
            float(np.median(source_periods)) if len(source_periods) else None
        ),
        "integration_period_median_s": (
            float(np.median(integrations)) if len(integrations) else None
        ),
        "valid_integration_ratio": (
            float(np.mean(np.isfinite(integrations) & (integrations > 0.0)))
            if len(integrations)
            else 0.0
        ),
        "nonzero_source_stamp_ratio": (
            float(np.mean(np.isfinite(stamps) & (stamps > 0.0)))
            if len(stamps)
            else 0.0
        ),
    }


def summarize(node, maximum_lag_s, lag_step_s, averaging_window_s):
    candidates = []
    lag_s = 0.0
    while lag_s <= maximum_lag_s + 1.0e-9:
        metrics = pair_metrics(match_samples(node.direct, node.fcu, lag_s, averaging_window_s))
        metrics["lag_s"] = lag_s
        candidates.append(metrics)
        lag_s += lag_step_s
    finite_candidates = [
        candidate for candidate in candidates
        if math.isfinite(candidate["flow_rate_correlation"])
    ]
    best = max(
        finite_candidates,
        key=lambda candidate: candidate["flow_rate_correlation"],
        default=pair_metrics([]),
    )
    gyro_coverage = (
        sum(sample["gyro_valid"] for sample in node.fcu) / len(node.fcu)
        if node.fcu else 0.0
    )
    direct_stream = stream_metrics(node.direct)
    fcu_stream = stream_metrics(node.fcu)
    rate_ratio = (
        fcu_stream["measured_rate_hz"] / direct_stream["measured_rate_hz"]
        if direct_stream["measured_rate_hz"] > 0.0
        else 0.0
    )
    integration_ratio = 0.0
    if (
        direct_stream["integration_period_median_s"] is not None
        and direct_stream["integration_period_median_s"] > 0.0
        and fcu_stream["integration_period_median_s"] is not None
    ):
        integration_ratio = (
            fcu_stream["integration_period_median_s"]
            / direct_stream["integration_period_median_s"]
        )
    passed = bool(
        best["matched_airborne_samples"] >= 80
        and math.isfinite(best["flow_rate_correlation"])
        and best["flow_rate_correlation"] >= 0.70
        and math.isfinite(best["flow_rate_scale"])
        and 0.75 <= best["flow_rate_scale"] <= 1.25
        and math.isfinite(best["flow_rate_normalized_rmse"])
        and best["flow_rate_normalized_rmse"] <= 1.0
        and best["distance_median_abs_error_m"] is not None
        and best["distance_median_abs_error_m"] <= 0.15
        and best["quality_median_abs_error"] is not None
        and best["quality_median_abs_error"] <= 10.0
        and gyro_coverage >= 0.95
        and 0.80 <= rate_ratio <= 1.20
        and 0.80 <= integration_ratio <= 1.20
        and direct_stream["valid_integration_ratio"] >= 0.99
        and fcu_stream["valid_integration_ratio"] >= 0.99
        and direct_stream["nonzero_source_stamp_ratio"] >= 0.99
        and fcu_stream["nonzero_source_stamp_ratio"] >= 0.99
        and node.fcu_regressions == 0
    )
    return {
        "direct_samples": len(node.direct),
        "fcu_samples": len(node.fcu),
        "best": best,
        "averaging_window_s": averaging_window_s,
        "fcu_gyro_coverage": gyro_coverage,
        "direct_stamp_regressions": node.direct_regressions,
        "fcu_stamp_regressions": node.fcu_regressions,
        "direct_stamp_regressions_are_reference_warning": True,
        "direct_stream": direct_stream,
        "fcu_routed_stream": fcu_stream,
        "routed_to_direct_rate_ratio": rate_ratio,
        "routed_to_direct_integration_ratio": integration_ratio,
        "passed": passed,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=100.0)
    parser.add_argument("--direct-topic", default="/sim/optical_flow/rad")
    parser.add_argument("--fcu-topic", default="/fcu/optical_flow/rad")
    parser.add_argument("--maximum-lag", type=float, default=0.6)
    parser.add_argument("--lag-step", type=float, default=0.01)
    parser.add_argument("--averaging-window", type=float, default=0.15)
    parser.add_argument("--output", required=True)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = RoundtripRecorder(args.direct_topic, args.fcu_topic)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    result = summarize(node, args.maximum_lag, args.lag_step, args.averaging_window)
    result["duration_s"] = args.duration
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if result["passed"] or not args.require_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
