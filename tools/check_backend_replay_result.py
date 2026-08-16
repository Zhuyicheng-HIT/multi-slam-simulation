#!/usr/bin/env python3
"""Apply lossless native-factor gates to one frozen backend replay."""

import argparse
import json
import math
from pathlib import Path


SUMMARY_PREFIX = "Unified backend final summary: "


def parse_summary(text):
    lines = [
        line.split(SUMMARY_PREFIX, 1)[1].strip()
        for line in text.splitlines()
        if SUMMARY_PREFIX in line
    ]
    if not lines:
        raise ValueError("backend final summary is missing")
    values = {}
    for field in lines[-1].split(";"):
        if "=" not in field:
            continue
        key, value = field.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def integer(values, key):
    try:
        return int(float(values[key]))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"backend summary field is missing or invalid: {key}") from error


def evaluate_lossless_replay(
    values,
    expected_native_count,
    expected_scan_request_count=0,
    expected_committed_count=None,
    allow_auxiliary_keyframes=False,
    maximum_uncommitted_native_count=None,
):
    expected = int(expected_native_count)
    expected_scan_requests = int(expected_scan_request_count)
    if expected <= 0:
        raise ValueError("expected native factor count must be positive")
    if expected_scan_requests < 0:
        raise ValueError("expected scan request count must be non-negative")
    expected_committed = (
        expected
        if expected_committed_count is None
        else int(expected_committed_count)
    )
    if expected_committed <= 0:
        raise ValueError("expected committed count must be positive")
    if expected_committed > expected and not allow_auxiliary_keyframes:
        raise ValueError(
            "expected committed count exceeds native count without auxiliary mode"
        )
    if maximum_uncommitted_native_count is None:
        maximum_uncommitted_native_count = max(
            0, expected - min(expected, expected_committed)
        )
    maximum_uncommitted_native_count = int(maximum_uncommitted_native_count)
    if maximum_uncommitted_native_count < 0:
        raise ValueError("maximum uncommitted native count must be non-negative")
    observed = {
        "native_received": integer(values, "native_received"),
        "states_committed": integer(values, "optimized_states_committed"),
        "imu_pair_timeouts": integer(values, "imu_pair_timeouts"),
        "native_queue_overflow": integer(values, "native_queue_overflow"),
        "native_queue_discarded": integer(values, "native_queue_discarded"),
        "native_consumed_without_state_commit": integer(
            values, "native_consumed_without_state_commit"
        ),
        "native_worker_errors": integer(values, "native_worker_errors"),
        "optimization_errors": integer(values, "optimization_errors"),
        "scan_prediction_cache_hits": integer(values, "scan_cache_hits"),
    }
    if allow_auxiliary_keyframes:
        observed.update({
            "auxiliary_keyframe_committed": integer(
                values, "auxiliary_keyframe_committed"
            ),
            "auxiliary_keyframe_rejected": integer(
                values, "auxiliary_keyframe_rejected"
            ),
            "auxiliary_keyframe_errors": integer(
                values, "auxiliary_keyframe_errors"
            ),
        })
    gates = {
        "received_every_native_factor": observed["native_received"] == expected,
        "committed_at_least_reference_trajectory": (
            observed["states_committed"] >= expected_committed
        ),
        "zero_imu_pair_timeouts": observed["imu_pair_timeouts"] == 0,
        "zero_native_queue_overflow": observed["native_queue_overflow"] == 0,
        "zero_native_queue_discarded": observed["native_queue_discarded"] == 0,
        "startup_uncommitted_not_above_reference_gap": (
            observed["native_consumed_without_state_commit"]
            <= maximum_uncommitted_native_count
        ),
        "zero_native_worker_errors": observed["native_worker_errors"] == 0,
        "zero_optimization_errors": observed["optimization_errors"] == 0,
        "prediction_chain_complete_or_not_recorded": (
            expected_scan_requests == 0
            or observed["scan_prediction_cache_hits"]
            >= max(0, expected_scan_requests - 1)
        ),
    }
    if allow_auxiliary_keyframes:
        gates.update({
            "auxiliary_keyframes_were_committed": (
                observed["auxiliary_keyframe_committed"] > 0
            ),
            "zero_auxiliary_keyframe_errors": (
                observed["auxiliary_keyframe_errors"] == 0
            ),
        })
    return {
        "schema_version": 1,
        "acceptance_basis": (
            "native_factor_replay_with_auxiliary_keyframes"
            if allow_auxiliary_keyframes
            else "lossless_native_factor_replay"
        ),
        "expected_native_factor_count": expected,
        "expected_minimum_committed_count": expected_committed,
        "maximum_uncommitted_native_count": maximum_uncommitted_native_count,
        "auxiliary_keyframes_allowed": bool(allow_auxiliary_keyframes),
        "expected_frontend_scan_request_count": expected_scan_requests,
        "prediction_chain_required": expected_scan_requests > 0,
        "observed": observed,
        "gates": gates,
        "passed": all(gates.values()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend-log", required=True)
    parser.add_argument("--expected-native-count", type=int, required=True)
    parser.add_argument(
        "--expected-scan-request-count", type=int, default=0
    )
    parser.add_argument("--expected-committed-count", type=int)
    parser.add_argument("--allow-auxiliary-keyframes", action="store_true")
    parser.add_argument("--maximum-uncommitted-native-count", type=int)
    parser.add_argument("--accuracy-json", default="")
    parser.add_argument(
        "--accuracy-policy", choices=("strict", "rmse"), default="strict"
    )
    parser.add_argument("--metrics-json", default="")
    parser.add_argument(
        "--require-time-calibration-lock", action="store_true"
    )
    parser.add_argument(
        "--require-time-calibration-applied", action="store_true"
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    backend_log = Path(args.backend_log)
    try:
        values = parse_summary(backend_log.read_text(encoding="utf-8"))
        report = evaluate_lossless_replay(
            values,
            args.expected_native_count,
            args.expected_scan_request_count,
            args.expected_committed_count,
            args.allow_auxiliary_keyframes,
            args.maximum_uncommitted_native_count,
        )
        if args.accuracy_json:
            accuracy = json.loads(
                Path(args.accuracy_json).read_text(encoding="utf-8")
            )
            strict_accuracy = bool(
                accuracy.get("acceptance", {}).get("passed", False)
            )
            rmse_m = float(
                accuracy.get("causal_ate", {})
                .get("three_dimensional", {})
                .get("rmse_m", float("inf"))
            )
            threshold_m = float(
                accuracy.get("acceptance", {}).get("threshold_m", 0.2)
            )
            report["accuracy"] = {
                "policy": args.accuracy_policy,
                "strict_passed": strict_accuracy,
                "causal_rmse_m": rmse_m,
                "threshold_m": threshold_m,
            }
            report["gates"]["causal_accuracy_passed"] = (
                strict_accuracy
                if args.accuracy_policy == "strict"
                else math.isfinite(rmse_m) and rmse_m < threshold_m
            )
        if args.metrics_json:
            metrics = json.loads(
                Path(args.metrics_json).read_text(encoding="utf-8")
            )
            latest = metrics.get("last_values", {})
            time_locked = str(
                latest.get("calibration_time_locked", "false")
            ).lower() in {"true", "1", "yes"}
            time_applied = (
                str(latest.get("calibration_mode", "")) == "time_apply"
                and time_locked
            )
            if args.require_time_calibration_lock:
                report["gates"]["online_time_calibration_locked"] = time_locked
            if args.require_time_calibration_applied:
                report["gates"]["online_time_calibration_applied"] = time_applied
        report["passed"] = all(report["gates"].values())
    except (OSError, ValueError) as error:
        report = {
            "schema_version": 1,
            "acceptance_basis": "lossless_native_factor_replay",
            "passed": False,
            "error": str(error),
        }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("passed") else 4


if __name__ == "__main__":
    raise SystemExit(main())
