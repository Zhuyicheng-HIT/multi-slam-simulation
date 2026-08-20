#!/usr/bin/env python3
"""Apply strict end-to-end acceptance gates to a unified validation run."""

import argparse
import json
import math
import re
from pathlib import Path


def _read_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _read_text(path):
    return Path(path).read_text(encoding="utf-8", errors="replace")


def _number(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"missing or invalid numeric field: {name}") from error
    if not math.isfinite(result):
        raise ValueError(f"non-finite numeric field: {name}")
    return result


def _integer(value, name):
    result = _number(value, name)
    if result != int(result):
        raise ValueError(f"field is not an integer: {name}")
    return int(result)


def _boolean(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _nested(mapping, *keys):
    value = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _empty_violation(value):
    return value in (None, False, 0, "", [], {})


def _stream_gates(stream, prefix, minimum_rate_hz, minimum_count):
    if not isinstance(stream, dict):
        return {f"{prefix}_stream_present": False}, {}
    observed = {
        "count": _integer(stream.get("count"), f"{prefix}.count"),
        "rate_hz": _number(stream.get("source_stamp_rate_hz"), f"{prefix}.rate_hz"),
        "max_gap_s": _number(stream.get("max_gap_s"), f"{prefix}.max_gap_s"),
        "source_age_p95_s": _number(
            _nested(stream, "source_age_s", "p95"), f"{prefix}.source_age_p95_s"
        ),
        "stale_stamps": _integer(
            stream.get("stale_stamp_over_0_25_s"), f"{prefix}.stale_stamps"
        ),
        "stamp_duplicates": _integer(
            stream.get("stamp_duplicates"), f"{prefix}.stamp_duplicates"
        ),
        "stamp_regressions": _integer(
            stream.get("stamp_regressions"), f"{prefix}.stamp_regressions"
        ),
        "zero_stamps": _integer(stream.get("zero_stamps"), f"{prefix}.zero_stamps"),
    }
    gates = {
        f"{prefix}_stream_present": True,
        f"{prefix}_sample_count": observed["count"] >= int(minimum_count),
        f"{prefix}_rate": observed["rate_hz"] >= float(minimum_rate_hz),
        f"{prefix}_maximum_gap": observed["max_gap_s"] <= 0.25,
        f"{prefix}_source_age": observed["source_age_p95_s"] <= 0.20,
        f"{prefix}_no_stale_stamps": observed["stale_stamps"] == 0,
        f"{prefix}_no_duplicate_stamps": observed["stamp_duplicates"] == 0,
        f"{prefix}_no_stamp_regressions": observed["stamp_regressions"] == 0,
        f"{prefix}_no_zero_stamps": observed["zero_stamps"] == 0,
    }
    return gates, observed


def evaluate_validation(
    accuracy,
    runtime,
    route_log,
    mavros_log,
    sitl_log,
    *,
    require_external_nav=False,
    require_time_lock=False,
    require_visual_time_lock=False,
    require_time_applied=False,
    require_visual_factors=False,
    require_automatic_loop_closure=False,
    mission_profile="rectangle",
    expected_route_feedback="unified_backend",
    expected_waypoints=4,
    minimum_matched_samples=300,
    minimum_motion_samples=50,
    minimum_sim_duration_s=120.0,
):
    causal = accuracy.get("causal_ate", {})
    three_d = causal.get("three_dimensional", {})
    horizontal = causal.get("horizontal", {})
    vertical = causal.get("vertical", {})
    initial_alignment = accuracy.get("initial_alignment", {})
    streams = runtime.get("streams", {})
    backend = runtime.get("backend_latest", {})

    matched_samples = _integer(accuracy.get("matched_samples"), "matched_samples")
    motion_samples = _integer(accuracy.get("motion_samples"), "motion_samples")
    rmse_m = _number(three_d.get("rmse_m"), "causal_ate.three_dimensional.rmse_m")
    p95_m = _number(three_d.get("p95_m"), "causal_ate.three_dimensional.p95_m")
    max_m = _number(three_d.get("max_m"), "causal_ate.three_dimensional.max_m")
    endpoint_m = _number(
        _nested(causal, "endpoint_error_m", "norm"), "causal_ate.endpoint_error_m.norm"
    )
    horizontal_rmse_m = _number(horizontal.get("rmse_m"), "horizontal.rmse_m")
    vertical_rmse_m = _number(vertical.get("rmse_m"), "vertical.rmse_m")
    sim_duration_s = _number(runtime.get("sim_duration_s"), "runtime.sim_duration_s")
    displacement_m = _number(
        _nested(streams, "unified_odom", "max_displacement_from_first_m"),
        "unified_odom.max_displacement_from_first_m",
    )
    landing_disarm_confirmed = (
        "LAND completed and FCU disarm confirmed." in route_log
    )
    termination_reason = runtime.get("termination_reason")
    runtime_completed = (
        termination_reason == "duration_complete"
        or (
            termination_reason == "early_landing"
            and landing_disarm_confirmed
        )
    )

    if mission_profile == "rectangle":
        route_phases = (
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: route_active",
            "Mission phase: landing",
        )
    elif mission_profile == "calibration":
        route_phases = (
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: calibration_excitation",
            "Mission phase: calibration_complete",
            "Mission phase: landing",
        )
    elif mission_profile == "figure_eight":
        route_phases = (
            "Mission phase: preflight",
            "Mission phase: post_takeoff_hold",
            "Mission phase: calibration_excitation",
            "Mission phase: route_active",
            "Mission phase: landing",
        )
    else:
        raise ValueError(f"unknown mission profile: {mission_profile}")
    waypoint_matches = re.findall(r"waypoint\s+(\d+)/(\d+):\s*\(", route_log)
    waypoint_indices = {int(index) for index, _ in waypoint_matches}
    waypoint_totals = {int(total) for _, total in waypoint_matches}
    expected_indices = set(range(1, int(expected_waypoints) + 1))
    waypoint_contract_ok = (
        not waypoint_matches
        if int(expected_waypoints) == 0
        else (
            waypoint_indices == expected_indices
            and waypoint_totals == {int(expected_waypoints)}
        )
    )

    gates = {
        "accuracy_acceptance_passed": bool(_nested(accuracy, "acceptance", "passed")),
        "causal_alignment_does_not_use_future_trajectory": (
            initial_alignment.get("future_trajectory_used") is False
        ),
        "truth_not_used_by_estimator": accuracy.get("truth_used_by_estimator") is False,
        "matched_sample_count": matched_samples >= int(minimum_matched_samples),
        "motion_sample_count": motion_samples >= int(minimum_motion_samples),
        "causal_3d_rmse_below_0_20_m": rmse_m < 0.20,
        "causal_3d_p95_below_0_20_m": p95_m < 0.20,
        "causal_3d_max_below_0_20_m": max_m < 0.20,
        "endpoint_below_0_20_m": endpoint_m < 0.20,
        "horizontal_rmse_below_0_20_m": horizontal_rmse_m < 0.20,
        "vertical_rmse_below_0_20_m": vertical_rmse_m < 0.20,
        "runtime_completed_requested_duration": (
            runtime_completed
            and sim_duration_s >= float(minimum_sim_duration_s)
        ),
        "runtime_graph_contract_clean": _empty_violation(
            runtime.get("graph_contract_violations")
        ),
        "vehicle_executed_nontrivial_motion": displacement_m >= 1.0,
        "mission_phase_sequence_present": all(phase in route_log for phase in route_phases),
        "all_expected_waypoints_present": waypoint_contract_ok,
        "landing_and_disarm_confirmed": landing_disarm_confirmed,
        "sitl_did_not_crash": (
            "Floating point exception" not in sitl_log
            and "Crash: Disarming" not in sitl_log
        ),
        "mavros_odometry_tf_contract_clean": "ODOM: Ex:" not in mavros_log,
    }
    figure_eight_observed = {}
    if mission_profile == "figure_eight":
        plan_match = re.search(
            r"Large figure-eight plan: one closed traversal, "
            r"planned_path_distance=([0-9.]+)m, .*"
            r"ratio_at_or_below_8m=([0-9.]+)%",
            route_log,
        )
        checkpoint_indices = {
            int(index) for index in re.findall(
                r"Mission checkpoint (\d+): large figure-eight single traversal",
                route_log,
            )
        }
        checkpoint_distances_m = [
            float(distance)
            for distance in re.findall(
                r"Mission checkpoint \d+: large figure-eight single traversal, "
                r"distance=([0-9.]+)m",
                route_log,
            )
        ]
        planned_distance_m = (
            _number(plan_match.group(1), "figure_eight.planned_distance_m")
            if plan_match else 0.0
        )
        low_altitude_ratio = (
            _number(plan_match.group(2), "figure_eight.low_altitude_percent") / 100.0
            if plan_match else 0.0
        )
        gates.update({
            "figure_eight_plan_present": plan_match is not None,
            "figure_eight_nontrivial_distance": planned_distance_m >= 25.0,
            "figure_eight_low_altitude_contract": low_altitude_ratio >= 0.50,
            "figure_eight_uses_requested_feedback": (
                "large figure-eight single traversal:" in route_log
                and f"feedback={expected_route_feedback}" in route_log
            ),
            "figure_eight_route_completed": (
                "large figure-eight single traversal: points=" in route_log
                and (
                    "Large figure-eight route completed:" in route_log
                    or "closed-loop return convergence" in route_log
                    or (
                        checkpoint_distances_m
                        and planned_distance_m - max(checkpoint_distances_m) <= 2.0
                    )
                )
                and landing_disarm_confirmed
            ),
        })
        figure_eight_observed = {
            "planned_distance_m": planned_distance_m,
            "low_altitude_ratio": low_altitude_ratio,
            "checkpoint_indices": sorted(checkpoint_indices),
            "checkpoint_distances_m": checkpoint_distances_m,
        }

    unified_gates, unified_observed = _stream_gates(
        streams.get("unified_odom"), "unified_odom", 4.0, minimum_matched_samples
    )
    gates.update(unified_gates)
    external_observed = {}
    if require_external_nav:
        external_gates, external_observed = _stream_gates(
            streams.get("externalnav_out"), "externalnav", 10.0, minimum_matched_samples
        )
        gates.update(external_gates)
        gates["ekf3_confirmed_external_nav_consumption"] = bool(
            re.search(r"EKF3 IMU\d+ is using external nav data", mavros_log)
        )
        reasons = runtime.get("externalnav_diagnostic_reasons", {})
        publishing = _integer(reasons.get("publishing", 0), "externalnav.publish reasons")
        total_reasons = sum(
            _integer(value, "externalnav diagnostic reason")
            for value in reasons.values()
        )
        gates["externalnav_publishing_ratio"] = (
            total_reasons > 0 and publishing / total_reasons >= 0.98
        )

    zero_backend_fields = (
        "optimization_errors",
        "optimization_rollbacks",
        "native_worker_errors",
        "native_worker_queue_overflow",
        "native_worker_queue_discarded",
        "native_consumed_without_state_commit",
    )
    for field in zero_backend_fields:
        gates[f"backend_{field}_zero"] = _integer(backend.get(field), field) == 0
    required_factors = ("lidar_factors", "imu_factors", "gnss_factors", "flow_factors")
    for field in required_factors:
        gates[f"backend_{field}_active"] = _integer(backend.get(field), field) > 0
    if require_visual_factors:
        gates["backend_visual_factors_active"] = (
            _integer(backend.get("visual_factors"), "visual_factors") > 0
        )
    automatic_loop_observed = {}
    if require_automatic_loop_closure:
        automatic_searches = _integer(
            runtime.get("automatic_loop_searches", 0),
            "automatic_loop_searches",
        )
        automatic_successes = _integer(
            runtime.get("automatic_loop_successes", 0),
            "automatic_loop_successes",
        )
        epoch_applied = _integer(
            runtime.get("fusion_epoch_applied", 0), "fusion_epoch_applied"
        )
        backend_resets = _integer(
            backend.get("relocalization_resets", 0), "relocalization_resets"
        )
        continuity = runtime.get("fusion_epoch_continuity", [])
        relocalization_timeline = runtime.get("relocalization_timeline", [])
        route_active_successes = [
            event for event in relocalization_timeline
            if event.get("mission_phase") == "route_active"
            and event.get("accepted") is True
            and str(event.get("reason", "")).startswith(
                "automatic_loop_candidate_accepted"
            )
        ]
        continuity_samples = []
        for event in continuity:
            stream = _nested(event, "streams", "unified_odom")
            if not isinstance(stream, dict) or not stream.get("available"):
                continue
            continuity_samples.append(stream)
        gates.update({
            "automatic_loop_search_executed": automatic_searches >= 1,
            "automatic_loop_candidate_accepted": automatic_successes >= 1,
            "automatic_loop_accepted_during_route": bool(
                route_active_successes
            ),
            "automatic_loop_epoch_applied": epoch_applied >= automatic_successes,
            "automatic_loop_backend_reset_applied": backend_resets >= automatic_successes,
            "automatic_loop_epoch_continuity_observed": (
                len(continuity_samples) >= automatic_successes
            ),
            "automatic_loop_position_step_bounded": (
                bool(continuity_samples)
                and max(_number(sample.get("position_step_m"), "position_step_m")
                        for sample in continuity_samples) <= 0.30
            ),
            "automatic_loop_yaw_step_bounded": (
                bool(continuity_samples)
                and max(_number(sample.get("yaw_step_rad"), "yaw_step_rad")
                        for sample in continuity_samples) <= 0.15
            ),
        })
        automatic_loop_observed = {
            "searches": automatic_searches,
            "successes": automatic_successes,
            "epoch_applied": epoch_applied,
            "backend_resets": backend_resets,
            "continuity": continuity_samples,
            "route_active_successes": route_active_successes,
        }
    gates["backend_covariance_is_not_fallback"] = backend.get("covariance_source") in {
        "window_marginal", "imu_propagated_anchor"
    }

    if require_time_lock:
        gates["online_time_calibration_locked"] = _boolean(
            backend.get("calibration_time_locked")
        )
        offset_s = _number(backend.get("calibration_time_offset_s"), "calibration_time_offset_s")
        gates["online_time_calibration_offset_bounded"] = abs(offset_s) <= 0.12
    if require_visual_time_lock:
        gates["visual_time_calibration_locked"] = _boolean(
            backend.get("visual_time_offset_locked")
        )
        visual_offset_s = _number(
            backend.get("visual_time_offset_s"), "visual_time_offset_s"
        )
        gates["visual_time_calibration_offset_bounded"] = (
            abs(visual_offset_s) <= 0.12
        )
    if require_time_applied:
        gates["online_time_calibration_applied"] = backend.get("calibration_mode") == "time_apply"

    observed = {
        "matched_samples": matched_samples,
        "motion_samples": motion_samples,
        "sim_duration_s": sim_duration_s,
        "causal_3d_rmse_m": rmse_m,
        "causal_3d_p95_m": p95_m,
        "causal_3d_max_m": max_m,
        "endpoint_error_m": endpoint_m,
        "horizontal_rmse_m": horizontal_rmse_m,
        "vertical_rmse_m": vertical_rmse_m,
        "maximum_displacement_m": displacement_m,
        "waypoint_indices": sorted(waypoint_indices),
        "figure_eight": figure_eight_observed,
        "automatic_loop_closure": automatic_loop_observed,
        "unified_odom": unified_observed,
        "externalnav": external_observed,
        "calibration_mode": backend.get("calibration_mode"),
        "calibration_time_locked": _boolean(backend.get("calibration_time_locked")),
        "calibration_time_offset_s": backend.get("calibration_time_offset_s"),
    }
    failed = sorted(name for name, passed in gates.items() if not passed)
    return {
        "schema_version": 2,
        "acceptance_basis": f"strict_unified_{mission_profile}_end_to_end",
        "requirements": {
            "require_external_nav": bool(require_external_nav),
            "require_time_lock": bool(require_time_lock),
            "require_visual_time_lock": bool(require_visual_time_lock),
            "require_time_applied": bool(require_time_applied),
            "require_visual_factors": bool(require_visual_factors),
            "require_automatic_loop_closure": bool(
                require_automatic_loop_closure
            ),
            "mission_profile": str(mission_profile),
            "expected_route_feedback": str(expected_route_feedback),
            "expected_waypoints": int(expected_waypoints),
            "minimum_matched_samples": int(minimum_matched_samples),
            "minimum_motion_samples": int(minimum_motion_samples),
            "minimum_sim_duration_s": float(minimum_sim_duration_s),
        },
        "observed": observed,
        "gates": gates,
        "failed_gates": failed,
        "passed": not failed,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--accuracy", required=True)
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--route-log", required=True)
    parser.add_argument("--mavros-log", required=True)
    parser.add_argument("--sitl-log", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--require-external-nav", action="store_true")
    parser.add_argument("--require-time-lock", action="store_true")
    parser.add_argument("--require-visual-time-lock", action="store_true")
    parser.add_argument("--require-time-applied", action="store_true")
    parser.add_argument("--require-visual-factors", action="store_true")
    parser.add_argument(
        "--require-automatic-loop-closure", action="store_true"
    )
    parser.add_argument("--expected-waypoints", type=int, default=4)
    parser.add_argument(
        "--mission-profile", choices=("rectangle", "calibration", "figure_eight"),
        default="rectangle",
    )
    parser.add_argument(
        "--expected-route-feedback",
        choices=("fcu_local", "unified_backend", "gazebo_truth"),
        default="unified_backend",
    )
    parser.add_argument("--minimum-matched-samples", type=int, default=300)
    parser.add_argument("--minimum-motion-samples", type=int, default=50)
    parser.add_argument("--minimum-sim-duration", type=float, default=120.0)
    args = parser.parse_args()

    try:
        report = evaluate_validation(
            _read_json(args.accuracy),
            _read_json(args.runtime),
            _read_text(args.route_log),
            _read_text(args.mavros_log),
            _read_text(args.sitl_log),
            require_external_nav=args.require_external_nav,
            require_time_lock=args.require_time_lock,
            require_visual_time_lock=args.require_visual_time_lock,
            require_time_applied=args.require_time_applied,
            require_visual_factors=args.require_visual_factors,
            require_automatic_loop_closure=args.require_automatic_loop_closure,
            mission_profile=args.mission_profile,
            expected_route_feedback=args.expected_route_feedback,
            expected_waypoints=args.expected_waypoints,
            minimum_matched_samples=args.minimum_matched_samples,
            minimum_motion_samples=args.minimum_motion_samples,
            minimum_sim_duration_s=args.minimum_sim_duration,
        )
    except (OSError, ValueError, json.JSONDecodeError) as error:
        report = {
            "schema_version": 2,
            "acceptance_basis": f"strict_unified_{args.mission_profile}_end_to_end",
            "passed": False,
            "error": str(error),
        }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("passed") else 4


if __name__ == "__main__":
    raise SystemExit(main())
