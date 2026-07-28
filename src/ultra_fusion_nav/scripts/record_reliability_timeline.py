#!/usr/bin/env python3
"""Record scheduler and injected-fault events for a bounded simulation run."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from uf_interfaces.msg import (
    FaultState,
    LioDiagnostics,
    ReliabilityScore,
    SchedulerState,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class ReliabilityTimelineRecorder(Node):
    def __init__(self):
        super().__init__("reliability_timeline_recorder")
        self.events = []
        self.started_monotonic = time.monotonic()
        self.create_subscription(
            SchedulerState,
            "/reliability/scheduler_state",
            self._scheduler,
            20,
        )
        self.create_subscription(
            FaultState,
            "/fault/state",
            self._fault,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray,
            "/fusion/unified/diagnostics",
            self._backend,
            20,
        )
        self.create_subscription(
            LioDiagnostics,
            "/lio/diagnostics",
            self._lio,
            20,
        )
        self.create_subscription(
            ReliabilityScore,
            "/reliability/optical_flow_score",
            self._flow_score,
            qos_profile_sensor_data,
        )

    def _relative_event(self, kind, msg):
        return {
            "kind": kind,
            "received_s": time.monotonic() - self.started_monotonic,
            "stamp_s": stamp_seconds(msg.header.stamp),
        }

    def _scheduler(self, msg):
        event = self._relative_event("scheduler", msg)
        event.update({
            "health_state": str(msg.health_state),
            "relocalization_requested": bool(msg.relocalization_requested),
            "weights": {
                name: float(weight)
                for name, weight in zip(msg.modality_names, msg.reliability_weights)
            },
            "degradation_scores": {
                name: float(score)
                for name, score in zip(msg.modality_names, msg.degradation_scores)
            },
            "factor_enabled": {
                name: bool(enabled)
                for name, enabled in zip(msg.modality_names, msg.factor_enabled)
            },
            "covariance_inflation": {
                name: float(value)
                for name, value in zip(msg.modality_names, msg.covariance_inflation)
            },
            "reasons": {
                name: str(reason)
                for name, reason in zip(msg.modality_names, msg.reasons)
            },
        })
        self.events.append(event)

    def _fault(self, msg):
        event = self._relative_event("fault", msg)
        event.update({
            "modality": str(msg.modality),
            "fault_type": str(msg.fault_type),
            "active": bool(msg.active),
            "magnitude": float(msg.magnitude),
            "affected_messages": int(msg.affected_messages),
            "timestamp_repairs": int(msg.timestamp_repairs),
        })
        self.events.append(event)

    def _flow_score(self, msg):
        event = self._relative_event("flow_score", msg)
        evidence = {
            name: float(value)
            for name, value in zip(msg.evidence_names, msg.evidence_values)
        }
        event.update({
            "degradation_score": float(msg.degradation_score),
            "reliability_weight": float(msg.reliability_weight),
            "valid": bool(msg.valid),
            "reasons": list(msg.reasons),
            "fcu_yaw_rate_abs_radps": float(
                evidence.get("fcu_yaw_rate_abs_radps", -1.0)
            ),
            "rotation_gate_weight": float(
                evidence.get("rotation_gate_weight", -1.0)
            ),
            "rotation_gate_phase_code": float(
                evidence.get("rotation_gate_phase_code", -1.0)
            ),
            "rotation_gate_translation_ready": float(
                evidence.get("rotation_gate_translation_ready", -1.0)
            ),
        })
        self.events.append(event)

    def _backend(self, msg):
        for status in msg.status:
            if status.name != "unified_backend_fusion":
                continue
            values = {item.key: item.value for item in status.values}
            event = self._relative_event("backend", msg)
            level = (
                int.from_bytes(status.level, byteorder="little")
                if isinstance(status.level, (bytes, bytearray))
                else int(status.level)
            )
            event.update({
                "level": level,
                "message": str(status.message),
                "lidar_factors": int(values.get("lidar_factors", 0)),
                "lidar_disabled": int(values.get("lidar_disabled", 0)),
                "gnss_factors": int(values.get("gnss_factors", 0)),
                "gnss_jump_rejected": int(values.get("gnss_jump_rejected", 0)),
                "reliability_mode": str(values.get("reliability_mode", "unknown")),
                "flow_factor_attempts": int(
                    values.get("flow_factor_attempts", 0)
                ),
                "flow_factors": int(values.get("flow_factors", 0)),
                "flow_disabled_quality": int(values.get("flow_disabled_quality", 0)),
                "flow_disabled_rotation": int(
                    values.get("flow_disabled_rotation", 0)
                ),
                "flow_rotation_phase": str(
                    values.get("flow_rotation_phase", "unavailable")
                ),
                "flow_rotation_weight": float(
                    values.get("flow_rotation_weight", -1.0)
                ),
                "flow_yaw_rate_abs_radps": float(
                    values.get("flow_yaw_rate_abs_radps", -1.0)
                ),
                "last_flow_reason": str(
                    values.get("last_flow_reason", "unavailable")
                ),
                "last_flow_factor_type": str(
                    values.get("last_flow_factor_type", "unavailable")
                ),
                "published": int(values.get("published", 0)),
                "optimization_errors": int(values.get("optimization_errors", 0)),
                "backend_solve_ms": float(values.get("backend_solve_ms", 0.0)),
                "backend_solve_mean_ms": float(
                    values.get("backend_solve_mean_ms", 0.0)
                ),
                "backend_solve_max_ms": float(
                    values.get("backend_solve_max_ms", 0.0)
                ),
                "callback_ms": float(values.get("callback_ms", 0.0)),
                "pending_native_worker_frames": int(
                    values.get("pending_native_worker_frames", 0)
                ),
                "imu_factors": int(values.get("imu_factors", 0)),
                "native_worker_queue_overflow": int(
                    values.get("native_worker_queue_overflow", 0)
                ),
                "imu_residual_updates": int(values.get("imu_residual_updates", 0)),
                "imu_residual_errors": int(values.get("imu_residual_errors", 0)),
                "imu_startup_reason": str(
                    values.get("imu_startup_reason", "not_attempted")
                ),
                "imu_startup_sample_count": int(
                    values.get("imu_startup_sample_count", 0)
                ),
                "imu_startup_span_s": float(
                    values.get("imu_startup_span_s", 0.0)
                ),
                "imu_startup_accel_bias": [
                    float(value) for value in values.get(
                        "imu_startup_accel_bias", "0,0,0"
                    ).split(",")
                ],
                "imu_startup_gyro_bias": [
                    float(value) for value in values.get(
                        "imu_startup_gyro_bias", "0,0,0"
                    ).split(",")
                ],
                "imu_startup_bias_accepted": int(
                    values.get("imu_startup_bias_accepted", 0)
                ),
                "imu_startup_bias_rejected": int(
                    values.get("imu_startup_bias_rejected", 0)
                ),
                "lidar_anchor_overrides": int(values.get("lidar_anchor_overrides", 0)),
                "native_lidar_received": int(values.get("native_lidar_received", 0)),
                "native_lidar_invalid": int(values.get("native_lidar_invalid", 0)),
                "native_lidar_factors": int(values.get("native_lidar_factors", 0)),
                "native_lidar_hard_disabled": int(
                    values.get("native_lidar_hard_disabled", 0)
                ),
                "native_lidar_pose_fallbacks": int(
                    values.get("native_lidar_pose_fallbacks", 0)
                ),
                "native_lidar_pair_timeouts": int(
                    values.get("native_lidar_pair_timeouts", 0)
                ),
                "lidar_factor_source": str(
                    values.get("lidar_factor_source", "unavailable")
                ),
                "native_lidar_matches": int(values.get("native_lidar_matches", 0)),
                "native_lidar_stamp_error_ms": float(
                    values.get("native_lidar_stamp_error_ms", -1.0)
                ),
                "lidar_prediction_position_innovation_m": float(
                    values.get("lidar_prediction_position_innovation_m", -1.0)
                ),
                "lidar_prediction_yaw_innovation_rad": float(
                    values.get("lidar_prediction_yaw_innovation_rad", -1.0)
                ),
                "imu_preintegration_residual_mahalanobis": float(
                    values.get("imu_preintegration_residual_mahalanobis", -1.0)
                ),
                "last_imu_residual_error": str(
                    values.get("last_imu_residual_error", "unavailable")
                ),
                "last_exception": str(values.get("last_exception", "unavailable")),
            })
            self.events.append(event)

    def _lio(self, msg):
        event = self._relative_event("lio", msg)
        event.update({
            "input_points": int(msg.input_points),
            "matched_points": int(msg.matched_points),
            "residual_mean_m": float(msg.residual_mean_m),
            "residual_p95_m": float(msg.residual_p95_m),
            "hessian_min_eigenvalue": float(min(msg.hessian_eigenvalues)),
            "hessian_condition": float(msg.hessian_condition),
            "normal_min_eigenvalue": float(min(msg.normal_covariance_eigenvalues)),
            "axial_penalty": float(msg.axial_penalty),
            "spatial_coverage": float(msg.spatial_coverage),
            "dynamic_ratio": float(msg.dynamic_ratio),
            "uncertain_ratio": float(msg.uncertain_ratio),
            "feature_repeatability": float(msg.feature_repeatability),
            "map_quality": float(msg.map_quality),
            "approximate": bool(msg.approximate),
            "source": str(msg.source),
        })
        self.events.append(event)


def summarize(events):
    scheduler = [event for event in events if event["kind"] == "scheduler"]
    faults = [event for event in events if event["kind"] == "fault"]
    backend = [event for event in events if event["kind"] == "backend"]
    lio = [event for event in events if event["kind"] == "lio"]
    flow_scores = [event for event in events if event["kind"] == "flow_score"]

    def finite_median(name):
        values = [event[name] for event in lio if math.isfinite(event[name])]
        return statistics.median(values) if values else None

    def backend_nonnegative_median(name):
        values = [
            event[name] for event in backend
            if math.isfinite(event[name]) and event[name] >= 0.0
        ]
        return statistics.median(values) if values else None

    def flow_nonnegative_median(name):
        values = [
            event[name] for event in flow_scores
            if math.isfinite(event[name]) and event[name] >= 0.0
        ]
        return statistics.median(values) if values else None
    states = []
    for event in scheduler:
        state = event["health_state"]
        if not states or states[-1] != state:
            states.append(state)
    active_faults = [event for event in faults if event["active"]]
    state_counts = {
        state: sum(event["health_state"] == state for event in scheduler)
        for state in (
            "NORMAL", "DEGRADED", "RISK", "RELOCALIZING", "RECOVERED",
            "FAILSAFE",
        )
    }
    return {
        "event_count": len(events),
        "scheduler_samples": len(scheduler),
        "fault_samples": len(faults),
        "scheduler_state_sequence": states,
        "scheduler_state_counts": state_counts,
        "active_fault_samples": len(active_faults),
        "fault_modalities": sorted({event["modality"] for event in faults}),
        "fault_types": sorted({event["fault_type"] for event in faults}),
        "backend_samples": len(backend),
        "backend_gnss_jump_rejected_max": max(
            (event["gnss_jump_rejected"] for event in backend), default=0
        ),
        "backend_optimization_errors_max": max(
            (event["optimization_errors"] for event in backend), default=0
        ),
        "backend_solve_ms_median": backend_nonnegative_median(
            "backend_solve_ms"
        ),
        "backend_solve_mean_ms_last": (
            backend[-1]["backend_solve_mean_ms"] if backend else None
        ),
        "backend_solve_max_ms_max": max(
            (event["backend_solve_max_ms"] for event in backend), default=0.0
        ),
        "backend_callback_ms_median": backend_nonnegative_median(
            "callback_ms"
        ),
        "backend_pending_native_worker_frames_max": max(
            (event["pending_native_worker_frames"] for event in backend),
            default=0,
        ),
        "backend_imu_factors_max": max(
            (event["imu_factors"] for event in backend), default=0
        ),
        "backend_native_worker_queue_overflow_max": max(
            (event["native_worker_queue_overflow"] for event in backend),
            default=0,
        ),
        "backend_reliability_modes": sorted({
            event["reliability_mode"] for event in backend
        }),
        "backend_flow_factor_attempts_max": max(
            (event["flow_factor_attempts"] for event in backend), default=0
        ),
        "backend_flow_factors_enabled_max": max(
            (event["flow_factors"] for event in backend), default=0
        ),
        "backend_flow_disabled_quality_max": max(
            (event["flow_disabled_quality"] for event in backend), default=0
        ),
        "backend_flow_disabled_rotation_max": max(
            (event["flow_disabled_rotation"] for event in backend), default=0
        ),
        "backend_flow_rotation_phases": sorted({
            event["flow_rotation_phase"] for event in backend
        }),
        "backend_flow_rotation_weight_median": backend_nonnegative_median(
            "flow_rotation_weight"
        ),
        "backend_flow_yaw_rate_abs_radps_median": backend_nonnegative_median(
            "flow_yaw_rate_abs_radps"
        ),
        "backend_flow_factor_types": sorted({
            event["last_flow_factor_type"] for event in backend
        }),
        "backend_lidar_disabled_max": max(
            (event["lidar_disabled"] for event in backend), default=0
        ),
        "backend_lidar_anchor_overrides_max": max(
            (event["lidar_anchor_overrides"] for event in backend), default=0
        ),
        "backend_native_lidar_received_max": max(
            (event["native_lidar_received"] for event in backend), default=0
        ),
        "backend_native_lidar_invalid_max": max(
            (event["native_lidar_invalid"] for event in backend), default=0
        ),
        "backend_native_lidar_factors_max": max(
            (event["native_lidar_factors"] for event in backend), default=0
        ),
        "backend_native_lidar_hard_disabled_max": max(
            (event["native_lidar_hard_disabled"] for event in backend), default=0
        ),
        "backend_native_lidar_pose_fallbacks_max": max(
            (event["native_lidar_pose_fallbacks"] for event in backend), default=0
        ),
        "backend_native_lidar_pair_timeouts_max": max(
            (event["native_lidar_pair_timeouts"] for event in backend), default=0
        ),
        "backend_native_lidar_matches_median": backend_nonnegative_median(
            "native_lidar_matches"
        ),
        "backend_native_lidar_stamp_error_ms_median": backend_nonnegative_median(
            "native_lidar_stamp_error_ms"
        ),
        "backend_lidar_factor_sources": sorted({
            event["lidar_factor_source"] for event in backend
        }),
        "backend_lidar_prediction_position_innovation_m_median": (
            backend_nonnegative_median("lidar_prediction_position_innovation_m")
        ),
        "backend_lidar_prediction_yaw_innovation_rad_median": (
            backend_nonnegative_median("lidar_prediction_yaw_innovation_rad")
        ),
        "backend_imu_residual_updates_max": max(
            (event["imu_residual_updates"] for event in backend), default=0
        ),
        "backend_imu_residual_errors_max": max(
            (event["imu_residual_errors"] for event in backend), default=0
        ),
        "backend_imu_startup_reasons": list(dict.fromkeys(
            event["imu_startup_reason"] for event in backend
        )),
        "backend_imu_startup_bias_accepted_max": max(
            (event["imu_startup_bias_accepted"] for event in backend), default=0
        ),
        "backend_imu_startup_bias_rejected_max": max(
            (event["imu_startup_bias_rejected"] for event in backend), default=0
        ),
        "backend_imu_startup_sample_count_max": max(
            (event["imu_startup_sample_count"] for event in backend), default=0
        ),
        "backend_imu_startup_span_s_max": max(
            (event["imu_startup_span_s"] for event in backend), default=0.0
        ),
        "backend_imu_startup_accel_bias_last": (
            backend[-1]["imu_startup_accel_bias"] if backend else None
        ),
        "backend_imu_startup_gyro_bias_last": (
            backend[-1]["imu_startup_gyro_bias"] if backend else None
        ),
        "flow_score_samples": len(flow_scores),
        "flow_score_degradation_median": flow_nonnegative_median(
            "degradation_score"
        ),
        "flow_score_rotation_weight_median": flow_nonnegative_median(
            "rotation_gate_weight"
        ),
        "flow_score_yaw_rate_abs_radps_median": flow_nonnegative_median(
            "fcu_yaw_rate_abs_radps"
        ),
        "flow_score_rotation_phase_codes": sorted({
            event["rotation_gate_phase_code"] for event in flow_scores
            if event["rotation_gate_phase_code"] >= 0.0
        }),
        "lio_samples": len(lio),
        "lio_matched_points_median": finite_median("matched_points"),
        "lio_residual_p95_m_median": finite_median("residual_p95_m"),
        "lio_hessian_min_eigenvalue_median": finite_median("hessian_min_eigenvalue"),
        "lio_hessian_condition_median": finite_median("hessian_condition"),
        "lio_normal_min_eigenvalue_median": finite_median("normal_min_eigenvalue"),
        "lio_axial_penalty_median": finite_median("axial_penalty"),
        "lio_dynamic_ratio_median": finite_median("dynamic_ratio"),
        "lio_uncertain_ratio_median": finite_median("uncertain_ratio"),
        "lio_feature_repeatability_median": finite_median("feature_repeatability"),
        "lio_map_quality_median": finite_median("map_quality"),
        "lio_native_samples": sum(not event["approximate"] for event in lio),
        "lio_approximate_samples": sum(event["approximate"] for event in lio),
        "lio_sources": sorted({event["source"] for event in lio}),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--expect-fault-modality", default="")
    parser.add_argument("--expect-fault-type", default="")
    args = parser.parse_args()
    rclpy.init()
    node = ReliabilityTimelineRecorder()
    started = time.monotonic()
    while rclpy.ok() and time.monotonic() - started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    payload = {
        "duration_s": time.monotonic() - started,
        "summary": summarize(node.events),
        "events": node.events,
    }
    expected_faults = [
        event for event in node.events
        if event["kind"] == "fault"
        and event["active"]
        and event["modality"] == args.expect_fault_modality
        and event["fault_type"] == args.expect_fault_type
    ]
    if args.expect_fault_modality or args.expect_fault_type:
        payload["summary"]["expected_fault_active_samples"] = len(expected_faults)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload["summary"], sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    scheduler_ok = payload["summary"]["scheduler_samples"] > 0
    expected_fault_ok = (
        not args.expect_fault_modality
        and not args.expect_fault_type
    ) or bool(expected_faults)
    return 0 if scheduler_ok and expected_fault_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
