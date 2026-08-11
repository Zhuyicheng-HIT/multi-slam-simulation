#!/usr/bin/env python3
"""Record scheduler and injected-fault events for a bounded simulation run."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
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
        super().__init__(
            "reliability_timeline_recorder",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.events = []
        self.started_wall = time.monotonic()
        self.first_event_ros_s = None
        self.invalid_header_stamp_counts = {}
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
            DiagnosticArray,
            "/vision/frontend_diagnostics",
            self._visual_frontend,
            20,
        )
        self.create_subscription(
            DiagnosticArray,
            "/fusion/unified/visual_timing",
            self._visual_timing,
            100,
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
        self.create_subscription(
            ReliabilityScore,
            "/reliability/vision_score",
            self._vision_score,
            qos_profile_sensor_data,
        )

    def _relative_event(self, kind, msg):
        source_stamp_s = stamp_seconds(msg.header.stamp)
        if source_stamp_s <= 0.0:
            self.invalid_header_stamp_counts[kind] = (
                self.invalid_header_stamp_counts.get(kind, 0) + 1
            )
            return None
        event_ros_s = source_stamp_s
        if self.first_event_ros_s is None:
            self.first_event_ros_s = event_ros_s
        elif event_ros_s < self.first_event_ros_s:
            shift_s = self.first_event_ros_s - event_ros_s
            self.first_event_ros_s = event_ros_s
            for event in self.events:
                event["received_s"] += shift_s
                event["elapsed_ros_s"] += shift_s
        elapsed_ros_s = event_ros_s - self.first_event_ros_s
        return {
            "kind": kind,
            # Backward-compatible alias, now intentionally on the ROS timeline.
            "received_s": elapsed_ros_s,
            "elapsed_ros_s": elapsed_ros_s,
            "arrival_elapsed_wall_s": time.monotonic() - self.started_wall,
            "stamp_s": source_stamp_s,
        }

    def _scheduler(self, msg):
        event = self._relative_event("scheduler", msg)
        if event is None:
            return
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
        if event is None:
            return
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
        if event is None:
            return
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

    def _vision_score(self, msg):
        event = self._relative_event("vision_score", msg)
        if event is None:
            return
        event.update({
            "degradation_score": float(msg.degradation_score),
            "reliability_weight": float(msg.reliability_weight),
            "valid": bool(msg.valid),
            "reasons": list(msg.reasons),
            "evidence": {
                name: float(value)
                for name, value in zip(
                    msg.evidence_names, msg.evidence_values
                )
            },
        })
        self.events.append(event)

    @staticmethod
    def _finite_float(values, name, default=-1.0):
        try:
            value = float(values.get(name, default))
        except (TypeError, ValueError):
            return float(default)
        return value if math.isfinite(value) else float(default)

    def _visual_frontend(self, msg):
        for status in msg.status:
            if status.name != "uf_rgbd_feature_frontend":
                continue
            values = {item.key: item.value for item in status.values}
            event = self._relative_event("visual_frontend", msg)
            if event is None:
                return
            integer_names = (
                "raw_color_frames", "raw_depth_frames", "raw_frames",
                "cadence_skipped", "keyframe_candidates",
                "tracking_initializations", "tracked_frames",
                "quality_valid_candidates", "quality_rejected_candidates",
                "published_candidates",
            )
            event.update({
                name: int(values.get(name, 0)) for name in integer_names
            })
            event.update({
                "keyframe_profile": values.get("keyframe_profile", "unknown"),
                "keyframe_period_s": self._finite_float(
                    values, "keyframe_period_s"
                ),
                "last_frontend_latency_s": self._finite_float(
                    values, "last_frontend_latency_s"
                ),
                "last_median_parallax_px": self._finite_float(
                    values, "last_median_parallax_px"
                ),
                "last_spatial_distribution": self._finite_float(
                    values, "last_spatial_distribution"
                ),
                "last_mean_reprojection_error_px": self._finite_float(
                    values, "last_mean_reprojection_error_px"
                ),
            })
            self.events.append(event)

    def _visual_timing(self, msg):
        for status in msg.status:
            if status.name != "visual_time_association":
                continue
            values = {item.key: item.value for item in status.values}
            event = self._relative_event("visual_timing", msg)
            if event is None:
                return
            event.update({
                "candidate_id": int(values.get("candidate_id", 0)),
                "outcome": values.get("outcome", "unknown"),
                "reason": values.get("reason", "unknown"),
                "missing_side": values.get("missing_side", "none"),
            })
            for name in (
                "visual_previous_stamp_s", "visual_timestamp_s",
                "arrival_ros_s", "ros_sim_time_s",
                "active_window_start_s", "active_window_end_s",
                "nearest_previous_state_stamp_s", "nearest_state_timestamp_s",
                "delta_previous_state_s", "delta_to_nearest_state_s",
                "visual_frontend_latency_s", "backend_queue_latency_s",
                "backend_queue_wall_latency_s", "keyframe_interval_s",
                "lidar_state_interval_median_s", "camera_imu_time_offset_s",
            ):
                event[name] = self._finite_float(values, name)
            self.events.append(event)

    def _backend(self, msg):
        for status in msg.status:
            if status.name != "unified_backend_fusion":
                continue
            values = {item.key: item.value for item in status.values}
            event = self._relative_event("backend", msg)
            if event is None:
                return
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
                "optimization_rejected": int(
                    values.get("optimization_rejected", 0)
                ),
                "optimization_rollbacks": int(
                    values.get("optimization_rollbacks", 0)
                ),
                "optimization_integrity_reason": str(
                    values.get("optimization_integrity_reason", "not_checked")
                ),
                "optimization_integrity_counts": str(
                    values.get("optimization_integrity_counts", "none")
                ),
                "optimization_integrity_translation_correction_m": float(
                    values.get("optimization_translation_correction_m", -1.0)
                ),
                "optimization_integrity_rotation_correction_rad": float(
                    values.get("optimization_rotation_correction_rad", -1.0)
                ),
                "optimization_integrity_velocity_correction_mps": float(
                    values.get("optimization_velocity_correction_mps", -1.0)
                ),
                "optimization_integrity_accel_bias_correction_mps2": float(
                    values.get("optimization_accel_bias_correction_mps2", -1.0)
                ),
                "optimization_integrity_gyro_bias_correction_radps": float(
                    values.get("optimization_gyro_bias_correction_radps", -1.0)
                ),
                "optimization_integrity_information_rank": int(
                    values.get("optimization_information_rank", -1)
                ),
                "optimization_integrity_initial_cost": float(
                    values.get("optimization_initial_cost", -1.0)
                ),
                "optimization_integrity_final_cost": float(
                    values.get("optimization_final_cost", -1.0)
                ),
                "optimization_integrity_information_condition": float(
                    values.get("optimization_information_condition", -1.0)
                ),
                "optimized_states_committed": int(
                    values.get("optimized_states_committed", 0)
                ),
                "visual_received": int(values.get("visual_received", 0)),
                "visual_factor_attempts": int(
                    values.get("visual_factor_attempts", 0)
                ),
                "visual_factors": int(values.get("visual_factors", 0)),
                "visual_rejected_time": int(
                    values.get("visual_rejected_time", 0)
                ),
                "visual_rejected_tracks": int(
                    values.get("visual_rejected_tracks", 0)
                ),
                "visual_window_associated_candidates": int(
                    values.get("visual_window_associated_candidates", 0)
                ),
                "visual_solver_accepted": int(
                    values.get("visual_solver_accepted", 0)
                ),
                "visual_solver_rejected": int(
                    values.get("visual_solver_rejected", 0)
                ),
                "visual_pending_enqueued": int(
                    values.get("visual_pending_enqueued", 0)
                ),
                "visual_pending_waits": int(
                    values.get("visual_pending_waits", 0)
                ),
                "visual_pending_expired": int(
                    values.get("visual_pending_expired", 0)
                ),
                "visual_pending_overflow": int(
                    values.get("visual_pending_overflow", 0)
                ),
                "visual_duplicate_candidates": int(
                    values.get("visual_duplicate_candidates", 0)
                ),
                "visual_quality_rejected_dv": int(
                    values.get("visual_quality_rejected_dv", 0)
                ),
                "visual_state_consistency_rejected": int(
                    values.get("visual_state_consistency_rejected", 0)
                ),
                "visual_linearization_invalid": int(
                    values.get("visual_linearization_invalid", 0)
                ),
                "visual_prefit_rmse_normalized": float(
                    values.get("visual_prefit_rmse_normalized", -1.0)
                ),
                "visual_prefit_rmse_px": float(
                    values.get("visual_prefit_rmse_px", -1.0)
                ),
                "visual_prefit_valid_track_ratio": float(
                    values.get("visual_prefit_valid_track_ratio", -1.0)
                ),
                "visual_prefit_jacobian_rank": int(
                    values.get("visual_prefit_jacobian_rank", 0)
                ),
                "visual_reprojection_rmse_normalized": float(
                    values.get("visual_reprojection_rmse_normalized", -1.0)
                ),
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
            event["performance_profiles"] = {}
            for name, value in values.items():
                if not name.startswith("profile_"):
                    continue
                try:
                    event["performance_profiles"][name] = float(value)
                except ValueError:
                    continue
            self.events.append(event)

    def _lio(self, msg):
        event = self._relative_event("lio", msg)
        if event is None:
            return
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


def record_for_ros_duration(node, duration_s, wall_timeout_s):
    wall_started = time.monotonic()
    last_progress_wall = wall_started
    ros_started_ns = None
    last_ros_ns = None
    elapsed_ros_s = 0.0
    elapsed_wall_s = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        now_ros_ns = node.get_clock().now().nanoseconds
        now_wall = time.monotonic()
        if now_ros_ns > 0 and ros_started_ns is None:
            ros_started_ns = now_ros_ns
        if last_ros_ns is not None and now_ros_ns < last_ros_ns:
            raise RuntimeError("ROS clock moved backwards while recording reliability timeline")
        if last_ros_ns is None or now_ros_ns > last_ros_ns:
            last_progress_wall = now_wall
        last_ros_ns = now_ros_ns
        elapsed_ros_s = (
            (now_ros_ns - ros_started_ns) * 1.0e-9
            if ros_started_ns is not None else 0.0
        )
        elapsed_wall_s = now_wall - wall_started
        if elapsed_ros_s >= duration_s:
            return elapsed_ros_s, elapsed_wall_s
        stalled_wall_s = now_wall - last_progress_wall
        if stalled_wall_s >= wall_timeout_s:
            raise RuntimeError(
                f"ROS clock stalled for {stalled_wall_s:.1f}s "
                f"after advancing {elapsed_ros_s:.1f}s"
            )
    return elapsed_ros_s, time.monotonic() - wall_started


def summarize(events):
    scheduler = [event for event in events if event["kind"] == "scheduler"]
    faults = [event for event in events if event["kind"] == "fault"]
    backend = [event for event in events if event["kind"] == "backend"]
    lio = [event for event in events if event["kind"] == "lio"]
    flow_scores = [event for event in events if event["kind"] == "flow_score"]
    vision_scores = [
        event for event in events if event["kind"] == "vision_score"
    ]
    visual_frontend = [
        event for event in events if event["kind"] == "visual_frontend"
    ]
    visual_timing = [
        event for event in events if event["kind"] == "visual_timing"
    ]

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

    def timing_nonnegative_median(name):
        values = [
            event[name] for event in visual_timing
            if math.isfinite(event[name]) and event[name] >= 0.0
        ]
        return statistics.median(values) if values else None

    def frontend_max(name):
        return max((event[name] for event in visual_frontend), default=0)
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
        "backend_gnss_factors_max": max(
            (event["gnss_factors"] for event in backend), default=0
        ),
        "backend_optimization_errors_max": max(
            (event["optimization_errors"] for event in backend), default=0
        ),
        "backend_optimization_rejected_max": max(
            (event["optimization_rejected"] for event in backend), default=0
        ),
        "backend_optimization_rollbacks_max": max(
            (event["optimization_rollbacks"] for event in backend), default=0
        ),
        "backend_optimization_integrity_reasons": sorted({
            event["optimization_integrity_reason"] for event in backend
        }),
        "backend_optimization_integrity_counts_last": (
            backend[-1]["optimization_integrity_counts"] if backend else "none"
        ),
        "backend_optimized_states_committed_max": max(
            (event["optimized_states_committed"] for event in backend), default=0
        ),
        "backend_visual_received_max": max(
            (event["visual_received"] for event in backend), default=0
        ),
        "backend_visual_factor_attempts_max": max(
            (event["visual_factor_attempts"] for event in backend), default=0
        ),
        "backend_visual_factors_max": max(
            (event["visual_factors"] for event in backend), default=0
        ),
        "backend_visual_rejected_time_max": max(
            (event["visual_rejected_time"] for event in backend), default=0
        ),
        "backend_visual_rejected_tracks_max": max(
            (event["visual_rejected_tracks"] for event in backend), default=0
        ),
        "backend_visual_window_associated_candidates_max": max(
            (event["visual_window_associated_candidates"] for event in backend),
            default=0,
        ),
        "backend_visual_solver_accepted_max": max(
            (event["visual_solver_accepted"] for event in backend), default=0
        ),
        "backend_visual_solver_rejected_max": max(
            (event["visual_solver_rejected"] for event in backend), default=0
        ),
        "backend_visual_pending_enqueued_max": max(
            (event["visual_pending_enqueued"] for event in backend), default=0
        ),
        "backend_visual_pending_waits_max": max(
            (event["visual_pending_waits"] for event in backend), default=0
        ),
        "backend_visual_pending_expired_max": max(
            (event["visual_pending_expired"] for event in backend), default=0
        ),
        "backend_visual_pending_overflow_max": max(
            (event["visual_pending_overflow"] for event in backend), default=0
        ),
        "backend_visual_duplicate_candidates_max": max(
            (event["visual_duplicate_candidates"] for event in backend),
            default=0,
        ),
        "backend_visual_quality_rejected_dv_max": max(
            (event["visual_quality_rejected_dv"] for event in backend), default=0
        ),
        "backend_visual_state_consistency_rejected_max": max(
            (event["visual_state_consistency_rejected"] for event in backend),
            default=0,
        ),
        "backend_visual_linearization_invalid_max": max(
            (event["visual_linearization_invalid"] for event in backend),
            default=0,
        ),
        "visual_frontend_samples": len(visual_frontend),
        "visual_raw_frames_max": frontend_max("raw_frames"),
        "visual_tracked_frames_max": frontend_max("tracked_frames"),
        "visual_keyframe_candidates_max": frontend_max("keyframe_candidates"),
        "visual_quality_valid_candidates_max": frontend_max(
            "quality_valid_candidates"
        ),
        "visual_quality_rejected_candidates_max": frontend_max(
            "quality_rejected_candidates"
        ),
        "visual_published_candidates_max": frontend_max("published_candidates"),
        "visual_keyframe_profiles": sorted({
            event["keyframe_profile"] for event in visual_frontend
        }),
        "visual_frontend_latency_s_median": (
            statistics.median([
                event["last_frontend_latency_s"] for event in visual_frontend
                if event["last_frontend_latency_s"] >= 0.0
            ]) if any(
                event["last_frontend_latency_s"] >= 0.0
                for event in visual_frontend
            ) else None
        ),
        "visual_timing_samples": len(visual_timing),
        "visual_timing_outcomes": {
            outcome: sum(event["outcome"] == outcome for event in visual_timing)
            for outcome in sorted({event["outcome"] for event in visual_timing})
        },
        "visual_timing_reasons": {
            reason: sum(event["reason"] == reason for event in visual_timing)
            for reason in sorted({event["reason"] for event in visual_timing})
        },
        "visual_timing_missing_sides": {
            side: sum(event["missing_side"] == side for event in visual_timing)
            for side in sorted({event["missing_side"] for event in visual_timing})
        },
        "visual_delta_to_nearest_state_s_median": timing_nonnegative_median(
            "delta_to_nearest_state_s"
        ),
        "visual_previous_delta_s_median": timing_nonnegative_median(
            "delta_previous_state_s"
        ),
        "visual_backend_queue_latency_s_median": timing_nonnegative_median(
            "backend_queue_latency_s"
        ),
        "visual_backend_queue_wall_latency_s_median": timing_nonnegative_median(
            "backend_queue_wall_latency_s"
        ),
        "visual_keyframe_interval_s_median": timing_nonnegative_median(
            "keyframe_interval_s"
        ),
        "visual_lidar_state_interval_s_median": timing_nonnegative_median(
            "lidar_state_interval_median_s"
        ),
        "backend_visual_reprojection_rmse_normalized_median": (
            backend_nonnegative_median("visual_reprojection_rmse_normalized")
        ),
        "backend_visual_prefit_rmse_normalized_median": (
            backend_nonnegative_median("visual_prefit_rmse_normalized")
        ),
        "backend_visual_prefit_rmse_px_median": backend_nonnegative_median(
            "visual_prefit_rmse_px"
        ),
        "backend_visual_prefit_valid_track_ratio_median": (
            backend_nonnegative_median("visual_prefit_valid_track_ratio")
        ),
        "backend_visual_prefit_jacobian_rank_median": (
            backend_nonnegative_median("visual_prefit_jacobian_rank")
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
        "backend_performance_profiles_last": (
            backend[-1]["performance_profiles"] if backend else {}
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
        "vision_score_samples": len(vision_scores),
        "vision_score_valid_samples": sum(
            event["valid"] for event in vision_scores
        ),
        "vision_reliability_weight_median": (
            statistics.median([
                event["reliability_weight"] for event in vision_scores
                if event["valid"]
            ])
            if any(event["valid"] for event in vision_scores) else None
        ),
        "vision_degradation_score_median": (
            statistics.median([
                event["degradation_score"] for event in vision_scores
            ]) if vision_scores else None
        ),
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
    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=0.0,
        help="wall seconds without ROS-clock progress; 0 selects a conservative limit",
    )
    args = parser.parse_args()
    rclpy.init()
    node = ReliabilityTimelineRecorder()
    wall_timeout_s = (
        args.wall_timeout if args.wall_timeout > 0.0
        else max(args.duration * 10.0, args.duration + 60.0)
    )
    record_started_wall = time.monotonic()
    interrupted = False
    try:
        duration_ros_s, duration_wall_s = record_for_ros_duration(
            node, args.duration, wall_timeout_s
        )
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
        duration_wall_s = time.monotonic() - record_started_wall
        duration_ros_s = max(
            (event["elapsed_ros_s"] for event in node.events), default=0.0
        )
    events = sorted(
        node.events,
        key=lambda event: (
            event["elapsed_ros_s"], event["arrival_elapsed_wall_s"]
        ),
    )
    payload = {
        "duration_s": duration_ros_s,
        "duration_ros_s": duration_ros_s,
        "duration_wall_s": duration_wall_s,
        "requested_duration_ros_s": args.duration,
        "interrupted": interrupted,
        "wall_timeout_s": wall_timeout_s,
        "wall_stall_timeout_s": wall_timeout_s,
        "event_time_basis": "valid_source_header_stamp_only",
        "invalid_header_stamp_counts": dict(node.invalid_header_stamp_counts),
        "summary": summarize(events),
        "events": events,
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
    if rclpy.ok():
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
