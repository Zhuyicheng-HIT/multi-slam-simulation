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
from uf_interfaces.msg import FaultState, LioDiagnostics, SchedulerState


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
            "factor_enabled": {
                name: bool(enabled)
                for name, enabled in zip(msg.modality_names, msg.factor_enabled)
            },
            "covariance_inflation": {
                name: float(value)
                for name, value in zip(msg.modality_names, msg.covariance_inflation)
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
                "flow_factors": int(values.get("flow_factors", 0)),
                "flow_disabled_quality": int(values.get("flow_disabled_quality", 0)),
                "published": int(values.get("published", 0)),
                "optimization_errors": int(values.get("optimization_errors", 0)),
                "imu_residual_updates": int(values.get("imu_residual_updates", 0)),
                "imu_residual_errors": int(values.get("imu_residual_errors", 0)),
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

    def finite_median(name):
        values = [event[name] for event in lio if math.isfinite(event[name])]
        return statistics.median(values) if values else None

    def backend_nonnegative_median(name):
        values = [
            event[name] for event in backend
            if math.isfinite(event[name]) and event[name] >= 0.0
        ]
        return statistics.median(values) if values else None
    states = []
    for event in scheduler:
        state = event["health_state"]
        if not states or states[-1] != state:
            states.append(state)
    active_faults = [event for event in faults if event["active"]]
    return {
        "event_count": len(events),
        "scheduler_samples": len(scheduler),
        "fault_samples": len(faults),
        "scheduler_state_sequence": states,
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
        "backend_flow_disabled_quality_max": max(
            (event["flow_disabled_quality"] for event in backend), default=0
        ),
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
