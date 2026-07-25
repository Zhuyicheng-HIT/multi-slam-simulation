#!/usr/bin/env python3
"""Record scheduler and injected-fault events for a bounded simulation run."""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from uf_interfaces.msg import FaultState, SchedulerState


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
                "gnss_factors": int(values.get("gnss_factors", 0)),
                "gnss_jump_rejected": int(values.get("gnss_jump_rejected", 0)),
                "published": int(values.get("published", 0)),
                "optimization_errors": int(values.get("optimization_errors", 0)),
            })
            self.events.append(event)


def summarize(events):
    scheduler = [event for event in events if event["kind"] == "scheduler"]
    faults = [event for event in events if event["kind"] == "fault"]
    backend = [event for event in events if event["kind"] == "backend"]
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
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output", required=True)
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
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload["summary"], sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if payload["summary"]["scheduler_samples"] > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
