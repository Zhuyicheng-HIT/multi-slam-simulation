#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import statistics
import time

import rclpy
from mavros_msgs.msg import GPSRAW
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from uf_interfaces.msg import GnssIntegrity, ReliabilityScore


class FcuGnssValidator(Node):
    def __init__(self):
        super().__init__("fcu_gnss_integrity_validator")
        self.phase = "startup"
        self.raw = []
        self.scores = []
        self.integrity = []
        self.parameter_client = self.create_client(ParamSetV2, "/mavros/param/set")
        self.create_subscription(GPSRAW, "/fcu/gnss/raw", self._raw, qos_profile_sensor_data)
        self.create_subscription(
            ReliabilityScore, "/reliability/gnss_score", self._score, 20
        )
        self.create_subscription(
            GnssIntegrity, "/reliability/gnss_integrity", self._integrity, 20
        )

    def _raw(self, msg):
        self.raw.append({
            "phase": self.phase,
            "fix_type": int(msg.fix_type),
            "satellites": int(msg.satellites_visible),
            "hdop": None if int(msg.eph) == 65535 else float(msg.eph) * 0.01,
            "latitude_deg": float(msg.lat) * 1.0e-7,
            "longitude_deg": float(msg.lon) * 1.0e-7,
        })

    def _score(self, msg):
        self.scores.append({
            "phase": self.phase,
            "degradation": float(msg.degradation_score),
            "valid": bool(msg.valid),
            "reasons": list(msg.reasons),
        })

    def _integrity(self, msg):
        self.integrity.append({
            "phase": self.phase,
            "jump": bool(msg.jump_detected),
            "synthetic_metadata": bool(msg.synthetic_metadata),
        })

    def spin_for(self, duration_s):
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_parameter(self, name, value):
        if not self.parameter_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MAVROS FCU parameter service is unavailable")
        request = ParamSetV2.Request()
        request.force_set = True
        request.param_id = name
        if isinstance(value, int):
            request.value.type = ParameterType.PARAMETER_INTEGER
            request.value.integer_value = value
        else:
            request.value.type = ParameterType.PARAMETER_DOUBLE
            request.value.double_value = float(value)
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Timed out setting {name}")
        result = future.result()
        if not result.success:
            raise RuntimeError(f"Failed to set {name}")


def phase_values(records, phase, key):
    return [record[key] for record in records if record["phase"] == phase]


def summarize(node):
    baseline_fix = phase_values(node.raw, "baseline", "fix_type")
    outage_fix = phase_values(node.raw, "outage", "fix_type")
    recovered_fix = phase_values(node.raw, "recovered", "fix_type")
    baseline_score = phase_values(node.scores, "baseline", "degradation")
    outage_score = phase_values(node.scores, "outage", "degradation")
    recovered_score = phase_values(node.scores, "recovered", "degradation")
    jump_flags = phase_values(node.integrity, "jump", "jump")
    restore_flags = phase_values(node.integrity, "restored", "jump")
    metadata_flags = [not record["synthetic_metadata"] for record in node.integrity]
    result = {
        "samples": {
            "raw": len(node.raw),
            "score": len(node.scores),
            "integrity": len(node.integrity),
        },
        "baseline_fix_types": sorted(set(baseline_fix)),
        "outage_fix_types": sorted(set(outage_fix)),
        "recovered_fix_types": sorted(set(recovered_fix)),
        "baseline_fix_type_median": statistics.median(baseline_fix) if baseline_fix else None,
        "outage_fix_type_median": statistics.median(outage_fix) if outage_fix else None,
        "recovered_fix_type_median": statistics.median(recovered_fix) if recovered_fix else None,
        "baseline_degradation_median": (
            statistics.median(baseline_score) if baseline_score else None
        ),
        "outage_degradation_median": (
            statistics.median(outage_score) if outage_score else None
        ),
        "recovered_degradation_median": (
            statistics.median(recovered_score) if recovered_score else None
        ),
        "jump_detected": any(jump_flags),
        "restore_jump_detected": any(restore_flags),
        "metadata_real_ratio": (
            sum(metadata_flags) / len(metadata_flags) if metadata_flags else 0.0
        ),
    }
    result["passed"] = bool(
        baseline_fix and statistics.median(baseline_fix) >= 3
        and outage_fix and statistics.median(outage_fix) <= 1
        and recovered_fix and statistics.median(recovered_fix) >= 3
        and baseline_score and outage_score and recovered_score
        and statistics.median(outage_score) >= statistics.median(baseline_score) + 0.15
        and statistics.median(recovered_score) <= statistics.median(baseline_score) + 0.05
        and result["jump_detected"]
        and result["metadata_real_ratio"] >= 0.95
    )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--jump-lat-deg", type=float, default=0.0003)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = FcuGnssValidator()
    try:
        node.phase = "baseline"
        node.spin_for(4.0)
        node.phase = "outage"
        node.set_parameter("SIM_GPS1_FIXTYPE", 1)
        node.spin_for(4.0)
        node.phase = "recovered"
        node.set_parameter("SIM_GPS1_FIXTYPE", 6)
        node.spin_for(5.0)
        node.phase = "jump"
        node.set_parameter("SIM_GPS1_GLTCH_X", float(args.jump_lat_deg))
        node.spin_for(3.0)
        node.phase = "restored"
        node.set_parameter("SIM_GPS1_GLTCH_X", 0.0)
        node.spin_for(5.0)
        result = summarize(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        result = {"passed": False, "error": "interrupted"}
    except Exception as exc:
        result = {"passed": False, "error": str(exc)}
    finally:
        try:
            node.set_parameter("SIM_GPS1_FIXTYPE", 6)
            node.set_parameter("SIM_GPS1_GLTCH_X", 0.0)
        except Exception as exc:
            result["restore_error"] = str(exc)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if result.get("passed", False) or not args.require_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
