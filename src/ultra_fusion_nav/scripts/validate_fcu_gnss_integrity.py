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
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from uf_interfaces.msg import GnssIntegrity, ReliabilityScore


def stamp_seconds(msg):
    try:
        return (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )
    except (AttributeError, TypeError, ValueError):
        return 0.0


class FcuGnssValidator(Node):
    def __init__(self):
        super().__init__(
            "fcu_gnss_integrity_validator",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.phase = "startup"
        self.raw = []
        self.scores = []
        self.integrity = []
        self.started_wall = time.monotonic()
        self.first_source_stamp_s = None
        self.invalid_header_stamp_counts = {"raw": 0, "score": 0, "integrity": 0}
        self.phase_timings = {}
        self.parameter_client = self.create_client(ParamSetV2, "/mavros/param/set")
        self.create_subscription(GPSRAW, "/fcu/gnss/raw", self._raw, qos_profile_sensor_data)
        self.create_subscription(
            ReliabilityScore, "/reliability/gnss_score", self._score, 20
        )
        self.create_subscription(
            GnssIntegrity, "/reliability/gnss_integrity", self._integrity, 20
        )

    def _timing(self, msg, stream):
        source_stamp_s = stamp_seconds(msg)
        if source_stamp_s <= 0.0:
            self.invalid_header_stamp_counts[stream] += 1
            return None
        if self.first_source_stamp_s is None:
            self.first_source_stamp_s = source_stamp_s
        elif source_stamp_s < self.first_source_stamp_s:
            self.first_source_stamp_s = source_stamp_s
            for records in (self.raw, self.scores, self.integrity):
                for record in records:
                    record["elapsed_ros_s"] = (
                        record["source_stamp_s"] - source_stamp_s
                    )
        elapsed_ros_s = source_stamp_s - self.first_source_stamp_s
        return {
            "source_stamp_s": source_stamp_s,
            "elapsed_ros_s": elapsed_ros_s,
            "arrival_elapsed_wall_s": time.monotonic() - self.started_wall,
        }

    def _raw(self, msg):
        timing = self._timing(msg, "raw")
        if timing is None:
            return
        record = {
            "phase": self.phase,
            "fix_type": int(msg.fix_type),
            "satellites": int(msg.satellites_visible),
            "hdop": None if int(msg.eph) == 65535 else float(msg.eph) * 0.01,
            "latitude_deg": float(msg.lat) * 1.0e-7,
            "longitude_deg": float(msg.lon) * 1.0e-7,
        }
        record.update(timing)
        self.raw.append(record)

    def _score(self, msg):
        timing = self._timing(msg, "score")
        if timing is None:
            return
        record = {
            "phase": self.phase,
            "degradation": float(msg.degradation_score),
            "valid": bool(msg.valid),
            "reasons": list(msg.reasons),
        }
        record.update(timing)
        self.scores.append(record)

    def _integrity(self, msg):
        timing = self._timing(msg, "integrity")
        if timing is None:
            return
        record = {
            "phase": self.phase,
            "jump": bool(msg.jump_detected),
            "synthetic_metadata": bool(msg.synthetic_metadata),
        }
        record.update(timing)
        self.integrity.append(record)

    def spin_for(self, duration_s, wall_timeout_s):
        if wall_timeout_s <= 0.0:
            wall_timeout_s = max(duration_s * 10.0, duration_s + 60.0)
        wall_started = time.monotonic()
        last_progress_wall = wall_started
        ros_started_ns = None
        last_ros_ns = None
        elapsed_ros_s = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now_ros_ns = self.get_clock().now().nanoseconds
            now_wall = time.monotonic()
            if now_ros_ns > 0 and ros_started_ns is None:
                ros_started_ns = now_ros_ns
            if last_ros_ns is not None and now_ros_ns < last_ros_ns:
                raise RuntimeError("ROS clock moved backwards during GNSS validation")
            if last_ros_ns is None or now_ros_ns > last_ros_ns:
                last_progress_wall = now_wall
            last_ros_ns = now_ros_ns
            elapsed_ros_s = (
                (now_ros_ns - ros_started_ns) * 1.0e-9
                if ros_started_ns is not None else 0.0
            )
            elapsed_wall_s = now_wall - wall_started
            if elapsed_ros_s >= duration_s:
                return {
                    "duration_s": elapsed_ros_s,
                    "duration_ros_s": elapsed_ros_s,
                    "duration_wall_s": elapsed_wall_s,
                    "wall_timeout_s": wall_timeout_s,
                    "wall_stall_timeout_s": wall_timeout_s,
                }
            stalled_wall_s = now_wall - last_progress_wall
            if stalled_wall_s >= wall_timeout_s:
                raise RuntimeError(
                    f"ROS clock stalled for {stalled_wall_s:.1f}s "
                    f"after advancing {elapsed_ros_s:.1f}s"
                )
        elapsed_wall_s = time.monotonic() - wall_started
        return {
            "duration_s": elapsed_ros_s,
            "duration_ros_s": elapsed_ros_s,
            "duration_wall_s": elapsed_wall_s,
            "wall_timeout_s": wall_timeout_s,
            "wall_stall_timeout_s": wall_timeout_s,
        }

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
        "timing": dict(node.phase_timings),
        "invalid_header_stamp_counts": dict(node.invalid_header_stamp_counts),
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
    parser.add_argument(
        "--phase-wall-timeout",
        type=float,
        default=0.0,
        help="wall seconds without ROS-clock progress; 0 selects a conservative limit",
    )
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = FcuGnssValidator()
    try:
        node.phase = "baseline"
        node.phase_timings["baseline"] = node.spin_for(
            4.0, args.phase_wall_timeout
        )
        node.phase = "outage"
        node.set_parameter("SIM_GPS1_FIXTYPE", 1)
        node.phase_timings["outage"] = node.spin_for(
            4.0, args.phase_wall_timeout
        )
        node.phase = "recovered"
        node.set_parameter("SIM_GPS1_FIXTYPE", 6)
        node.phase_timings["recovered"] = node.spin_for(
            5.0, args.phase_wall_timeout
        )
        node.phase = "jump"
        node.set_parameter("SIM_GPS1_GLTCH_X", float(args.jump_lat_deg))
        node.phase_timings["jump"] = node.spin_for(
            3.0, args.phase_wall_timeout
        )
        node.phase = "restored"
        node.set_parameter("SIM_GPS1_GLTCH_X", 0.0)
        node.phase_timings["restored"] = node.spin_for(
            5.0, args.phase_wall_timeout
        )
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
