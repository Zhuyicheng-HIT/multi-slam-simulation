#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from collections import Counter, defaultdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from mavros_msgs.msg import OpticalFlowRad
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rclpy.utilities import remove_ros_args
from uf_interfaces.msg import (
    FaultState,
    FusionEpoch,
    LidarCalibrationMotion,
    ReliabilityScore,
    RelocalizationResult,
    SchedulerState,
)


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def diagnostic_level(value):
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little")
    return int(value)


def scheduler_clock_domain_error_s(source_ns, observer_ns):
    """Return clock error, deferring validation until the observer has /clock."""
    source_ns = int(source_ns)
    observer_ns = int(observer_ns)
    if observer_ns <= 0:
        return None
    if source_ns <= 0:
        return math.inf
    return abs(source_ns - observer_ns) * 1.0e-9


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(message):
    orientation = message.pose.pose.orientation
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def normalized_quaternion_xyzw(message):
    orientation = message.pose.pose.orientation
    quaternion = tuple(float(value) for value in (
        orientation.x, orientation.y, orientation.z, orientation.w
    ))
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in quaternion)


def rotate_body_to_map(vector, quaternion_xyzw):
    x, y, z, w = quaternion_xyzw
    vx, vy, vz = vector
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


class StreamStats:
    def __init__(self):
        self.wall_arrivals = []
        self.observer_ros_times = []
        self.source_stamp_ns = []
        self.source_times = []
        self.source_ages_s = []
        self.positions = []
        self.yaws = []
        self.quaternions_xyzw = []
        self.linear_velocities_body = []
        self.last_stamp = None
        self.regressions = 0
        self.duplicates = 0
        self.zero_stamps = 0

    def add(self, message, observer_ros_ns=None):
        self.wall_arrivals.append(time.monotonic())
        if observer_ros_ns is not None and int(observer_ros_ns) > 0:
            self.observer_ros_times.append(int(observer_ros_ns) * 1.0e-9)
        stamp = stamp_ns(message)
        if stamp <= 0:
            self.zero_stamps += 1
            return
        self.source_stamp_ns.append(stamp)
        self.source_times.append(stamp * 1.0e-9)
        if observer_ros_ns is not None and int(observer_ros_ns) > 0:
            self.source_ages_s.append(
                (int(observer_ros_ns) - stamp) * 1.0e-9
            )
        self.positions.append((
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        ))
        self.yaws.append(yaw_from_odometry(message))
        self.quaternions_xyzw.append(normalized_quaternion_xyzw(message))
        self.linear_velocities_body.append((
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
            float(message.twist.twist.linear.z),
        ))
        if self.last_stamp is not None:
            self.regressions += int(stamp < self.last_stamp)
            self.duplicates += int(stamp == self.last_stamp)
        self.last_stamp = stamp

    def epoch_continuity(self, event_stamp_ns, local_window_s=0.5):
        ordered = sorted(
            range(len(self.source_stamp_ns)),
            key=lambda index: (self.source_stamp_ns[index], index),
        )
        before = None
        after = None
        for index in ordered:
            source_stamp_ns = self.source_stamp_ns[index]
            if source_stamp_ns < event_stamp_ns:
                before = index
            elif after is None:
                after = index
                break
        if before is None or after is None:
            return {
                "available": False,
                "event_stamp_s": event_stamp_ns * 1.0e-9,
                "reason": "missing_bracketing_samples",
            }

        def position_delta(first, second):
            return math.sqrt(sum(
                (value - reference) ** 2
                for value, reference in zip(
                    self.positions[second], self.positions[first]
                )
            ))

        bracket_gap_s = (
            self.source_stamp_ns[after] - self.source_stamp_ns[before]
        ) * 1.0e-9
        local_window_ns = int(round(local_window_s * 1.0e9))
        local_indices = [
            index for index in ordered
            if abs(self.source_stamp_ns[index] - event_stamp_ns) <= local_window_ns
        ]
        local_steps = []
        local_yaw_steps = []
        for first, second in zip(local_indices, local_indices[1:]):
            if first == before and second == after:
                continue
            local_steps.append(position_delta(first, second))
            local_yaw_steps.append(abs(wrap_angle(
                self.yaws[second] - self.yaws[first]
            )))
        position_jump_m = position_delta(before, after)
        yaw_jump_rad = abs(wrap_angle(self.yaws[after] - self.yaws[before]))
        pre_velocity_body = self.linear_velocities_body[before]
        post_velocity_body = self.linear_velocities_body[after]
        pre_velocity_map = rotate_body_to_map(
            pre_velocity_body, self.quaternions_xyzw[before]
        )
        predicted_post = tuple(
            position + velocity * bracket_gap_s
            for position, velocity in zip(
                self.positions[before], pre_velocity_map
            )
        )
        constant_velocity_residual_m = math.sqrt(sum(
            (actual - predicted) ** 2
            for actual, predicted in zip(self.positions[after], predicted_post)
        ))
        velocity_step_mps = math.sqrt(sum(
            (post - pre) ** 2
            for pre, post in zip(pre_velocity_body, post_velocity_body)
        ))
        return {
            "available": True,
            "event_stamp_s": event_stamp_ns * 1.0e-9,
            "before_stamp_s": self.source_stamp_ns[before] * 1.0e-9,
            "after_stamp_s": self.source_stamp_ns[after] * 1.0e-9,
            "before_to_epoch_s": (
                event_stamp_ns - self.source_stamp_ns[before]
            ) * 1.0e-9,
            "epoch_to_after_s": (
                self.source_stamp_ns[after] - event_stamp_ns
            ) * 1.0e-9,
            "bracket_gap_s": bracket_gap_s,
            "before_position_m": self.positions[before],
            "after_position_m": self.positions[after],
            "position_step_m": position_jump_m,
            "before_yaw_rad": self.yaws[before],
            "after_yaw_rad": self.yaws[after],
            "yaw_step_rad": yaw_jump_rad,
            "before_linear_velocity_body_mps": pre_velocity_body,
            "after_linear_velocity_body_mps": post_velocity_body,
            "before_speed_mps": math.sqrt(sum(
                value * value for value in pre_velocity_body
            )),
            "after_speed_mps": math.sqrt(sum(
                value * value for value in post_velocity_body
            )),
            "linear_velocity_step_mps": velocity_step_mps,
            "constant_velocity_position_residual_m": constant_velocity_residual_m,
            "angular_velocity_available": False,
            "local_window_s": local_window_s,
            "local_sample_count": len(local_indices),
            "neighbor_max_step_m": max(local_steps) if local_steps else None,
            "neighbor_max_yaw_step_rad": (
                max(local_yaw_steps) if local_yaw_steps else None
            ),
        }

    def report(self):
        gaps = [b - a for a, b in zip(self.source_times, self.source_times[1:])]
        observer_gaps = [
            b - a for a, b in zip(
                self.observer_ros_times, self.observer_ros_times[1:]
            )
        ]
        maximum_gap_index = (
            max(range(len(gaps)), key=lambda index: gaps[index])
            if gaps else None
        )
        duration = (
            self.source_times[-1] - self.source_times[0]
            if len(self.source_times) > 1 else 0.0
        )
        wall_duration = (
            self.wall_arrivals[-1] - self.wall_arrivals[0]
            if len(self.wall_arrivals) > 1 else 0.0
        )
        displacement = []
        if self.positions:
            origin = self.positions[0]
            displacement = [math.sqrt(sum(
                (value - reference) ** 2
                for value, reference in zip(position, origin)
            )) for position in self.positions]
        first_over_five = next((
            self.source_times[index] - self.source_times[0]
            for index, value in enumerate(displacement) if value > 5.0
        ), None) if len(self.source_times) == len(displacement) else None
        return {
            "count": len(self.wall_arrivals),
            "rate_hz": (len(self.source_times) - 1) / duration if duration else 0.0,
            "source_stamp_rate_hz": (
                (len(self.source_times) - 1) / duration if duration else 0.0
            ),
            "wall_arrival_rate_hz": (
                (len(self.wall_arrivals) - 1) / wall_duration
                if wall_duration else 0.0
            ),
            "observer_ros_arrival_rate_hz": (
                (len(self.observer_ros_times) - 1)
                / (self.observer_ros_times[-1] - self.observer_ros_times[0])
                if len(self.observer_ros_times) > 1
                and self.observer_ros_times[-1] > self.observer_ros_times[0]
                else 0.0
            ),
            "max_gap_s": max(gaps) if gaps else None,
            "max_gap_start_s": (
                self.source_times[maximum_gap_index] - self.source_times[0]
                if maximum_gap_index is not None else None
            ),
            "max_gap_end_s": (
                self.source_times[maximum_gap_index + 1] - self.source_times[0]
                if maximum_gap_index is not None else None
            ),
            "gaps_over_0_25_s": sum(gap > 0.25 for gap in gaps),
            "observer_ros_max_gap_s": (
                max(observer_gaps) if observer_gaps else None
            ),
            "observer_ros_gaps_over_0_25_s": sum(
                gap > 0.25 for gap in observer_gaps
            ),
            "observer_ros_gaps_over_0_5_s": sum(
                gap > 0.5 for gap in observer_gaps
            ),
            "observer_ros_gaps_over_1_s": sum(
                gap > 1.0 for gap in observer_gaps
            ),
            "stamp_regressions": self.regressions,
            "stamp_duplicates": self.duplicates,
            "zero_stamps": self.zero_stamps,
            "max_displacement_from_first_m": max(displacement) if displacement else None,
            "first_displacement_over_5m_s": first_over_five,
            "source_age_s": numeric_summary([
                (index, value)
                for index, value in enumerate(self.source_ages_s)
            ]),
            "absolute_source_age_s": numeric_summary([
                (index, abs(value))
                for index, value in enumerate(self.source_ages_s)
            ]),
            "future_stamp_over_0_05_s": sum(
                age < -0.05 for age in self.source_ages_s
            ),
            "stale_stamp_over_0_25_s": sum(
                age > 0.25 for age in self.source_ages_s
            ),
            "clock_domain_mismatch_over_1_s": sum(
                abs(age) > 1.0 for age in self.source_ages_s
            ),
            "latest_source_age_s": (
                self.source_ages_s[-1] if self.source_ages_s else None
            ),
        }


class HeaderStreamStats:
    """Bounded diagnostics for estimator-facing stamped sensor streams."""

    def __init__(self):
        self.wall_arrivals = []
        self.observer_ros_times = []
        self.source_times = []
        self.source_ages_s = []
        self.last_stamp_ns = None
        self.regressions = 0
        self.duplicates = 0
        self.zero_stamps = 0

    def add(self, message, observer_ros_ns, wall_arrival_s=None):
        self.wall_arrivals.append(
            time.monotonic() if wall_arrival_s is None else float(wall_arrival_s)
        )
        observer_ros_ns = int(observer_ros_ns)
        if observer_ros_ns > 0:
            self.observer_ros_times.append(observer_ros_ns * 1.0e-9)
        source_stamp_ns = stamp_ns(message)
        if source_stamp_ns <= 0:
            self.zero_stamps += 1
            return
        self.source_times.append(source_stamp_ns * 1.0e-9)
        if observer_ros_ns > 0:
            self.source_ages_s.append(
                (observer_ros_ns - source_stamp_ns) * 1.0e-9
            )
        if self.last_stamp_ns is not None:
            self.regressions += int(source_stamp_ns < self.last_stamp_ns)
            self.duplicates += int(source_stamp_ns == self.last_stamp_ns)
        self.last_stamp_ns = source_stamp_ns

    @staticmethod
    def _gap_report(values):
        gaps = [second - first for first, second in zip(values, values[1:])]
        nonnegative_gap_indices = [
            index for index, gap in enumerate(gaps) if gap >= 0.0
        ]
        maximum_gap_index = (
            max(nonnegative_gap_indices, key=lambda index: gaps[index])
            if nonnegative_gap_indices else None
        )
        duration = max(values) - min(values) if len(values) > 1 else 0.0
        return {
            "rate_hz": (
                (len(values) - 1) / duration if duration > 0.0 else 0.0
            ),
            "max_gap_s": (
                gaps[maximum_gap_index]
                if maximum_gap_index is not None else None
            ),
            "max_gap_start_s": (
                values[maximum_gap_index] - values[0]
                if maximum_gap_index is not None else None
            ),
            "max_gap_end_s": (
                values[maximum_gap_index + 1] - values[0]
                if maximum_gap_index is not None else None
            ),
            "gaps_over_0_25_s": sum(gap > 0.25 for gap in gaps),
            "gaps_over_0_5_s": sum(gap > 0.5 for gap in gaps),
            "gaps_over_1_s": sum(gap > 1.0 for gap in gaps),
        }

    def report(self):
        source = self._gap_report(self.source_times)
        observer = self._gap_report(self.observer_ros_times)
        wall = self._gap_report(self.wall_arrivals)
        age_samples = [
            (index, value) for index, value in enumerate(self.source_ages_s)
        ]
        absolute_age_samples = [
            (index, abs(value)) for index, value in enumerate(self.source_ages_s)
        ]
        return {
            "count": len(self.wall_arrivals),
            "valid_source_stamp_count": len(self.source_times),
            "source_stamp_rate_hz": source["rate_hz"],
            "observer_ros_arrival_rate_hz": observer["rate_hz"],
            "wall_arrival_rate_hz": wall["rate_hz"],
            "source_max_gap_s": source["max_gap_s"],
            "source_max_gap_start_s": source["max_gap_start_s"],
            "source_max_gap_end_s": source["max_gap_end_s"],
            "source_gaps_over_0_25_s": source["gaps_over_0_25_s"],
            "source_gaps_over_0_5_s": source["gaps_over_0_5_s"],
            "source_gaps_over_1_s": source["gaps_over_1_s"],
            "observer_ros_max_gap_s": observer["max_gap_s"],
            "observer_ros_gaps_over_0_25_s": observer["gaps_over_0_25_s"],
            "observer_ros_gaps_over_0_5_s": observer["gaps_over_0_5_s"],
            "observer_ros_gaps_over_1_s": observer["gaps_over_1_s"],
            "wall_max_gap_s": wall["max_gap_s"],
            "stamp_regressions": self.regressions,
            "stamp_duplicates": self.duplicates,
            "zero_stamps": self.zero_stamps,
            "source_age_s": numeric_summary(age_samples),
            "absolute_source_age_s": numeric_summary(absolute_age_samples),
            "future_stamp_over_0_05_s": sum(
                age < -0.05 for age in self.source_ages_s
            ),
            "stale_stamp_over_0_25_s": sum(
                age > 0.25 for age in self.source_ages_s
            ),
            "clock_domain_mismatch_over_1_s": sum(
                abs(age) > 1.0 for age in self.source_ages_s
            ),
            "latest_source_age_s": (
                self.source_ages_s[-1] if self.source_ages_s else None
            ),
        }


class ReliabilityScoreStats(HeaderStreamStats):
    def __init__(self):
        super().__init__()
        self.valid_count = 0
        self.degradation_scores = []
        self.reliability_weights = []
        self.observation_counts = []
        self.reasons = Counter()

    def add(self, message, observer_ros_ns, wall_arrival_s=None):
        super().add(message, observer_ros_ns, wall_arrival_s)
        self.valid_count += int(message.valid)
        sample_index = len(self.degradation_scores)
        self.degradation_scores.append((
            sample_index, float(message.degradation_score)
        ))
        self.reliability_weights.append((
            sample_index, float(message.reliability_weight)
        ))
        self.observation_counts.append(int(message.observation_count))
        self.reasons.update(str(reason) for reason in message.reasons)

    def report(self):
        report = super().report()
        count = len(self.degradation_scores)
        report.update({
            "valid_ratio": self.valid_count / count if count else None,
            "degradation_score": numeric_summary(self.degradation_scores),
            "reliability_weight": numeric_summary(self.reliability_weights),
            "observation_count_min": (
                min(self.observation_counts) if self.observation_counts else None
            ),
            "observation_count_max": (
                max(self.observation_counts) if self.observation_counts else None
            ),
            "reasons": dict(self.reasons),
        })
        return report


class FaultStateStats:
    def __init__(self):
        self.events = 0
        self.active_events = 0
        self.timestamp_repaired_events = 0
        self.max_timestamp_repairs = 0
        self.max_affected_messages = 0
        self.fault_types = Counter()

    def add(self, message):
        self.events += 1
        self.active_events += int(message.active)
        self.timestamp_repaired_events += int(message.timestamp_repaired)
        self.max_timestamp_repairs = max(
            self.max_timestamp_repairs, int(message.timestamp_repairs)
        )
        self.max_affected_messages = max(
            self.max_affected_messages, int(message.affected_messages)
        )
        self.fault_types[str(message.fault_type)] += 1

    def report(self):
        return {
            "events": self.events,
            "active_events": self.active_events,
            "timestamp_repaired_events": self.timestamp_repaired_events,
            "max_timestamp_repairs": self.max_timestamp_repairs,
            "max_affected_messages": self.max_affected_messages,
            "fault_types": dict(self.fault_types),
        }


def numeric_summary(samples):
    if not samples:
        return None
    values = sorted(value for _, value in samples)
    index_95 = min(len(values) - 1, math.ceil(0.95 * len(values)) - 1)
    middle = len(values) // 2
    median = (
        values[middle]
        if len(values) % 2
        else 0.5 * (values[middle - 1] + values[middle])
    )
    return {
        "count": len(values),
        "min": values[0],
        "median": median,
        "p95": values[index_95],
        "max": values[-1],
    }


def parse_named_counts(value):
    """Parse a stable ``name:count`` diagnostic field."""
    text = str(value).strip()
    if not text or text == "none":
        return {}
    counts = {}
    for entry in text.split(","):
        name, separator, count_text = entry.rpartition(":")
        if not separator or not name:
            raise ValueError(f"invalid named count entry: {entry}")
        count = int(count_text)
        if count < 0:
            raise ValueError(f"negative named count: {entry}")
        counts[name] = count
    return counts


def parse_diagnostic_bool(value):
    text = str(value).strip().lower()
    if text in ("true", "1", "yes"):
        return True
    if text in ("false", "0", "no"):
        return False
    raise ValueError(f"invalid diagnostic boolean: {value}")


def calibration_diagnostic_sample(values, stamp_s, mission_phase):
    """Convert backend calibration diagnostics into a typed timeline sample."""
    if "calibration_reason" not in values:
        return None
    sample = {
        "stamp_s": float(stamp_s),
        "mission_phase": str(mission_phase),
    }
    for key in (
        "calibration_reason",
        "calibration_motion_reason",
        "calibration_mode",
        "calibration_time_candidate_reason",
    ):
        if key in values:
            sample[key] = str(values[key])
    for key in (
        "calibration_motion_received",
        "calibration_motion_rejected",
        "calibration_updates",
        "calibration_accepted",
        "calibration_pair_count",
        "calibration_time_candidate_pairs",
    ):
        try:
            sample[key] = int(values[key])
        except (KeyError, TypeError, ValueError):
            continue
    for key in (
        "calibration_time_offset_s",
        "calibration_time_correlation",
        "calibration_time_margin",
        "calibration_rotation_residual_rad",
        "calibration_time_candidate_offset_s",
        "calibration_excitation_ratio",
        "calibration_accumulated_rotation_rad",
        "calibration_unweighted_accumulated_rotation_rad",
        "calibration_weighted_accumulated_rotation_rad",
        "calibration_imu_accumulated_rotation_rad",
        "calibration_motion_weight_mean",
        "calibration_rotation_inlier_ratio",
    ):
        try:
            value = float(values[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            sample[key] = value
    for key in (
        "calibration_time_candidate_valid",
        "calibration_time_locked",
        "calibration_rotation_locked",
        "calibration_locked",
    ):
        try:
            sample[key] = parse_diagnostic_bool(values[key])
        except (KeyError, TypeError, ValueError):
            continue
    eigenvalue_text = values.get("calibration_excitation_eigenvalues")
    if eigenvalue_text is not None:
        try:
            eigenvalues = [
                float(value) for value in str(eigenvalue_text).split(",")
            ]
        except ValueError:
            eigenvalues = []
        if eigenvalues and all(math.isfinite(value) for value in eigenvalues):
            sample["calibration_excitation_eigenvalues"] = eigenvalues
    return sample


def scheduler_phase_summary(timeline):
    phases = {}
    for entry in timeline:
        phase = str(entry.get("mission_phase", "unreported"))
        current = phases.setdefault(phase, {
            "sample_count": 0,
            "first_stamp_s": None,
            "last_stamp_s": None,
            "health_states": Counter(),
            "factor_samples": Counter(),
            "factor_enabled": Counter(),
            "degradation_sum": defaultdict(float),
            "estimator_support_sum": 0.0,
        })
        current["sample_count"] += 1
        stamp_s = float(entry["stamp_s"])
        if current["first_stamp_s"] is None:
            current["first_stamp_s"] = stamp_s
        current["last_stamp_s"] = stamp_s
        current["health_states"][str(entry["health_state"])] += 1
        current["estimator_support_sum"] += float(entry["estimator_support"])
        for name, enabled in entry.get("factor_enabled", {}).items():
            current["factor_samples"][str(name)] += 1
            current["factor_enabled"][str(name)] += int(bool(enabled))
        for name, score in entry.get("degradation_scores", {}).items():
            current["degradation_sum"][str(name)] += float(score)

    output = {}
    for phase, current in phases.items():
        count = current["sample_count"]
        output[phase] = {
            "sample_count": count,
            "first_stamp_s": current["first_stamp_s"],
            "last_stamp_s": current["last_stamp_s"],
            "duration_s": (
                current["last_stamp_s"] - current["first_stamp_s"]
                if count > 1 else 0.0
            ),
            "health_states": dict(current["health_states"]),
            "factor_enabled_ratio": {
                name: current["factor_enabled"][name] / sample_count
                for name, sample_count in current["factor_samples"].items()
                if sample_count
            },
            "degradation_score_mean": {
                name: total / count
                for name, total in current["degradation_sum"].items()
                if count
            },
            "estimator_support_mean": (
                current["estimator_support_sum"] / count if count else None
            ),
        }
    return output


class CalibrationMotionStats:
    """Summarize the independent raw-scan calibration branch by mission phase."""

    def __init__(self):
        self.phases = {}

    @staticmethod
    def _bucket():
        return {
            "count": 0,
            "accepted": 0,
            "reasons": Counter(),
            "interval_s": [],
            "rotation_angle_rad": [],
            "translation_m": [],
            "quality_weight": [],
        }

    @staticmethod
    def _report_bucket(bucket):
        count = int(bucket["count"])
        return {
            "count": count,
            "accepted": int(bucket["accepted"]),
            "accepted_ratio": bucket["accepted"] / count if count else None,
            "reasons": dict(bucket["reasons"]),
            "accepted_interval_s": numeric_summary(bucket["interval_s"]),
            "accepted_rotation_angle_rad": numeric_summary(
                bucket["rotation_angle_rad"]
            ),
            "accepted_translation_m": numeric_summary(bucket["translation_m"]),
            "accepted_quality_weight": numeric_summary(bucket["quality_weight"]),
        }

    def add(self, message, mission_phase):
        phase = str(mission_phase)
        bucket = self.phases.setdefault(phase, self._bucket())
        bucket["count"] += 1
        bucket["reasons"][str(message.reason)] += 1
        if not bool(message.accepted) or not bool(message.converged):
            return
        start_s = (
            float(message.start_stamp.sec)
            + float(message.start_stamp.nanosec) * 1.0e-9
        )
        end_s = stamp_ns(message) * 1.0e-9
        quaternion = message.relative_rotation
        quaternion_values = [
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
        ]
        norm = math.sqrt(sum(value * value for value in quaternion_values))
        translation = message.relative_translation
        translation_m = math.sqrt(
            float(translation.x) ** 2
            + float(translation.y) ** 2
            + float(translation.z) ** 2
        )
        residual_rms_m = float(message.residual_rms_m)
        inlier_ratio = float(message.inlier_ratio)
        values = (start_s, end_s, norm, translation_m, residual_rms_m, inlier_ratio)
        if not all(math.isfinite(value) for value in values) or norm <= 1.0e-12:
            return
        rotation_angle = 2.0 * math.acos(
            min(1.0, abs(quaternion_values[3] / norm))
        )
        quality_weight = min(
            1.0,
            max(0.05, inlier_ratio * math.exp(-residual_rms_m / 0.15)),
        )
        sample_index = int(bucket["accepted"])
        bucket["accepted"] += 1
        bucket["interval_s"].append((sample_index, end_s - start_s))
        bucket["rotation_angle_rad"].append((sample_index, rotation_angle))
        bucket["translation_m"].append((sample_index, translation_m))
        bucket["quality_weight"].append((sample_index, quality_weight))

    def report(self):
        total = self._bucket()
        for bucket in self.phases.values():
            total["count"] += bucket["count"]
            total["accepted"] += bucket["accepted"]
            total["reasons"].update(bucket["reasons"])
            for key in (
                "interval_s",
                "rotation_angle_rad",
                "translation_m",
                "quality_weight",
            ):
                total[key].extend(bucket[key])
        report = self._report_bucket(total)
        report["phase_summary"] = {
            phase: self._report_bucket(bucket)
            for phase, bucket in self.phases.items()
        }
        return report


class Metrics(Node):
    def __init__(self):
        super().__init__("unified_externalnav_metrics")
        self.declare_parameter("external_nav_topic", "/mavros/odometry/out")
        external_nav_topic = self.get_parameter("external_nav_topic").value
        self.streams = {"unified_odom": StreamStats(), "externalnav_out": StreamStats()}
        self.sensor_streams = {
            "gnss": HeaderStreamStats(),
            "optical_flow": HeaderStreamStats(),
        }
        self.reliability_streams = {
            "gnss": ReliabilityScoreStats(),
            "optical_flow": ReliabilityScoreStats(),
        }
        self.fault_states = defaultdict(FaultStateStats)
        self.sensor_contract_latest = {}
        self.mission_phase = "unreported"
        self.mission_phase_timeline = []
        self.states = Counter()
        self.enabled = Counter()
        self.samples = Counter()
        self.capability_sum = defaultdict(float)
        self.capability_count = Counter()
        self.support = []
        self.reasons = Counter()
        self.externalnav_gate_latest = {}
        self.backend_latest = {}
        self.started_wall_s = time.monotonic()
        self.started_ros_s = None
        self.last_ros_s = None
        self.backend_numeric = defaultdict(list)
        self.covariance_sources = Counter()
        self.backend_diagnostic_messages = 0
        self.relocalization_states = Counter()
        self.relocalization_successes = 0
        self.fusion_epoch_events = []
        self.calibration_motion_stats = CalibrationMotionStats()
        self.calibration_timeline = []
        self.last_calibration_timeline_s = None
        self.last_calibration_signature = None
        self.scheduler_timeline = []
        self.last_scheduler_timeline_ns = None
        self.last_scheduler_signature = None
        self.optimization_integrity_reasons = Counter()
        self.optimization_integrity_count_source = "diagnostic_sample"
        self.scheduler_clock_domain_violations = 0
        self.scheduler_clock_domain_deferred = 0
        self.scheduler_clock_domain_max_error_s = 0.0
        self.scheduler_clock_domain_examples = []
        self.graph_contract_violations = []
        self.last_graph_contract_check_wall_s = None
        self.create_subscription(
            Odometry, "/fusion/unified/odom",
            lambda m: self.odom_stream("unified_odom", m), 50
        )
        self.create_subscription(
            Odometry, external_nav_topic,
            lambda m: self.odom_stream("externalnav_out", m), 50
        )
        self.create_subscription(
            NavSatFix,
            "/sensors/gnss/fix",
            lambda m: self.sensor_stream("gnss", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OpticalFlowRad,
            "/sensors/optical_flow/rad",
            lambda m: self.sensor_stream("optical_flow", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ReliabilityScore,
            "/reliability/gnss_score",
            lambda m: self.reliability_score("gnss", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ReliabilityScore,
            "/reliability/optical_flow_score",
            lambda m: self.reliability_score("optical_flow", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FaultState, "/fault/state", self.fault_state, qos_profile_sensor_data
        )
        self.create_subscription(
            DiagnosticArray,
            "/sensor_contract/diagnostics",
            self.sensor_contract,
            10,
        )
        self.create_subscription(String, "/mission/phase", self.mission_phase_event, 10)
        self.create_subscription(SchedulerState, "/reliability/scheduler_state", self.scheduler, 20)
        self.create_subscription(DiagnosticArray, "/external_nav/diagnostics", self.diagnostics, 10)
        self.create_subscription(DiagnosticArray, "/fusion/unified/diagnostics", self.diagnostics, 10)
        self.create_subscription(RelocalizationResult, "/relocalization/result", self.relocalization, 10)
        epoch_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            FusionEpoch, "/fusion/unified/epoch", self.fusion_epoch, epoch_qos
        )
        self.create_subscription(
            LidarCalibrationMotion,
            "/calibration/lidar_relative_motion",
            self.calibration_motion,
            qos_profile_sensor_data,
        )

    def scheduler(self, message):
        source_ns = stamp_ns(message)
        now_ns = self.get_clock().now().nanoseconds
        clock_error_s = scheduler_clock_domain_error_s(source_ns, now_ns)
        if clock_error_s is None:
            self.scheduler_clock_domain_deferred += 1
            return
        if not math.isfinite(clock_error_s) or clock_error_s > 1.0:
            self.scheduler_clock_domain_violations += 1
            self.scheduler_clock_domain_max_error_s = max(
                self.scheduler_clock_domain_max_error_s, clock_error_s
            )
            if len(self.scheduler_clock_domain_examples) < 8:
                self.scheduler_clock_domain_examples.append({
                    "source_stamp_s": source_ns * 1.0e-9,
                    "observer_stamp_s": now_ns * 1.0e-9,
                    "absolute_error_s": clock_error_s,
                    "health_state": str(message.health_state),
                })
            return
        self.states[message.health_state] += 1
        for name, flag in zip(message.modality_names, message.factor_enabled):
            self.samples[name] += 1
            self.enabled[name] += int(flag)
        for name, value in zip(message.capability_names, message.capability_support):
            self.capability_count[name] += 1
            self.capability_sum[name] += float(value)
        self.support.append(float(message.estimator_support))
        signature = (
            str(message.health_state),
            tuple(bool(value) for value in message.factor_enabled),
            tuple(bool(value) for value in message.capability_observable),
            bool(message.relocalization_requested),
        )
        periodic_due = (
            self.last_scheduler_timeline_ns is None
            or source_ns - self.last_scheduler_timeline_ns >= 500_000_000
        )
        if source_ns > 0 and (
            periodic_due or signature != self.last_scheduler_signature
        ):
            self.scheduler_timeline.append({
                "stamp_ns": source_ns,
                "stamp_s": source_ns * 1.0e-9,
                "health_state": str(message.health_state),
                "estimator_support": float(message.estimator_support),
                "degradation_scores": {
                    name: float(value) for name, value in zip(
                        message.modality_names, message.degradation_scores
                    )
                },
                "factor_enabled": {
                    name: bool(value) for name, value in zip(
                        message.modality_names, message.factor_enabled
                    )
                },
                "reasons": {
                    name: str(value) for name, value in zip(
                        message.modality_names, message.reasons
                    )
                },
                "capability_support": {
                    name: float(value) for name, value in zip(
                        message.capability_names, message.capability_support
                    )
                },
                "capability_observable": {
                    name: bool(value) for name, value in zip(
                        message.capability_names, message.capability_observable
                    )
                },
                "relocalization_requested": bool(
                    message.relocalization_requested
                ),
                "mission_phase": self.mission_phase,
            })
            self.last_scheduler_timeline_ns = source_ns
            self.last_scheduler_signature = signature

    def odom_stream(self, name, message):
        if name not in self.streams:
            return
        self.streams[name].add(message, self.get_clock().now().nanoseconds)

    def sensor_stream(self, name, message):
        if name not in self.sensor_streams:
            return
        self.sensor_streams[name].add(
            message, self.get_clock().now().nanoseconds
        )

    def reliability_score(self, name, message):
        if name not in self.reliability_streams:
            return
        self.reliability_streams[name].add(
            message, self.get_clock().now().nanoseconds
        )

    def fault_state(self, message):
        self.fault_states[str(message.modality)].add(message)

    def sensor_contract(self, message):
        for status in message.status:
            if not status.name.startswith("sensor_contract/"):
                continue
            modality = status.name.split("/", 1)[1]
            self.sensor_contract_latest[modality] = {
                "level": diagnostic_level(status.level),
                "message": str(status.message),
                "values": {
                    item.key: str(item.value) for item in status.values
                },
            }

    def mission_phase_event(self, message):
        phase = str(message.data).strip() or "unreported"
        if phase == self.mission_phase:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        self.mission_phase = phase
        self.mission_phase_timeline.append({
            "stamp_ns": int(now_ns),
            "stamp_s": int(now_ns) * 1.0e-9,
            "phase": phase,
        })

    @staticmethod
    def _endpoint_identity(endpoint):
        return {
            "node_name": str(endpoint.node_name),
            "node_namespace": str(endpoint.node_namespace),
            "topic_type": str(endpoint.topic_type),
            "endpoint_gid": list(endpoint.endpoint_gid),
        }

    def graph_contract_valid(self):
        now_wall_s = time.monotonic()
        if (
            self.last_graph_contract_check_wall_s is not None
            and now_wall_s - self.last_graph_contract_check_wall_s < 1.0
        ):
            return not self.graph_contract_violations
        self.last_graph_contract_check_wall_s = now_wall_s
        contracts = {
            "/clock": 1,
            "/reliability/scheduler_state": 1,
        }
        for topic, expected_count in contracts.items():
            endpoints = self.get_publishers_info_by_topic(topic)
            if len(endpoints) == expected_count:
                continue
            if len(self.graph_contract_violations) < 8:
                self.graph_contract_violations.append({
                    "topic": topic,
                    "expected_publishers": expected_count,
                    "observed_publishers": len(endpoints),
                    "endpoints": [
                        self._endpoint_identity(endpoint)
                        for endpoint in endpoints
                    ],
                })
            return False
        return True

    def observe_ros_time(self, now_ros_s=None):
        if now_ros_s is None:
            now_ros_s = self.get_clock().now().nanoseconds * 1.0e-9
        if now_ros_s <= 0.0:
            return False
        if self.last_ros_s is not None and now_ros_s < self.last_ros_s:
            raise RuntimeError("ROS simulation clock moved backwards")
        self.last_ros_s = now_ros_s
        if self.started_ros_s is None:
            self.started_ros_s = now_ros_s
        return True

    def diagnostics(self, message):
        for status in message.status:
            if status.name == "external_nav/gate":
                self.reasons[status.message] += 1
                self.externalnav_gate_latest = {
                    item.key: item.value for item in status.values
                }
            elif status.name == "unified_backend_fusion":
                self.backend_diagnostic_messages += 1
                self.backend_latest = {
                    item.key: item.value for item in status.values
                }
                now_ros_s = self.get_clock().now().nanoseconds * 1.0e-9
                if not self.observe_ros_time(now_ros_s):
                    continue
                elapsed = now_ros_s - self.started_ros_s
                for key in (
                    "backend_solve_ms", "backend_marginalization_ms",
                    "callback_ms", "backend_cost",
                    "lidar_prediction_position_innovation_m",
                    "lidar_prediction_yaw_innovation_rad",
                ):
                    try:
                        value = float(self.backend_latest[key])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if math.isfinite(value):
                        self.backend_numeric[key].append((elapsed, value))
                source = self.backend_latest.get("covariance_source")
                if source:
                    self.covariance_sources[source] += 1
                calibration_sample = calibration_diagnostic_sample(
                    self.backend_latest, now_ros_s, self.mission_phase
                )
                if calibration_sample is not None:
                    signature = (
                        calibration_sample.get("calibration_reason"),
                        calibration_sample.get("calibration_time_candidate_reason"),
                        calibration_sample.get("calibration_pair_count"),
                        calibration_sample.get("calibration_time_locked"),
                        calibration_sample.get("calibration_rotation_locked"),
                        calibration_sample.get("calibration_locked"),
                    )
                    periodic_due = (
                        self.last_calibration_timeline_s is None
                        or now_ros_s - self.last_calibration_timeline_s >= 0.5
                    )
                    if periodic_due or signature != self.last_calibration_signature:
                        self.calibration_timeline.append(calibration_sample)
                        self.last_calibration_timeline_s = now_ros_s
                        self.last_calibration_signature = signature
                integrity_counts = self.backend_latest.get(
                    "optimization_integrity_counts"
                )
                if integrity_counts is not None:
                    try:
                        parsed = parse_named_counts(integrity_counts)
                    except (TypeError, ValueError):
                        parsed = None
                    if parsed is not None:
                        self.optimization_integrity_reasons = Counter(parsed)
                        self.optimization_integrity_count_source = (
                            "backend_cumulative"
                        )
                elif self.optimization_integrity_count_source != (
                    "backend_cumulative"
                ):
                    integrity_reason = self.backend_latest.get(
                        "optimization_integrity_reason"
                    )
                    if integrity_reason:
                        self.optimization_integrity_reasons[
                            integrity_reason
                        ] += 1

    def relocalization(self, message):
        self.relocalization_states[message.state_name] += 1
        self.relocalization_successes += int(message.accepted)

    def fusion_epoch(self, message):
        event_stamp_ns = stamp_ns(message)
        self.fusion_epoch_events.append({
            "stamp_ns": event_stamp_ns,
            "stamp_s": event_stamp_ns * 1.0e-9,
            "applied": bool(message.applied),
            "session_id": int(message.session_id),
            "transaction_id": int(message.transaction_id),
            "reset_counter": int(message.reset_counter),
            "candidate_id": int(message.candidate_id),
            "reason": str(message.reason),
        })

    def calibration_motion(self, message):
        self.calibration_motion_stats.add(message, self.mission_phase)

    def report(self, now_ros_s=None):
        if now_ros_s is None:
            now_ros_s = self.get_clock().now().nanoseconds * 1.0e-9
        sim_duration_s = (
            0.0
            if self.started_ros_s is None or now_ros_s <= 0.0
            else max(0.0, now_ros_s - self.started_ros_s)
        )
        first_threshold_crossing = {}
        for key, threshold in (
            ("callback_ms", 100.0),
            ("lidar_prediction_position_innovation_m", 0.5),
            ("lidar_prediction_yaw_innovation_rad", 0.5),
        ):
            first_threshold_crossing[key] = next((
                elapsed for elapsed, value in self.backend_numeric[key]
                if abs(value) > threshold
            ), None)
        epoch_continuity = []
        for event in self.fusion_epoch_events:
            if not event["applied"]:
                continue
            epoch_continuity.append({
                "session_id": event["session_id"],
                "reset_counter": event["reset_counter"],
                "transaction_id": event["transaction_id"],
                "candidate_id": event["candidate_id"],
                "reason": event["reason"],
                "stamp_s": event["stamp_s"],
                "streams": {
                    name: stats.epoch_continuity(event["stamp_ns"])
                    for name, stats in self.streams.items()
                },
            })
        return {
            "sim_duration_s": sim_duration_s,
            "wall_duration_s": time.monotonic() - self.started_wall_s,
            "algorithm_clock": "ros_sim_time",
            "performance_clock": "wall_monotonic",
            "streams": {name: value.report() for name, value in self.streams.items()},
            "sensor_streams": {
                name: value.report()
                for name, value in self.sensor_streams.items()
            },
            "reliability_streams": {
                name: value.report()
                for name, value in self.reliability_streams.items()
            },
            "fault_injector": {
                name: value.report() for name, value in self.fault_states.items()
            },
            "sensor_contract_latest": self.sensor_contract_latest,
            "scheduler_states": dict(self.states),
            "scheduler_timeline": self.scheduler_timeline,
            "scheduler_phase_summary": scheduler_phase_summary(
                self.scheduler_timeline
            ),
            "mission_phase_timeline": self.mission_phase_timeline,
            "scheduler_clock_domain_violations": (
                self.scheduler_clock_domain_violations
            ),
            "scheduler_clock_domain_deferred": (
                self.scheduler_clock_domain_deferred
            ),
            "scheduler_clock_domain_max_error_s": (
                self.scheduler_clock_domain_max_error_s
            ),
            "scheduler_clock_domain_examples": (
                self.scheduler_clock_domain_examples
            ),
            "graph_contract_violations": self.graph_contract_violations,
            "factor_enabled_ratio": {name: self.enabled[name] / count for name, count in self.samples.items() if count},
            "capability_support_mean": {name: self.capability_sum[name] / count for name, count in self.capability_count.items() if count},
            "estimator_support_mean": sum(self.support) / len(self.support) if self.support else None,
            "estimator_support_min": min(self.support) if self.support else None,
            "externalnav_diagnostic_reasons": dict(self.reasons),
            "externalnav_gate_latest": self.externalnav_gate_latest,
            "backend_diagnostic_messages": self.backend_diagnostic_messages,
            "backend_latest": self.backend_latest,
            "calibration_timeline": self.calibration_timeline,
            "calibration_motion": self.calibration_motion_stats.report(),
            "backend_numeric_summary": {
                key: numeric_summary(samples)
                for key, samples in self.backend_numeric.items()
            },
            "backend_first_threshold_crossing_s": first_threshold_crossing,
            "optimization_integrity_reasons": dict(
                self.optimization_integrity_reasons
            ),
            "optimization_integrity_count_source": (
                self.optimization_integrity_count_source
            ),
            "covariance_sources": dict(self.covariance_sources),
            "relocalization_states": dict(self.relocalization_states),
            "relocalization_successes": self.relocalization_successes,
            "fusion_epoch_events": self.fusion_epoch_events,
            "fusion_epoch_continuity": epoch_continuity,
            "fusion_epoch_applied": sum(
                int(event["applied"]) for event in self.fusion_epoch_events
            ),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--wall-timeout", type=float, default=0.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = Metrics()
    started_ros_s = None
    last_ros_s = None
    started_wall_s = time.monotonic()
    wall_timeout = (
        args.wall_timeout if args.wall_timeout > 0.0
        else max(60.0, args.duration * 10.0)
    )
    termination_reason = "duration_complete"
    pending_error = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now_ros_s = node.get_clock().now().nanoseconds * 1.0e-9
            if now_ros_s <= 0.0:
                if time.monotonic() - started_wall_s >= wall_timeout:
                    raise RuntimeError(
                        "wall watchdog expired waiting for ROS simulation time"
                    )
                continue
            if last_ros_s is not None and now_ros_s < last_ros_s:
                raise RuntimeError("ROS simulation clock moved backwards")
            last_ros_s = now_ros_s
            if started_ros_s is None:
                started_ros_s = now_ros_s
            node.observe_ros_time(now_ros_s)
            if time.monotonic() - started_wall_s >= 3.0:
                if not node.graph_contract_valid():
                    raise RuntimeError("ROS graph publisher contract violated")
            if node.scheduler_clock_domain_violations:
                raise RuntimeError(
                    "scheduler state arrived from a different ROS clock domain"
                )
            if now_ros_s - started_ros_s >= args.duration:
                break
            if time.monotonic() - started_wall_s >= wall_timeout:
                raise RuntimeError(
                    "wall watchdog expired waiting for simulation time"
                )
        else:
            termination_reason = "ros_shutdown"
    except (KeyboardInterrupt, ExternalShutdownException):
        termination_reason = "interrupted"
    except Exception as error:  # Preserve partial evidence before failing.
        termination_reason = f"error:{type(error).__name__}:{error}"
        pending_error = error
    finally:
        report = node.report(last_ros_s)
        report["termination_reason"] = termination_reason
        with open(args.output, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(report, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if pending_error is not None:
        raise pending_error


if __name__ == "__main__":
    main()
