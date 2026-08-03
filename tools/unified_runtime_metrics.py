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
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from uf_interfaces.msg import RelocalizationResult, SchedulerState


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class StreamStats:
    def __init__(self):
        self.wall_arrivals = []
        self.source_times = []
        self.positions = []
        self.last_stamp = None
        self.regressions = 0
        self.duplicates = 0
        self.zero_stamps = 0

    def add(self, message):
        self.wall_arrivals.append(time.monotonic())
        stamp = stamp_ns(message)
        if stamp <= 0:
            self.zero_stamps += 1
            return
        self.source_times.append(stamp * 1.0e-9)
        self.positions.append((
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        ))
        if self.last_stamp is not None:
            self.regressions += int(stamp < self.last_stamp)
            self.duplicates += int(stamp == self.last_stamp)
        self.last_stamp = stamp

    def report(self):
        gaps = [b - a for a, b in zip(self.source_times, self.source_times[1:])]
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
            "max_gap_s": max(gaps) if gaps else None,
            "gaps_over_0_25_s": sum(gap > 0.25 for gap in gaps),
            "stamp_regressions": self.regressions,
            "stamp_duplicates": self.duplicates,
            "zero_stamps": self.zero_stamps,
            "max_displacement_from_first_m": max(displacement) if displacement else None,
            "first_displacement_over_5m_s": first_over_five,
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


class Metrics(Node):
    def __init__(self):
        super().__init__("unified_externalnav_metrics")
        self.streams = {"unified_odom": StreamStats(), "externalnav_out": StreamStats()}
        self.states = Counter()
        self.enabled = Counter()
        self.samples = Counter()
        self.capability_sum = defaultdict(float)
        self.capability_count = Counter()
        self.support = []
        self.reasons = Counter()
        self.backend_latest = {}
        self.started_wall_s = time.monotonic()
        self.started_ros_s = None
        self.last_ros_s = None
        self.backend_numeric = defaultdict(list)
        self.covariance_sources = Counter()
        self.backend_diagnostic_messages = 0
        self.relocalization_states = Counter()
        self.relocalization_successes = 0
        self.create_subscription(Odometry, "/fusion/unified/odom", lambda m: self.streams["unified_odom"].add(m), 50)
        self.create_subscription(Odometry, "/mavros/odometry/out", lambda m: self.streams["externalnav_out"].add(m), 50)
        self.create_subscription(SchedulerState, "/reliability/scheduler_state", self.scheduler, 20)
        self.create_subscription(DiagnosticArray, "/external_nav/diagnostics", self.diagnostics, 10)
        self.create_subscription(DiagnosticArray, "/fusion/unified/diagnostics", self.diagnostics, 10)
        self.create_subscription(RelocalizationResult, "/relocalization/result", self.relocalization, 10)

    def scheduler(self, message):
        self.states[message.health_state] += 1
        for name, flag in zip(message.modality_names, message.factor_enabled):
            self.samples[name] += 1
            self.enabled[name] += int(flag)
        for name, value in zip(message.capability_names, message.capability_support):
            self.capability_count[name] += 1
            self.capability_sum[name] += float(value)
        self.support.append(float(message.estimator_support))

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

    def relocalization(self, message):
        self.relocalization_states[message.state_name] += 1
        self.relocalization_successes += int(message.accepted)

    def report(self):
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
        return {
            "sim_duration_s": sim_duration_s,
            "wall_duration_s": time.monotonic() - self.started_wall_s,
            "algorithm_clock": "ros_sim_time",
            "performance_clock": "wall_monotonic",
            "streams": {name: value.report() for name, value in self.streams.items()},
            "scheduler_states": dict(self.states),
            "factor_enabled_ratio": {name: self.enabled[name] / count for name, count in self.samples.items() if count},
            "capability_support_mean": {name: self.capability_sum[name] / count for name, count in self.capability_count.items() if count},
            "estimator_support_mean": sum(self.support) / len(self.support) if self.support else None,
            "estimator_support_min": min(self.support) if self.support else None,
            "externalnav_diagnostic_reasons": dict(self.reasons),
            "backend_diagnostic_messages": self.backend_diagnostic_messages,
            "backend_latest": self.backend_latest,
            "backend_numeric_summary": {
                key: numeric_summary(samples)
                for key, samples in self.backend_numeric.items()
            },
            "backend_first_threshold_crossing_s": first_threshold_crossing,
            "covariance_sources": dict(self.covariance_sources),
            "relocalization_states": dict(self.relocalization_states),
            "relocalization_successes": self.relocalization_successes,
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
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        now_ros_s = node.get_clock().now().nanoseconds * 1.0e-9
        if now_ros_s <= 0.0:
            if time.monotonic() - started_wall_s >= wall_timeout:
                raise RuntimeError("wall watchdog expired waiting for ROS simulation time")
            continue
        if last_ros_s is not None and now_ros_s < last_ros_s:
            raise RuntimeError("ROS simulation clock moved backwards")
        last_ros_s = now_ros_s
        if started_ros_s is None:
            started_ros_s = now_ros_s
        node.observe_ros_time(now_ros_s)
        if now_ros_s - started_ros_s >= args.duration:
            break
        if time.monotonic() - started_wall_s >= wall_timeout:
            raise RuntimeError("wall watchdog expired waiting for simulation time")
    report = node.report()
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(report, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
