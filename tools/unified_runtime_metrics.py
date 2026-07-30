#!/usr/bin/env python3
import argparse
import json
import time
from collections import Counter, defaultdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from uf_interfaces.msg import SchedulerState


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class StreamStats:
    def __init__(self):
        self.arrivals = []
        self.last_stamp = None
        self.regressions = 0
        self.duplicates = 0

    def add(self, message):
        self.arrivals.append(time.monotonic())
        stamp = stamp_ns(message)
        if self.last_stamp is not None:
            self.regressions += int(stamp < self.last_stamp)
            self.duplicates += int(stamp == self.last_stamp)
        self.last_stamp = stamp

    def report(self):
        gaps = [b - a for a, b in zip(self.arrivals, self.arrivals[1:])]
        duration = self.arrivals[-1] - self.arrivals[0] if len(self.arrivals) > 1 else 0.0
        return {
            "count": len(self.arrivals),
            "rate_hz": (len(self.arrivals) - 1) / duration if duration else 0.0,
            "max_gap_s": max(gaps) if gaps else None,
            "gaps_over_0_25_s": sum(gap > 0.25 for gap in gaps),
            "stamp_regressions": self.regressions,
            "stamp_duplicates": self.duplicates,
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
        self.create_subscription(Odometry, "/fusion/unified/odom", lambda m: self.streams["unified_odom"].add(m), 50)
        self.create_subscription(Odometry, "/mavros/odometry/out", lambda m: self.streams["externalnav_out"].add(m), 50)
        self.create_subscription(SchedulerState, "/reliability/scheduler_state", self.scheduler, 20)
        self.create_subscription(DiagnosticArray, "/external_nav/diagnostics", self.diagnostics, 10)

    def scheduler(self, message):
        self.states[message.health_state] += 1
        for name, flag in zip(message.modality_names, message.factor_enabled):
            self.samples[name] += 1
            self.enabled[name] += int(flag)
        for name, value in zip(message.capability_names, message.capability_support):
            self.capability_count[name] += 1
            self.capability_sum[name] += float(value)
        self.support.append(float(message.estimator_support))

    def diagnostics(self, message):
        for status in message.status:
            if status.name == "external_nav/gate":
                self.reasons[status.message] += 1

    def report(self):
        return {
            "streams": {name: value.report() for name, value in self.streams.items()},
            "scheduler_states": dict(self.states),
            "factor_enabled_ratio": {name: self.enabled[name] / count for name, count in self.samples.items() if count},
            "capability_support_mean": {name: self.capability_sum[name] / count for name, count in self.capability_count.items() if count},
            "estimator_support_mean": sum(self.support) / len(self.support) if self.support else None,
            "estimator_support_min": min(self.support) if self.support else None,
            "externalnav_diagnostic_reasons": dict(self.reasons),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=125.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Metrics()
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    report = node.report()
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(report, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
