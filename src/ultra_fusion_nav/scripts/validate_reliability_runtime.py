#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, Imu, NavSatFix, NavSatStatus
from uf_interfaces.msg import LioDiagnostics, ReliabilityScore


MODALITIES = ("lidar", "gnss", "imu", "optical_flow", "vision")


class ReliabilityProbe(Node):
    def __init__(self):
        super().__init__(
            "reliability_validation_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.lidar_pub = self.create_publisher(LioDiagnostics, "/lio/diagnostics", 20)
        self.odom_pub = self.create_publisher(Odometry, "/lio/odom", 20)
        self.gnss_pub = self.create_publisher(NavSatFix, "/sensors/gnss/fix", 20)
        self.imu_pub = self.create_publisher(Imu, "/sensors/imu", 50)
        self.flow_pub = self.create_publisher(OpticalFlowRad, "/sensors/optical_flow/rad", 20)
        self.depth_pub = self.create_publisher(Image, "/sensors/rgbd/depth", 20)
        self.color_pub = self.create_publisher(Image, "/sensors/rgbd/color", 20)
        self.samples = {modality: [] for modality in MODALITIES}
        self.valid_samples = {modality: [] for modality in MODALITIES}
        self.weight_samples = {modality: [] for modality in MODALITIES}
        self.coverage_samples = {modality: [] for modality in MODALITIES}
        self.invalid_header_stamp_counts = {modality: 0 for modality in MODALITIES}
        for modality in MODALITIES:
            self.create_subscription(
                ReliabilityScore, f"/reliability/{modality}_score",
                lambda msg, key=modality: self._score(msg, key), 20
            )
        self.sequence = 0

    def _score(self, msg, modality):
        stamp_s = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )
        if stamp_s <= 0.0:
            self.invalid_header_stamp_counts[modality] += 1
            return
        evidence = dict(zip(msg.evidence_names, msg.evidence_values))
        self.samples[modality].append(float(msg.degradation_score))
        self.valid_samples[modality].append(1.0 if msg.valid else 0.0)
        self.weight_samples[modality].append(float(msg.reliability_weight))
        self.coverage_samples[modality].append(float(evidence.get("evidence_weight_coverage", -1.0)))

    def _stamp(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "validation"

    def publish_set(self, degraded):
        self.sequence += 1
        lidar = LioDiagnostics()
        self._stamp(lidar)
        lidar.input_points = 1200
        lidar.matched_points = 80 if degraded else 1000
        lidar.hessian_eigenvalues = ([1e-8, 1e-8, 1e-6, 10.0, 20.0, 30.0]
                                      if degraded else [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        lidar.hessian_condition = 3e9 if degraded else 6.0
        lidar.normal_covariance_eigenvalues = [0.0, 0.0, 1.0] if degraded else [0.1, 0.2, 0.7]
        lidar.axial_penalty = 0.8 if degraded else 0.0
        lidar.approximate = True
        lidar.source = "validation"
        self.lidar_pub.publish(lidar)

        odom = Odometry()
        self._stamp(odom)
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        gnss = NavSatFix()
        self._stamp(gnss)
        gnss.status.status = NavSatStatus.STATUS_FIX
        gnss.status.service = NavSatStatus.SERVICE_GPS
        offset = 0.001 if degraded and self.sequence % 2 else 0.0
        gnss.latitude = 31.0 + offset
        gnss.longitude = 121.0 - offset
        gnss.altitude = 10.0
        gnss.position_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.gnss_pub.publish(gnss)

        imu = Imu()
        self._stamp(imu)
        sign = -1.0 if self.sequence % 2 else 1.0
        if degraded:
            imu.linear_acceleration.x = 60.0
            imu.angular_velocity.z = 12.0
        else:
            imu.linear_acceleration.x = sign
            imu.linear_acceleration.z = 9.80665
            imu.angular_velocity.z = 0.35 * sign
        self.imu_pub.publish(imu)

        flow = OpticalFlowRad()
        self._stamp(flow)
        flow.integration_time_us = 50000
        flow.integrated_x = 0.20 if degraded else 0.001
        flow.integrated_y = 0.20 if degraded else 0.001
        flow.quality = 5 if degraded else 220
        flow.distance = 3.0
        self.flow_pub.publish(flow)

        depth = Image()
        self._stamp(depth)
        depth.height = 32
        depth.width = 32
        depth.encoding = "32FC1"
        depth.step = depth.width * 4
        depth_values = np.zeros((32, 32), dtype=np.float32) if degraded else np.full((32, 32), 3.0, dtype=np.float32)
        depth.data = depth_values.tobytes()
        self.depth_pub.publish(depth)

        color = Image()
        self._stamp(color)
        color.height = 64
        color.width = 64
        color.encoding = "rgb8"
        color.step = color.width * 3
        if degraded:
            pixels = np.full((64, 64, 3), 127, dtype=np.uint8)
        else:
            checker = ((np.indices((64, 64)).sum(axis=0) // 4) % 2 * 255).astype(np.uint8)
            pixels = np.repeat(checker[:, :, None], 3, axis=2)
        color.data = pixels.tobytes()
        self.color_pub.publish(color)


def run_phase(node, degraded, duration, wall_timeout_s, publish_rate_hz):
    for sample_group in (
        node.samples, node.valid_samples, node.weight_samples, node.coverage_samples
    ):
        for values in sample_group.values():
            values.clear()
    wall_started = time.monotonic()
    last_progress_wall = wall_started
    ros_started_s = None
    last_ros_s = None
    next_publish_ros_s = None
    period_s = 1.0 / max(1.0, publish_rate_hz)
    elapsed_ros_s = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        now_ros_s = node.get_clock().now().nanoseconds * 1.0e-9
        now_wall = time.monotonic()
        if now_ros_s > 0.0 and ros_started_s is None:
            ros_started_s = now_ros_s
            next_publish_ros_s = now_ros_s
        if last_ros_s is not None and now_ros_s < last_ros_s:
            raise RuntimeError("ROS clock moved backwards during reliability validation")
        if last_ros_s is None or now_ros_s > last_ros_s:
            last_progress_wall = now_wall
        last_ros_s = now_ros_s
        elapsed_ros_s = (
            now_ros_s - ros_started_s if ros_started_s is not None else 0.0
        )
        elapsed_wall_s = now_wall - wall_started
        if elapsed_ros_s >= duration:
            break
        stalled_wall_s = now_wall - last_progress_wall
        if stalled_wall_s >= wall_timeout_s:
            raise RuntimeError(
                f"ROS clock stalled for {stalled_wall_s:.1f}s "
                f"after advancing {elapsed_ros_s:.1f}s"
            )
        if (
            next_publish_ros_s is not None
            and now_ros_s + 1.0e-12 >= next_publish_ros_s
        ):
            node.publish_set(degraded)
            next_publish_ros_s = now_ros_s + period_s
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.0)
    scores = {
        key: float(np.median(values[-20:])) if values else math.nan
        for key, values in node.samples.items()
    }
    evidence = {}
    for key in MODALITIES:
        evidence[key] = {
            "valid_rate": float(np.mean(node.valid_samples[key][-20:])) if node.valid_samples[key] else math.nan,
            "weight_median": float(np.median(node.weight_samples[key][-20:])) if node.weight_samples[key] else math.nan,
            "coverage_median": float(np.median(node.coverage_samples[key][-20:])) if node.coverage_samples[key] else math.nan,
        }
    return scores, evidence, {
        "duration_s": elapsed_ros_s,
        "duration_ros_s": elapsed_ros_s,
        "duration_wall_s": time.monotonic() - wall_started,
        "wall_timeout_s": wall_timeout_s,
        "wall_stall_timeout_s": wall_timeout_s,
        "publish_rate_ros_hz": publish_rate_hz,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--phase-duration", type=float, default=4.0)
    parser.add_argument("--publish-rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--wall-timeout-per-phase",
        type=float,
        default=0.0,
        help="wall seconds without ROS-clock progress; 0 selects a conservative limit",
    )
    args = parser.parse_args()
    rclpy.init()
    node = ReliabilityProbe()
    phase_wall_timeout_s = (
        args.wall_timeout_per_phase
        if args.wall_timeout_per_phase > 0.0
        else max(args.phase_duration * 10.0, args.phase_duration + 60.0)
    )
    _, _, warmup_timing = run_phase(
        node, False, 1.0, phase_wall_timeout_s, args.publish_rate_hz
    )
    healthy, healthy_evidence, healthy_timing = run_phase(
        node,
        False,
        args.phase_duration,
        phase_wall_timeout_s,
        args.publish_rate_hz,
    )
    degraded, degraded_evidence, degraded_timing = run_phase(
        node,
        True,
        args.phase_duration,
        phase_wall_timeout_s,
        args.publish_rate_hz,
    )
    deltas = {key: degraded[key] - healthy[key] for key in MODALITIES}
    direction_passed = all(
        math.isfinite(deltas[key]) and deltas[key] >= 0.15 for key in MODALITIES
    )
    complete_modalities = ("lidar", "gnss")
    incomplete_modalities = ("imu", "optical_flow", "vision")
    evidence_passed = all(
        healthy_evidence[key]["valid_rate"] >= 0.95
        and degraded_evidence[key]["valid_rate"] >= 0.95
        for key in complete_modalities
    ) and all(
        healthy_evidence[key]["valid_rate"] <= 0.05
        and degraded_evidence[key]["valid_rate"] <= 0.05
        and abs(healthy_evidence[key]["weight_median"]) <= 1.0e-6
        and abs(degraded_evidence[key]["weight_median"]) <= 1.0e-6
        for key in incomplete_modalities
    )
    passed = direction_passed and evidence_passed
    result = {
        "healthy": healthy,
        "degraded": degraded,
        "delta": deltas,
        "evidence": {"healthy": healthy_evidence, "degraded": degraded_evidence},
        "timing": {
            "warmup": warmup_timing,
            "healthy": healthy_timing,
            "degraded": degraded_timing,
        },
        "invalid_header_stamp_counts": dict(node.invalid_header_stamp_counts),
        "direction_passed": direction_passed,
        "evidence_policy_passed": evidence_passed,
        "passed": passed,
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
