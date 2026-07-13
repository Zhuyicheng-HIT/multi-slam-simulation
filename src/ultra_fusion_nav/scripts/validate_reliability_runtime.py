#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from mavros_msgs.msg import OpticalFlowRad
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix, NavSatStatus
from uf_interfaces.msg import LioDiagnostics, ReliabilityScore


MODALITIES = ("lidar", "gnss", "imu", "optical_flow", "vision")


class ReliabilityProbe(Node):
    def __init__(self):
        super().__init__("reliability_validation_probe")
        self.lidar_pub = self.create_publisher(LioDiagnostics, "/lio/diagnostics", 20)
        self.gnss_pub = self.create_publisher(NavSatFix, "/sensors/gnss/fix", 20)
        self.imu_pub = self.create_publisher(Imu, "/sensors/imu", 50)
        self.flow_pub = self.create_publisher(OpticalFlowRad, "/sensors/optical_flow/rad", 20)
        self.depth_pub = self.create_publisher(Image, "/sensors/rgbd/depth", 20)
        self.color_pub = self.create_publisher(Image, "/sensors/rgbd/color", 20)
        self.samples = {modality: [] for modality in MODALITIES}
        for modality in MODALITIES:
            self.create_subscription(
                ReliabilityScore, f"/reliability/{modality}_score",
                lambda msg, key=modality: self.samples[key].append(float(msg.degradation_score)), 20
            )
        self.sequence = 0

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


def run_phase(node, degraded, duration):
    for values in node.samples.values():
        values.clear()
    started = time.monotonic()
    while time.monotonic() - started < duration:
        node.publish_set(degraded)
        rclpy.spin_once(node, timeout_sec=0.03)
        time.sleep(0.02)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.03)
    return {
        key: float(np.median(values[-20:])) if values else math.nan
        for key, values in node.samples.items()
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--phase-duration", type=float, default=4.0)
    args = parser.parse_args()
    rclpy.init()
    node = ReliabilityProbe()
    time.sleep(1.0)
    healthy = run_phase(node, False, args.phase_duration)
    degraded = run_phase(node, True, args.phase_duration)
    deltas = {key: degraded[key] - healthy[key] for key in MODALITIES}
    passed = all(math.isfinite(deltas[key]) and deltas[key] >= 0.15 for key in MODALITIES)
    result = {"healthy": healthy, "degraded": degraded, "delta": deltas, "passed": passed}
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
