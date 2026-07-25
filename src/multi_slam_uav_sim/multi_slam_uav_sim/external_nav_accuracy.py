import bisect
import json
import math
import os
import threading
import time
from collections import deque

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np
import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ExternalNavAccuracy(Node):
    """Evaluate ExternalNav against Gazebo truth without feeding truth to the estimator."""

    def __init__(self):
        super().__init__("external_nav_accuracy")
        self.declare_parameter("odom_topic", "/fusion/gps_flow/odom")
        self.declare_parameter("world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("model_name", "apm_iris")
        self.declare_parameter("output_path", "")
        self.declare_parameter("maximum_pose_gap_s", 0.20)
        self.declare_parameter("rpe_interval_s", 1.0)
        self.declare_parameter("minimum_motion_speed_mps", 0.03)
        self.declare_parameter("maximum_samples", 30000)
        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.maximum_pose_gap = float(self.get_parameter("maximum_pose_gap_s").value)
        self.rpe_interval = float(self.get_parameter("rpe_interval_s").value)
        self.minimum_motion_speed = float(
            self.get_parameter("minimum_motion_speed_mps").value)
        maximum_samples = int(self.get_parameter("maximum_samples").value)
        self.lock = threading.Lock()
        self.truth_samples = deque(maxlen=maximum_samples * 3)
        self.odom_samples = deque(maxlen=maximum_samples)
        self.started_s = time.monotonic()
        self.last_report = None
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom, 20)
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            Pose_V, f"/world/{self.world_name}/dynamic_pose/info", self._gz_pose)
        self.gz_node.subscribe(
            Pose_V, f"/world/{self.world_name}/pose/info", self._gz_pose)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"ExternalNav accuracy evaluator active: odom={self.get_parameter('odom_topic').value}, "
            f"truth=Gazebo/{self.model_name}, report={self.output_path or 'log_only'}")

    def _odom(self, msg):
        now = time.monotonic()
        point = msg.pose.pose.position
        with self.lock:
            self.odom_samples.append((now, float(point.x), float(point.y), float(point.z)))

    def _gz_pose(self, msg):
        now = time.monotonic()
        for pose in msg.pose:
            if pose.name == self.model_name or pose.name.endswith(f"::{self.model_name}"):
                with self.lock:
                    if not self.truth_samples or now > self.truth_samples[-1][0]:
                        self.truth_samples.append((
                            now,
                            float(pose.position.x),
                            float(pose.position.y),
                            float(pose.position.z),
                        ))
                return

    def _truth_at(self, samples, times, stamp_s):
        index = bisect.bisect_left(times, stamp_s)
        if index == 0 or index >= len(samples):
            return None
        before = samples[index - 1]
        after = samples[index]
        interval = after[0] - before[0]
        if interval <= 0.0 or interval > self.maximum_pose_gap:
            return None
        ratio = (stamp_s - before[0]) / interval
        return np.asarray(before[1:4], dtype=float) + ratio * (
            np.asarray(after[1:4], dtype=float) - np.asarray(before[1:4], dtype=float)
        )

    @staticmethod
    def _align_xy(estimate, truth):
        estimate_mean = np.mean(estimate, axis=0)
        truth_mean = np.mean(truth, axis=0)
        estimate_centered = estimate - estimate_mean
        truth_centered = truth - truth_mean
        u, _, vt = np.linalg.svd(estimate_centered.T @ truth_centered)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T
        translation = truth_mean - rotation @ estimate_mean
        aligned = (rotation @ estimate.T).T + translation
        denominator = float(np.sum(estimate_centered * estimate_centered))
        rotated_centered = (rotation @ estimate_centered.T).T
        scale = (
            float(np.sum(rotated_centered * truth_centered)) / denominator
            if denominator > 1.0e-12 else 0.0
        )
        return aligned, rotation, translation, scale

    def _calculate(self):
        with self.lock:
            truth_samples = list(self.truth_samples)
            odom_samples = list(self.odom_samples)
        if len(truth_samples) < 2 or len(odom_samples) < 20:
            return None
        truth_times = [sample[0] for sample in truth_samples]
        matched_times = []
        estimate = []
        truth = []
        for stamp_s, x, y, z in odom_samples:
            reference = self._truth_at(truth_samples, truth_times, stamp_s)
            if reference is None:
                continue
            matched_times.append(stamp_s)
            estimate.append((x, y, z))
            truth.append(reference)
        if len(estimate) < 20:
            return None
        estimate = np.asarray(estimate, dtype=float)
        truth = np.asarray(truth, dtype=float)
        aligned_xy, rotation, translation, scale = self._align_xy(
            estimate[:, :2], truth[:, :2])
        z_offset = float(np.mean(truth[:, 2] - estimate[:, 2]))
        aligned = np.column_stack((aligned_xy, estimate[:, 2] + z_offset))
        errors = np.linalg.norm(aligned - truth, axis=1)

        motion_mask = np.zeros(len(truth), dtype=bool)
        for index in range(1, len(truth)):
            dt = matched_times[index] - matched_times[index - 1]
            if dt > 0.0:
                speed = float(np.linalg.norm(truth[index] - truth[index - 1]) / dt)
                motion_mask[index] = speed >= self.minimum_motion_speed
        motion_errors = errors[motion_mask]

        rpe_errors = []
        for index, stamp_s in enumerate(matched_times):
            target = bisect.bisect_left(matched_times, stamp_s + self.rpe_interval)
            if target >= len(matched_times):
                continue
            if abs(matched_times[target] - stamp_s - self.rpe_interval) > 0.15:
                continue
            estimate_delta_xy = rotation @ (
                estimate[target, :2] - estimate[index, :2])
            truth_delta_xy = truth[target, :2] - truth[index, :2]
            estimate_delta_z = estimate[target, 2] - estimate[index, 2]
            truth_delta_z = truth[target, 2] - truth[index, 2]
            rpe_errors.append(math.sqrt(
                float(np.sum((estimate_delta_xy - truth_delta_xy) ** 2))
                + float((estimate_delta_z - truth_delta_z) ** 2)
            ))
        rpe_errors = np.asarray(rpe_errors, dtype=float)
        return {
            "schema_version": 1,
            "wall_duration_s": time.monotonic() - self.started_s,
            "matched_samples": len(errors),
            "motion_samples": int(np.count_nonzero(motion_mask)),
            "alignment": {
                "yaw_deg": math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])),
                "translation_x_m": float(translation[0]),
                "translation_y_m": float(translation[1]),
                "translation_z_m": z_offset,
                "diagnostic_scale_not_applied": scale,
            },
            "ate": {
                "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
                "median_m": float(np.median(errors)),
                "p95_m": float(np.percentile(errors, 95)),
                "max_m": float(np.max(errors)),
                "motion_rmse_m": (
                    float(np.sqrt(np.mean(motion_errors ** 2)))
                    if motion_errors.size else None
                ),
            },
            "rpe_1s": {
                "samples": len(rpe_errors),
                "rmse_m": (
                    float(np.sqrt(np.mean(rpe_errors ** 2)))
                    if rpe_errors.size else None
                ),
                "p95_m": float(np.percentile(rpe_errors, 95)) if rpe_errors.size else None,
            },
            "truth_used_by_estimator": False,
        }

    def _write(self, report):
        if not self.output_path:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        temporary = self.output_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, self.output_path)

    def _report(self):
        report = self._calculate()
        if report is None:
            return
        self.last_report = report
        self._write(report)
        self.get_logger().info(
            "EXTERNAL_NAV_ACCURACY "
            f"samples={report['matched_samples']} "
            f"ate_rmse={report['ate']['rmse_m']:.3f}m "
            f"motion_rmse={report['ate']['motion_rmse_m']} "
            f"rpe_1s={report['rpe_1s']['rmse_m']} "
            f"scale={report['alignment']['diagnostic_scale_not_applied']:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = ExternalNavAccuracy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
