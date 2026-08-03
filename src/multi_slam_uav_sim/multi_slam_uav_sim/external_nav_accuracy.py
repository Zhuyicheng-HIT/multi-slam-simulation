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
        self.started_wall_s = time.monotonic()
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
        stamp_s = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )
        if stamp_s <= 0.0:
            return
        point = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self._quaternion_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w)
        with self.lock:
            self.odom_samples.append(
                (stamp_s, float(point.x), float(point.y), float(point.z), yaw)
            )

    def _gz_pose(self, msg):
        try:
            stamp_s = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nsec) * 1.0e-9
            )
        except Exception:
            return
        if stamp_s <= 0.0:
            return
        for pose in msg.pose:
            if pose.name == self.model_name or pose.name.endswith(f"::{self.model_name}"):
                with self.lock:
                    if not self.truth_samples or stamp_s > self.truth_samples[-1][0]:
                        self.truth_samples.append((
                            stamp_s,
                            float(pose.position.x),
                            float(pose.position.y),
                            float(pose.position.z),
                            self._quaternion_yaw(
                                pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w,
                            ),
                        ))
                return

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _quaternion_yaw(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm <= 1.0e-12:
            return 0.0
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

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
        position = np.asarray(before[1:4], dtype=float) + ratio * (
            np.asarray(after[1:4], dtype=float) - np.asarray(before[1:4], dtype=float)
        )
        yaw = before[4] + ratio * self._wrap_angle(after[4] - before[4])
        return position, self._wrap_angle(yaw)

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
        estimate_yaw = []
        truth_yaw = []
        for stamp_s, x, y, z, yaw in odom_samples:
            reference = self._truth_at(truth_samples, truth_times, stamp_s)
            if reference is None:
                continue
            reference_position, reference_yaw = reference
            matched_times.append(stamp_s)
            estimate.append((x, y, z))
            truth.append(reference_position)
            estimate_yaw.append(yaw)
            truth_yaw.append(reference_yaw)
        if len(estimate) < 20:
            return None
        estimate = np.asarray(estimate, dtype=float)
        truth = np.asarray(truth, dtype=float)
        aligned_xy, rotation, translation, scale = self._align_xy(
            estimate[:, :2], truth[:, :2])
        z_offset = float(np.mean(truth[:, 2] - estimate[:, 2]))
        aligned = np.column_stack((aligned_xy, estimate[:, 2] + z_offset))
        errors = np.linalg.norm(aligned - truth, axis=1)
        alignment_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        yaw_errors = np.asarray([
            self._wrap_angle(estimate_value + alignment_yaw - truth_value)
            for estimate_value, truth_value in zip(estimate_yaw, truth_yaw)
        ], dtype=float)

        motion_mask = np.zeros(len(truth), dtype=bool)
        for index in range(1, len(truth)):
            dt = matched_times[index] - matched_times[index - 1]
            if dt > 0.0:
                speed = float(np.linalg.norm(truth[index] - truth[index - 1]) / dt)
                motion_mask[index] = speed >= self.minimum_motion_speed
        motion_errors = errors[motion_mask]
        motion_yaw_errors = yaw_errors[motion_mask]

        turning_mask = np.zeros(len(truth), dtype=bool)
        for index in range(1, len(truth_yaw)):
            dt = matched_times[index] - matched_times[index - 1]
            if dt > 0.0:
                yaw_rate = abs(self._wrap_angle(
                    truth_yaw[index] - truth_yaw[index - 1])) / dt
                turning_mask[index] = yaw_rate >= math.radians(5.0)
        turning_yaw_errors = yaw_errors[turning_mask]

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
        yaw_rpe_errors = []
        for index, stamp_s in enumerate(matched_times):
            target = bisect.bisect_left(matched_times, stamp_s + self.rpe_interval)
            if target >= len(matched_times):
                continue
            if abs(matched_times[target] - stamp_s - self.rpe_interval) > 0.15:
                continue
            estimate_delta = self._wrap_angle(
                estimate_yaw[target] - estimate_yaw[index])
            truth_delta = self._wrap_angle(truth_yaw[target] - truth_yaw[index])
            yaw_rpe_errors.append(self._wrap_angle(estimate_delta - truth_delta))
        yaw_rpe_errors = np.asarray(yaw_rpe_errors, dtype=float)
        yaw_abs_deg = np.degrees(np.abs(yaw_errors))
        return {
            "schema_version": 2,
            "sim_duration_s": matched_times[-1] - matched_times[0],
            "wall_duration_s": time.monotonic() - self.started_wall_s,
            "association_basis": "source_header_stamp",
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
            "yaw": {
                "samples": len(yaw_errors),
                "rmse_deg": float(math.degrees(
                    math.sqrt(float(np.mean(yaw_errors ** 2))))),
                "median_abs_deg": float(np.median(yaw_abs_deg)),
                "p95_abs_deg": float(np.percentile(yaw_abs_deg, 95)),
                "max_abs_deg": float(np.max(yaw_abs_deg)),
                "motion_rmse_deg": (
                    float(math.degrees(math.sqrt(
                        float(np.mean(motion_yaw_errors ** 2)))))
                    if motion_yaw_errors.size else None
                ),
                "turning_samples": int(np.count_nonzero(turning_mask)),
                "turning_rmse_deg": (
                    float(math.degrees(math.sqrt(
                        float(np.mean(turning_yaw_errors ** 2)))))
                    if turning_yaw_errors.size else None
                ),
            },
            "yaw_rpe_1s": {
                "samples": len(yaw_rpe_errors),
                "rmse_deg": (
                    float(math.degrees(math.sqrt(
                        float(np.mean(yaw_rpe_errors ** 2)))))
                    if yaw_rpe_errors.size else None
                ),
                "p95_abs_deg": (
                    float(np.percentile(np.degrees(np.abs(yaw_rpe_errors)), 95))
                    if yaw_rpe_errors.size else None
                ),
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
            f"yaw_rmse={report['yaw']['rmse_deg']:.3f}deg "
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
