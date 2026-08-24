import bisect
import csv
import json
import math
import os
import threading
import time
from collections import deque

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class ExternalNavAccuracy(Node):
    """Evaluate ExternalNav against truth.

    Truth remains confined to this evaluator and is not published to the
    estimator.
    """

    def __init__(self):
        super().__init__("external_nav_accuracy")
        self.declare_parameter("odom_topic", "/fusion/gps_flow/odom")
        self.declare_parameter("world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("model_name", "apm_iris")
        self.declare_parameter("truth_odom_topic", "")
        self.declare_parameter("truth_odom_qos_reliability", "best_effort")
        self.declare_parameter("truth_odom_qos_depth", 20)
        self.declare_parameter("output_path", "")
        self.declare_parameter("samples_output_path", "")
        self.declare_parameter("maximum_pose_gap_s", 0.20)
        self.declare_parameter("rpe_interval_s", 1.0)
        self.declare_parameter("minimum_motion_speed_mps", 0.03)
        self.declare_parameter("initial_alignment_duration_s", 10.0)
        self.declare_parameter("acceptance_threshold_m", 0.20)
        self.declare_parameter("maximum_sustained_exceedance_s", 0.50)
        self.declare_parameter("maximum_samples", 30000)
        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.truth_odom_topic = str(
            self.get_parameter("truth_odom_topic").value).strip()
        self.output_path = str(self.get_parameter("output_path").value)
        self.samples_output_path = str(
            self.get_parameter("samples_output_path").value).strip()
        if not self.samples_output_path and self.output_path:
            output_root, _ = os.path.splitext(self.output_path)
            self.samples_output_path = output_root + ".samples.csv"
        self.maximum_pose_gap = float(
            self.get_parameter("maximum_pose_gap_s").value)
        self.rpe_interval = float(self.get_parameter("rpe_interval_s").value)
        self.minimum_motion_speed = float(
            self.get_parameter("minimum_motion_speed_mps").value)
        self.initial_alignment_duration = max(
            0.5,
            float(self.get_parameter("initial_alignment_duration_s").value),
        )
        self.acceptance_threshold = max(
            0.0, float(self.get_parameter("acceptance_threshold_m").value))
        self.maximum_sustained_exceedance = max(
            0.0,
            float(self.get_parameter("maximum_sustained_exceedance_s").value),
        )
        maximum_samples = int(self.get_parameter("maximum_samples").value)
        self.lock = threading.Lock()
        self.truth_samples = deque(maxlen=maximum_samples * 3)
        self.odom_samples = deque(maxlen=maximum_samples)
        self.started_wall_s = time.monotonic()
        self.last_report = None
        self.last_causal_samples = []
        self.dynamic_truth_seen = False
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom,
            20,
        )
        self.gz_node = None
        if self.truth_odom_topic:
            truth_qos = self._truth_subscription_qos(
                str(self.get_parameter("truth_odom_qos_reliability").value),
                int(self.get_parameter("truth_odom_qos_depth").value),
            )
            self.create_subscription(
                Odometry, self.truth_odom_topic, self._truth_odom, truth_qos)
        else:
            self.gz_node = GzNode()
            self.gz_node.subscribe(
                Pose_V,
                f"/world/{self.world_name}/dynamic_pose/info",
                self._gz_dynamic_pose,
            )
            self.gz_node.subscribe(
                Pose_V,
                f"/world/{self.world_name}/pose/info",
                self._gz_fallback_pose,
            )
        self.create_timer(5.0, self._report)
        odom_topic = self.get_parameter("odom_topic").value
        self.get_logger().info(
            f"ExternalNav accuracy evaluator active: odom={odom_topic}, "
            f"truth={self.truth_odom_topic or ('Gazebo/' + self.model_name)}, "
            f"report={self.output_path or 'log_only'}")

    @staticmethod
    def _truth_subscription_qos(reliability, depth):
        reliability_name = str(reliability).strip().lower()
        reliability_policies = {
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
            "reliable": ReliabilityPolicy.RELIABLE,
            "system_default": ReliabilityPolicy.SYSTEM_DEFAULT,
        }
        if reliability_name not in reliability_policies:
            raise ValueError(
                "truth_odom_qos_reliability must be best_effort, reliable, "
                "or system_default"
            )
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(depth)),
            reliability=reliability_policies[reliability_name],
            durability=DurabilityPolicy.VOLATILE,
        )

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

    def _gz_dynamic_pose(self, msg):
        self._gz_pose(msg, dynamic_source=True)

    def _gz_fallback_pose(self, msg):
        self._gz_pose(msg, dynamic_source=False)

    @staticmethod
    def _accept_truth_source(dynamic_truth_seen, dynamic_source):
        return bool(dynamic_source or not dynamic_truth_seen)

    def _gz_pose(self, msg, *, dynamic_source):
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
            if (
                pose.name == self.model_name
                or pose.name.endswith(f"::{self.model_name}")
            ):
                with self.lock:
                    if not self._accept_truth_source(
                        self.dynamic_truth_seen, dynamic_source
                    ):
                        return
                    if (
                        not self.truth_samples
                        or stamp_s > self.truth_samples[-1][0]
                    ):
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
                        self.dynamic_truth_seen = (
                            self.dynamic_truth_seen or dynamic_source
                        )
                return

    def _truth_odom(self, msg):
        stamp_s = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )
        if stamp_s <= 0.0:
            return
        point = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        sample = (
            stamp_s,
            float(point.x),
            float(point.y),
            float(point.z),
            self._quaternion_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        with self.lock:
            if not self.truth_samples or stamp_s > self.truth_samples[-1][0]:
                self.truth_samples.append(sample)

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
            np.asarray(after[1:4], dtype=float)
            - np.asarray(before[1:4], dtype=float)
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

    @staticmethod
    def _circular_mean(angles):
        angles = np.asarray(angles, dtype=float)
        return math.atan2(
            float(np.mean(np.sin(angles))),
            float(np.mean(np.cos(angles))),
        )

    @classmethod
    def _initial_pose_alignment(
            cls, estimate, truth, estimate_yaw, truth_yaw,
            matched_times, duration_s):
        """Align from the initial interval without using future motion."""
        times = np.asarray(matched_times, dtype=float)
        cutoff = float(times[0]) + float(duration_s)
        indices = np.flatnonzero(times <= cutoff)
        if indices.size < 3:
            indices = np.arange(min(3, len(times)))
        yaw_offsets = np.asarray([
            cls._wrap_angle(reference - measured)
            for measured, reference in zip(
                np.asarray(estimate_yaw)[indices],
                np.asarray(truth_yaw)[indices],
            )
        ])
        yaw_offset = cls._circular_mean(yaw_offsets)
        rotation = np.asarray([
            [math.cos(yaw_offset), -math.sin(yaw_offset)],
            [math.sin(yaw_offset), math.cos(yaw_offset)],
        ])
        rotated_xy = (rotation @ estimate[:, :2].T).T
        translation_xy = np.mean(
            truth[indices, :2] - rotated_xy[indices], axis=0)
        z_offset = float(np.mean(
            truth[indices, 2] - estimate[indices, 2]))
        aligned = np.column_stack((
            rotated_xy + translation_xy,
            estimate[:, 2] + z_offset,
        ))
        return (
            aligned,
            rotation,
            translation_xy,
            z_offset,
            yaw_offset,
            int(indices.size),
        )

    @staticmethod
    def _position_error_summary(aligned, truth):
        residual = (
            np.asarray(aligned, dtype=float) - np.asarray(truth, dtype=float))
        horizontal = np.linalg.norm(residual[:, :2], axis=1)
        vertical = np.abs(residual[:, 2])
        three_dimensional = np.linalg.norm(residual, axis=1)

        def summary(values):
            return {
                "rmse_m": float(np.sqrt(np.mean(values ** 2))),
                "median_m": float(np.median(values)),
                "p95_m": float(np.percentile(values, 95)),
                "max_m": float(np.max(values)),
            }

        return {
            "three_dimensional": summary(three_dimensional),
            "horizontal": summary(horizontal),
            "vertical": summary(vertical),
            "axis_rmse_m": {
                "x": float(np.sqrt(np.mean(residual[:, 0] ** 2))),
                "y": float(np.sqrt(np.mean(residual[:, 1] ** 2))),
                "z": float(np.sqrt(np.mean(residual[:, 2] ** 2))),
            },
            "endpoint_error_m": {
                "x": float(residual[-1, 0]),
                "y": float(residual[-1, 1]),
                "z": float(residual[-1, 2]),
                "norm": float(three_dimensional[-1]),
            },
        }

    @staticmethod
    def _threshold_exceedance_summary(
            matched_times, errors, threshold_m, maximum_gap_s):
        times = np.asarray(matched_times, dtype=float)
        errors = np.asarray(errors, dtype=float)
        over_threshold = errors > float(threshold_m)
        maximum_duration_s = 0.0
        total_duration_s = 0.0
        run_start_s = None
        previous_s = None
        first_exceedance_s = None
        last_exceedance_s = None

        for stamp_s, exceeds in zip(times, over_threshold):
            stamp_s = float(stamp_s)
            if not exceeds:
                run_start_s = None
                previous_s = None
                continue
            if first_exceedance_s is None:
                first_exceedance_s = stamp_s
            last_exceedance_s = stamp_s
            if (
                run_start_s is None
                or previous_s is None
                or stamp_s - previous_s > float(maximum_gap_s)
            ):
                run_start_s = stamp_s
            else:
                total_duration_s += stamp_s - previous_s
            previous_s = stamp_s
            maximum_duration_s = max(
                maximum_duration_s, stamp_s - run_start_s)

        return {
            "threshold_m": float(threshold_m),
            "sample_count": int(np.count_nonzero(over_threshold)),
            "sample_ratio": float(np.mean(over_threshold)),
            "total_duration_s": float(total_duration_s),
            "maximum_contiguous_duration_s": float(maximum_duration_s),
            "first_stamp_s": first_exceedance_s,
            "last_stamp_s": last_exceedance_s,
        }

    @staticmethod
    def _position_rpe_errors(
            estimate, truth, matched_times, rotation, interval_s):
        errors = []
        for index, stamp_s in enumerate(matched_times):
            target = bisect.bisect_left(
                matched_times, stamp_s + float(interval_s))
            if target >= len(matched_times):
                continue
            if abs(matched_times[target] - stamp_s - interval_s) > 0.15:
                continue
            estimate_delta_xy = rotation @ (
                estimate[target, :2] - estimate[index, :2])
            truth_delta_xy = truth[target, :2] - truth[index, :2]
            estimate_delta_z = estimate[target, 2] - estimate[index, 2]
            truth_delta_z = truth[target, 2] - truth[index, 2]
            errors.append(math.sqrt(
                float(np.sum((estimate_delta_xy - truth_delta_xy) ** 2))
                + float((estimate_delta_z - truth_delta_z) ** 2)
            ))
        return np.asarray(errors, dtype=float)

    @classmethod
    def _yaw_rpe_errors(
            cls, estimate_yaw, truth_yaw, matched_times, interval_s):
        errors = []
        for index, stamp_s in enumerate(matched_times):
            target = bisect.bisect_left(
                matched_times, stamp_s + float(interval_s))
            if target >= len(matched_times):
                continue
            if abs(matched_times[target] - stamp_s - interval_s) > 0.15:
                continue
            estimate_delta = cls._wrap_angle(
                estimate_yaw[target] - estimate_yaw[index])
            truth_delta = cls._wrap_angle(
                truth_yaw[target] - truth_yaw[index])
            errors.append(cls._wrap_angle(estimate_delta - truth_delta))
        return np.asarray(errors, dtype=float)

    @staticmethod
    def _rpe_summary(errors):
        errors = np.asarray(errors, dtype=float)
        return {
            "samples": len(errors),
            "rmse_m": (
                float(np.sqrt(np.mean(errors ** 2))) if errors.size else None
            ),
            "p95_m": float(np.percentile(errors, 95)) if errors.size else None,
        }

    @staticmethod
    def _yaw_error_summary(yaw_errors, motion_mask, turning_mask):
        yaw_errors = np.asarray(yaw_errors, dtype=float)
        yaw_abs_deg = np.degrees(np.abs(yaw_errors))
        motion_errors = yaw_errors[motion_mask]
        turning_errors = yaw_errors[turning_mask]
        return {
            "samples": len(yaw_errors),
            "rmse_deg": float(math.degrees(
                math.sqrt(float(np.mean(yaw_errors ** 2))))),
            "median_abs_deg": float(np.median(yaw_abs_deg)),
            "p95_abs_deg": float(np.percentile(yaw_abs_deg, 95)),
            "max_abs_deg": float(np.max(yaw_abs_deg)),
            "motion_rmse_deg": (
                float(math.degrees(math.sqrt(
                    float(np.mean(motion_errors ** 2)))))
                if motion_errors.size else None
            ),
            "turning_samples": int(np.count_nonzero(turning_mask)),
            "turning_rmse_deg": (
                float(math.degrees(math.sqrt(
                    float(np.mean(turning_errors ** 2)))))
                if turning_errors.size else None
            ),
        }

    @staticmethod
    def _yaw_rpe_summary(errors):
        errors = np.asarray(errors, dtype=float)
        return {
            "samples": len(errors),
            "rmse_deg": (
                float(math.degrees(math.sqrt(float(np.mean(errors ** 2)))))
                if errors.size else None
            ),
            "p95_abs_deg": (
                float(np.percentile(np.degrees(np.abs(errors)), 95))
                if errors.size else None
            ),
        }

    @staticmethod
    def _causal_sample_rows(
            matched_times, estimate, causal_aligned, truth,
            causal_yaw_errors, threshold_m):
        residual = causal_aligned - truth
        horizontal = np.linalg.norm(residual[:, :2], axis=1)
        three_dimensional = np.linalg.norm(residual, axis=1)
        rows = []
        for index, stamp_s in enumerate(matched_times):
            rows.append({
                "stamp_s": float(stamp_s),
                "estimate_raw_x_m": float(estimate[index, 0]),
                "estimate_raw_y_m": float(estimate[index, 1]),
                "estimate_raw_z_m": float(estimate[index, 2]),
                "estimate_aligned_x_m": float(causal_aligned[index, 0]),
                "estimate_aligned_y_m": float(causal_aligned[index, 1]),
                "estimate_aligned_z_m": float(causal_aligned[index, 2]),
                "truth_x_m": float(truth[index, 0]),
                "truth_y_m": float(truth[index, 1]),
                "truth_z_m": float(truth[index, 2]),
                "error_x_m": float(residual[index, 0]),
                "error_y_m": float(residual[index, 1]),
                "error_z_m": float(residual[index, 2]),
                "horizontal_error_m": float(horizontal[index]),
                "error_3d_m": float(three_dimensional[index]),
                "yaw_error_deg": float(math.degrees(causal_yaw_errors[index])),
                "above_threshold": int(
                    three_dimensional[index] > float(threshold_m)),
            })
        return rows

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
        (
            causal_aligned,
            causal_rotation,
            causal_translation,
            causal_z_offset,
            causal_yaw_offset,
            causal_alignment_samples,
        ) = self._initial_pose_alignment(
            estimate,
            truth,
            estimate_yaw,
            truth_yaw,
            matched_times,
            self.initial_alignment_duration,
        )
        causal_position_errors = np.linalg.norm(causal_aligned - truth, axis=1)
        causal_error = self._position_error_summary(causal_aligned, truth)
        threshold_exceedance = self._threshold_exceedance_summary(
            matched_times,
            causal_position_errors,
            self.acceptance_threshold,
            self.maximum_pose_gap,
        )
        causal_error["threshold_exceedance"] = threshold_exceedance
        alignment_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        legacy_yaw_errors = np.asarray([
            self._wrap_angle(estimate_value + alignment_yaw - truth_value)
            for estimate_value, truth_value in zip(estimate_yaw, truth_yaw)
        ], dtype=float)
        causal_yaw_errors = np.asarray([
            self._wrap_angle(
                estimate_value + causal_yaw_offset - truth_value)
            for estimate_value, truth_value in zip(estimate_yaw, truth_yaw)
        ], dtype=float)

        motion_mask = np.zeros(len(truth), dtype=bool)
        for index in range(1, len(truth)):
            dt = matched_times[index] - matched_times[index - 1]
            if dt > 0.0:
                speed = float(
                    np.linalg.norm(truth[index] - truth[index - 1]) / dt)
                motion_mask[index] = speed >= self.minimum_motion_speed
        motion_errors = errors[motion_mask]
        causal_motion_errors = causal_position_errors[motion_mask]
        causal_error["motion_rmse_m"] = (
            float(np.sqrt(np.mean(causal_motion_errors ** 2)))
            if causal_motion_errors.size else None
        )

        turning_mask = np.zeros(len(truth), dtype=bool)
        for index in range(1, len(truth_yaw)):
            dt = matched_times[index] - matched_times[index - 1]
            if dt > 0.0:
                yaw_rate = abs(self._wrap_angle(
                    truth_yaw[index] - truth_yaw[index - 1])) / dt
                turning_mask[index] = yaw_rate >= math.radians(5.0)
        causal_rpe_errors = self._position_rpe_errors(
            estimate, truth, matched_times, causal_rotation, self.rpe_interval)
        legacy_rpe_errors = self._position_rpe_errors(
            estimate, truth, matched_times, rotation, self.rpe_interval)
        causal_yaw_rpe_errors = self._yaw_rpe_errors(
            estimate_yaw, truth_yaw, matched_times, self.rpe_interval)
        legacy_yaw_rpe_errors = causal_yaw_rpe_errors.copy()
        self.last_causal_samples = self._causal_sample_rows(
            matched_times,
            estimate,
            causal_aligned,
            truth,
            causal_yaw_errors,
            self.acceptance_threshold,
        )

        causal_yaw_summary = self._yaw_error_summary(
            causal_yaw_errors, motion_mask, turning_mask)
        legacy_yaw_summary = self._yaw_error_summary(
            legacy_yaw_errors, motion_mask, turning_mask)
        causal_rpe_summary = self._rpe_summary(causal_rpe_errors)
        legacy_rpe_summary = self._rpe_summary(legacy_rpe_errors)
        causal_yaw_rpe_summary = self._yaw_rpe_summary(causal_yaw_rpe_errors)
        legacy_yaw_rpe_summary = self._yaw_rpe_summary(legacy_yaw_rpe_errors)
        acceptance_gates = {
            "three_dimensional_rmse_below_threshold": (
                causal_error["three_dimensional"]["rmse_m"]
                < self.acceptance_threshold
            ),
            "three_dimensional_p95_below_threshold": (
                causal_error["three_dimensional"]["p95_m"]
                < self.acceptance_threshold
            ),
            "endpoint_below_threshold": (
                causal_error["endpoint_error_m"]["norm"]
                < self.acceptance_threshold
            ),
            "sustained_exceedance_below_limit": (
                threshold_exceedance["maximum_contiguous_duration_s"]
                <= self.maximum_sustained_exceedance
            ),
            "horizontal_rmse_below_threshold": (
                causal_error["horizontal"]["rmse_m"]
                < self.acceptance_threshold
            ),
            "vertical_rmse_below_threshold": (
                causal_error["vertical"]["rmse_m"]
                < self.acceptance_threshold
            ),
        }
        return {
            "schema_version": 4,
            "sim_duration_s": matched_times[-1] - matched_times[0],
            "wall_duration_s": time.monotonic() - self.started_wall_s,
            "association_basis": "source_header_stamp",
            "matched_samples": len(errors),
            "motion_samples": int(np.count_nonzero(motion_mask)),
            "alignment": {
                "yaw_deg": math.degrees(
                    math.atan2(rotation[1, 0], rotation[0, 0])),
                "translation_x_m": float(translation[0]),
                "translation_y_m": float(translation[1]),
                "translation_z_m": z_offset,
                "diagnostic_scale_not_applied": scale,
            },
            "initial_alignment": {
                "duration_s": self.initial_alignment_duration,
                "samples": causal_alignment_samples,
                "yaw_deg": math.degrees(causal_yaw_offset),
                "translation_x_m": float(causal_translation[0]),
                "translation_y_m": float(causal_translation[1]),
                "translation_z_m": causal_z_offset,
                "future_trajectory_used": False,
            },
            "causal_ate": causal_error,
            "acceptance": {
                "metric_basis": "frozen_initial_alignment",
                "threshold_m": self.acceptance_threshold,
                "maximum_sustained_exceedance_s": (
                    self.maximum_sustained_exceedance
                ),
                **acceptance_gates,
                "passed": all(acceptance_gates.values()),
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
            "rpe_1s": causal_rpe_summary,
            "yaw": causal_yaw_summary,
            "yaw_rpe_1s": causal_yaw_rpe_summary,
            "legacy_aligned_rpe_1s": legacy_rpe_summary,
            "legacy_aligned_yaw": legacy_yaw_summary,
            "legacy_aligned_yaw_rpe_1s": legacy_yaw_rpe_summary,
            "samples_output_path": self.samples_output_path or None,
            "truth_used_by_estimator": False,
        }

    def _write(self, report):
        if self.output_path:
            os.makedirs(
                os.path.dirname(os.path.abspath(self.output_path)),
                exist_ok=True,
            )
            temporary = self.output_path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(
                    report, stream, indent=2, sort_keys=True, allow_nan=False)
                stream.write("\n")
            os.replace(temporary, self.output_path)
        if self.samples_output_path and self.last_causal_samples:
            os.makedirs(
                os.path.dirname(os.path.abspath(self.samples_output_path)),
                exist_ok=True,
            )
            temporary = self.samples_output_path + ".tmp"
            with open(temporary, "w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=list(self.last_causal_samples[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.last_causal_samples)
            os.replace(temporary, self.samples_output_path)

    def _report(self):
        report = self._calculate()
        if report is None:
            return
        self.last_report = report
        self._write(report)
        causal_ate = report["causal_ate"]["three_dimensional"]
        self.get_logger().info(
            "EXTERNAL_NAV_ACCURACY "
            f"samples={report['matched_samples']} "
            f"causal_ate_rmse={causal_ate['rmse_m']:.3f}m "
            f"causal_ate_p95={causal_ate['p95_m']:.3f}m "
            f"legacy_ate_rmse={report['ate']['rmse_m']:.3f}m "
            f"rpe_1s={report['rpe_1s']['rmse_m']} "
            f"yaw_rmse={report['yaw']['rmse_deg']:.3f}deg "
            f"accepted={report['acceptance']['passed']} "
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
