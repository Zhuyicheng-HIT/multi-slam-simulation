
#!/usr/bin/env python3
import csv
import json
import math
import statistics
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rtabmap_msgs.msg import Info, OdomInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage
from uf_interfaces.msg import ReliabilityScore

from multi_slam_uav_sim.d435i_pair_health import ExactStampPairHealth


STATES = ("NORMAL", "WEAK", "RISK", "LOST", "RELOCALIZING", "RECOVERED")


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def low_is_bad(value, healthy, bad):
    if value is None or not math.isfinite(float(value)):
        return 0.5
    if healthy <= bad:
        return 0.0
    return clamp((healthy - float(value)) / (healthy - bad))


def high_is_bad(value, healthy, bad):
    if value is None or not math.isfinite(float(value)):
        return 0.5
    if bad <= healthy:
        return 0.0
    return clamp((float(value) - healthy) / (bad - healthy))


def median(values, fallback):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return float(statistics.median(finite)) if finite else float(fallback)


class D435iVisualReliability(Node):
    """Explainable visual reliability monitor. Ground truth is never consumed."""

    def __init__(self):
        super().__init__("d435i_visual_reliability")
        defaults = {
            "rgb_topic": "/front/d435i/color/image_raw",
            "depth_topic": "/front/d435i/aligned_depth_to_color/image_raw",
            "tracking_topic": "/front/d435i/transport/frame_tracking",
            "pair_tracking_topic": (
                "/front/d435i/degraded/transport/frame_tracking"),
            "odom_topic": "/rtabmap/odom",
            "odom_info_topic": "/rtabmap/odom_info",
            "map_info_topic": "/rtabmap/info",
            "relocalization_phase_topic": "/vision/relocalization_phase",
            "output_csv": "",
            "publish_hz": 5.0,
            "calibration_s": 8.0,
            "minimum_state_dwell_s": 1.0,
            "recovered_hold_s": 3.0,
            "weak_threshold": 0.30,
            "risk_threshold": 0.55,
            "state_hysteresis": 0.05,
            "dominant_direct_weak_threshold": 0.80,
            "hard_image_gap_s": 1.0,
            "hard_latency_ms": 1000.0,
            "delay_slope_risk_ms_s": 5.0,
            "feature_healthy_absolute": 120.0,
            "feature_bad_absolute": 20.0,
            "inlier_healthy_absolute": 30.0,
            "inlier_bad_absolute": 5.0,
            "depth_healthy_absolute": 0.70,
            "depth_bad_absolute": 0.15,
            "latency_healthy_absolute_ms": 120.0,
            "latency_bad_absolute_ms": 400.0,
            "weight_feature": 0.15,
            "weight_inlier": 0.25,
            "weight_depth": 0.15,
            "weight_image": 0.15,
            "weight_delay": 0.15,
            "weight_motion": 0.05,
            "weight_covariance": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.values = {
            name: self.get_parameter(name).value for name in defaults}
        weight_names = [name for name in defaults if name.startswith("weight_")]
        weight_sum = sum(float(self.values[name]) for name in weight_names)
        if abs(weight_sum - 1.0) > 1.0e-6:
            raise ValueError(f"D_V weights must sum to 1.0, got {weight_sum}")

        self.started_s = time.monotonic()
        self.last_publish_s = self.started_s
        self.latest_color = None
        self.latest_depth = None
        self.pair_health = ExactStampPairHealth(
            max_pending=120, max_intervals=120)
        self.pair_health_lock = threading.Lock()
        self.pair_tracking_group = MutuallyExclusiveCallbackGroup()
        self.image_pair_health = ExactStampPairHealth(
            max_pending=120, max_intervals=120)
        self.odom_arrivals = deque(maxlen=120)
        self.latency_history = deque(maxlen=80)
        self.tracking_by_stamp = {}
        self.tf_last_stamp = {}
        self.tf_backward_jumps = 0
        self.previous_tf_backward_jumps = 0
        self.current_lost = False
        self.zero_inliers_since_s = None
        self.features = None
        self.matches = None
        self.inliers = None
        self.inlier_ratio = None
        self.words = None
        self.global_closures = 0
        self.rejected_closures = 0
        self.closure_keys = set()
        self.map_id = None
        self.covariance_trace = None
        self.horizontal_speed = 0.0
        self.vertical_speed = 0.0
        self.yaw_rate_deg_s = 0.0
        self.latest_latency_ms = None
        self.relocalization_active = False
        self.state = "NORMAL"
        self.state_since_s = self.started_s
        self.recovered_since_s = None
        self.baseline_ready = False
        self.baseline = {}
        self.calibration = {
            key: [] for key in (
                "features", "inliers", "inlier_ratio", "depth_valid_ratio",
                "laplacian_variance", "contrast", "latency_ms", "frame_hz",
                "covariance_trace")}
        self.last_metrics = {}

        output_csv = str(self.values["output_csv"]).strip()
        self.csv_file = None
        self.csv_writer = None
        if output_csv:
            path = Path(output_csv).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open(
                "w", newline="", encoding="utf-8", buffering=1)
            fields = [
                "steady_s", "elapsed_s", "ros_time_s", "score", "state",
                "hard_reason", "baseline_ready", "features", "matches",
                "inliers", "inlier_ratio", "brightness_mean",
                "underexposed_ratio", "overexposed_ratio", "contrast",
                "laplacian_variance", "depth_valid_ratio", "rgb_depth_delta_ms",
                "pair_sequence", "pair_observed_count", "pair_sequence_gaps",
                "source_sequence_gaps",
                "source_drop_ratio", "image_pair_sequence",
                "unmatched_color", "unmatched_depth",
                "frame_hz", "longest_frame_interval_ms", "image_gap_s",
                "rtab_hz", "latency_ms", "latency_slope_ms_s",
                "covariance_trace", "horizontal_speed_mps",
                "vertical_speed_mps", "yaw_rate_deg_s", "tf_backward_jumps",
                "global_closures", "rejected_closures", "map_id",
                "d_feature", "d_inlier", "d_depth", "d_image", "d_delay",
                "d_motion", "d_covariance", "d_pair_drop"]
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fields)
            self.csv_writer.writeheader()

        tracking_qos = QoSProfile(depth=500)
        tracking_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            Image, str(self.values["rgb_topic"]), self._color_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.values["depth_topic"]), self._depth_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.values["tracking_topic"]), self._tracking_cb,
            tracking_qos)
        self.create_subscription(
            String, str(self.values["pair_tracking_topic"]),
            self._pair_tracking_cb, tracking_qos,
            callback_group=self.pair_tracking_group)
        self.create_subscription(
            Odometry, str(self.values["odom_topic"]), self._odom_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            OdomInfo, str(self.values["odom_info_topic"]), self._odom_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Info, str(self.values["map_info_topic"]), self._info_cb, 20)
        self.create_subscription(TFMessage, "/tf", self._tf_cb, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.values["relocalization_phase_topic"]),
            self._relocalization_cb, 20)

        self.score_pub = self.create_publisher(
            Float32, "/vision/reliability_score", 20)
        self.state_pub = self.create_publisher(
            String, "/vision/reliability_state", 20)
        self.metrics_pub = self.create_publisher(
            String, "/vision/reliability_metrics", 20)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/vision/reliability_diagnostics", 20)
        # D_V_rgbd is the single owner of the scheduler-facing visual score in
        # the integrated stack.  RTAB odometry is the only visual backend
        # factor; image/feature evidence is used only to weight that factor.
        self.uf_score_pub = self.create_publisher(
            ReliabilityScore, "/reliability/vision_score", 20)
        self.create_timer(1.0 / max(1.0, float(self.values["publish_hz"])),
                          self._publish)
        self.get_logger().info(
            "D435i visual reliability V1 active; online inputs exclude ground truth")

    def _color_cb(self, message):
        now_s = time.monotonic()
        self.latest_color = message
        self.image_pair_health.observe(
            "color", stamp_ns(message.header.stamp), now_s)

    def _depth_cb(self, message):
        now_s = time.monotonic()
        self.latest_depth = message
        self.image_pair_health.observe(
            "depth", stamp_ns(message.header.stamp), now_s)

    def _tracking_cb(self, message):
        try:
            values = [int(value) for value in message.data.split(",")]
            if len(values) != 8:
                return
            self.tracking_by_stamp[values[1]] = values[4] * 1.0e-9
            if len(self.tracking_by_stamp) > 1000:
                for key in sorted(self.tracking_by_stamp)[:200]:
                    self.tracking_by_stamp.pop(key, None)
        except (TypeError, ValueError):
            return

    def _pair_tracking_cb(self, message):
        try:
            values = [int(value) for value in message.data.split(",")]
            if len(values) != 8:
                return
            with self.pair_health_lock:
                self.pair_health.observe_pair(
                    values[1], values[5] * 1.0e-9, sequence=values[0],
                    source_sequence=values[6])
        except (TypeError, ValueError):
            return

    def _odom_cb(self, message):
        now_s = time.monotonic()
        self.odom_arrivals.append(now_s)
        source_s = self.tracking_by_stamp.get(stamp_ns(message.header.stamp))
        if source_s is not None:
            self.latest_latency_ms = max(0.0, (now_s - source_s) * 1000.0)
            self.latency_history.append((now_s, self.latest_latency_ms))
        covariance = message.pose.covariance
        self.covariance_trace = sum(float(covariance[index]) for index in (0, 7, 14))
        twist = message.twist.twist
        self.horizontal_speed = math.hypot(twist.linear.x, twist.linear.y)
        self.vertical_speed = abs(float(twist.linear.z))
        self.yaw_rate_deg_s = abs(math.degrees(float(twist.angular.z)))

    def _odom_info_cb(self, message):
        self.current_lost = bool(message.lost)
        self.features = int(message.features)
        self.matches = int(message.matches)
        self.inliers = int(message.inliers)
        self.inlier_ratio = (
            float(self.inliers) / float(self.matches) if self.matches > 0 else 0.0)
        if self.inliers <= 0:
            if self.zero_inliers_since_s is None:
                self.zero_inliers_since_s = time.monotonic()
        else:
            self.zero_inliers_since_s = None

    def _info_cb(self, message):
        self.map_id = int(message.ref_id)
        self.words = sum(int(value) for value in message.weights_values)
        if int(message.loop_closure_id) > 0:
            key = (int(message.ref_id), int(message.loop_closure_id))
            if key not in self.closure_keys:
                self.closure_keys.add(key)
                self.global_closures += 1
        for key, value in zip(message.stats_keys, message.stats_values):
            lowered = key.lower()
            if "rejected" in lowered and "closure" in lowered and float(value) > 0.0:
                self.rejected_closures += 1

    def _tf_cb(self, message):
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            value = stamp_ns(transform.header.stamp)
            previous = self.tf_last_stamp.get(key)
            if previous is not None and value < previous:
                self.tf_backward_jumps += 1
            self.tf_last_stamp[key] = max(value, previous or value)

    def _relocalization_cb(self, message):
        phase = message.data.strip().upper()
        self.relocalization_active = phase in ("START", "RELOCALIZING")

    def _color_metrics(self):
        message = self.latest_color
        if message is None or message.width <= 0 or message.height <= 0:
            return {}
        channels = 1 if message.encoding in ("mono8", "8UC1") else 3
        if message.encoding in ("rgba8", "bgra8"):
            channels = 4
        raw = np.frombuffer(message.data, dtype=np.uint8)
        expected = int(message.height) * int(message.step)
        if raw.size < expected:
            return {}
        rows = raw[:expected].reshape((message.height, message.step))
        useful = rows[:, :message.width * channels]
        pixels = useful.reshape((message.height, message.width, channels))
        sample = pixels[::8, ::8]
        if channels == 1:
            gray = sample[:, :, 0]
        else:
            rgb = sample[:, :, :3].astype(np.float32)
            if message.encoding.startswith("bgr"):
                rgb = rgb[:, :, ::-1]
            gray = (0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] +
                    0.114 * rgb[:, :, 2]).astype(np.uint8)
        return {
            "brightness_mean": float(np.mean(gray)),
            "underexposed_ratio": float(np.mean(gray <= 10)),
            "overexposed_ratio": float(np.mean(gray >= 245)),
            "contrast": float(np.std(gray)),
            "laplacian_variance": float(cv2.Laplacian(
                gray, cv2.CV_64F).var()),
        }

    def _depth_metrics(self):
        message = self.latest_depth
        if message is None or message.width <= 0 or message.height <= 0:
            return {}
        if message.encoding in ("16UC1", "mono16"):
            dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
            scale = 0.001
        elif message.encoding == "32FC1":
            dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
            scale = 1.0
        else:
            return {}
        try:
            depth = np.ndarray(
                (message.height, message.width), dtype=dtype,
                buffer=message.data,
                strides=(message.step, dtype.itemsize))[::8, ::8].astype(np.float32)
        except (TypeError, ValueError):
            return {}
        depth *= scale
        valid = np.isfinite(depth) & (depth > 0.05) & (depth < 10.0)
        return {"depth_valid_ratio": float(np.mean(valid))}

    @staticmethod
    def _rate(values):
        if len(values) < 2:
            return 0.0
        duration = values[-1] - values[0]
        return float((len(values) - 1) / duration) if duration > 0.0 else 0.0

    def _latency_slope(self):
        if len(self.latency_history) < 8:
            return 0.0
        x = np.asarray([item[0] for item in self.latency_history], dtype=float)
        y = np.asarray([item[1] for item in self.latency_history], dtype=float)
        x -= x[0]
        if np.ptp(x) < 1.0:
            return 0.0
        return float(np.polyfit(x, y, 1)[0])

    def _freeze_baseline(self, metrics):
        if self.baseline_ready:
            return
        elapsed = time.monotonic() - self.started_s
        for key, values in self.calibration.items():
            value = metrics.get(key)
            if value is not None and math.isfinite(float(value)):
                values.append(float(value))
        if elapsed < float(self.values["calibration_s"]):
            return
        fallbacks = {
            "features": self.values["feature_healthy_absolute"],
            "inliers": self.values["inlier_healthy_absolute"],
            "inlier_ratio": 0.50,
            "depth_valid_ratio": self.values["depth_healthy_absolute"],
            "laplacian_variance": 100.0,
            "contrast": 25.0,
            "latency_ms": self.values["latency_healthy_absolute_ms"],
            "frame_hz": 25.0,
            "covariance_trace": 1.0e-5,
        }
        self.baseline = {
            key: median(values, fallbacks[key])
            for key, values in self.calibration.items()}
        self.baseline_ready = True
        self.get_logger().info("D_V calibration frozen: " + json.dumps(
            self.baseline, sort_keys=True))

    def _components(self, metrics):
        baseline = self.baseline
        features_healthy = max(
            float(self.values["feature_healthy_absolute"]),
            0.75 * baseline.get("features", 0.0))
        features_bad = min(
            float(self.values["feature_bad_absolute"]),
            0.25 * baseline.get("features", 80.0))
        d_feature = low_is_bad(
            metrics.get("features"), features_healthy, features_bad)

        inlier_healthy = max(
            float(self.values["inlier_healthy_absolute"]),
            0.65 * baseline.get("inliers", 0.0))
        inlier_bad = min(
            float(self.values["inlier_bad_absolute"]),
            0.15 * baseline.get("inliers", 30.0))
        d_inlier_count = low_is_bad(
            metrics.get("inliers"), inlier_healthy, inlier_bad)
        d_inlier_ratio = low_is_bad(
            metrics.get("inlier_ratio"),
            max(0.35, 0.70 * baseline.get("inlier_ratio", 0.5)), 0.10)
        d_inlier = max(d_inlier_count, d_inlier_ratio)

        depth_healthy = max(
            float(self.values["depth_healthy_absolute"]),
            0.90 * baseline.get("depth_valid_ratio", 0.7))
        d_depth = low_is_bad(
            metrics.get("depth_valid_ratio"), depth_healthy,
            float(self.values["depth_bad_absolute"]))

        lap_base = max(1.0, baseline.get("laplacian_variance", 100.0))
        contrast_base = max(1.0, baseline.get("contrast", 25.0))
        d_blur = low_is_bad(
            metrics.get("laplacian_variance"), 0.60 * lap_base,
            0.10 * lap_base)
        d_contrast = low_is_bad(
            metrics.get("contrast"), 0.65 * contrast_base,
            0.15 * contrast_base)
        exposure = max(
            float(metrics.get("underexposed_ratio", 0.0)),
            float(metrics.get("overexposed_ratio", 0.0)))
        d_exposure = high_is_bad(exposure, 0.20, 0.80)
        d_image = max(d_blur, d_contrast, d_exposure)

        latency_healthy = max(
            float(self.values["latency_healthy_absolute_ms"]),
            1.50 * baseline.get("latency_ms", 80.0))
        d_latency = high_is_bad(
            metrics.get("latency_ms"), latency_healthy,
            float(self.values["latency_bad_absolute_ms"]))
        frame_base = max(1.0, baseline.get("frame_hz", 25.0))
        d_rate = low_is_bad(metrics.get("frame_hz"), 0.75 * frame_base,
                            0.25 * frame_base)
        d_gap = high_is_bad(metrics.get("image_gap_s"), 0.15, 0.75)
        d_pair_drop = high_is_bad(
            metrics.get("source_drop_ratio"), 0.02, 0.30)
        d_delay = max(d_latency, d_rate, d_gap, d_pair_drop)

        d_motion = max(
            high_is_bad(metrics.get("horizontal_speed_mps"), 0.75, 1.25),
            high_is_bad(metrics.get("vertical_speed_mps"), 0.50, 0.90),
            high_is_bad(metrics.get("yaw_rate_deg_s"), 30.0, 60.0))

        cov_base = max(1.0e-12, baseline.get("covariance_trace", 1.0e-5))
        covariance = metrics.get("covariance_trace")
        covariance_ratio = (
            float(covariance) / cov_base if covariance is not None else None)
        d_covariance = high_is_bad(covariance_ratio, 4.0, 50.0)
        return {
            "d_feature": d_feature,
            "d_inlier": d_inlier,
            "d_depth": d_depth,
            "d_image": d_image,
            "d_delay": d_delay,
            "d_motion": d_motion,
            "d_covariance": d_covariance,
            "d_pair_drop": d_pair_drop,
        }

    def _hard_reason(self, metrics):
        now_s = time.monotonic()
        if self.relocalization_active:
            return "relocalizing"
        if self.current_lost:
            return "odometry_lost"
        if (self.zero_inliers_since_s is not None and
                now_s - self.zero_inliers_since_s >= 1.0):
            return "quality_zero_sustained"
        if (metrics.get("pair_observed_count", 0) > 0 and
                metrics.get("image_gap_s", 0.0) >=
                float(self.values["hard_image_gap_s"])):
            return "rgbd_stopped"
        if (metrics.get("latency_ms") is not None and
                metrics["latency_ms"] >= float(self.values["hard_latency_ms"])):
            return "latency_hard_limit"
        if self.tf_backward_jumps > self.previous_tf_backward_jumps:
            return "tf_time_backward"
        return ""

    def _target_state(self, score, hard_reason, metrics, components):
        if hard_reason == "relocalizing":
            return "RELOCALIZING"
        if hard_reason:
            return "LOST"
        if (metrics.get("latency_slope_ms_s", 0.0) >=
                float(self.values["delay_slope_risk_ms_s"]) and
                float(metrics.get("latency_ms") or 0.0) > 200.0):
            return "RISK"
        if score >= float(self.values["risk_threshold"]):
            return "RISK"
        direct_evidence = max(
            components.get("d_image", 0.0),
            components.get("d_pair_drop", 0.0))
        if direct_evidence >= float(
                self.values["dominant_direct_weak_threshold"]):
            return "WEAK"
        if score >= float(self.values["weak_threshold"]):
            return "WEAK"
        return "NORMAL"

    def _advance_state(self, target, score, now_s):
        if target in ("LOST", "RELOCALIZING"):
            if self.state != target:
                self.state = target
                self.state_since_s = now_s
            return
        if self.state in ("LOST", "RISK", "RELOCALIZING") and target == "NORMAL":
            self.state = "RECOVERED"
            self.state_since_s = now_s
            self.recovered_since_s = now_s
            return
        if self.state == "RECOVERED":
            if target != "NORMAL":
                self.state = target
                self.state_since_s = now_s
                self.recovered_since_s = None
            elif (self.recovered_since_s is not None and
                  now_s - self.recovered_since_s >=
                  float(self.values["recovered_hold_s"])):
                self.state = "NORMAL"
                self.state_since_s = now_s
                self.recovered_since_s = None
            return
        if target == self.state:
            return
        hysteresis = float(self.values["state_hysteresis"])
        if self.state == "WEAK" and target == "NORMAL" and score >= (
                float(self.values["weak_threshold"]) - hysteresis):
            return
        if self.state == "RISK" and target == "WEAK" and score >= (
                float(self.values["risk_threshold"]) - hysteresis):
            return
        if now_s - self.state_since_s < float(self.values["minimum_state_dwell_s"]):
            return
        self.state = target
        self.state_since_s = now_s

    def _publish(self):
        now_s = time.monotonic()
        color = self._color_metrics()
        depth = self._depth_metrics()
        with self.pair_health_lock:
            intervals = list(self.pair_health.intervals)
            pair_sequence = self.pair_health.pair_sequence
            pair_observed_count = self.pair_health.observed_pair_count
            pair_sequence_gaps = self.pair_health.pair_sequence_gaps
            source_sequence_gaps = self.pair_health.source_sequence_gaps
            source_drop_ratio = self.pair_health.source_drop_ratio
            last_pair_arrival_s = self.pair_health.last_pair_arrival_s
            last_stamp_delta_ms = self.pair_health.last_stamp_delta_ms
        frame_hz = 1.0 / statistics.mean(intervals) if intervals else 0.0
        longest_ms = max(intervals) * 1000.0 if intervals else None
        image_gap_s = (
            now_s - last_pair_arrival_s
            if last_pair_arrival_s is not None
            else now_s - self.started_s)
        metrics = {
            **color, **depth,
            "features": self.features,
            "matches": self.matches,
            "inliers": self.inliers,
            "inlier_ratio": self.inlier_ratio,
            "pair_sequence": pair_sequence,
            "pair_observed_count": pair_observed_count,
            "pair_sequence_gaps": pair_sequence_gaps,
            "source_sequence_gaps": source_sequence_gaps,
            "source_drop_ratio": source_drop_ratio,
            "image_pair_sequence": self.image_pair_health.pair_sequence,
            "unmatched_color": len(self.image_pair_health.color_arrivals),
            "unmatched_depth": len(self.image_pair_health.depth_arrivals),
            "frame_hz": frame_hz,
            "longest_frame_interval_ms": longest_ms,
            "image_gap_s": image_gap_s,
            "rgb_depth_delta_ms": last_stamp_delta_ms,
            "rtab_hz": self._rate(list(self.odom_arrivals)),
            "latency_ms": self.latest_latency_ms,
            "latency_slope_ms_s": self._latency_slope(),
            "covariance_trace": self.covariance_trace,
            "horizontal_speed_mps": self.horizontal_speed,
            "vertical_speed_mps": self.vertical_speed,
            "yaw_rate_deg_s": self.yaw_rate_deg_s,
            "tf_backward_jumps": self.tf_backward_jumps,
            "global_closures": self.global_closures,
            "rejected_closures": self.rejected_closures,
            "map_id": self.map_id,
        }
        self._freeze_baseline(metrics)
        components = self._components(metrics)
        weighted_components = (
            "feature", "inlier", "depth", "image", "delay", "motion",
            "covariance")
        score = sum(
            float(self.values[f"weight_{name}"]) * components[f"d_{name}"]
            for name in weighted_components)
        score = clamp(score)
        hard_reason = self._hard_reason(metrics)
        target = self._target_state(score, hard_reason, metrics, components)
        self._advance_state(target, score, now_s)
        self.previous_tf_backward_jumps = self.tf_backward_jumps
        metrics.update(components)
        metrics.update({
            "score": score,
            "state": self.state,
            "hard_reason": hard_reason,
            "baseline_ready": self.baseline_ready,
            "baseline": self.baseline,
        })
        self.last_metrics = metrics

        score_message = Float32()
        score_message.data = float(score)
        self.score_pub.publish(score_message)
        state_message = String()
        state_message.data = self.state
        self.state_pub.publish(state_message)
        metrics_message = String()
        metrics_message.data = json.dumps(metrics, sort_keys=True, allow_nan=False)
        self.metrics_pub.publish(metrics_message)
        self._publish_uf_score(metrics)
        self._publish_diagnostics(metrics)

        if self.csv_writer is not None:
            row = {key: metrics.get(key, "") for key in self.csv_writer.fieldnames}
            row.update({
                "steady_s": now_s,
                "elapsed_s": now_s - self.started_s,
                "ros_time_s": self.get_clock().now().nanoseconds * 1.0e-9,
                "score": score,
                "state": self.state,
                "hard_reason": hard_reason,
                "baseline_ready": int(self.baseline_ready),
            })
            self.csv_writer.writerow(row)
        self.last_publish_s = now_s

    def _publish_uf_score(self, metrics):
        message = ReliabilityScore()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "d435i_link"
        message.modality = "vision"
        state = str(metrics.get("state", "LOST"))
        raw_score = clamp(float(metrics.get("score", 1.0)))
        if state in ("LOST", "RELOCALIZING"):
            raw_score = 1.0
        observation_count = max(
            0, int(metrics.get("pair_observed_count") or 0))
        complete = bool(
            metrics.get("baseline_ready")
            and observation_count >= 10
            and metrics.get("features") is not None
            and metrics.get("inliers") is not None
            and metrics.get("depth_valid_ratio") is not None
            and float(metrics.get("rtab_hz") or 0.0) > 0.0
        )
        message.degradation_score = float(raw_score)
        message.reliability_weight = float(1.0 - raw_score) if complete else 0.0
        message.valid = complete
        message.observation_count = observation_count
        message.minimum_observation_count = 10
        message.reasons = [f"dv_rgbd_state:{state.lower()}"]
        hard_reason = str(metrics.get("hard_reason") or "")
        if hard_reason:
            message.reasons.append(f"dv_rgbd_hard_gate:{hard_reason}")
        if not metrics.get("baseline_ready"):
            message.reasons.append("dv_rgbd_calibrating")
        evidence = {
            "dv_rgbd_v1": 1.0,
            "baseline_ready": 1.0 if metrics.get("baseline_ready") else 0.0,
            "features": metrics.get("features"),
            "inliers": metrics.get("inliers"),
            "inlier_ratio": metrics.get("inlier_ratio"),
            "depth_valid_ratio": metrics.get("depth_valid_ratio"),
            "rtab_hz": metrics.get("rtab_hz"),
            "latency_ms": metrics.get("latency_ms"),
            "d_feature": metrics.get("d_feature"),
            "d_inlier": metrics.get("d_inlier"),
            "d_depth": metrics.get("d_depth"),
            "d_image": metrics.get("d_image"),
            "d_delay": metrics.get("d_delay"),
            "d_motion": metrics.get("d_motion"),
            "d_covariance": metrics.get("d_covariance"),
            "d_pair_drop": metrics.get("d_pair_drop"),
        }
        for name, value in evidence.items():
            if value is None:
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(numeric):
                continue
            message.evidence_names.append(name)
            message.evidence_values.append(numeric)
        self.uf_score_pub.publish(message)

    def _publish_diagnostics(self, metrics):
        status = DiagnosticStatus()
        status.name = "D435i visual reliability D_V V1"
        status.hardware_id = "front/d435i"
        status.level = {
            "NORMAL": DiagnosticStatus.OK,
            "RECOVERED": DiagnosticStatus.OK,
            "WEAK": DiagnosticStatus.WARN,
            "RISK": DiagnosticStatus.WARN,
            "RELOCALIZING": DiagnosticStatus.WARN,
            "LOST": DiagnosticStatus.ERROR,
        }[self.state]
        status.message = self.state
        keys = [
            "score", "hard_reason", "features", "inliers", "inlier_ratio",
            "depth_valid_ratio", "laplacian_variance", "latency_ms", "frame_hz",
            "rtab_hz", "d_feature", "d_inlier", "d_depth", "d_image",
            "d_delay", "d_motion", "d_covariance", "d_pair_drop",
            "source_drop_ratio"]
        status.values = [
            KeyValue(key=key, value=str(metrics.get(key, ""))) for key in keys]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D435iVisualReliability()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
