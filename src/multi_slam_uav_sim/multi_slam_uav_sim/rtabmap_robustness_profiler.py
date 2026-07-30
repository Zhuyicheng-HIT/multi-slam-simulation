#!/usr/bin/env python3
import bisect
import csv
import json
import math
import os
import re
import subprocess
import time
from pathlib import Path

import numpy as np
import psutil
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from rtabmap_msgs.msg import Info, OdomInfo
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def finite_summary(values):
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return None
    return {
        "count": int(len(array)),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p05": float(np.percentile(array, 5)),
        "p95": float(np.percentile(array, 95)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def rate_summary(arrivals_ns):
    if len(arrivals_ns) < 2:
        return {
            "count": len(arrivals_ns), "mean_hz": 0.0,
            "median_hz": 0.0, "p05_hz": 0.0, "p95_hz": 0.0,
            "min_hz": 0.0, "max_hz": 0.0,
            "longest_interval_ms": None,
        }
    seconds = np.asarray(arrivals_ns, dtype=float) * 1.0e-9
    intervals = np.diff(seconds)
    rates = 1.0 / np.maximum(intervals, 1.0e-9)
    return {
        "count": len(arrivals_ns),
        "mean_hz": float((len(arrivals_ns) - 1) / (seconds[-1] - seconds[0])),
        "median_hz": float(np.median(rates)),
        "p05_hz": float(np.percentile(rates, 5)),
        "p95_hz": float(np.percentile(rates, 95)),
        "min_hz": float(np.min(rates)),
        "max_hz": float(np.max(rates)),
        "longest_interval_ms": float(np.max(intervals) * 1000.0),
    }


class RtabmapRobustnessProfiler(Node):
    def __init__(self):
        super().__init__("rtabmap_robustness_profiler")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("profile", "stationary")
        self.declare_parameter("duration_s", 0.0)
        self.declare_parameter("rtabmap_log_path", "")
        self.declare_parameter(
            "tracking_topic", "/front/d435i/transport/frame_tracking")
        self.declare_parameter(
            "ground_truth_topic", "/d435i_visual_slam/ground_truth")
        self.declare_parameter(
            "mavros_odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("rtabmap_odom_topic", "/rtabmap/odom")
        self.declare_parameter("stage_topic", "/d435i_visual_slam/stage")

        output_dir = str(self.get_parameter("output_dir").value).strip()
        if not output_dir:
            output_dir = f"/tmp/d435i_robustness_{int(time.time())}"
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.profile = str(self.get_parameter("profile").value)
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.rtabmap_log_path = Path(
            str(self.get_parameter("rtabmap_log_path").value)).expanduser()
        self.started_ns = time.monotonic_ns()
        self.stage = "initializing"
        self.done = False
        self.stop_requested = False

        self.frames = []
        self.frames_by_stamp = {}
        self.odom_events = []
        self.odom_outputs = {}
        self.info_events = []
        self.trajectories = {"ground_truth": [], "mavros": [], "rtabmap": []}
        self.clock_samples = []
        self.resource_samples = []
        self.tf_last_stamp = {}
        self.tf_time_jumps = 0
        self.quality_values = []
        self.features_values = []
        self.lost_events = 0
        self.recoveries = 0
        self.last_lost = False
        self.latest_grid = None

        self.trajectory_file, self.trajectory_writer = self._csv(
            "trajectory.csv",
            ["stream", "stamp", "raw_header_stamp", "arrival_steady_s",
             "x", "y", "z", "qx", "qy", "qz", "qw", "stage"],
        )
        self.resource_file, self.resource_writer = self._csv(
            "resource_usage.csv",
            ["elapsed_s", "stage", "rtf", "total_cpu_percent",
             "pipeline_cpu_percent", "gazebo_cpu_percent",
             "bridge_cpu_percent", "rtab_cpu_percent", "pipeline_rss_bytes",
             "gazebo_rss_bytes", "bridge_rss_bytes", "rtab_rss_bytes",
             "memory_used_bytes", "gpu_util_percent", "gpu_memory_used_mb"],
        )
        self.ground_truth_file, self.ground_truth_writer = self._csv(
            "ground_truth.csv",
            ["stamp", "x", "y", "z", "qx", "qy", "qz", "qw", "stage"],
        )
        self.odom_features_file, self.odom_features_writer = self._csv(
            "odometry_features.csv",
            ["stamp", "features", "words", "word_matches", "word_inliers",
             "key_frame_added", "odometry_type", "stage"],
        )
        self.tum_file = (self.output_dir / "trajectory.tum").open(
            "w", encoding="utf-8", buffering=1)
        self.gt_tum_file = (self.output_dir / "ground_truth.tum").open(
            "w", encoding="utf-8", buffering=1)
        self.mavros_tum_file = (self.output_dir / "mavros.tum").open(
            "w", encoding="utf-8", buffering=1)

        tracking_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=500,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.transport_callbacks = MutuallyExclusiveCallbackGroup()
        self.odom_info_callbacks = MutuallyExclusiveCallbackGroup()
        self.map_info_callbacks = MutuallyExclusiveCallbackGroup()
        self.support_callbacks = MutuallyExclusiveCallbackGroup()
        self.resource_callbacks = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            String, str(self.get_parameter("tracking_topic").value),
            self._tracking_cb, tracking_qos,
            callback_group=self.transport_callbacks)
        self.create_subscription(
            OdomInfo, "/rtabmap/odom_info", self._odom_info_cb,
            qos_profile_sensor_data,
            callback_group=self.odom_info_callbacks)
        self.create_subscription(
            Info, "/rtabmap/info", self._info_cb, 20,
            callback_group=self.map_info_callbacks)
        self.create_subscription(
            Odometry, str(self.get_parameter("ground_truth_topic").value),
            lambda message: self._odom_cb("ground_truth", message), 30,
            callback_group=self.support_callbacks)
        self.create_subscription(
            Odometry, str(self.get_parameter("mavros_odom_topic").value),
            lambda message: self._odom_cb("mavros", message),
            qos_profile_sensor_data,
            callback_group=self.support_callbacks)
        self.create_subscription(
            Odometry, str(self.get_parameter("rtabmap_odom_topic").value),
            lambda message: self._odom_cb("rtabmap", message),
            qos_profile_sensor_data,
            callback_group=self.transport_callbacks)
        self.create_subscription(
            Clock, "/clock", self._clock_cb, 50,
            callback_group=self.support_callbacks)
        self.create_subscription(
            String, str(self.get_parameter("stage_topic").value),
            self._stage_cb, 20, callback_group=self.support_callbacks)
        self.create_subscription(
            TFMessage, "/tf", self._tf_cb, qos_profile_sensor_data,
            callback_group=self.support_callbacks)
        self.create_subscription(
            TFMessage, "/tf_static", self._tf_cb, 10,
            callback_group=self.support_callbacks)
        self.create_subscription(
            OccupancyGrid, "/rtabmap/grid_map", self._grid_cb, 1,
            callback_group=self.support_callbacks)
        self.create_timer(
            1.0, self._resource_timer,
            callback_group=self.resource_callbacks)
        self.get_logger().info(
            f"RTAB robustness profiler active: profile={self.profile} "
            f"output={self.output_dir}")

    def _csv(self, name, fields):
        handle = (self.output_dir / name).open(
            "w", newline="", encoding="utf-8", buffering=1)
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        return handle, writer

    def _stage_cb(self, message):
        self.stage = message.data or "unlabelled"

    def _tracking_cb(self, message):
        arrival_ns = time.monotonic_ns()
        try:
            values = [int(value) for value in message.data.split(",")]
            if len(values) != 8:
                raise ValueError(f"expected 8 fields, got {len(values)}")
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Invalid bridge tracking record: {error}")
            return
        frame = {
            "pair_sequence": values[0],
            "stamp_ns": values[1],
            "color_source_steady_ns": values[2],
            "depth_source_steady_ns": values[3],
            "matched_steady_ns": values[4],
            "published_steady_ns": values[5],
            "color_source_sequence": values[6],
            "depth_source_sequence": values[7],
            "tracker_arrival_steady_ns": arrival_ns,
            "stage": self.stage,
        }
        self.frames.append(frame)
        self.frames_by_stamp[frame["stamp_ns"]] = frame

    def _odom_info_cb(self, message):
        arrival_ns = time.monotonic_ns()
        input_stamp_ns = stamp_ns(message.header.stamp)
        lost = bool(message.lost)
        if lost and not self.last_lost:
            self.lost_events += 1
        if not lost and self.last_lost:
            self.recoveries += 1
        self.last_lost = lost
        self.quality_values.append(int(message.inliers))
        self.features_values.append(int(message.features))
        event = {
            "input_stamp_ns": input_stamp_ns,
            "odom_info_arrival_steady_ns": arrival_ns,
            "quality_inliers": int(message.inliers),
            "features": int(message.features),
            "matches": int(message.matches),
            "lost": int(lost),
            "key_frame_added": int(message.key_frame_added),
            "local_map_size": int(message.local_map_size),
            "local_key_frames": int(message.local_key_frames),
            "front_end_time_ms": float(message.time_estimation) * 1000.0,
            "particle_filter_time_ms": float(message.time_particle_filtering) * 1000.0,
            "local_bundle_time_ms": float(message.local_bundle_time) * 1000.0,
            "stage": self.stage,
            "pair_sequence": None,
        }
        self.odom_events.append(event)
        self.odom_features_writer.writerow({
            "stamp": input_stamp_ns * 1.0e-9,
            "features": int(message.features),
            "words": len(message.words_keys),
            "word_matches": len(message.word_matches),
            "word_inliers": len(message.word_inliers),
            "key_frame_added": int(message.key_frame_added),
            "odometry_type": int(message.type),
            "stage": self.stage,
        })

    def _recompute_correlations(self):
        """Correlate after capture using the lightweight odometry output time.

        OdomInfo is a large diagnostic message and its Python callback can arrive
        well after /rtabmap/odom. Using that callback as the endpoint measures the
        observer queue, not the SLAM path. The navigation odometry callback is the
        primary endpoint; OdomInfo timing is retained as observer-delay evidence.
        """
        ordered_frames = sorted(
            self.frames, key=lambda item: item["published_steady_ns"])
        published_times = [
            frame["published_steady_ns"] for frame in ordered_frames]
        sequence_frames = sorted(
            self.frames, key=lambda item: item["pair_sequence"])
        sequences = [frame["pair_sequence"] for frame in sequence_frames]

        info_by_stamp = {
            event["input_stamp_ns"]: event for event in self.odom_events}
        correlated = []
        for input_stamp_ns, output_ns in self.odom_outputs.items():
            frame = self.frames_by_stamp.get(input_stamp_ns)
            if frame is None:
                continue
            event = info_by_stamp.get(input_stamp_ns, {
                "input_stamp_ns": input_stamp_ns,
                "odom_info_arrival_steady_ns": None,
                "stage": frame["stage"],
                "pair_sequence": None,
            })
            event["pair_sequence"] = int(frame["pair_sequence"])
            event["source_matched_steady_ns"] = frame["matched_steady_ns"]
            event["published_steady_ns"] = frame["published_steady_ns"]
            event["odom_output_arrival_steady_ns"] = output_ns
            event["end_to_end_latency_ms"] = (
                output_ns - frame["matched_steady_ns"]) * 1.0e-6
            if event["odom_info_arrival_steady_ns"] is not None:
                event["odom_info_callback_delay_ms"] = max(
                    0.0,
                    (event["odom_info_arrival_steady_ns"] - output_ns) * 1.0e-6)
            correlated.append(event)

        previous_sequence = None
        for event in sorted(
                correlated,
                key=lambda item: item["odom_output_arrival_steady_ns"]):
            output_ns = event["odom_output_arrival_steady_ns"]
            sequence = event["pair_sequence"]
            latest_index = bisect.bisect_right(published_times, output_ns) - 1
            latest = (ordered_frames[latest_index]
                      if latest_index >= 0 else self.frames_by_stamp[
                          event["input_stamp_ns"]])
            event["latest_published_sequence"] = latest["pair_sequence"]
            event["lag_frames"] = max(
                0, latest["pair_sequence"] - sequence)
            event["latest_frame_age_ms"] = max(
                0.0, (output_ns - latest["matched_steady_ns"]) * 1.0e-6)
            event["processed_sequence_gap"] = (
                0 if previous_sequence is None else sequence - previous_sequence)
            event["skipped_frames"] = (
                0 if previous_sequence is None
                else max(0, sequence - previous_sequence - 1))
            first_index = (0 if previous_sequence is None else
                           bisect.bisect_right(sequences, previous_sequence))
            last_index = bisect.bisect_right(sequences, sequence)
            candidates = sequence_frames[first_index:last_index]
            event["queue_age_upper_bound_ms"] = (
                max(0.0, (
                    output_ns - min(
                        item["matched_steady_ns"] for item in candidates)
                ) * 1.0e-6) if candidates else 0.0)
            previous_sequence = sequence
        self.correlated_events = correlated

    @staticmethod
    def _stat(stats, *names):
        for name in names:
            if name in stats:
                return float(stats[name])
        lowered = [(key.lower(), value) for key, value in stats.items()]
        for name in names:
            needle = name.lower().strip("/")
            for key, value in lowered:
                if needle in key:
                    return float(value)
        return None

    def _info_cb(self, message):
        stats = dict(zip(message.stats_keys, message.stats_values))
        likelihood_by_id = dict(zip(
            (int(key) for key in message.likelihood_keys),
            (float(value) for value in message.likelihood_values)))
        raw_likelihood_by_id = dict(zip(
            (int(key) for key in message.raw_likelihood_keys),
            (float(value) for value in message.raw_likelihood_values)))
        posterior = [
            (int(key), float(value))
            for key, value in zip(message.posterior_keys, message.posterior_values)
            if int(key) > 0 and int(key) != int(message.ref_id)
        ]
        candidate_id = 0
        candidate_similarity = 0.0
        if posterior:
            candidate_id, candidate_similarity = max(posterior, key=lambda item: item[1])
        highest_id = self._stat(stats, "Loop/Highest_hypothesis_id/")
        highest_value = self._stat(stats, "Loop/Highest_hypothesis_value/")
        if highest_id is not None and int(round(highest_id)) > 0:
            candidate_id = int(round(highest_id))
        if highest_value is not None:
            candidate_similarity = highest_value
        map_to_odom = message.odom_cache.map_to_odom
        transform = map_to_odom.translation
        rotation = map_to_odom.rotation
        loop_detection_parts = [self._stat(stats, name) for name in (
            "Timing/Likelihood_computation/ms",
            "Timing/Posterior_computation/ms",
            "Timing/Hypotheses_creation/ms",
            "Timing/Hypotheses_validation/ms",
        )]
        loop_detection_parts = [
            value for value in loop_detection_parts if value is not None]
        event = {
            "stamp_ns": stamp_ns(message.header.stamp),
            "arrival_steady_ns": time.monotonic_ns(),
            "ref_id": int(message.ref_id),
            "loop_closure_id": int(message.loop_closure_id),
            "proximity_detection_id": int(message.proximity_detection_id),
            "candidate_id": candidate_id,
            "candidate_similarity": candidate_similarity,
            "candidate_likelihood": likelihood_by_id.get(candidate_id, 0.0),
            "candidate_raw_likelihood": raw_likelihood_by_id.get(
                candidate_id, 0.0),
            "posterior_best": max(
                (value for _, value in posterior), default=0.0),
            "likelihood_best": max(likelihood_by_id.values(), default=0.0),
            "raw_likelihood_best": max(
                raw_likelihood_by_id.values(), default=0.0),
            "visual_matches": self._stat(
                stats, "Loop/Visual_matches/", "Loop/Matches/") or 0.0,
            "rejected_hypothesis": self._stat(
                stats, "Loop/RejectedHypothesis/") or 0.0,
            "geometric_inliers": self._stat(
                stats, "Loop/Visual_inliers/", "Loop/Inliers/") or 0.0,
            "map_id": self._stat(stats, "Loop/Map_id/"),
            "update_time_ms": self._stat(stats, "RtabmapROS/TimeTotal/ms"),
            "loop_detection_time_ms": (
                sum(loop_detection_parts) if loop_detection_parts else None),
            "optimization_time_ms": self._stat(
                stats, "Timing/Map_optimization/ms"),
            "optimization_error": self._stat(
                stats, "Loop/Optimization_error/"),
            "optimization_max_error": self._stat(
                stats, "Loop/Optimization_max_error/"),
            "map_to_odom_x": float(transform.x),
            "map_to_odom_y": float(transform.y),
            "map_to_odom_z": float(transform.z),
            "map_to_odom_yaw": yaw_from_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w),
            "stage": self.stage,
        }
        self.info_events.append(event)

    def _odom_cb(self, stream, message):
        arrival_ns = time.monotonic_ns()
        raw_stamp = stamp_seconds(message.header.stamp)
        evaluation_stamp = raw_stamp
        if stream == "mavros" and self.clock_samples:
            evaluation_stamp = self.clock_samples[-1][1]
        pose = message.pose.pose
        sample = (
            evaluation_stamp,
            float(pose.position.x), float(pose.position.y), float(pose.position.z),
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
            self.stage,
        )
        self.trajectories[stream].append(sample)
        self.trajectory_writer.writerow({
            "stream": stream,
            "stamp": f"{evaluation_stamp:.9f}",
            "raw_header_stamp": f"{raw_stamp:.9f}",
            "arrival_steady_s": f"{arrival_ns * 1.0e-9:.9f}",
            "x": sample[1], "y": sample[2], "z": sample[3],
            "qx": sample[4], "qy": sample[5], "qz": sample[6],
            "qw": sample[7], "stage": self.stage,
        })
        tum_line = (
            f"{evaluation_stamp:.9f} {sample[1]:.9f} {sample[2]:.9f} "
            f"{sample[3]:.9f} {sample[4]:.9f} {sample[5]:.9f} "
            f"{sample[6]:.9f} {sample[7]:.9f}\n")
        if stream == "rtabmap":
            self.tum_file.write(tum_line)
            self.odom_outputs[stamp_ns(message.header.stamp)] = arrival_ns
        elif stream == "ground_truth":
            self.gt_tum_file.write(tum_line)
            self.ground_truth_writer.writerow({
                "stamp": f"{evaluation_stamp:.9f}",
                "x": sample[1], "y": sample[2], "z": sample[3],
                "qx": sample[4], "qy": sample[5], "qz": sample[6],
                "qw": sample[7], "stage": self.stage,
            })
        else:
            self.mavros_tum_file.write(tum_line)

    def _clock_cb(self, message):
        self.clock_samples.append((time.monotonic_ns(), stamp_seconds(message.clock)))
        if len(self.clock_samples) > 2000:
            self.clock_samples = self.clock_samples[-1000:]

    def _tf_cb(self, message):
        for transform in message.transforms:
            current = stamp_ns(transform.header.stamp)
            previous = self.tf_last_stamp.get(transform.child_frame_id)
            if previous is not None and current < previous:
                self.tf_time_jumps += 1
            self.tf_last_stamp[transform.child_frame_id] = current

    def _grid_cb(self, message):
        self.latest_grid = message

    def _rtf(self):
        if len(self.clock_samples) < 2:
            return float("nan")
        first_wall, first_sim = self.clock_samples[0]
        last_wall, last_sim = self.clock_samples[-1]
        wall_s = (last_wall - first_wall) * 1.0e-9
        return (last_sim - first_sim) / wall_s if wall_s > 0.0 else float("nan")

    @staticmethod
    def _gpu_metrics():
        try:
            result = subprocess.run(
                ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
                 "--format=csv,noheader,nounits"],
                capture_output=True, text=True, timeout=1.0, check=False)
            values = result.stdout.strip().splitlines()[0].split(",")
            return float(values[0]), float(values[1])
        except (FileNotFoundError, IndexError, ValueError, subprocess.TimeoutExpired):
            return float("nan"), float("nan")

    @staticmethod
    def _process_resources():
        totals = {
            "pipeline_cpu_percent": 0.0, "pipeline_rss_bytes": 0,
            "gazebo_cpu_percent": 0.0, "gazebo_rss_bytes": 0,
            "bridge_cpu_percent": 0.0, "bridge_rss_bytes": 0,
            "rtab_cpu_percent": 0.0, "rtab_rss_bytes": 0,
        }
        markers = (
            "gz sim", "arducopter", "mavros", "d435i_sim_bridge",
            "d435i_rgbd_bridge", "parameter_bridge", "rtabmap",
            "rgbd_odometry", "gazebo_ground_truth_bridge",
        )
        for process in psutil.process_iter(["cmdline", "memory_info"]):
            try:
                if process.pid == os.getpid():
                    continue
                command = " ".join(process.info.get("cmdline") or [])
                if not any(marker in command for marker in markers):
                    continue
                cpu = float(process.cpu_percent(None))
                rss = int(process.info["memory_info"].rss)
                totals["pipeline_cpu_percent"] += cpu
                totals["pipeline_rss_bytes"] += rss
                if "gz sim" in command:
                    totals["gazebo_cpu_percent"] += cpu
                    totals["gazebo_rss_bytes"] += rss
                if "d435i_rgbd_bridge" in command:
                    totals["bridge_cpu_percent"] += cpu
                    totals["bridge_rss_bytes"] += rss
                if "rgbd_odometry" in command or "rtabmap" in command:
                    totals["rtab_cpu_percent"] += cpu
                    totals["rtab_rss_bytes"] += rss
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                continue
        return totals

    def _resource_timer(self):
        elapsed = (time.monotonic_ns() - self.started_ns) * 1.0e-9
        process = self._process_resources()
        gpu_util, gpu_memory = self._gpu_metrics()
        row = {
            "elapsed_s": elapsed,
            "stage": self.stage,
            "rtf": self._rtf(),
            "total_cpu_percent": psutil.cpu_percent(None),
            "memory_used_bytes": int(psutil.virtual_memory().used),
            "gpu_util_percent": gpu_util,
            "gpu_memory_used_mb": gpu_memory,
            **process,
        }
        self.resource_samples.append(row)
        self.resource_writer.writerow(row)
        if self.duration_s > 0.0 and elapsed >= self.duration_s:
            self.stop_requested = True

    @staticmethod
    def _associate(reference, estimate, tolerance_s=0.12):
        if not reference or not estimate:
            return [], []
        stamps = [sample[0] for sample in reference]
        matched_reference = []
        matched_estimate = []
        for sample in estimate:
            index = bisect.bisect_left(stamps, sample[0])
            candidates = [candidate for candidate in (index - 1, index)
                          if 0 <= candidate < len(reference)]
            if not candidates:
                continue
            best = min(candidates, key=lambda candidate: abs(
                reference[candidate][0] - sample[0]))
            if abs(reference[best][0] - sample[0]) <= tolerance_s:
                matched_reference.append(reference[best])
                matched_estimate.append(sample)
        return matched_reference, matched_estimate

    @classmethod
    def _trajectory_metrics(cls, reference, estimate):
        reference, estimate = cls._associate(reference, estimate)
        if len(reference) < 3:
            return {"available": False, "matched": len(reference)}
        reference_position = np.asarray([sample[1:4] for sample in reference])
        estimate_position = np.asarray([sample[1:4] for sample in estimate])
        estimate_center = estimate_position.mean(axis=0)
        reference_center = reference_position.mean(axis=0)
        covariance = (
            (estimate_position - estimate_center).T
            @ (reference_position - reference_center))
        left, _, right_t = np.linalg.svd(covariance)
        rotation = left @ right_t
        if np.linalg.det(rotation) < 0.0:
            left[:, -1] *= -1.0
            rotation = left @ right_t
        aligned = (estimate_position - estimate_center) @ rotation + reference_center
        error = aligned - reference_position
        norms = np.linalg.norm(error, axis=1)
        horizontal = np.linalg.norm(error[:, :2], axis=1)
        reference_yaw = np.asarray([
            yaw_from_quaternion(*sample[4:8]) for sample in reference])
        estimate_yaw = np.asarray([
            yaw_from_quaternion(*sample[4:8]) for sample in estimate])
        yaw_offset = math.atan2(
            np.sin(reference_yaw - estimate_yaw).mean(),
            np.cos(reference_yaw - estimate_yaw).mean())
        yaw_error = np.asarray([
            abs(wrap_angle(reference_value - estimate_value - yaw_offset))
            for reference_value, estimate_value
            in zip(reference_yaw, estimate_yaw)
        ])
        rpe = np.linalg.norm(np.diff(aligned, axis=0)
                             - np.diff(reference_position, axis=0), axis=1)
        steps = np.linalg.norm(np.diff(estimate_position, axis=0), axis=1)
        return {
            "available": True,
            "matched": len(reference),
            "ate_rmse_m": float(np.sqrt(np.mean(norms ** 2))),
            "rpe_translation_rmse_m": float(np.sqrt(np.mean(rpe ** 2))),
            "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal ** 2))),
            "height_rmse_m": float(np.sqrt(np.mean(error[:, 2] ** 2))),
            "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
            "trajectory_breaks_over_1m": int(np.count_nonzero(steps > 1.0)),
            "largest_raw_step_m": float(np.max(steps)) if len(steps) else 0.0,
        }

    @staticmethod
    def _closure_metrics(samples):
        if len(samples) < 2:
            return None
        first = samples[0]
        last = samples[-1]
        position_error = math.sqrt(sum(
            (last[index] - first[index]) ** 2 for index in (1, 2, 3)))
        first_yaw = yaw_from_quaternion(*first[4:8])
        last_yaw = yaw_from_quaternion(*last[4:8])
        return {
            "position_error_m": position_error,
            "yaw_error_deg": abs(math.degrees(wrap_angle(last_yaw - first_yaw))),
        }

    @staticmethod
    def _linear_trend(values, times):
        if len(values) < 3:
            return {"slope_per_s": None, "r_squared": None}
        x = np.asarray(times, dtype=float)
        y = np.asarray(values, dtype=float)
        x = x - x[0]
        slope, intercept = np.polyfit(x, y, 1)
        predicted = slope * x + intercept
        total = np.sum((y - np.mean(y)) ** 2)
        residual = np.sum((y - predicted) ** 2)
        r_squared = 0.0 if total <= 1.0e-12 else 1.0 - residual / total
        return {"slope_per_s": float(slope), "r_squared": float(r_squared)}

    def _log_counts(self):
        if not self.rtabmap_log_path.is_file():
            return {"lost": 0, "reset": 0}
        text = self.rtabmap_log_path.read_text(encoding="utf-8", errors="ignore")
        return {
            "lost": len(re.findall(r"Odometry lost|lost=true", text, re.I)),
            "reset": len(re.findall(
                r"Odometry automatically reset|resetting odometry|Odometry reset",
                text, re.I)),
        }

    def _write_frame_tracking(self):
        fields = [
            "pair_sequence", "stamp_ns", "stamp_s",
            "color_source_sequence", "depth_source_sequence",
            "color_source_steady_s", "depth_source_steady_s",
            "matched_steady_s", "published_steady_s",
            "tracker_arrival_steady_s", "tracking_delivery_ms", "stage",
        ]
        with (self.output_dir / "frame_tracking.csv").open(
                "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for frame in sorted(self.frames, key=lambda item: item["pair_sequence"]):
                writer.writerow({
                    "pair_sequence": frame["pair_sequence"],
                    "stamp_ns": frame["stamp_ns"],
                    "stamp_s": frame["stamp_ns"] * 1.0e-9,
                    "color_source_sequence": frame["color_source_sequence"],
                    "depth_source_sequence": frame["depth_source_sequence"],
                    "color_source_steady_s": frame["color_source_steady_ns"] * 1.0e-9,
                    "depth_source_steady_s": frame["depth_source_steady_ns"] * 1.0e-9,
                    "matched_steady_s": frame["matched_steady_ns"] * 1.0e-9,
                    "published_steady_s": frame["published_steady_ns"] * 1.0e-9,
                    "tracker_arrival_steady_s": frame["tracker_arrival_steady_ns"] * 1.0e-9,
                    "tracking_delivery_ms": (
                        frame["tracker_arrival_steady_ns"]
                        - frame["published_steady_ns"]) * 1.0e-6,
                    "stage": frame["stage"],
                })

    def _write_latency(self):
        fields = [
            "pair_sequence", "input_stamp_ns", "input_stamp_s",
            "source_matched_steady_s", "published_steady_s",
            "odom_info_arrival_steady_s", "odom_output_arrival_steady_s",
            "end_to_end_latency_ms", "odom_info_callback_delay_ms",
            "latest_published_sequence", "lag_frames", "latest_frame_age_ms",
            "queue_age_upper_bound_ms", "processed_sequence_gap", "skipped_frames",
            "front_end_time_ms", "particle_filter_time_ms", "local_bundle_time_ms",
            "quality_inliers", "features", "matches", "lost", "key_frame_added",
            "local_map_size", "local_key_frames", "stage",
        ]
        with (self.output_dir / "rtab_latency.csv").open(
                "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for event in self.correlated_events:
                output_ns = self.odom_outputs.get(event["input_stamp_ns"])
                writer.writerow({
                    "pair_sequence": event.get("pair_sequence", ""),
                    "input_stamp_ns": event["input_stamp_ns"],
                    "input_stamp_s": event["input_stamp_ns"] * 1.0e-9,
                    "source_matched_steady_s": (
                        event.get("source_matched_steady_ns", 0) * 1.0e-9
                        if event.get("source_matched_steady_ns") else ""),
                    "published_steady_s": (
                        event.get("published_steady_ns", 0) * 1.0e-9
                        if event.get("published_steady_ns") else ""),
                    "odom_info_arrival_steady_s": (
                        event["odom_info_arrival_steady_ns"] * 1.0e-9
                        if event.get("odom_info_arrival_steady_ns") else ""),
                    "odom_output_arrival_steady_s": (
                        output_ns * 1.0e-9 if output_ns else ""),
                    "end_to_end_latency_ms": event.get("end_to_end_latency_ms", ""),
                    "odom_info_callback_delay_ms": event.get(
                        "odom_info_callback_delay_ms", ""),
                    "latest_published_sequence": event.get(
                        "latest_published_sequence", ""),
                    "lag_frames": event.get("lag_frames", ""),
                    "latest_frame_age_ms": event.get("latest_frame_age_ms", ""),
                    "queue_age_upper_bound_ms": event.get(
                        "queue_age_upper_bound_ms", ""),
                    "processed_sequence_gap": event.get("processed_sequence_gap", ""),
                    "skipped_frames": event.get("skipped_frames", ""),
                    **{key: event[key] for key in fields if key in event},
                })

    def _write_loops(self):
        fields = [
            "stamp_ns", "stamp_s", "arrival_steady_s", "ref_id",
            "candidate_id", "candidate_similarity", "geometric_inliers",
            "candidate_likelihood", "candidate_raw_likelihood",
            "posterior_best", "likelihood_best", "raw_likelihood_best",
            "visual_matches",
            "loop_closure_id", "proximity_detection_id", "rejected_hypothesis",
            "map_id", "update_time_ms", "loop_detection_time_ms",
            "optimization_time_ms", "optimization_error",
            "optimization_max_error", "map_to_odom_x", "map_to_odom_y",
            "map_to_odom_z", "map_to_odom_yaw", "stage",
        ]
        with (self.output_dir / "loop_closure.csv").open(
                "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for event in self.info_events:
                row = {key: event.get(key, "") for key in fields}
                row["stamp_s"] = event["stamp_ns"] * 1.0e-9
                row["arrival_steady_s"] = event["arrival_steady_ns"] * 1.0e-9
                writer.writerow(row)

    def _write_map(self):
        if self.latest_grid is None:
            return
        width = int(self.latest_grid.info.width)
        height = int(self.latest_grid.info.height)
        data = list(self.latest_grid.data)
        pixels = bytearray(
            205 if value < 0 else (0 if value >= 65 else 254)
            for value in data)
        pgm_path = self.output_dir / "map.pgm"
        with pgm_path.open("wb") as handle:
            handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            for row in range(height - 1, -1, -1):
                start = row * width
                handle.write(pixels[start:start + width])
        origin = self.latest_grid.info.origin.position
        (self.output_dir / "map.yaml").write_text(
            f"image: map.pgm\nresolution: {self.latest_grid.info.resolution}\n"
            f"origin: [{origin.x}, {origin.y}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8")

    def _finish(self):
        if self.done:
            return
        self.done = True
        self._recompute_correlations()
        self._write_frame_tracking()
        self._write_latency()
        self._write_loops()
        self._write_map()

        matched_events = self.correlated_events
        sequences = [event["pair_sequence"] for event in matched_events]
        sequence_span = (
            max(sequences) - min(sequences) + 1 if sequences else 0)
        unique_processed = len(set(sequences))
        processed_ratio = (
            unique_processed / sequence_span if sequence_span else 0.0)
        skipped_ratio = 1.0 - processed_ratio if sequence_span else 0.0
        bridge_gaps = sum(
            max(0, current["pair_sequence"] - previous["pair_sequence"] - 1)
            for previous, current in zip(self.frames, self.frames[1:]))
        latency_values = [
            event["end_to_end_latency_ms"] for event in matched_events]
        latency_times = [
            (event["odom_output_arrival_steady_ns"] - self.started_ns) * 1.0e-9
            for event in matched_events]
        latency = finite_summary(latency_values)
        latency_trend = self._linear_trend(latency_values, latency_times)
        lag = finite_summary([event.get("lag_frames", float("nan"))
                              for event in matched_events])
        latest_age = finite_summary([
            event.get("latest_frame_age_ms", float("nan"))
            for event in matched_events])
        queue_age = finite_summary([
            event.get("queue_age_upper_bound_ms", float("nan"))
            for event in matched_events])
        info_callback_delay = finite_summary([
            event.get("odom_info_callback_delay_ms", float("nan"))
            for event in matched_events])
        tracking_delivery = finite_summary([
            (frame["tracker_arrival_steady_ns"] - frame["published_steady_ns"])
            * 1.0e-6 for frame in self.frames])
        frontend = finite_summary([
            event["front_end_time_ms"] for event in self.odom_events
            if event.get("front_end_time_ms") is not None])
        map_update = finite_summary([
            event["update_time_ms"] for event in self.info_events
            if event["update_time_ms"] is not None])
        logs = self._log_counts()
        accepted_global = [event for event in self.info_events
                           if event["loop_closure_id"] > 0]
        accepted_proximity = [event for event in self.info_events
                              if event["proximity_detection_id"] > 0]
        accepted = [event for event in self.info_events
                    if (event["loop_closure_id"] > 0
                        or event["proximity_detection_id"] > 0)]
        rejected = [event for event in self.info_events
                    if event["rejected_hypothesis"] > 0.0]
        candidates = [event for event in self.info_events
                      if (event["candidate_id"] > 0
                          and (event["candidate_likelihood"] > 0.0
                               or event["posterior_best"] > 0.0
                               or event["loop_closure_id"] > 0
                               or event["rejected_hypothesis"] > 0.0))]
        trajectory = self._trajectory_metrics(
            self.trajectories["ground_truth"], self.trajectories["rtabmap"])
        closure = {
            stream: self._closure_metrics(samples)
            for stream, samples in self.trajectories.items()
        }
        resources = {
            key: finite_summary([row[key] for row in self.resource_samples])
            for key in (
                "rtf", "total_cpu_percent", "pipeline_cpu_percent",
                "gazebo_cpu_percent", "bridge_cpu_percent", "rtab_cpu_percent",
                "pipeline_rss_bytes", "gazebo_rss_bytes", "bridge_rss_bytes",
                "rtab_rss_bytes", "gpu_util_percent", "gpu_memory_used_mb")
        }

        fail_reasons = []
        warn_reasons = []
        if self.profile != "t0":
            if self.lost_events or logs["lost"]:
                fail_reasons.append("odometry lost")
            if logs["reset"]:
                fail_reasons.append("odometry reset")
            if self.tf_time_jumps:
                fail_reasons.append("TF time moved backward")
            if trajectory.get("trajectory_breaks_over_1m", 0):
                fail_reasons.append("trajectory step exceeded 1 m")
            if any(value == 0 for value in self.quality_values):
                fail_reasons.append("RTAB quality reached zero")
            if (latency_trend["slope_per_s"] is not None
                    and latency_trend["slope_per_s"] > 5.0
                    and latency_trend["r_squared"] > 0.5):
                fail_reasons.append("end-to-end latency grew continuously")
            if trajectory.get("available") and trajectory["ate_rmse_m"] > 0.10:
                warn_reasons.append("ATE exceeded 10 cm")
            if self.quality_values and min(self.quality_values) < 20:
                warn_reasons.append("quality dropped below 20")
            if rejected:
                warn_reasons.append("loop candidate was rejected")
            if latency and latency["max"] > 500.0:
                warn_reasons.append("an end-to-end latency sample exceeded 500 ms")
        elif bridge_gaps:
            fail_reasons.append("bridge tracking sequence gap")
        classification = "FAIL" if fail_reasons else (
            "WARN" if warn_reasons else "PASS")

        summary = {
            "profile": self.profile,
            "classification": classification,
            "duration_s": (time.monotonic_ns() - self.started_ns) * 1.0e-9,
            "bridge": {
                "rate": rate_summary([
                    frame["published_steady_ns"] for frame in self.frames]),
                "frames": len(self.frames),
                "pair_sequence_gaps": bridge_gaps,
            },
            "rtab": {
                "rate": rate_summary([
                    arrival_ns for arrival_ns in self.odom_outputs.values()]),
                "processed_frames": len(self.odom_outputs),
                "correlated_frames": len(matched_events),
                "processing_ratio": processed_ratio,
                "skipped_ratio": skipped_ratio,
                "latency_ms": latency,
                "latency_trend_ms_per_s": latency_trend,
                "lag_frames": lag,
                "latest_frame_age_ms": latest_age,
                "queue_age_upper_bound_ms": queue_age,
                "odom_info_callback_delay_ms": info_callback_delay,
                "tracking_delivery_ms": tracking_delivery,
                "front_end_time_ms": frontend,
                "map_update_time_ms": map_update,
                "quality": finite_summary(self.quality_values),
                "features": finite_summary(self.features_values),
                "lost_events": self.lost_events,
                "recoveries": self.recoveries,
                "lost_log_count": logs["lost"],
                "reset_log_count": logs["reset"],
            },
            "loop_closure": {
                "candidate_events": len(candidates),
                "accepted_events": len(accepted),
                "global_accepted_events": len(accepted_global),
                "proximity_accepted_events": len(accepted_proximity),
                "rejected_events": len(rejected),
                "global_accepted_ids": sorted(set(
                    event["loop_closure_id"] for event in accepted_global)),
                "proximity_accepted_ids": sorted(set(
                    event["proximity_detection_id"]
                    for event in accepted_proximity)),
                "wrong_loop_suspected": bool(
                    accepted and trajectory.get("trajectory_breaks_over_1m", 0)),
            },
            "trajectory": trajectory,
            "closure": closure,
            "tf_time_jumps": self.tf_time_jumps,
            "resources": resources,
            "fail_reasons": fail_reasons,
            "warn_reasons": warn_reasons,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        self._write_summary(summary)
        for handle in (
                self.trajectory_file, self.resource_file, self.tum_file,
                self.gt_tum_file, self.mavros_tum_file,
                self.ground_truth_file, self.odom_features_file):
            handle.flush()
            handle.close()
        self.get_logger().info(
            f"Robustness profile complete: {self.output_dir / 'summary.md'}")

    def _write_summary(self, summary):
        def number(section, key, digits=3):
            if not section or section.get(key) is None:
                return "n/a"
            return f"{section[key]:.{digits}f}"

        bridge = summary["bridge"]["rate"]
        rtab = summary["rtab"]
        rtab_rate = rtab["rate"]
        latency = rtab["latency_ms"] or {}
        trend = rtab["latency_trend_ms_per_s"]
        quality = rtab["quality"] or {}
        features = rtab["features"] or {}
        trajectory = summary["trajectory"]
        lines = [
            f"# D435i robustness profile: {self.profile}", "",
            f"Result: **{summary['classification']}**", "",
            "| Metric | Result |", "|---|---:|",
            f"| duration | {summary['duration_s']:.1f} s |",
            f"| bridge pair mean Hz | {bridge['mean_hz']:.3f} |",
            f"| bridge longest interval | {number(bridge, 'longest_interval_ms')} ms |",
            f"| bridge sequence gaps | {summary['bridge']['pair_sequence_gaps']} |",
            f"| RTAB odometry mean Hz | {rtab_rate['mean_hz']:.3f} |",
            f"| RTAB processing ratio | {rtab['processing_ratio']:.3f} |",
            f"| RTAB skipped-frame ratio | {rtab['skipped_ratio']:.3f} |",
            f"| RTAB output E2E mean / p95 / max | {number(latency, 'mean')} / "
            f"{number(latency, 'p95')} / {number(latency, 'max')} ms |",
            f"| latency slope / R^2 | {number(trend, 'slope_per_s')} ms/s / "
            f"{number(trend, 'r_squared')} |",
            f"| tracking delivery mean / p95 | "
            f"{number(rtab['tracking_delivery_ms'], 'mean')} / "
            f"{number(rtab['tracking_delivery_ms'], 'p95')} ms |",
            f"| OdomInfo observer delay mean / p95 | "
            f"{number(rtab['odom_info_callback_delay_ms'], 'mean')} / "
            f"{number(rtab['odom_info_callback_delay_ms'], 'p95')} ms |",
            f"| front-end mean / p95 | {number(rtab['front_end_time_ms'], 'mean')} / "
            f"{number(rtab['front_end_time_ms'], 'p95')} ms |",
            f"| map update mean / p95 | {number(rtab['map_update_time_ms'], 'mean')} / "
            f"{number(rtab['map_update_time_ms'], 'p95')} ms |",
            f"| quality mean / minimum | {number(quality, 'mean')} / "
            f"{number(quality, 'min')} |",
            f"| features mean / minimum | {number(features, 'mean')} / "
            f"{number(features, 'min')} |",
            f"| lost / reset | {rtab['lost_events'] + rtab['lost_log_count']} / "
            f"{rtab['reset_log_count']} |",
            f"| loop candidate / global / proximity / rejected | "
            f"{summary['loop_closure']['candidate_events']} / "
            f"{summary['loop_closure']['global_accepted_events']} / "
            f"{summary['loop_closure']['proximity_accepted_events']} / "
            f"{summary['loop_closure']['rejected_events']} |",
            f"| TF backward jumps | {summary['tf_time_jumps']} |",
        ]
        if trajectory.get("available"):
            lines.extend([
                f"| ATE RMSE | {trajectory['ate_rmse_m']:.4f} m |",
                f"| RPE translation RMSE | "
                f"{trajectory['rpe_translation_rmse_m']:.4f} m |",
                f"| horizontal / height RMSE | "
                f"{trajectory['horizontal_rmse_m']:.4f} / "
                f"{trajectory['height_rmse_m']:.4f} m |",
                f"| yaw RMSE | {trajectory['yaw_rmse_deg']:.3f} deg |",
                f"| largest raw trajectory step | "
                f"{trajectory['largest_raw_step_m']:.4f} m |",
            ])
        lines.extend(["", "## Classification evidence", ""])
        if summary["fail_reasons"]:
            lines.extend(f"- FAIL: {reason}" for reason in summary["fail_reasons"])
        if summary["warn_reasons"]:
            lines.extend(f"- WARN: {reason}" for reason in summary["warn_reasons"])
        if not summary["fail_reasons"] and not summary["warn_reasons"]:
            lines.append("- No failure or warning criterion was observed.")
        lines.extend([
            "",
            "Frame sequence is carried on a side diagnostic topic; image payloads "
            "and RTAB-Map parameters are unchanged. End-to-end latency ends at the "
            "lightweight `/rtabmap/odom` callback. OdomInfo callback delay is "
            "reported separately so observer backlog cannot masquerade as SLAM "
            "latency. `queue_age_upper_bound_ms` is an observer-side upper bound "
            "between consecutive processed sequences, not internal queue access.",
            "",
        ])
        (self.output_dir / "summary.md").write_text(
            "\n".join(lines), encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapRobustnessProfiler()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stop_requested:
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        executor.remove_node(node)
        if not node.done:
            node._finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
