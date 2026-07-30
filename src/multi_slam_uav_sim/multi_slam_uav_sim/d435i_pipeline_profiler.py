#!/usr/bin/env python3
import bisect
import csv
import json
import math
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import psutil
import rclpy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(qx, qy, qz, qw):
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class D435iPipelineProfiler(Node):
    def __init__(self):
        super().__init__("d435i_pipeline_profiler")
        self.declare_parameter("duration_s", 60.0)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("color_topic", "/front/d435i/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/front/d435i/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "ground_truth_topic", "/d435i_visual_slam/ground_truth")
        self.declare_parameter("mavros_odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("rtabmap_odom_topic", "/rtabmap/odom")
        self.declare_parameter("stage_topic", "/d435i_visual_slam/stage")
        self.declare_parameter("image_qos_reliability", "best_effort")
        self.declare_parameter("image_qos_depth", 1)

        self.duration_s = max(float(self.get_parameter("duration_s").value), 5.0)
        output_dir = str(self.get_parameter("output_dir").value).strip()
        if not output_dir:
            output_dir = f"/tmp/d435i_profile_{int(time.time())}"
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.started_wall = time.monotonic()
        self.done = False
        self.stop_requested = False
        self.stage = "unlabelled"

        self.image_arrivals = {"color": [], "depth": []}
        self.image_stamps = {"color": [], "depth": []}
        self.image_transport_latency = {"color": [], "depth": []}
        self.rgb_depth_deltas = []
        self.trajectories = {"ground_truth": [], "mavros": [], "rtabmap": []}
        self.clock_samples = []
        self.system_samples = []
        self.odom_quality = []
        self.odom_info_arrivals = []
        self.odom_lost_events = 0
        self.odom_recoveries = 0
        self.last_odom_lost = False
        self.quality_zero_count = 0
        self.accepted_loop_closures = 0
        self.rejected_loop_closures = 0
        self.last_loop_closure_id = 0
        self.map_ids = []
        self.map_id_changes = 0
        self.last_map_id = None
        self.rtabmap_update_times_ms = []
        self.tf_last_stamp = {}
        self.tf_time_jumps = 0
        self.process_handles = {}

        self.trajectory_file = self._csv_file(
            "trajectories.csv",
            ["stream", "stamp", "raw_header_stamp", "x", "y", "z",
             "qx", "qy", "qz", "qw", "stage"],
        )
        self.tum_files = {
            name: (self.output_dir / f"{name}.tum").open(
                "w", encoding="utf-8", buffering=1)
            for name in self.trajectories
        }
        self.image_file = self._csv_file(
            "image_timing.csv",
            ["stream", "arrival_wall_s", "stamp", "interval_s",
             "transport_latency_s", "rgb_depth_stamp_delta_s"],
        )
        self.system_file = self._csv_file(
            "system_metrics.csv",
            ["elapsed_s", "rtf", "cpu_percent", "memory_used_bytes",
             "memory_percent", "swap_used_bytes", "pipeline_cpu_percent",
             "pipeline_rss_bytes", "gpu_util_percent", "gpu_memory_used_mb",
             "gpu_memory_total_mb"],
        )
        self.rtabmap_file = self._csv_file(
            "rtabmap_metrics.csv",
            ["stamp", "quality_inliers", "features", "matches", "lost",
             "local_map_size", "time_estimation_s", "memory_usage_mb", "stage"],
        )
        self.rtabmap_info_file = self._csv_file(
            "rtabmap_info.csv",
            ["stamp", "ref_id", "map_id", "loop_closure_id",
             "rejected_hypothesis", "update_time_ms", "stage"],
        )

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(self.get_parameter("image_qos_depth").value)),
            durability=DurabilityPolicy.VOLATILE,
        )
        if str(self.get_parameter("image_qos_reliability").value) == "reliable":
            image_qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Image, color_topic,
            lambda message: self._image_cb("color", message), image_qos)
        self.create_subscription(
            Image, depth_topic,
            lambda message: self._image_cb("depth", message), image_qos)
        self.create_subscription(
            Odometry, str(self.get_parameter("ground_truth_topic").value),
            lambda message: self._odom_cb("ground_truth", message), 30)
        self.create_subscription(
            Odometry, str(self.get_parameter("mavros_odom_topic").value),
            lambda message: self._odom_cb("mavros", message), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter("rtabmap_odom_topic").value),
            lambda message: self._odom_cb("rtabmap", message), qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self._clock_cb, 20)
        self.create_subscription(
            OdomInfo, "/rtabmap/odom_info", self._odom_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Info, "/rtabmap/info", self._info_cb, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter("stage_topic").value), self._stage_cb, 10)
        self.create_subscription(TFMessage, "/tf", self._tf_cb, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf_static", self._tf_cb, 10)
        self.system_callback_group = ReentrantCallbackGroup()
        self.create_timer(
            1.0, self._system_timer,
            callback_group=self.system_callback_group)
        self.get_logger().info(
            f"Profiling D435i visual pipeline for {self.duration_s:.1f}s -> "
            f"{self.output_dir}")

    def _csv_file(self, name, fields):
        handle = (self.output_dir / name).open(
            "w", newline="", encoding="utf-8", buffering=1)
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        return handle, writer

    def _stage_cb(self, message):
        self.stage = message.data or "unlabelled"

    def _image_cb(self, stream, message):
        now = time.monotonic()
        arrival = now - self.started_wall
        stamp = stamp_seconds(message.header.stamp)
        previous = self.image_arrivals[stream][-1] if self.image_arrivals[stream] else None
        interval = arrival - previous if previous is not None else 0.0
        self.image_arrivals[stream].append(arrival)
        self.image_stamps[stream].append(stamp)

        transport_latency = ""
        if self.clock_samples:
            clock_wall, clock_sim = self.clock_samples[-1]
            clock_rate = 1.0
            if len(self.clock_samples) >= 2:
                previous_wall, previous_sim = self.clock_samples[-2]
                wall_delta = clock_wall - previous_wall
                if wall_delta > 1.0e-6:
                    clock_rate = max(
                        0.0, min(2.0, (clock_sim - previous_sim) / wall_delta))
            estimated_sim_now = clock_sim + (now - clock_wall) * clock_rate
            latency_value = estimated_sim_now - stamp
            if 0.0 <= latency_value <= 5.0:
                self.image_transport_latency[stream].append(latency_value)
                transport_latency = latency_value

        delta = ""
        if stream == "depth" and self.image_stamps["color"]:
            delta_value = abs(stamp - self.image_stamps["color"][-1])
            self.rgb_depth_deltas.append(delta_value)
            delta = delta_value
        _, writer = self.image_file
        writer.writerow({
            "stream": stream,
            "arrival_wall_s": f"{arrival:.9f}",
            "stamp": f"{stamp:.9f}",
            "interval_s": f"{interval:.9f}",
            "transport_latency_s": transport_latency,
            "rgb_depth_stamp_delta_s": delta,
        })

    def _odom_cb(self, stream, message):
        raw_stamp = stamp_seconds(message.header.stamp)
        stamp = raw_stamp
        # MAVROS uses host time in this baseline, while Gazebo and RTAB-Map
        # use /clock. Convert only the evaluation timestamp and retain the raw
        # header timestamp in the CSV for auditability.
        if stream == "mavros" and self.clock_samples:
            stamp = self.clock_samples[-1][1]
        pose = message.pose.pose
        sample = (
            stamp, float(pose.position.x), float(pose.position.y),
            float(pose.position.z), float(pose.orientation.x),
            float(pose.orientation.y), float(pose.orientation.z),
            float(pose.orientation.w),
        )
        self.trajectories[stream].append(sample)
        _, writer = self.trajectory_file
        writer.writerow(dict(zip(
            ["stream", "stamp", "raw_header_stamp", "x", "y", "z",
             "qx", "qy", "qz", "qw", "stage"],
            [stream, stamp, raw_stamp, *sample[1:], self.stage],
        )))
        self.tum_files[stream].write(
            "{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(
                *sample))

    def _clock_cb(self, message):
        sim_s = stamp_seconds(message.clock)
        self.clock_samples.append((time.monotonic(), sim_s))
        if len(self.clock_samples) > 1000:
            self.clock_samples = self.clock_samples[-500:]

    def _odom_info_cb(self, message):
        stamp = stamp_seconds(message.header.stamp)
        quality = int(message.inliers)
        lost = bool(message.lost)
        self.odom_quality.append(quality)
        self.odom_info_arrivals.append(time.monotonic() - self.started_wall)
        if quality == 0:
            self.quality_zero_count += 1
        if lost and not self.last_odom_lost:
            self.odom_lost_events += 1
        if not lost and self.last_odom_lost:
            self.odom_recoveries += 1
        self.last_odom_lost = lost
        _, writer = self.rtabmap_file
        writer.writerow({
            "stamp": f"{stamp:.9f}",
            "quality_inliers": quality,
            "features": int(message.features),
            "matches": int(message.matches),
            "lost": int(lost),
            "local_map_size": int(message.local_map_size),
            "time_estimation_s": float(message.time_estimation),
            "memory_usage_mb": int(message.memory_usage),
            "stage": self.stage,
        })

    def _info_cb(self, message):
        stats = dict(zip(message.stats_keys, message.stats_values))
        loop_id = int(message.loop_closure_id)
        if loop_id > 0 and loop_id != self.last_loop_closure_id:
            self.accepted_loop_closures += 1
            self.last_loop_closure_id = loop_id
        rejected = float(stats.get("Loop/RejectedHypothesis/", 0.0))
        if rejected > 0.0:
            self.rejected_loop_closures += 1
        map_id_value = stats.get("Loop/Map_id/")
        map_id = ""
        if map_id_value is not None:
            map_id = int(round(float(map_id_value)))
            self.map_ids.append(map_id)
            if self.last_map_id is not None and map_id != self.last_map_id:
                self.map_id_changes += 1
            self.last_map_id = map_id
        update_time_ms = stats.get("RtabmapROS/TimeTotal/ms")
        if update_time_ms is not None:
            self.rtabmap_update_times_ms.append(float(update_time_ms))
        _, writer = self.rtabmap_info_file
        writer.writerow({
            "stamp": f"{stamp_seconds(message.header.stamp):.9f}",
            "ref_id": int(message.ref_id),
            "map_id": map_id,
            "loop_closure_id": loop_id,
            "rejected_hypothesis": rejected,
            "update_time_ms": "" if update_time_ms is None else update_time_ms,
            "stage": self.stage,
        })

    def _tf_cb(self, message):
        for transform in message.transforms:
            stamp = stamp_seconds(transform.header.stamp)
            key = transform.child_frame_id
            previous = self.tf_last_stamp.get(key)
            if previous is not None and stamp + 1.0e-6 < previous:
                self.tf_time_jumps += 1
            self.tf_last_stamp[key] = stamp

    def _rtf(self):
        if len(self.clock_samples) < 2:
            return float("nan")
        first_wall, first_sim = self.clock_samples[0]
        last_wall, last_sim = self.clock_samples[-1]
        wall_delta = last_wall - first_wall
        return (last_sim - first_sim) / wall_delta if wall_delta > 0.0 else float("nan")

    def _pipeline_resources(self):
        cpu = 0.0
        rss = 0
        markers = (
            "gz sim", "arducopter", "mavros", "d435i_sim_bridge",
            "d435i_rgbd_bridge", "parameter_bridge", "rtabmap",
            "rgbd_odometry", "gazebo_ground_truth_bridge",
        )
        live_pids = set()
        for process in psutil.process_iter(["pid", "cmdline"]):
            try:
                command = " ".join(process.info.get("cmdline") or [])
                if not any(marker in command for marker in markers):
                    continue
                pid = int(process.info["pid"])
                live_pids.add(pid)
                handle = self.process_handles.setdefault(pid, psutil.Process(pid))
                cpu += handle.cpu_percent(None)
                rss += handle.memory_info().rss
            except (psutil.Error, OSError):
                continue
        self.process_handles = {
            pid: handle for pid, handle in self.process_handles.items()
            if pid in live_pids
        }
        return cpu, rss

    @staticmethod
    def _gpu_metrics():
        if shutil.which("nvidia-smi") is None:
            return (float("nan"), float("nan"), float("nan"))
        try:
            result = subprocess.run([
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,memory.total",
                "--format=csv,noheader,nounits",
            ], check=True, capture_output=True, text=True, timeout=2.0)
            values = [float(value.strip()) for value in
                      result.stdout.splitlines()[0].split(",")]
            return tuple(values[:3])
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            return (float("nan"), float("nan"), float("nan"))

    def _system_timer(self):
        if self.done:
            return
        elapsed = time.monotonic() - self.started_wall
        memory = psutil.virtual_memory()
        swap = psutil.swap_memory()
        pipeline_cpu, pipeline_rss = self._pipeline_resources()
        gpu_util, gpu_used, gpu_total = self._gpu_metrics()
        row = {
            "elapsed_s": elapsed,
            "rtf": self._rtf(),
            "cpu_percent": psutil.cpu_percent(None),
            "memory_used_bytes": int(memory.used),
            "memory_percent": float(memory.percent),
            "swap_used_bytes": int(swap.used),
            "pipeline_cpu_percent": pipeline_cpu,
            "pipeline_rss_bytes": pipeline_rss,
            "gpu_util_percent": gpu_util,
            "gpu_memory_used_mb": gpu_used,
            "gpu_memory_total_mb": gpu_total,
        }
        self.system_samples.append(row)
        _, writer = self.system_file
        writer.writerow(row)
        if elapsed >= self.duration_s:
            self.stop_requested = True

    @staticmethod
    def _rate_summary(arrivals):
        if len(arrivals) < 2:
            return {"count": len(arrivals), "mean_hz": 0.0,
                    "median_hz": 0.0, "p05_hz": 0.0,
                    "p95_hz": 0.0, "min_hz": 0.0, "max_hz": 0.0,
                    "longest_interval_s": None}
        intervals = np.diff(np.asarray(arrivals, dtype=float))
        rates = 1.0 / np.maximum(intervals, 1.0e-9)
        return {
            "count": len(arrivals),
            "mean_hz": float((len(arrivals) - 1) / (arrivals[-1] - arrivals[0])),
            "median_hz": float(np.median(rates)),
            "p05_hz": float(np.percentile(rates, 5)),
            "p95_hz": float(np.percentile(rates, 95)),
            "min_hz": float(np.min(rates)),
            "max_hz": float(np.max(rates)),
            "longest_interval_s": float(np.max(intervals)),
        }

    @staticmethod
    def _associate(reference, estimate, tolerance_s=0.12):
        if not reference or not estimate:
            return [], []
        reference_stamps = [sample[0] for sample in reference]
        matched_reference = []
        matched_estimate = []
        for sample in estimate:
            index = bisect.bisect_left(reference_stamps, sample[0])
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
            return {"matched": len(reference), "available": False}
        ref_position = np.asarray([sample[1:4] for sample in reference])
        est_position = np.asarray([sample[1:4] for sample in estimate])
        est_center = est_position.mean(axis=0)
        ref_center = ref_position.mean(axis=0)
        covariance = (est_position - est_center).T @ (ref_position - ref_center)
        left, _, right_t = np.linalg.svd(covariance)
        rotation = left @ right_t
        if np.linalg.det(rotation) < 0.0:
            left[:, -1] *= -1.0
            rotation = left @ right_t
        aligned = (est_position - est_center) @ rotation + ref_center
        error = aligned - ref_position
        norms = np.linalg.norm(error, axis=1)
        horizontal = np.linalg.norm(error[:, :2], axis=1)

        ref_yaw = np.asarray([quaternion_yaw(*sample[4:8]) for sample in reference])
        est_yaw = np.asarray([quaternion_yaw(*sample[4:8]) for sample in estimate])
        yaw_offset = math.atan2(
            np.sin(ref_yaw - est_yaw).mean(),
            np.cos(ref_yaw - est_yaw).mean())
        yaw_error = np.asarray([
            abs(wrap_angle(ref - est - yaw_offset))
            for ref, est in zip(ref_yaw, est_yaw)
        ])

        ref_delta = np.diff(ref_position, axis=0)
        est_delta = np.diff(aligned, axis=0)
        rpe = np.linalg.norm(est_delta - ref_delta, axis=1)
        steps = np.linalg.norm(np.diff(est_position, axis=0), axis=1)
        return {
            "available": True,
            "matched": len(reference),
            "ate_rmse_m": float(np.sqrt(np.mean(norms ** 2))),
            "ate_median_m": float(np.median(norms)),
            "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal ** 2))),
            "height_rmse_m": float(np.sqrt(np.mean(error[:, 2] ** 2))),
            "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
            "rpe_translation_rmse_m": float(np.sqrt(np.mean(rpe ** 2))) if len(rpe) else 0.0,
            "trajectory_breaks_over_1m": int(np.count_nonzero(steps > 1.0)),
            "largest_raw_step_m": float(np.max(steps)) if len(steps) else 0.0,
        }

    @staticmethod
    def _finite_summary(samples, key):
        values = np.asarray([row[key] for row in samples], dtype=float)
        values = values[np.isfinite(values)]
        if not len(values):
            return None
        return {"mean": float(np.mean(values)), "median": float(np.median(values)),
                "p05": float(np.percentile(values, 5)),
                "p95": float(np.percentile(values, 95)),
                "min": float(np.min(values)), "max": float(np.max(values))}

    @staticmethod
    def _value_summary(values):
        finite = np.asarray(values, dtype=float)
        finite = finite[np.isfinite(finite)]
        if not len(finite):
            return None
        return {"count": int(len(finite)), "mean": float(np.mean(finite)),
                "median": float(np.median(finite)),
                "p05": float(np.percentile(finite, 5)),
                "p95": float(np.percentile(finite, 95)),
                "min": float(np.min(finite)), "max": float(np.max(finite))}

    @staticmethod
    def _stamp_delta_summary(color_stamps, depth_stamps):
        if not color_stamps or not depth_stamps:
            return {"count": 0, "mean": None, "max": None,
                    "exact_match_fraction": None,
                    "exact_pair_count": 0,
                    "exact_sync_failure_messages": None}
        ordered_color = sorted(color_stamps)
        deltas = []
        for stamp in depth_stamps:
            index = bisect.bisect_left(ordered_color, stamp)
            candidates = [candidate for candidate in (index - 1, index)
                          if 0 <= candidate < len(ordered_color)]
            deltas.append(min(
                abs(stamp - ordered_color[candidate])
                for candidate in candidates))
        values = np.asarray(deltas, dtype=float)
        exact_count = int(np.count_nonzero(values <= 1.0e-9))
        return {
            "count": len(deltas),
            "mean": float(np.mean(values)),
            "max": float(np.max(values)),
            "exact_match_fraction": float(np.mean(values <= 1.0e-9)),
            "exact_pair_count": exact_count,
            "exact_sync_failure_messages": int(
                len(color_stamps) + len(depth_stamps) - 2 * exact_count),
        }

    def _finish(self):
        if self.done:
            return
        self.done = True
        summary = {
            "duration_s": time.monotonic() - self.started_wall,
            "color": self._rate_summary(self.image_arrivals["color"]),
            "aligned_depth": self._rate_summary(self.image_arrivals["depth"]),
            "rgb_depth_stamp_delta_s": self._stamp_delta_summary(
                self.image_stamps["color"], self.image_stamps["depth"]),
            "transport_latency_s": {
                stream: self._value_summary(values)
                for stream, values in self.image_transport_latency.items()
            },
            "rtf": self._finite_summary(self.system_samples, "rtf"),
            "cpu_percent": self._finite_summary(self.system_samples, "cpu_percent"),
            "pipeline_cpu_percent": self._finite_summary(
                self.system_samples, "pipeline_cpu_percent"),
            "memory_used_bytes": self._finite_summary(
                self.system_samples, "memory_used_bytes"),
            "pipeline_rss_bytes": self._finite_summary(
                self.system_samples, "pipeline_rss_bytes"),
            "swap_used_bytes": self._finite_summary(
                self.system_samples, "swap_used_bytes"),
            "gpu_util_percent": self._finite_summary(
                self.system_samples, "gpu_util_percent"),
            "gpu_memory_used_mb": self._finite_summary(
                self.system_samples, "gpu_memory_used_mb"),
            "rtabmap": {
                "processing_rate": self._rate_summary(
                    self.odom_info_arrivals),
                "quality_count": len(self.odom_quality),
                "quality_mean": float(np.mean(self.odom_quality)) if self.odom_quality else None,
                "quality_min": int(min(self.odom_quality)) if self.odom_quality else None,
                "quality_zero_count": self.quality_zero_count,
                "odometry_lost_events": self.odom_lost_events,
                "odometry_recoveries": self.odom_recoveries,
                "accepted_loop_closures": self.accepted_loop_closures,
                "rejected_loop_closures": self.rejected_loop_closures,
                "map_ids": sorted(set(self.map_ids)),
                "map_id_changes": self.map_id_changes,
                "update_time_ms": {
                    "mean": float(np.mean(self.rtabmap_update_times_ms))
                    if self.rtabmap_update_times_ms else None,
                    "max": float(np.max(self.rtabmap_update_times_ms))
                    if self.rtabmap_update_times_ms else None,
                },
            },
            "tf_time_jumps": self.tf_time_jumps,
            "trajectory_metrics": {
                "rtabmap_vs_ground_truth": self._trajectory_metrics(
                    self.trajectories["ground_truth"], self.trajectories["rtabmap"]),
                "mavros_vs_ground_truth": self._trajectory_metrics(
                    self.trajectories["ground_truth"], self.trajectories["mavros"]),
            },
            "trajectory_sample_counts": {
                name: len(samples) for name, samples in self.trajectories.items()
            },
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        self._write_markdown_summary(summary)
        for handle, _ in (self.trajectory_file, self.image_file,
                          self.system_file, self.rtabmap_file,
                          self.rtabmap_info_file):
            handle.flush()
            handle.close()
        for handle in self.tum_files.values():
            handle.flush()
            handle.close()
        self.get_logger().info(
            f"D435i profile complete: {self.output_dir / 'summary.json'}")

    def _write_markdown_summary(self, summary):
        def value(section, key, digits=3):
            item = section.get(key)
            return "n/a" if item is None else f"{item:.{digits}f}"

        color = summary["color"]
        depth = summary["aligned_depth"]
        rtabmap = summary["rtabmap"]
        rtf = summary.get("rtf") or {}
        trajectory = summary["trajectory_metrics"]["rtabmap_vs_ground_truth"]
        color_latency = summary["transport_latency_s"].get("color") or {}
        depth_latency = summary["transport_latency_s"].get("depth") or {}
        lines = [
            "# D435i visual pipeline profile",
            "",
            f"Duration: {summary['duration_s']:.1f} s",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| RGB mean / median Hz | {color['mean_hz']:.3f} / {color['median_hz']:.3f} |",
            f"| RGB p5 / p95 Hz | {color['p05_hz']:.3f} / {color['p95_hz']:.3f} |",
            f"| RGB minimum / maximum Hz | {color['min_hz']:.3f} / {color['max_hz']:.3f} |",
            f"| aligned depth mean / median Hz | {depth['mean_hz']:.3f} / {depth['median_hz']:.3f} |",
            f"| aligned depth p5 / p95 Hz | {depth['p05_hz']:.3f} / {depth['p95_hz']:.3f} |",
            f"| aligned depth minimum / maximum Hz | {depth['min_hz']:.3f} / {depth['max_hz']:.3f} |",
            f"| RGB longest interval | {value(color, 'longest_interval_s')} s |",
            f"| depth longest interval | {value(depth, 'longest_interval_s')} s |",
            f"| RGB/depth maximum stamp delta | {value(summary['rgb_depth_stamp_delta_s'], 'max', 6)} s |",
            f"| RGB/depth exact-stamp match fraction | {value(summary['rgb_depth_stamp_delta_s'], 'exact_match_fraction', 3)} |",
            f"| RGB/depth exact-sync failure messages | {summary['rgb_depth_stamp_delta_s'].get('exact_sync_failure_messages', 'n/a')} |",
            f"| RGB transport latency mean / p95 | {value(color_latency, 'mean')} / {value(color_latency, 'p95')} s |",
            f"| depth transport latency mean / p95 | {value(depth_latency, 'mean')} / {value(depth_latency, 'p95')} s |",
            f"| RTF median / minimum | {value(rtf, 'median')} / {value(rtf, 'min')} |",
            f"| RTAB inlier quality mean / minimum | {rtabmap['quality_mean']} / {rtabmap['quality_min']} |",
            f"| RTAB actual processing mean / median Hz | {rtabmap['processing_rate']['mean_hz']:.3f} / {rtabmap['processing_rate']['median_hz']:.3f} |",
            f"| RTAB quality=0 samples | {rtabmap['quality_zero_count']} |",
            f"| odometry lost / recoveries | {rtabmap['odometry_lost_events']} / {rtabmap['odometry_recoveries']} |",
            f"| accepted loop closures | {rtabmap['accepted_loop_closures']} |",
            f"| rejected loop closures | {rtabmap['rejected_loop_closures']} |",
            f"| observed map IDs / changes | {rtabmap['map_ids']} / {rtabmap['map_id_changes']} |",
            f"| RTAB update time mean / max | {value(rtabmap['update_time_ms'], 'mean')} / {value(rtabmap['update_time_ms'], 'max')} ms |",
            f"| TF backward time jumps | {summary['tf_time_jumps']} |",
        ]
        if trajectory.get("available"):
            lines.extend([
                f"| aligned ATE RMSE | {trajectory['ate_rmse_m']:.4f} m |",
                f"| RPE translation RMSE | {trajectory['rpe_translation_rmse_m']:.4f} m |",
                f"| horizontal / height RMSE | {trajectory['horizontal_rmse_m']:.4f} / {trajectory['height_rmse_m']:.4f} m |",
                f"| yaw RMSE | {trajectory['yaw_rmse_deg']:.3f} deg |",
            ])
        else:
            lines.append(
                f"| trajectory accuracy | unavailable ({trajectory.get('matched', 0)} matched samples) |")
        lines.extend([
            "",
            "The profiler treats RTAB-Map odometry inliers as the quality value. "
            "Trajectory errors use timestamp association followed by rigid SE(3) "
            "alignment without scale.",
            "",
        ])
        (self.output_dir / "summary.md").write_text(
            "\n".join(lines), encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = D435iPipelineProfiler()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stop_requested:
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=5.0)
        if not node.done:
            node._finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
