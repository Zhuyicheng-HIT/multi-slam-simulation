#!/usr/bin/env python3
import argparse
import json
import math
import struct
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, PointCloud2, PointField


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def rotate_to_initial(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([c * x + s * y, -s * x + c * y])


def align_xy(estimate, truth):
    """Rigidly align estimate to truth without changing trajectory scale."""
    estimate = np.asarray(estimate, dtype=float)
    truth = np.asarray(truth, dtype=float)
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
    return (rotation @ estimate.T).T + translation, rotation, translation


def wrap_angle(values):
    values = np.asarray(values, dtype=float)
    return np.arctan2(np.sin(values), np.cos(values))


def cloud_xyz(msg, max_points=6000):
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"}.issubset(fields) or msg.point_step <= 0:
        return np.empty((0, 3), dtype=np.float64)
    count = min(int(msg.width) * int(msg.height), len(msg.data) // int(msg.point_step))
    if count <= 0:
        return np.empty((0, 3), dtype=np.float64)
    stride = max(1, count // max_points)
    unpackers = {}
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype == PointField.FLOAT32:
            unpackers[name] = (field.offset, "<f")
        elif field.datatype == PointField.FLOAT64:
            unpackers[name] = (field.offset, "<d")
        else:
            return np.empty((0, 3), dtype=np.float64)
    points = []
    data = msg.data
    step = int(msg.point_step)
    for index in range(0, count, stride):
        base = index * step
        xyz = [struct.unpack_from(fmt, data, base + offset)[0]
               for offset, fmt in unpackers.values()]
        if all(math.isfinite(value) for value in xyz):
            points.append(xyz)
    return np.asarray(points, dtype=np.float64)


class SlamDriftAnalyzer(Node):
    def __init__(self, voxel_size):
        super().__init__("slam_drift_analyzer")
        self.voxel_size = voxel_size
        self.fast = []
        self.truth = []
        self.imu = []
        self.raw_stamp_regressions = 0
        self.raw_stamp_duplicates = 0
        self.registered_stamp_regressions = 0
        self.registered_stamp_duplicates = 0
        self.imu_stamp_regressions = 0
        self.imu_stamp_duplicates = 0
        self.last_raw_stamp = 0
        self.last_registered_stamp = 0
        self.last_imu_stamp = 0
        self.raw_timing = []
        self.registered_timing = []
        self.cloud_overlaps = []
        self.cloud_centroid_jumps = []
        self.previous_voxels = None
        self.previous_centroid = None

        self.create_subscription(Odometry, "/Odometry", self._fast_cb, 20)
        self.create_subscription(
            Odometry, "/sim/mid360/ground_truth_odom", self._truth_cb, qos_profile_sensor_data)
        self.create_subscription(
            Imu, "/livox/imu", self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, "/sim/mid360/points_raw", self._raw_cloud_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, "/cloud_registered", self._registered_cloud_cb, qos_profile_sensor_data)

    @staticmethod
    def _odom_record(msg):
        p = msg.pose.pose.position
        return (
            time.monotonic(), stamp_ns(msg.header), float(p.x), float(p.y), float(p.z),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def _fast_cb(self, msg):
        self.fast.append(self._odom_record(msg))

    def _truth_cb(self, msg):
        self.truth.append(self._odom_record(msg))

    def _imu_cb(self, msg):
        stamp = stamp_ns(msg.header)
        if stamp < self.last_imu_stamp:
            self.imu_stamp_regressions += 1
        elif stamp == self.last_imu_stamp and stamp != 0:
            self.imu_stamp_duplicates += 1
        self.last_imu_stamp = max(stamp, self.last_imu_stamp)
        self.imu.append((time.monotonic(), stamp, float(msg.angular_velocity.z)))

    def _raw_cloud_cb(self, msg):
        arrival = time.monotonic()
        stamp = stamp_ns(msg.header)
        if stamp < self.last_raw_stamp:
            self.raw_stamp_regressions += 1
        elif stamp == self.last_raw_stamp and stamp != 0:
            self.raw_stamp_duplicates += 1
        self.last_raw_stamp = max(stamp, self.last_raw_stamp)
        self.raw_timing.append((arrival, stamp))

    def _registered_cloud_cb(self, msg):
        arrival = time.monotonic()
        stamp = stamp_ns(msg.header)
        if stamp < self.last_registered_stamp:
            self.registered_stamp_regressions += 1
        elif stamp == self.last_registered_stamp and stamp != 0:
            self.registered_stamp_duplicates += 1
        self.last_registered_stamp = max(stamp, self.last_registered_stamp)
        self.registered_timing.append((arrival, stamp))
        points = cloud_xyz(msg)
        if len(points) < 20:
            return
        centroid = np.median(points, axis=0)
        voxels = {tuple(row) for row in np.floor(points / self.voxel_size).astype(np.int32)}
        if self.previous_voxels:
            union = len(voxels | self.previous_voxels)
            if union:
                self.cloud_overlaps.append(len(voxels & self.previous_voxels) / union)
        if self.previous_centroid is not None:
            self.cloud_centroid_jumps.append(float(np.linalg.norm(centroid - self.previous_centroid)))
        self.previous_voxels = voxels
        self.previous_centroid = centroid


def nearest_records(source, target, max_delta_s=0.05):
    if not source or not target:
        return []
    target_times = np.asarray([row[1] * 1.0e-9 for row in target])
    matches = []
    for row in source:
        source_time = row[1] * 1.0e-9
        index = int(np.searchsorted(target_times, source_time))
        candidates = [i for i in (index - 1, index) if 0 <= i < len(target)]
        best = min(candidates, key=lambda i: abs(target_times[i] - source_time))
        if abs(target_times[best] - source_time) <= max_delta_s:
            matches.append((row, target[best]))
    return matches


def percentile(values, q, default=None):
    return float(np.percentile(values, q)) if values else default


def timing_stats(records):
    if len(records) < 2:
        return {
            "samples": len(records),
            "stamp_period_median_ms": None,
            "stamp_period_p95_ms": None,
            "wall_arrival_minus_stamp_period_jitter_p95_ms": None,
            "arrival_minus_stamp_jitter_p95_ms": None,
        }
    arrivals = np.asarray([row[0] for row in records], dtype=np.float64)
    stamps = np.asarray([row[1] for row in records], dtype=np.float64) * 1.0e-9
    stamp_period = np.diff(stamps)
    scheduling_jitter = np.diff(arrivals) - stamp_period
    wall_jitter_p95_ms = float(
        np.percentile(np.abs(scheduling_jitter), 95) * 1.0e3
    )
    return {
        "samples": len(records),
        "stamp_period_median_ms": float(np.median(stamp_period) * 1.0e3),
        "stamp_period_p95_ms": float(np.percentile(stamp_period, 95) * 1.0e3),
        "wall_arrival_minus_stamp_period_jitter_p95_ms": wall_jitter_p95_ms,
        "arrival_minus_stamp_jitter_p95_ms": wall_jitter_p95_ms,
    }


def assess_coupling(coupling_corr, truth_imu_corr, threshold=0.65):
    failures = []
    warnings = []
    reference_valid = truth_imu_corr is not None and truth_imu_corr >= threshold
    if coupling_corr is None or truth_imu_corr is None:
        failures.append("有效偏航转动样本不足")
    elif not reference_valid:
        warnings.append("真值偏航与飞控陀螺参考相关系数低于 0.65，不能裁决 FAST-LIO 耦合")
    elif coupling_corr < threshold:
        failures.append("FAST-LIO 偏航与飞控陀螺相关系数低于 0.65")
    return reference_valid, failures, warnings


def build_report(node, sim_duration, wall_duration=None):
    matches = nearest_records(node.fast, node.truth)
    match_stamp_delta_ms = [abs(fast[1] - truth[1]) * 1.0e-6 for fast, truth in matches]
    report = {
        "duration_s": sim_duration,
        "sim_duration_s": sim_duration,
        "wall_duration_s": wall_duration,
        "observed_rtf": (
            None
            if wall_duration is None or wall_duration <= 0.0
            else sim_duration / wall_duration
        ),
        "samples": {
            "fast_lio_odom": len(node.fast),
            "ground_truth_odom": len(node.truth),
            "matched_odom": len(matches),
            "fast_lio_fcu_imu": len(node.imu),
            "registered_cloud_pairs": len(node.cloud_overlaps),
        },
        "timestamp_regressions": {
            "raw_cloud": node.raw_stamp_regressions,
            "registered_cloud": node.registered_stamp_regressions,
            "fast_lio_fcu_imu": node.imu_stamp_regressions,
        },
        "timestamp_duplicates": {
            "raw_cloud": node.raw_stamp_duplicates,
            "registered_cloud": node.registered_stamp_duplicates,
            "fast_lio_fcu_imu": node.imu_stamp_duplicates,
        },
        "pointcloud": {
            "voxel_overlap_p05": percentile(node.cloud_overlaps, 5),
            "voxel_overlap_median": percentile(node.cloud_overlaps, 50),
            "centroid_jump_p95_m": percentile(node.cloud_centroid_jumps, 95),
            "centroid_jump_max_m": max(node.cloud_centroid_jumps, default=None),
        },
        "timing": {
            "association_basis": "header_stamp",
            "algorithm_clock": "ros_sim_time",
            "performance_clock": "wall_monotonic",
            "fast_lio_odom": timing_stats(node.fast),
            "ground_truth_odom": timing_stats(node.truth),
            "fast_lio_fcu_imu": timing_stats(node.imu),
            "raw_cloud": timing_stats(node.raw_timing),
            "registered_cloud": timing_stats(node.registered_timing),
            "fast_truth_stamp_delta_p95_ms": percentile(match_stamp_delta_ms, 95),
            "fast_truth_stamp_delta_max_ms": max(match_stamp_delta_ms, default=None),
        },
    }

    if len(matches) < 10:
        report["passed"] = False
        report["failures"] = ["FAST-LIO 与真值里程计的匹配样本不足"]
        return report

    fast_rows = [pair[0] for pair in matches]
    truth_rows = [pair[1] for pair in matches]
    fast_xy = np.asarray([[row[2], row[3]] for row in fast_rows])
    truth_xy = np.asarray([[row[2], row[3]] for row in truth_rows])
    aligned_fast_xy, alignment_rotation, _ = align_xy(fast_xy, truth_xy)
    position_errors = np.linalg.norm(aligned_fast_xy - truth_xy, axis=1)
    fast_yaws = np.unwrap([row[5] for row in fast_rows])
    truth_yaws = np.unwrap([row[5] for row in truth_rows])
    alignment_yaw = math.atan2(alignment_rotation[1, 0], alignment_rotation[0, 0])
    aligned_fast_yaw = fast_yaws + alignment_yaw
    yaw_errors = np.rad2deg(wrap_angle(aligned_fast_yaw - truth_yaws))

    times = np.asarray([row[1] * 1.0e-9 for row in fast_rows])
    dt = np.diff(times)
    fast_yaw_rate = np.diff(fast_yaws) / np.maximum(dt, 1.0e-3)
    truth_yaw_rate = np.diff(truth_yaws) / np.maximum(dt, 1.0e-3)
    imu_times = np.asarray([row[1] * 1.0e-9 for row in node.imu])
    imu_z = np.asarray([row[2] for row in node.imu])
    best = None
    for lag_s in np.arange(-0.5, 0.501, 0.02):
        interval = []
        shifted = imu_times + lag_s
        for start, end in zip(times[:-1], times[1:]):
            mask = (shifted >= start) & (shifted < end)
            interval.append(float(np.mean(imu_z[mask])) if np.any(mask) else math.nan)
        interval = np.asarray(interval)
        candidate_turning = np.isfinite(interval) & (np.abs(truth_yaw_rate) > math.radians(5.0))
        if np.count_nonzero(candidate_turning) < 5:
            continue
        score = float(np.corrcoef(truth_yaw_rate[candidate_turning], interval[candidate_turning])[0, 1])
        if best is None or score > best[0]:
            best = (score, float(lag_s), interval, candidate_turning)
    if best is None:
        interval_imu = np.full_like(truth_yaw_rate, math.nan)
        turning = np.zeros_like(truth_yaw_rate, dtype=bool)
        imu_lag_s = None
    else:
        _, imu_lag_s, interval_imu, turning = best
    settled = np.abs(truth_yaw_rate) <= math.radians(5.0)
    if np.count_nonzero(turning) >= 5:
        coupling_corr = float(np.corrcoef(fast_yaw_rate[turning], interval_imu[turning])[0, 1])
        coupling_rmse = float(np.sqrt(np.mean((fast_yaw_rate[turning] - interval_imu[turning]) ** 2)))
        truth_imu_corr = float(np.corrcoef(truth_yaw_rate[turning], interval_imu[turning])[0, 1])
    else:
        coupling_corr = None
        coupling_rmse = None
        truth_imu_corr = None

    report["trajectory"] = {
        "position_rmse_m": float(np.sqrt(np.mean(np.square(position_errors)))),
        "position_error_max_m": max(position_errors),
        "yaw_rmse_deg": float(np.sqrt(np.mean(np.square(yaw_errors)))),
        "yaw_error_max_abs_deg": float(np.max(np.abs(yaw_errors))),
        "yaw_error_settled_p95_deg": float(np.percentile(np.abs(yaw_errors[1:][settled]), 95))
        if np.any(settled) else None,
        "final_position_error_m": position_errors[-1],
        "final_yaw_error_abs_deg": float(abs(yaw_errors[-1])),
        "fast_yaw_vs_fcu_gyro_corr": coupling_corr,
        "truth_yaw_vs_fcu_gyro_corr": truth_imu_corr,
        "fast_yaw_vs_fcu_gyro_rmse_rad_s": coupling_rmse,
        "estimated_fcu_imu_lag_s": imu_lag_s,
        "turning_samples": int(np.count_nonzero(turning)),
        "coupling_reference_valid": bool(
            truth_imu_corr is not None and truth_imu_corr >= 0.65
        ),
    }

    failures = []
    warnings = []
    if any(report["timestamp_regressions"].values()):
        failures.append("存在点云或 IMU 时间戳回退")
    if report["trajectory"]["position_rmse_m"] > 0.75:
        failures.append("位置 RMSE 超过 0.75 m")
    if report["trajectory"]["position_error_max_m"] > 1.5:
        failures.append("最大位置误差超过 1.5 m")
    if report["trajectory"]["yaw_rmse_deg"] > 12.0:
        failures.append("偏航 RMSE 超过 12 度")
    if report["trajectory"]["yaw_error_max_abs_deg"] > 40.0:
        failures.append("转向动态最大偏航误差超过 40 度")
    if report["trajectory"]["final_position_error_m"] > 0.5:
        failures.append("轨迹结束后的残余位置误差超过 0.5 m")
    if report["trajectory"]["final_yaw_error_abs_deg"] > 15.0:
        failures.append("轨迹结束后的残余偏航误差超过 15 度")
    settled_p95 = report["trajectory"]["yaw_error_settled_p95_deg"]
    if settled_p95 is not None and settled_p95 > 15.0:
        failures.append("非转向阶段偏航误差 P95 超过 15 度")
    _, coupling_failures, coupling_warnings = assess_coupling(coupling_corr, truth_imu_corr)
    failures.extend(coupling_failures)
    warnings.extend(coupling_warnings)
    overlap_p05 = report["pointcloud"]["voxel_overlap_p05"]
    if overlap_p05 is not None and overlap_p05 < 0.05:
        failures.append("连续注册点云体素重叠率出现突降")
    centroid_p95 = report["pointcloud"]["centroid_jump_p95_m"]
    if (centroid_p95 is not None and centroid_p95 > 3.5 and
            overlap_p05 is not None and overlap_p05 < 0.15):
        failures.append("注册点云质心跳变且体素重叠率过低")
    report["failures"] = failures
    report["warnings"] = warnings
    report["passed"] = not failures
    return report


def main():
    parser = argparse.ArgumentParser(description="Measure FAST-LIO drift, yaw/IMU coupling and cloud jumps")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--output", default="/tmp/multi_slam_slam_report.json")
    parser.add_argument("--voxel-size", type=float, default=0.5)
    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=0.0,
        help="Wall-clock watchdog only; zero selects max(60 s, 10x sim duration)",
    )
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    node = SlamDriftAnalyzer(args.voxel_size)
    wall_started = time.monotonic()
    wall_timeout = (
        args.wall_timeout
        if args.wall_timeout > 0.0
        else max(60.0, args.duration * 10.0)
    )
    ros_started = None
    last_ros_s = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now_ros_s = node.get_clock().now().nanoseconds * 1.0e-9
            if now_ros_s > 0.0 and ros_started is None:
                ros_started = now_ros_s
            if last_ros_s is not None and now_ros_s < last_ros_s:
                raise RuntimeError("ROS simulation clock moved backwards during evaluation")
            last_ros_s = now_ros_s
            if ros_started is not None and now_ros_s - ros_started >= args.duration:
                break
            if time.monotonic() - wall_started >= wall_timeout:
                raise RuntimeError(
                    "wall-clock watchdog expired while waiting for ROS simulation time"
                )
    finally:
        wall_duration = time.monotonic() - wall_started
        sim_duration = (
            0.0
            if ros_started is None or last_ros_s is None
            else max(0.0, last_ros_s - ros_started)
        )
        report = build_report(node, sim_duration, wall_duration)
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
