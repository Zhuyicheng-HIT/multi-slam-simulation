import bisect
import math
import os
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np
import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from mavros_msgs.msg import OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from multi_slam_uav_sim.optical_flow_model import sensor_displacement_frd


def stamp_seconds(stamp):
    """Return a ROS/Gazebo stamp in seconds, or zero when unavailable."""
    try:
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    except AttributeError:
        try:
            value = float(stamp.sec) + float(stamp.nsec) * 1.0e-9
        except AttributeError:
            return 0.0
    return value if math.isfinite(value) and value > 0.0 else 0.0


def accuracy_row_arrays(rows):
    """Decode captured rows without relying on positional fields at call sites."""
    estimates = np.asarray([[row[6], row[7]] for row in rows], dtype=float)
    truth = np.asarray([[row[4], row[5]] for row in rows], dtype=float)
    integration = np.asarray([row[2] for row in rows], dtype=float)
    arrival_intervals = np.asarray([row[3] for row in rows], dtype=float)
    quality = np.asarray([row[8] for row in rows], dtype=float)
    distance = np.asarray([row[9] for row in rows], dtype=float)
    return estimates, truth, integration, arrival_intervals, quality, distance


def select_association_basis(pose_source_times, flow_source_times,
                             maximum_clock_offset_s=1.0):
    """Choose source stamps only when both streams share one clock domain."""
    pose_times = np.asarray(pose_source_times, dtype=float)
    flow_times = np.asarray(flow_source_times, dtype=float)
    if pose_times.size < 2 or flow_times.size < 2:
        return "arrival"
    if np.any(~np.isfinite(pose_times)) or np.any(~np.isfinite(flow_times)):
        return "arrival"
    pose_mid = float(np.median(pose_times))
    flow_mid = float(np.median(flow_times))
    if abs(pose_mid - flow_mid) > float(maximum_clock_offset_s):
        return "arrival"
    overlap_start = max(float(np.min(pose_times)), float(np.min(flow_times)))
    overlap_end = min(float(np.max(pose_times)), float(np.max(flow_times)))
    return "source_stamp" if overlap_end > overlap_start else "arrival"


def yaw_rate_from_quaternions(start_xyzw, end_xyzw, interval_s):
    """Return the shortest absolute yaw rate between two body poses."""
    if not math.isfinite(float(interval_s)) or float(interval_s) <= 0.0:
        return float("nan")

    def yaw(quaternion):
        x, y, z, w = (float(value) for value in quaternion)
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    delta = yaw(end_xyzw) - yaw(start_xyzw)
    delta = math.atan2(math.sin(delta), math.cos(delta))
    return abs(delta) / float(interval_s)


class FlowGazeboAccuracy(Node):
    """Evaluate compensated flow displacement against the simulated sensor pose."""

    def __init__(self):
        super().__init__("flow_gazebo_accuracy")
        self.declare_parameter("flow_topic", "/sim/optical_flow/rad")
        self.declare_parameter("native_flow_topic", "/sim/optical_flow/rad_native")
        self.declare_parameter("gazebo_world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("gazebo_model", "apm_iris")
        self.declare_parameter("duration_s", 120.0)
        self.declare_parameter("min_quality", 80)
        self.declare_parameter("min_ground_distance_m", 0.6)
        self.declare_parameter("min_truth_speed_mps", 0.06)
        self.declare_parameter("max_truth_speed_mps", 2.0)
        self.declare_parameter("max_vertical_speed_mps", 0.35)
        self.declare_parameter("no_turn_max_yaw_rate_radps", 0.08)
        self.declare_parameter("no_turn_min_samples", 50)
        self.declare_parameter("max_pose_gap_s", 0.20)
        self.declare_parameter("association_basis", "source_stamp")
        self.declare_parameter("maximum_clock_offset_s", 1.0)
        self.declare_parameter("sensor_offset_z_down_m", 0.35)
        self.declare_parameter("csv_path", "")

        self.flow_topic = str(self.get_parameter("flow_topic").value)
        self.native_flow_topic = str(self.get_parameter("native_flow_topic").value)
        self.world_name = str(self.get_parameter("gazebo_world_name").value)
        self.model_name = str(self.get_parameter("gazebo_model").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.min_quality = int(self.get_parameter("min_quality").value)
        self.min_ground_distance_m = float(self.get_parameter("min_ground_distance_m").value)
        self.min_truth_speed_mps = float(self.get_parameter("min_truth_speed_mps").value)
        self.max_truth_speed_mps = float(self.get_parameter("max_truth_speed_mps").value)
        self.max_vertical_speed_mps = float(self.get_parameter("max_vertical_speed_mps").value)
        self.no_turn_max_yaw_rate = float(
            self.get_parameter("no_turn_max_yaw_rate_radps").value
        )
        self.no_turn_min_samples = int(
            self.get_parameter("no_turn_min_samples").value
        )
        self.max_pose_gap_s = float(self.get_parameter("max_pose_gap_s").value)
        self.association_basis = str(
            self.get_parameter("association_basis").value).lower()
        if self.association_basis not in {"auto", "source_stamp"}:
            raise ValueError(
                "accuracy association must use source_stamp (auto is an alias)")
        self.maximum_clock_offset_s = float(
            self.get_parameter("maximum_clock_offset_s").value)
        self.lever_arm_frd = (
            0.0,
            0.0,
            float(self.get_parameter("sensor_offset_z_down_m").value),
        )
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.lock = threading.Lock()
        self.pose_samples = []
        self.flow_samples = []
        self.native_flow_samples = {}
        self.last_flow_arrival = None
        self.start_ros_s = None
        self.last_ros_s = None
        self.start_wall_s = time.monotonic()
        self.done = False
        self.last_report = 0.0

        self.create_subscription(
            OpticalFlowRad, self.flow_topic, self._flow_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            OpticalFlowRad,
            self.native_flow_topic,
            self._native_flow_cb,
            qos_profile_sensor_data,
        )
        self.gz_node = GzNode()
        self.gz_pose_topics = (
            f"/world/{self.world_name}/dynamic_pose/info",
            f"/world/{self.world_name}/pose/info",
        )
        for topic in self.gz_pose_topics:
            self.gz_node.subscribe(Pose_V, topic, self._gz_pose_cb)
        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info(
            f"Recording compensated flow displacement for {self.duration_s:.1f}s: "
            f"flow={self.flow_topic}, native_flow={self.native_flow_topic}, "
            f"gazebo_model={self.model_name}"
        )

    @staticmethod
    def _stamp_key(stamp):
        return int(stamp.sec), int(stamp.nanosec)

    def _native_flow_cb(self, msg):
        key = self._stamp_key(msg.header.stamp)
        sample = (
            float(msg.integrated_x),
            float(msg.integrated_y),
            float(msg.integrated_xgyro),
            float(msg.integrated_ygyro),
        )
        with self.lock:
            self.native_flow_samples[key] = sample
            if len(self.native_flow_samples) > 2000:
                oldest = next(iter(self.native_flow_samples))
                self.native_flow_samples.pop(oldest, None)

    def _flow_cb(self, msg):
        arrival_time = time.monotonic()
        arrival_interval = (
            0.0
            if self.last_flow_arrival is None
            else arrival_time - self.last_flow_arrival
        )
        self.last_flow_arrival = arrival_time
        flow_stamp = stamp_seconds(msg.header.stamp)
        if flow_stamp <= 0.0:
            return
        integration_s = float(msg.integration_time_us) * 1.0e-6
        distance = float(msg.distance)
        raw_x = float(msg.integrated_x)
        raw_y = float(msg.integrated_y)
        gyro_x = float(msg.integrated_xgyro)
        gyro_y = float(msg.integrated_ygyro)
        estimated_dx = (raw_y - gyro_y) * distance
        estimated_dy = -(raw_x - gyro_x) * distance
        with self.lock:
            native = self.native_flow_samples.get(
                self._stamp_key(msg.header.stamp),
                (float("nan"),) * 4,
            )
            self.flow_samples.append((
                flow_stamp,
                arrival_time,
                integration_s,
                arrival_interval,
                estimated_dx,
                estimated_dy,
                int(msg.quality),
                distance,
                raw_x,
                raw_y,
                gyro_x,
                gyro_y,
                *native,
            ))

    def _gz_pose_cb(self, msg):
        arrival_time = time.monotonic()
        source_stamp = stamp_seconds(msg.header.stamp)
        if source_stamp <= 0.0:
            return
        for pose in msg.pose:
            if pose.name == self.model_name or pose.name.endswith(f"::{self.model_name}"):
                sample = (
                    source_stamp,
                    arrival_time,
                    (
                        float(pose.position.x),
                        float(pose.position.y),
                        float(pose.position.z),
                    ),
                    (
                        float(pose.orientation.x),
                        float(pose.orientation.y),
                        float(pose.orientation.z),
                        float(pose.orientation.w),
                    ),
                )
                with self.lock:
                    if not self.pose_samples or source_stamp > self.pose_samples[-1][0]:
                        self.pose_samples.append(sample)
                return

    def _timer_cb(self):
        ros_now_s = self.get_clock().now().nanoseconds * 1.0e-9
        if ros_now_s <= 0.0:
            return
        if self.last_ros_s is not None and ros_now_s < self.last_ros_s:
            raise RuntimeError("ROS simulation clock moved backwards during flow evaluation")
        self.last_ros_s = ros_now_s
        if self.start_ros_s is None:
            self.start_ros_s = ros_now_s
        elapsed = ros_now_s - self.start_ros_s
        now = time.monotonic()
        if now - self.last_report > 4.0:
            with self.lock:
                pose_count = len(self.pose_samples)
                flow_count = len(self.flow_samples)
                recent_quality = [
                    sample[6] for sample in self.flow_samples[-50:]
                ]
            quality_median = float(np.median(recent_quality)) if recent_quality else 0.0
            self.get_logger().info(
                f"accuracy capture elapsed={elapsed:.1f}s pose_samples={pose_count} "
                f"flow_samples={flow_count} recent_quality_median={quality_median:.1f}"
            )
            self.last_report = now
        if elapsed >= self.duration_s:
            self._finish()

    def close_gazebo_transport(self):
        """Stop pybind callbacks before Python begins interpreter teardown."""
        for topic in self.gz_pose_topics:
            try:
                self.gz_node.unsubscribe(topic)
            except Exception as exc:
                self.get_logger().warning(
                    f"Gazebo pose unsubscribe failed for {topic}: {exc}"
                )

    def _pose_at(self, poses, times, timestamp_s):
        index = bisect.bisect_left(times, timestamp_s)
        if index == 0 or index >= len(poses):
            return None
        before = poses[index - 1]
        after = poses[index]
        dt = after[0] - before[0]
        if dt <= 0.0 or dt > self.max_pose_gap_s:
            return None
        ratio = (timestamp_s - before[0]) / dt
        if ratio < 0.0 or ratio > 1.0:
            return None
        position = np.asarray(before[1]) + ratio * (
            np.asarray(after[1]) - np.asarray(before[1])
        )
        q0 = np.asarray(before[2])
        q1 = np.asarray(after[2])
        if float(np.dot(q0, q1)) < 0.0:
            q1 = -q1
        quaternion = q0 + ratio * (q1 - q0)
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1.0e-9:
            return None
        quaternion /= norm
        return tuple(position), tuple(quaternion)

    def _evaluate_mapping(self, estimates, truth, swap, sign_x, sign_y):
        mapped = estimates[:, ::-1] if swap else estimates.copy()
        mapped *= np.asarray([sign_x, sign_y])
        denominator = float(np.sum(mapped * mapped))
        scale = max(
            0.0,
            float(np.sum(mapped * truth) / denominator) if denominator > 1.0e-12 else 0.0,
        )
        predicted = scale * mapped
        errors = predicted - truth
        rmse = float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))
        truth_scale = max(1.0e-9, float(np.median(np.linalg.norm(truth, axis=1))))
        if float(np.std(predicted)) < 1.0e-9 or float(np.std(truth)) < 1.0e-9:
            correlation = -1.0
        else:
            correlation = float(np.corrcoef(predicted.ravel(), truth.ravel())[0, 1])
        return {
            "swap": swap,
            "sign_x": sign_x,
            "sign_y": sign_y,
            "scale": scale,
            "rmse_m": rmse,
            "normalized_rmse": rmse / truth_scale,
            "correlation": correlation,
        }

    def _mapping_summary(self, rows):
        estimates, truth, _, _, _, _ = accuracy_row_arrays(rows)
        mappings = [
            self._evaluate_mapping(estimates, truth, swap, sign_x, sign_y)
            for swap in (False, True)
            for sign_x in (-1.0, 1.0)
            for sign_y in (-1.0, 1.0)
        ]
        best = min(mappings, key=lambda value: value["normalized_rmse"])
        expected_mapping = (
            not best["swap"]
            and best["sign_x"] == 1.0
            and best["sign_y"] == 1.0
        )
        passed = bool(
            expected_mapping
            and 0.70 <= best["scale"] <= 1.30
            and best["correlation"] >= 0.50
            and best["normalized_rmse"] <= 0.75
        )
        return best, expected_mapping, passed

    def _finish(self):
        if self.done:
            return
        self.done = True
        with self.lock:
            poses = list(self.pose_samples)
            flows = list(self.flow_samples)
        basis = self.association_basis
        if basis == "auto":
            basis = "source_stamp"
        pose_time_index = 0
        poses = [
            (sample[pose_time_index], sample[2], sample[3])
            for sample in poses
        ]
        pose_times = [sample[0] for sample in poses]
        rows = []
        for (source_stamp, arrival_time, integration_s, arrival_interval, estimated_dx,
             estimated_dy, quality, distance, raw_x, raw_y, gyro_x, gyro_y,
             native_raw_x, native_raw_y, native_gyro_x, native_gyro_y) in flows:
            if (
                integration_s <= 1.0e-4
                or quality < self.min_quality
                or distance < self.min_ground_distance_m
            ):
                continue
            stamp = source_stamp
            start_pose = self._pose_at(poses, pose_times, stamp - integration_s)
            end_pose = self._pose_at(poses, pose_times, stamp)
            if start_pose is None or end_pose is None:
                continue
            truth = sensor_displacement_frd(start_pose, end_pose, self.lever_arm_frd)
            if not all(math.isfinite(value) for value in (
                estimated_dx, estimated_dy, truth[0], truth[1], truth[2],
            )):
                continue
            speed = math.hypot(truth[0], truth[1]) / integration_s
            vertical_speed = abs(truth[2]) / integration_s
            if not self.min_truth_speed_mps <= speed <= self.max_truth_speed_mps:
                continue
            if vertical_speed > self.max_vertical_speed_mps:
                continue
            yaw_rate = yaw_rate_from_quaternions(
                start_pose[1], end_pose[1], integration_s
            )
            if not math.isfinite(yaw_rate):
                continue
            rows.append((
                stamp,
                arrival_time,
                integration_s,
                arrival_interval,
                truth[0],
                truth[1],
                estimated_dx,
                estimated_dy,
                quality,
                distance,
                yaw_rate,
                raw_x,
                raw_y,
                gyro_x,
                gyro_y,
                native_raw_x,
                native_raw_y,
                native_gyro_x,
                native_gyro_y,
            ))

        if not rows:
            self.get_logger().error("FLOW_ACCURACY no aligned displacement samples")
            return
        estimates, truth, integration, arrival_intervals, quality, distance = accuracy_row_arrays(rows)
        best, expected_mapping, passed = self._mapping_summary(rows)
        alternate_gyro_x_rows = [
            row[:7]
            + (-(row[11] + row[13]) * row[9],)
            + row[8:]
            for row in rows
        ]
        alternate_gyro_x_best, _, _ = self._mapping_summary(alternate_gyro_x_rows)
        no_turn_rows = [
            row for row in rows
            if math.isfinite(row[10]) and row[10] <= self.no_turn_max_yaw_rate
        ]
        no_turn_best = None
        no_turn_expected_mapping = False
        no_turn_passed = False
        if len(no_turn_rows) >= self.no_turn_min_samples:
            no_turn_best, no_turn_expected_mapping, no_turn_passed = (
                self._mapping_summary(no_turn_rows)
            )
        no_turn_text = (
            "no_turn_scale=nan no_turn_rmse_m=nan no_turn_normalized_rmse=nan "
            "no_turn_corr=nan no_turn_expected_mapping=False"
            if no_turn_best is None else
            f"no_turn_scale={no_turn_best['scale']:.3f} "
            f"no_turn_rmse_m={no_turn_best['rmse_m']:.4f} "
            f"no_turn_normalized_rmse={no_turn_best['normalized_rmse']:.3f} "
            f"no_turn_corr={no_turn_best['correlation']:.3f} "
            f"no_turn_expected_mapping={no_turn_expected_mapping}"
        )
        self.get_logger().info(
            "FLOW_ACCURACY "
            f"association_basis={basis} matched={len(rows)} "
            f"poses={len(poses)} flows={len(flows)} "
            f"mapping=[swap={best['swap']},sx={best['sign_x']:+.0f},sy={best['sign_y']:+.0f}] "
            f"expected_mapping={expected_mapping} scale={best['scale']:.3f} "
            f"rmse_m={best['rmse_m']:.4f} normalized_rmse={best['normalized_rmse']:.3f} "
            f"corr={best['correlation']:.3f} quality_median={float(np.median(quality)):.1f} "
            f"distance_median={float(np.median(distance)):.2f}m "
            f"arrival_gap_outliers={int(np.count_nonzero(arrival_intervals > np.maximum(0.20, 3.0 * integration)))} "
            f"alternate_gyro_x_scale={alternate_gyro_x_best['scale']:.3f} "
            f"alternate_gyro_x_normalized_rmse={alternate_gyro_x_best['normalized_rmse']:.3f} "
            f"alternate_gyro_x_corr={alternate_gyro_x_best['correlation']:.3f} "
            f"passed={passed} no_turn_matched={len(no_turn_rows)} "
            f"no_turn_yaw_rate_max_radps={self.no_turn_max_yaw_rate:.3f} "
            f"{no_turn_text} no_turn_passed={no_turn_passed}"
        )

        if self.csv_path:
            with open(self.csv_path, "w", encoding="utf-8") as stream:
                stream.write(
                    "t,arrival_time_s,integration_time_s,arrival_interval_s,truth_dx,truth_dy,"
                    "flow_dx,flow_dy,quality,distance,yaw_rate_radps,raw_x,raw_y,gyro_x,gyro_y,"
                    "native_raw_x,native_raw_y,native_gyro_x,native_gyro_y\n"
                )
                for row in rows:
                    stream.write(",".join(str(value) for value in row) + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = FlowGazeboAccuracy()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        node._finish()
    finally:
        node.close_gazebo_transport()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
