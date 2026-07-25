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


class FlowGazeboAccuracy(Node):
    """Evaluate compensated flow displacement against the simulated sensor pose."""

    def __init__(self):
        super().__init__("flow_gazebo_accuracy")
        self.declare_parameter("flow_topic", "/sim/optical_flow/rad")
        self.declare_parameter("gazebo_world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("gazebo_model", "apm_iris")
        self.declare_parameter("duration_s", 120.0)
        self.declare_parameter("min_quality", 80)
        self.declare_parameter("min_ground_distance_m", 0.6)
        self.declare_parameter("min_truth_speed_mps", 0.06)
        self.declare_parameter("max_truth_speed_mps", 2.0)
        self.declare_parameter("max_vertical_speed_mps", 0.35)
        self.declare_parameter("max_pose_gap_s", 0.20)
        self.declare_parameter("sensor_offset_z_down_m", 0.35)
        self.declare_parameter("csv_path", "")

        self.flow_topic = str(self.get_parameter("flow_topic").value)
        self.world_name = str(self.get_parameter("gazebo_world_name").value)
        self.model_name = str(self.get_parameter("gazebo_model").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.min_quality = int(self.get_parameter("min_quality").value)
        self.min_ground_distance_m = float(self.get_parameter("min_ground_distance_m").value)
        self.min_truth_speed_mps = float(self.get_parameter("min_truth_speed_mps").value)
        self.max_truth_speed_mps = float(self.get_parameter("max_truth_speed_mps").value)
        self.max_vertical_speed_mps = float(self.get_parameter("max_vertical_speed_mps").value)
        self.max_pose_gap_s = float(self.get_parameter("max_pose_gap_s").value)
        self.lever_arm_frd = (
            0.0,
            0.0,
            float(self.get_parameter("sensor_offset_z_down_m").value),
        )
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.lock = threading.Lock()
        self.pose_samples = []
        self.flow_samples = []
        self.last_flow_arrival = None
        self.start_time = time.monotonic()
        self.done = False
        self.last_report = 0.0

        self.create_subscription(
            OpticalFlowRad, self.flow_topic, self._flow_cb, qos_profile_sensor_data
        )
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            Pose_V, f"/world/{self.world_name}/dynamic_pose/info", self._gz_pose_cb
        )
        self.gz_node.subscribe(Pose_V, f"/world/{self.world_name}/pose/info", self._gz_pose_cb)
        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info(
            f"Recording compensated flow displacement for {self.duration_s:.1f}s: "
            f"flow={self.flow_topic}, gazebo_model={self.model_name}"
        )

    def _flow_cb(self, msg):
        now = time.monotonic()
        arrival_interval = (
            0.0 if self.last_flow_arrival is None else now - self.last_flow_arrival
        )
        self.last_flow_arrival = now
        integration_s = float(msg.integration_time_us) * 1.0e-6
        distance = float(msg.distance)
        estimated_dx = (float(msg.integrated_y) - float(msg.integrated_ygyro)) * distance
        estimated_dy = -(float(msg.integrated_x) - float(msg.integrated_xgyro)) * distance
        with self.lock:
            self.flow_samples.append((
                now,
                integration_s,
                arrival_interval,
                estimated_dx,
                estimated_dy,
                int(msg.quality),
                distance,
            ))

    def _gz_pose_cb(self, msg):
        now = time.monotonic()
        for pose in msg.pose:
            if pose.name == self.model_name or pose.name.endswith(f"::{self.model_name}"):
                sample = (
                    now,
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
                    if not self.pose_samples or now > self.pose_samples[-1][0]:
                        self.pose_samples.append(sample)
                return

    def _timer_cb(self):
        elapsed = time.monotonic() - self.start_time
        now = time.monotonic()
        if now - self.last_report > 4.0:
            with self.lock:
                pose_count = len(self.pose_samples)
                flow_count = len(self.flow_samples)
                recent_quality = [sample[5] for sample in self.flow_samples[-50:]]
            quality_median = float(np.median(recent_quality)) if recent_quality else 0.0
            self.get_logger().info(
                f"accuracy capture elapsed={elapsed:.1f}s pose_samples={pose_count} "
                f"flow_samples={flow_count} recent_quality_median={quality_median:.1f}"
            )
            self.last_report = now
        if elapsed >= self.duration_s:
            self._finish()

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

    def _finish(self):
        if self.done:
            return
        self.done = True
        with self.lock:
            poses = list(self.pose_samples)
            flows = list(self.flow_samples)
        pose_times = [sample[0] for sample in poses]
        rows = []
        for (stamp, integration_s, arrival_interval, estimated_dx, estimated_dy,
             quality, distance) in flows:
            if (
                integration_s <= 1.0e-4
                or quality < self.min_quality
                or distance < self.min_ground_distance_m
            ):
                continue
            start_pose = self._pose_at(poses, pose_times, stamp - integration_s)
            end_pose = self._pose_at(poses, pose_times, stamp)
            if start_pose is None or end_pose is None:
                continue
            truth = sensor_displacement_frd(start_pose, end_pose, self.lever_arm_frd)
            speed = math.hypot(truth[0], truth[1]) / integration_s
            vertical_speed = abs(truth[2]) / integration_s
            if not self.min_truth_speed_mps <= speed <= self.max_truth_speed_mps:
                continue
            if vertical_speed > self.max_vertical_speed_mps:
                continue
            rows.append((
                stamp,
                integration_s,
                arrival_interval,
                truth[0],
                truth[1],
                estimated_dx,
                estimated_dy,
                quality,
                distance,
            ))

        if not rows:
            self.get_logger().error("FLOW_ACCURACY no aligned displacement samples")
            return
        estimates = np.asarray([[row[5], row[6]] for row in rows])
        truth = np.asarray([[row[3], row[4]] for row in rows])
        mappings = [
            self._evaluate_mapping(estimates, truth, swap, sign_x, sign_y)
            for swap in (False, True)
            for sign_x in (-1.0, 1.0)
            for sign_y in (-1.0, 1.0)
        ]
        best = min(mappings, key=lambda value: value["normalized_rmse"])
        expected_mapping = not best["swap"] and best["sign_x"] == 1.0 and best["sign_y"] == 1.0
        passed = bool(
            expected_mapping
            and 0.70 <= best["scale"] <= 1.30
            and best["correlation"] >= 0.50
            and best["normalized_rmse"] <= 0.75
        )
        self.get_logger().info(
            "FLOW_ACCURACY "
            f"matched={len(rows)} poses={len(poses)} flows={len(flows)} "
            f"mapping=[swap={best['swap']},sx={best['sign_x']:+.0f},sy={best['sign_y']:+.0f}] "
            f"expected_mapping={expected_mapping} scale={best['scale']:.3f} "
            f"rmse_m={best['rmse_m']:.4f} normalized_rmse={best['normalized_rmse']:.3f} "
            f"corr={best['correlation']:.3f} quality_median={float(np.median([r[7] for r in rows])):.1f} "
            f"distance_median={float(np.median([r[8] for r in rows])):.2f}m "
            f"arrival_gap_outliers={sum(r[2] > max(0.20, 3.0 * r[1]) for r in rows)} "
            f"passed={passed}"
        )

        if self.csv_path:
            with open(self.csv_path, "w", encoding="utf-8") as stream:
                stream.write(
                    "t,integration_time_s,arrival_interval_s,truth_dx,truth_dy,"
                    "flow_dx,flow_dy,quality,distance\n"
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
