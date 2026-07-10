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
from mavros_msgs.msg import OpticalFlow
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class FlowGazeboAccuracy(Node):
    """Compare simulated optical-flow planar velocity with Gazebo model motion."""

    def __init__(self):
        super().__init__("flow_gazebo_accuracy")
        self.declare_parameter("flow_topic", "/sim/optical_flow/raw")
        self.declare_parameter("gazebo_world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("gazebo_model", "apm_iris")
        self.declare_parameter("duration_s", 120.0)
        self.declare_parameter("min_quality", 80)
        self.declare_parameter("min_ground_distance_m", 0.6)
        self.declare_parameter("min_truth_speed_mps", 0.02)
        self.declare_parameter("max_truth_speed_mps", 2.0)
        self.declare_parameter("max_vertical_speed_mps", 0.35)
        self.declare_parameter("max_time_offset_s", 0.20)
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
        self.max_time_offset_s = float(self.get_parameter("max_time_offset_s").value)
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.lock = threading.Lock()
        self.pose_samples = []
        self.flow_samples = []
        self.start_time = time.monotonic()
        self.done = False
        self.last_report = 0.0

        self.create_subscription(
            OpticalFlow, self.flow_topic, self._flow_cb, qos_profile_sensor_data
        )
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            Pose_V, f"/world/{self.world_name}/dynamic_pose/info", self._gz_pose_cb
        )
        self.gz_node.subscribe(
            Pose_V, f"/world/{self.world_name}/pose/info", self._gz_pose_cb
        )
        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info(
            f"Recording optical-flow accuracy for {self.duration_s:.1f}s: "
            f"flow={self.flow_topic}, gazebo_model={self.model_name}"
        )

    def _flow_cb(self, msg):
        t = time.monotonic()
        distance = float(msg.ground_distance)
        # MAVROS OpticalFlow flow_comp_m is angular flow. Compare Gazebo speed
        # against angular rate multiplied by measured ground distance.
        vx = float(msg.flow_rate.x) * distance
        vy = float(msg.flow_rate.y) * distance
        with self.lock:
            self.flow_samples.append(
                (
                    t,
                    vx,
                    vy,
                    int(msg.quality),
                    distance,
                )
            )

    def _gz_pose_cb(self, msg):
        t = time.monotonic()
        for pose in msg.pose:
            if pose.name == self.model_name or pose.name.endswith(f"::{self.model_name}"):
                with self.lock:
                    self.pose_samples.append(
                        (
                            t,
                            float(pose.position.x),
                            float(pose.position.y),
                            float(pose.position.z),
                        )
                    )
                return

    def _timer_cb(self):
        elapsed = time.monotonic() - self.start_time
        now = time.monotonic()
        if now - self.last_report > 4.0:
            with self.lock:
                pose_n = len(self.pose_samples)
                flow_n = len(self.flow_samples)
                recent_q = [s[3] for s in self.flow_samples[-50:]]
            q_med = float(np.median(recent_q)) if recent_q else 0.0
            self.get_logger().info(
                f"accuracy capture elapsed={elapsed:.1f}s pose_samples={pose_n} "
                f"flow_samples={flow_n} recent_quality_median={q_med:.1f}"
            )
            self.last_report = now
        if elapsed >= self.duration_s:
            self._finish()

    def _pose_velocities(self, poses):
        velocities = []
        for a, b in zip(poses[:-1], poses[1:]):
            t0, x0, y0, z0 = a
            t1, x1, y1, z1 = b
            dt = t1 - t0
            if not math.isfinite(dt) or dt <= 0.001 or dt > 0.5:
                continue
            velocities.append(((t0 + t1) * 0.5, (x1 - x0) / dt, (y1 - y0) / dt, (z1 - z0) / dt, (z0 + z1) * 0.5))
        return velocities

    def _corr(self, a, b):
        if len(a) < 3:
            return float("nan")
        aa = np.asarray(a, dtype=np.float64)
        bb = np.asarray(b, dtype=np.float64)
        if float(np.std(aa)) < 1.0e-6 or float(np.std(bb)) < 1.0e-6:
            return float("nan")
        return float(np.corrcoef(aa, bb)[0, 1])

    def _evaluate_mapping(self, rows, swap, sx, sy):
        errors = []
        pred_x = []
        pred_y = []
        truth_x = []
        truth_y = []
        for _, gvx, gvy, fx, fy, _, _, _, _ in rows:
            px = sx * (fy if swap else fx)
            py = sy * (fx if swap else fy)
            errors.append((px - gvx, py - gvy))
            pred_x.append(px)
            pred_y.append(py)
            truth_x.append(gvx)
            truth_y.append(gvy)
        err = np.asarray(errors, dtype=np.float64)
        rmse = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
        mae = float(np.mean(np.linalg.norm(err, axis=1)))
        corr_x = self._corr(pred_x, truth_x)
        corr_y = self._corr(pred_y, truth_y)
        finite_corr = [c for c in (corr_x, corr_y) if math.isfinite(c)]
        corr = float(np.mean(finite_corr)) if finite_corr else float("nan")
        return {
            "swap": swap,
            "sx": sx,
            "sy": sy,
            "rmse": rmse,
            "mae": mae,
            "corr_x": corr_x,
            "corr_y": corr_y,
            "corr": corr,
        }

    def _finish(self):
        if self.done:
            return
        self.done = True
        with self.lock:
            poses = list(self.pose_samples)
            flows = [f for f in self.flow_samples if f[3] >= self.min_quality]

        velocities = self._pose_velocities(poses)
        velocity_times = [v[0] for v in velocities]
        rows = []
        pose_z = [p[3] for p in poses]
        for ft, fx, fy, quality, distance in flows:
            if not velocity_times:
                break
            idx = bisect.bisect_left(velocity_times, ft)
            candidates = []
            if idx < len(velocities):
                candidates.append(velocities[idx])
            if idx > 0:
                candidates.append(velocities[idx - 1])
            if not candidates:
                continue
            best = min(candidates, key=lambda v: abs(v[0] - ft))
            offset = abs(best[0] - ft)
            if offset <= self.max_time_offset_s:
                _, gvx, gvy, gvz, gz = best
                speed_xy = math.hypot(gvx, gvy)
                if distance < self.min_ground_distance_m:
                    continue
                if abs(gvz) > self.max_vertical_speed_mps:
                    continue
                if speed_xy < self.min_truth_speed_mps or speed_xy > self.max_truth_speed_mps:
                    continue
                rows.append((ft, gvx, gvy, fx, fy, quality, distance, gvz, gz))

        if not rows:
            self.get_logger().error(
                "FLOW_ACCURACY no aligned samples. Check /sim/optical_flow/raw and Gazebo pose topics."
            )
            return

        mappings = [
            self._evaluate_mapping(rows, swap, sx, sy)
            for swap in (False, True)
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
        ]
        best = min(mappings, key=lambda m: m["rmse"])
        qualities = [r[5] for r in rows]
        distances = [r[6] for r in rows]
        offsets = []
        for ft, *_ in rows:
            idx = bisect.bisect_left(velocity_times, ft)
            near = []
            if idx < len(velocities):
                near.append(abs(velocities[idx][0] - ft))
            if idx > 0:
                near.append(abs(velocities[idx - 1][0] - ft))
            if near:
                offsets.append(min(near))

        mapping_text = (
            f"gazebo_vx ~= {best['sx']:+.0f}*flow_{'y' if best['swap'] else 'x'}, "
            f"gazebo_vy ~= {best['sy']:+.0f}*flow_{'x' if best['swap'] else 'y'}"
        )
        self.get_logger().info(
            "FLOW_ACCURACY "
            f"matched={len(rows)} poses={len(poses)} flows={len(flows)} "
            f"filters=[q>={self.min_quality},dist>={self.min_ground_distance_m:.2f},"
            f"speed={self.min_truth_speed_mps:.2f}..{self.max_truth_speed_mps:.2f},"
            f"|vz|<={self.max_vertical_speed_mps:.2f}] "
            f"mapping=[{mapping_text}] "
            f"rmse={best['rmse']:.3f}m/s mae={best['mae']:.3f}m/s "
            f"corr_x={best['corr_x']:.3f} corr_y={best['corr_y']:.3f} "
            f"quality_median={float(np.median(qualities)):.1f} "
            f"distance_median={float(np.median(distances)):.2f}m "
            f"gazebo_z_min={float(np.min(pose_z)) if pose_z else 0.0:.2f}m "
            f"gazebo_z_max={float(np.max(pose_z)) if pose_z else 0.0:.2f}m "
            f"time_offset_median={float(np.median(offsets)) if offsets else 0.0:.3f}s"
        )

        if self.csv_path:
            with open(self.csv_path, "w", encoding="utf-8") as f:
                f.write("t,gazebo_vx,gazebo_vy,flow_velocity_x,flow_velocity_y,quality,distance,gazebo_vz,gazebo_z\n")
                for row in rows:
                    f.write(",".join(str(v) for v in row) + "\n")
            self.get_logger().info(f"Wrote aligned flow accuracy CSV: {self.csv_path}")


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
