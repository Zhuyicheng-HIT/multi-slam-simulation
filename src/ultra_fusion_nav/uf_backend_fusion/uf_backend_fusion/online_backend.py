"""Online first-pass Ultra-Fusion-style local backend.

The node keeps the estimator boundary explicit: LIO is the local pose anchor,
raw GNSS and optical-flow are optional observation factors, and the scheduler
controls factor weights. It intentionally does not subscribe to FCU fused
local position or Gazebo truth. The current backend is linear/tangent-space;
the IMU factor is therefore marked approximate until the bias-aware SE(3)
backend replaces it.
"""

from bisect import bisect_left, bisect_right
from collections import deque
import copy
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from uf_interfaces.msg import ReliabilityScore, SchedulerState

from .imu_preintegration import ImuSample, _quat_to_rotvec, preintegrate
from .window import SlidingWindowBackend
from uf_reliability.scoring import (
    gnss_score,
    optical_flow_displacement_frd,
    optical_flow_score,
)


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
MIN_FLOW_QUALITY = 20
MAX_COVARIANCE_INFLATION = 20.0


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def quaternion_to_yaw(quaternion):
    x, y, z, w = (
        float(quaternion.x), float(quaternion.y),
        float(quaternion.z), float(quaternion.w),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw):
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def unwrap_yaw(previous_yaw, wrapped_yaw):
    """Keep yaw residuals continuous across the +/- pi branch cut."""
    if previous_yaw is None:
        return float(wrapped_yaw)
    delta = math.atan2(
        math.sin(float(wrapped_yaw) - float(previous_yaw)),
        math.cos(float(wrapped_yaw) - float(previous_yaw)),
    )
    return float(previous_yaw) + delta


def rotate_planar(forward, left, yaw):
    cosine, sine = math.cos(float(yaw)), math.sin(float(yaw))
    return (
        cosine * float(forward) - sine * float(left),
        sine * float(forward) + cosine * float(left),
    )


def frd_to_enu_delta(forward, right, yaw):
    return rotate_planar(float(forward), -float(right), yaw)


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    latitude = math.radians(float(latitude_deg))
    longitude = math.radians(float(longitude_deg))
    sin_latitude = math.sin(latitude)
    prime_vertical = WGS84_A_M / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )
    return (
        (prime_vertical + altitude_m) * math.cos(latitude) * math.cos(longitude),
        (prime_vertical + altitude_m) * math.cos(latitude) * math.sin(longitude),
        (prime_vertical * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
    )


class LocalEnuProjector:
    def __init__(self, latitude_deg, longitude_deg, altitude_m):
        self.latitude = math.radians(float(latitude_deg))
        self.longitude = math.radians(float(longitude_deg))
        self.origin = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)

    def project(self, latitude_deg, longitude_deg, altitude_m):
        x, y, z = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        dx, dy, dz = x - self.origin[0], y - self.origin[1], z - self.origin[2]
        sin_latitude, cos_latitude = math.sin(self.latitude), math.cos(self.latitude)
        sin_longitude, cos_longitude = math.sin(self.longitude), math.cos(self.longitude)
        return (
            -sin_longitude * dx + cos_longitude * dy,
            -sin_latitude * cos_longitude * dx
            - sin_latitude * sin_longitude * dy
            + cos_latitude * dz,
            cos_latitude * cos_longitude * dx
            + cos_latitude * sin_longitude * dy
            + sin_latitude * dz,
        )


def scheduler_decision(weight=1.0, enabled=True, inflation=1.0):
    weight = max(0.0, min(1.0, float(weight)))
    inflation = max(1.0, min(MAX_COVARIANCE_INFLATION, float(inflation)))
    return {
        "factor_enabled": bool(enabled) and weight > 0.0,
        "reliability_weight": weight if enabled else 0.0,
        "covariance_inflation": inflation,
    }


def gnss_jump_rejected(current_position, gnss_position, gate_m=20.0):
    current = np.asarray(current_position, dtype=float)
    measurement = np.asarray(gnss_position, dtype=float)
    if current.shape != (3,) or measurement.shape != (3,):
        raise ValueError("GNSS jump gate expects two 3-vectors")
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(measurement)):
        return True
    return float(np.linalg.norm(current - measurement)) > float(gate_m)


def flow_observation_delta(flow_records, yaw):
    """Aggregate valid MAVLink optical-flow increments into map ENU."""
    delta = np.zeros(2, dtype=float)
    qualities = []
    distances = []
    for flow in flow_records:
        distance = float(flow["distance_m"])
        if distance <= 0.0 or not math.isfinite(distance):
            continue
        displacement = optical_flow_displacement_frd(
            flow["integrated_x"], flow["integrated_y"],
            flow["integrated_xgyro"], flow["integrated_ygyro"],
            distance,
        )
        if displacement is None:
            continue
        delta += np.asarray(
            frd_to_enu_delta(displacement[0], displacement[1], yaw),
            dtype=float,
        )
        qualities.append(float(flow["quality"]))
        distances.append(distance)
    if not qualities:
        return None
    return {
        "delta_position": [float(delta[0]), float(delta[1]), 0.0],
        "quality": float(np.mean(qualities)),
        "distance_m": float(np.mean(distances)),
        "sample_count": len(qualities),
    }


class UnifiedBackendNode(Node):
    def __init__(self):
        super().__init__("unified_backend_fusion")
        defaults = {
            "lio_topic": "/lio/odom",
            "gnss_topic": "/sensors/gnss/fix",
            "flow_topic": "/sensors/optical_flow/rad",
            "imu_topic": "/sensors/imu",
            "scheduler_topic": "/reliability/scheduler_state",
            "output_topic": "/fusion/unified/odom",
            "path_topic": "/fusion/unified/path",
            "diagnostic_topic": "/fusion/unified/diagnostics",
            "map_frame": "map",
            "body_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("window_size", 20)
        self.declare_parameter("gnss_max_age_s", 2.0)
        self.declare_parameter("flow_max_age_s", 1.0)
        self.declare_parameter("imu_buffer_s", 5.0)
        self.declare_parameter("minimum_flow_quality", MIN_FLOW_QUALITY)
        self.declare_parameter("minimum_flow_distance_m", 0.08)
        self.declare_parameter("maximum_flow_distance_m", 12.0)
        self.declare_parameter("gnss_default_variance_m2", 4.0)
        self.declare_parameter("gnss_jump_gate_m", 20.0)
        self.declare_parameter("imu_factor_enabled", True)
        self.declare_parameter("preserve_lio_anchor", True)
        self.declare_parameter("imu_covariance_scale", 50.0)
        self.declare_parameter("imu_bias_random_walk_variance", 1.0e-4)
        self.declare_parameter("scheduler_timeout_s", 1.0)
        self.declare_parameter("publish_path_length", 2000)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.gnss_max_age_s = float(self.get_parameter("gnss_max_age_s").value)
        self.flow_max_age_s = float(self.get_parameter("flow_max_age_s").value)
        self.imu_buffer_s = float(self.get_parameter("imu_buffer_s").value)
        self.minimum_flow_quality = int(self.get_parameter("minimum_flow_quality").value)
        self.minimum_flow_distance_m = float(
            self.get_parameter("minimum_flow_distance_m").value)
        self.maximum_flow_distance_m = float(
            self.get_parameter("maximum_flow_distance_m").value)
        self.gnss_default_variance = float(
            self.get_parameter("gnss_default_variance_m2").value)
        self.gnss_jump_gate_m = float(
            self.get_parameter("gnss_jump_gate_m").value)
        self.imu_factor_enabled = bool(self.get_parameter("imu_factor_enabled").value)
        self.preserve_lio_anchor = bool(self.get_parameter("preserve_lio_anchor").value)
        self.imu_covariance_scale = float(
            self.get_parameter("imu_covariance_scale").value)
        self.imu_bias_random_walk_variance = float(
            self.get_parameter("imu_bias_random_walk_variance").value)
        self.scheduler_timeout_s = float(
            self.get_parameter("scheduler_timeout_s").value)
        self.max_path = int(self.get_parameter("publish_path_length").value)

        self.backend = SlidingWindowBackend(
            max_states=max(2, int(self.get_parameter("window_size").value))
        )
        self.path = Path()
        self.path.poses = []
        self.imu_buffer = deque(maxlen=10000)
        self.flow_buffer = deque(maxlen=3000)
        self.latest_gnss = None
        self.projector = None
        self.lio_origin = None
        self.last_lio_stamp = None
        self.last_lio_position = None
        self.last_lio_yaw = 0.0
        self.flow_clock_offset_s = None
        self.scheduler = {}
        self.scheduler_arrival = None
        self.scheduler_health = "UNAVAILABLE"
        self.scores = {}
        self.counts = {
            "lio": 0, "published": 0, "lidar_factors": 0,
            "lidar_disabled": 0, "gnss_factors": 0, "gnss_jump_rejected": 0,
            "flow_factors": 0, "flow_disabled_quality": 0,
            "imu_factors": 0, "imu_invalid": 0, "optimization_errors": 0,
        }
        self.last_reason = "waiting_for_lio"
        self.last_callback_ms = 0.0
        self.last_imu_reason = "unavailable"
        self.last_flow_reason = "unavailable"
        self.last_output = None

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostic_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("lio_topic").value),
            self._lio, 20)
        self.create_subscription(
            NavSatFix, str(self.get_parameter("gnss_topic").value),
            self._gnss, qos_profile_sensor_data)
        self.create_subscription(
            OpticalFlowRad, str(self.get_parameter("flow_topic").value),
            self._flow, qos_profile_sensor_data)
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value),
            self._imu, qos_profile_sensor_data)
        self.create_subscription(
            SchedulerState, str(self.get_parameter("scheduler_topic").value),
            self._scheduler, 20)
        for modality in ("lidar", "gnss", "imu", "optical_flow"):
            self.create_subscription(
                ReliabilityScore, f"/reliability/{modality}_score",
                lambda msg, name=modality: self._score(name, msg),
                qos_profile_sensor_data,
            )
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            "Unified backend active: LIO anchor + GNSS/flow factors; "
            f"IMU bias-aware local factor={'on' if self.imu_factor_enabled else 'off'}")

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _score(self, modality, msg):
        self.scores[modality] = {
            "weight": float(msg.reliability_weight) if msg.valid else 0.0,
            "valid": bool(msg.valid),
            "stamp_mono": time.monotonic(),
        }

    def _scheduler(self, msg):
        lengths = (
            len(msg.modality_names), len(msg.reliability_weights),
            len(msg.covariance_inflation), len(msg.factor_enabled),
        )
        if min(lengths) != max(lengths):
            self.last_reason = "malformed_scheduler_state"
            return
        self.scheduler = {
            name: (float(weight), bool(enabled), float(inflation))
            for name, weight, enabled, inflation in zip(
                msg.modality_names, msg.reliability_weights,
                msg.factor_enabled, msg.covariance_inflation,
            )
        }
        self.scheduler_health = str(msg.health_state)
        self.scheduler_arrival = time.monotonic()

    def _decision(self, modality, default_enabled=False):
        now = time.monotonic()
        # LIO is the local estimator anchor. A missing/stale diagnostic must
        # not silently remove its pose factor and leave rotation unobservable.
        score_item = self.scores.get(modality)
        score_fresh = (
            score_item is not None
            and score_item["valid"]
            and now - score_item["stamp_mono"] <= self.scheduler_timeout_s
        )
        if modality == "lidar" and self.preserve_lio_anchor and not score_fresh:
            return scheduler_decision(1.0, default_enabled, 1.0)
        if self.scheduler_arrival is not None and now - self.scheduler_arrival <= self.scheduler_timeout_s:
            item = self.scheduler.get(modality)
            if item is not None:
                decision = scheduler_decision(item[0], item[1], item[2])
                if modality == "lidar" and self.preserve_lio_anchor and not decision["factor_enabled"]:
                    decision["factor_enabled"] = True
                    decision["reliability_weight"] = max(0.05, decision["reliability_weight"])
                    decision["covariance_inflation"] = max(
                        1.0, min(MAX_COVARIANCE_INFLATION, decision["covariance_inflation"])
                    )
                return decision
        item = self.scores.get(modality)
        if item is not None and now - item["stamp_mono"] <= self.scheduler_timeout_s:
            decision = scheduler_decision(item["weight"], item["valid"], 1.0)
            if modality == "lidar" and self.preserve_lio_anchor and not decision["factor_enabled"]:
                decision["factor_enabled"] = True
                decision["reliability_weight"] = max(0.05, decision["reliability_weight"])
            return decision
        return scheduler_decision(1.0, default_enabled, 1.0)

    def _imu(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self._now_s()
        sample = ImuSample(
            stamp,
            (
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            ),
            (
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ),
        )
        self.imu_buffer.append(sample)
        if self.last_lio_stamp is not None:
            cutoff = self.last_lio_stamp - self.imu_buffer_s
            while self.imu_buffer and self.imu_buffer[0].stamp_s < cutoff:
                self.imu_buffer.popleft()

    def _flow_stamp(self, stamp):
        if stamp <= 0.0:
            return self._now_s()
        if self.flow_clock_offset_s is None and self.last_lio_stamp is not None:
            if abs(self.last_lio_stamp - stamp) > 1000.0:
                self.flow_clock_offset_s = self.last_lio_stamp - stamp
        return stamp if self.flow_clock_offset_s is None else stamp + self.flow_clock_offset_s

    def _flow(self, msg):
        stamp = self._flow_stamp(stamp_seconds(msg.header.stamp))
        self.flow_buffer.append({
            "stamp_s": stamp,
            "integrated_x": float(msg.integrated_x),
            "integrated_y": float(msg.integrated_y),
            "integrated_xgyro": float(msg.integrated_xgyro),
            "integrated_ygyro": float(msg.integrated_ygyro),
            "quality": int(msg.quality),
            "distance_m": float(msg.distance),
        })

    def _gnss(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        values = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
        if not all(math.isfinite(value) for value in values):
            return
        if self.projector is None:
            self.projector = LocalEnuProjector(*values)
        covariance = [
            max(0.04, float(msg.position_covariance[0])),
            max(0.04, float(msg.position_covariance[4])),
            max(0.04, float(msg.position_covariance[8])),
        ]
        self.latest_gnss = {
            "stamp_s": stamp_seconds(msg.header.stamp),
            "position_enu": self.projector.project(*values),
            "covariance": covariance,
            "status": int(msg.status.status),
        }

    def _imu_factor(self, previous_stamp, current_stamp, previous_yaw, previous_index, current_index):
        if not self.imu_factor_enabled or len(self.imu_buffer) < 2:
            return
        samples = list(self.imu_buffer)
        stamps = [sample.stamp_s for sample in samples]
        start = max(0, bisect_left(stamps, previous_stamp) - 1)
        end = min(len(samples), bisect_right(stamps, current_stamp) + 1)
        result = preintegrate(samples[start:end], previous_stamp, current_stamp)
        self.last_imu_reason = result.reason
        if not result.valid:
            self.counts["imu_invalid"] += 1
            return
        world_position = rotate_planar(
            result.delta_position[0], result.delta_position[1], previous_yaw)
        world_velocity = rotate_planar(
            result.delta_velocity[0], result.delta_velocity[1], previous_yaw)
        yaw_cosine, yaw_sine = math.cos(previous_yaw), math.sin(previous_yaw)
        map_rotation = np.array([
            [yaw_cosine, -yaw_sine, 0.0],
            [yaw_sine, yaw_cosine, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        delta_position = np.asarray(
            [world_position[0], world_position[1], result.delta_position[2]], dtype=float)
        delta_velocity = np.asarray(
            [world_velocity[0], world_velocity[1], result.delta_velocity[2]], dtype=float)
        delta_rotation = map_rotation @ _quat_to_rotvec(np.asarray(result.delta_quaternion))
        position_accel_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_position_accel_bias).reshape(3, 3)
        position_gyro_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_position_gyro_bias).reshape(3, 3)
        velocity_accel_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_velocity_accel_bias).reshape(3, 3)
        velocity_gyro_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_velocity_gyro_bias).reshape(3, 3)
        rotation_gyro_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_rotation_gyro_bias).reshape(3, 3)
        covariance = np.asarray(result.covariance, dtype=float)
        covariance = np.maximum(covariance * self.imu_covariance_scale, 1.0e-6)
        decision = self._decision("imu", default_enabled=True)
        self.backend.add_bias_aware_imu(
            previous_index, current_index, result.dt_s,
            delta_position, delta_velocity, delta_rotation,
            position_accel_jacobian.ravel(), position_gyro_jacobian.ravel(),
            velocity_accel_jacobian.ravel(), velocity_gyro_jacobian.ravel(),
            rotation_gyro_jacobian.ravel(),
            # The data-layer preintegrator already adds gravity.  Passing a
            # zero vector here avoids adding it twice; the eventual manifold
            # implementation will carry gravity outside the delta instead.
            gravity=(0.0, 0.0, 0.0),
            covariance=covariance,
            bias_random_walk_covariance=np.full(6, self.imu_bias_random_walk_variance),
            decision=decision,
        )
        self.counts["imu_factors"] += 1

    def _gnss_factor(self, stamp, position, index):
        if self.latest_gnss is None or self.projector is None or self.lio_origin is None:
            return
        age = stamp - self.latest_gnss["stamp_s"]
        if age < -0.5 or age > self.gnss_max_age_s:
            return
        gnss_position = np.asarray(self.lio_origin) + np.asarray(
            self.latest_gnss["position_enu"], dtype=float)
        covariance = np.asarray(self.latest_gnss["covariance"], dtype=float)
        current = np.asarray(position, dtype=float)
        innovation = current - gnss_position
        mahalanobis = float(np.sum(innovation * innovation / covariance))
        score, _, _ = gnss_score(
            1.0 if self.latest_gnss["status"] >= 0 else 0.0,
            float(np.sum(covariance)), mahalanobis,
        )
        decision = self._decision("gnss", default_enabled=True)
        decision["degradation_score"] = float(score)
        if gnss_jump_rejected(current, gnss_position, self.gnss_jump_gate_m):
            decision["factor_enabled"] = False
            decision["reliability_weight"] = 0.0
            decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
            decision["degradation_score"] = 1.0
            decision["reasons"] = ["gnss_jump_hard_gate"]
            self.counts["gnss_jump_rejected"] += 1
        self.backend.add_gnss(index, gnss_position, covariance=covariance, decision=decision)
        self.counts["gnss_factors"] += 1

    def _flow_factor(self, previous_stamp, current_stamp, previous_yaw, previous_index, current_index, lio_delta):
        if not self.flow_buffer:
            self.last_flow_reason = "no_samples"
            return
        if self.flow_clock_offset_s is None:
            latest_stamp = self.flow_buffer[-1]["stamp_s"]
            if abs(previous_stamp - latest_stamp) > 1000.0:
                self.flow_clock_offset_s = previous_stamp - latest_stamp
                self.flow_buffer = deque(
                    [
                        dict(item, stamp_s=item["stamp_s"] + self.flow_clock_offset_s)
                        for item in self.flow_buffer
                    ],
                    maxlen=3000,
                )
        stamps = [item["stamp_s"] for item in self.flow_buffer]
        start = bisect_right(stamps, previous_stamp)
        end = bisect_right(stamps, current_stamp)
        records = list(self.flow_buffer)[start:end]
        self.flow_buffer = deque(
            [item for item in self.flow_buffer if item["stamp_s"] > current_stamp],
            maxlen=3000,
        )
        observation = flow_observation_delta(records, self.last_lio_yaw)
        if observation is None:
            self.last_flow_reason = "no_valid_observation"
            return
        score, evidence, reasons = optical_flow_score(
            observation["delta_position"],
            [float(lio_delta[0]), float(lio_delta[1])],
            observation["quality"], observation["distance_m"],
        )
        decision = self._decision("optical_flow", default_enabled=True)
        decision["degradation_score"] = float(score)
        decision["evidence"] = evidence
        decision["reasons"] = list(reasons)
        if (
            observation["quality"] < self.minimum_flow_quality
            or not self.minimum_flow_distance_m <= observation["distance_m"] <= self.maximum_flow_distance_m
        ):
            decision["factor_enabled"] = False
            decision["reliability_weight"] = 0.0
            decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
            self.counts["flow_disabled_quality"] += 1
            self.last_flow_reason = "quality_or_distance_gate"
        else:
            self.last_flow_reason = "accepted"
        self.backend.add_optical_flow(
            previous_index, current_index, observation["delta_position"],
            covariance=[0.10 ** 2, 0.10 ** 2, 1.0], decision=decision,
        )
        self.counts["flow_factors"] += 1

    def _lio(self, msg):
        started = time.perf_counter_ns()
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self._now_s()
        if self.last_lio_stamp is not None and stamp <= self.last_lio_stamp:
            self.last_reason = "nonmonotonic_lio_stamp"
            return
        pose = msg.pose.pose
        position = np.asarray(
            [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
            dtype=float,
        )
        wrapped_yaw = quaternion_to_yaw(pose.orientation)
        yaw = unwrap_yaw(self.last_lio_yaw if self.last_lio_stamp is not None else None, wrapped_yaw)
        current_index = self.backend.add_state()
        rotation = [0.0, 0.0, yaw]
        if self.last_lio_stamp is None:
            self.lio_origin = position.copy()
            self.backend.add_prior(
                current_index,
                np.concatenate((position, rotation, np.zeros(9, dtype=float))),
                covariance=np.full(15, 1.0e-4),
            )
        lidar_decision = self._decision("lidar", default_enabled=True)
        self.counts["lidar_factors"] += 1
        if not lidar_decision["factor_enabled"]:
            self.counts["lidar_disabled"] += 1
        self.backend.add_lidar_pose(
            current_index, position, rotation,
            covariance=[0.05 ** 2] * 3 + [0.03 ** 2] * 3,
            decision=lidar_decision,
        )
        if self.last_lio_stamp is not None:
            previous_index = current_index - 1
            lio_delta = position - self.last_lio_position
            self._gnss_factor(stamp, position, current_index)
            self._flow_factor(
                self.last_lio_stamp, stamp, self.last_lio_yaw,
                previous_index, current_index, lio_delta,
            )
            self._imu_factor(
                self.last_lio_stamp, stamp, self.last_lio_yaw,
                previous_index, current_index,
            )
        try:
            self.backend.optimize()
            estimate = self.backend.state(current_index)
            self._publish(msg.header, estimate)
            self.counts["lio"] += 1
            self.last_reason = "ok"
        except (np.linalg.LinAlgError, ValueError, IndexError) as error:
            self.counts["optimization_errors"] += 1
            self.last_reason = f"optimization_error:{type(error).__name__}"
        self.last_lio_stamp = stamp
        self.last_lio_position = position
        self.last_lio_yaw = yaw
        self.last_callback_ms = (time.perf_counter_ns() - started) * 1.0e-6

    def _publish(self, header, state):
        yaw = float(state[5])
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        output = Odometry()
        output.header = copy.deepcopy(header)
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.body_frame
        output.pose.pose.position.x = float(state[0])
        output.pose.pose.position.y = float(state[1])
        output.pose.pose.position.z = float(state[2])
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.twist.twist.linear.x = float(
            math.cos(yaw) * state[6] + math.sin(yaw) * state[7])
        output.twist.twist.linear.y = float(
            -math.sin(yaw) * state[6] + math.cos(yaw) * state[7])
        output.twist.twist.linear.z = float(state[8])
        output.pose.covariance[0] = 0.05 ** 2
        output.pose.covariance[7] = 0.05 ** 2
        output.pose.covariance[14] = 0.10 ** 2
        output.pose.covariance[35] = 0.03 ** 2
        output.twist.covariance[0] = 0.25
        output.twist.covariance[7] = 0.25
        output.twist.covariance[14] = 0.50
        self.odom_pub.publish(output)
        self.last_output = output
        pose = PoseStamped()
        pose.header = copy.deepcopy(output.header)
        pose.pose = copy.deepcopy(output.pose.pose)
        self.path.header = copy.deepcopy(output.header)
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_path:
            self.path.poses = self.path.poses[-self.max_path:]
        self.path_pub.publish(self.path)
        self.counts["published"] += 1

    @staticmethod
    def _key(name, value):
        item = KeyValue()
        item.key = str(name)
        item.value = str(value)
        return item

    def _diagnostics(self):
        diagnostic = DiagnosticStatus()
        diagnostic.name = "unified_backend_fusion"
        diagnostic.hardware_id = "companion_computer"
        healthy = self.last_reason == "ok" and self.counts["optimization_errors"] == 0
        diagnostic.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        diagnostic.message = self.last_reason
        diagnostic.values = [
            self._key("scheduler_health", self.scheduler_health),
            self._key("window_states", self.backend.state_count),
            self._key("window_factors", self.backend.factor_count),
            self._key("callback_ms", f"{self.last_callback_ms:.3f}"),
            self._key("last_imu_reason", self.last_imu_reason),
            self._key("last_flow_reason", self.last_flow_reason),
        ]
        diagnostic.values.extend(
            self._key(name, value) for name, value in self.counts.items()
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(diagnostic)
        self.diagnostic_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedBackendNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
