"""Multi-channel, profile-driven fault injector for Robustness V3.

The node is deliberately outside the estimator.  It interposes isolated raw
topics and republishes canonical inputs, so a fault cannot silently alter the
fusion implementation, thresholds, or transaction semantics.
"""

import copy
import math
from typing import Dict, Iterable, List

import numpy as np
import rclpy
from fast_lio.msg import FrontendScanRequest, NativeLidarFactor
from mavros_msgs.msg import OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix
from uf_interfaces.msg import (
    FaultState,
    ReliabilityScore,
    VisualFeatureTracks,
)

from .fault_models import add_gnss_jump, shift_stamp
from .fault_profiles import FaultProfile, FaultSpec, load_fault_profile


CHANNEL_TYPES = {
    "native_lidar": NativeLidarFactor,
    "imu": Imu,
    "gnss": NavSatFix,
    "optical_flow": OpticalFlowRad,
    "vision": VisualFeatureTracks,
}
DEFAULT_INPUTS = {
    "native_lidar": "/robustness/raw/native_lidar_factor",
    "imu": "/robustness/raw/imu",
    "gnss": "/robustness/raw/gnss",
    "optical_flow": "/robustness/raw/optical_flow",
    "vision": "/robustness/raw/visual_tracks",
}
DEFAULT_OUTPUTS = {
    "native_lidar": "/fast_lio/native_lidar_factor",
    "imu": "/sensors/imu",
    "gnss": "/sensors/gnss/fix",
    "optical_flow": "/sensors/optical_flow/rad",
    "vision": "/vision/feature_tracks",
}


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_multiply_xyzw(a, b):
    ax, ay, az, aw = [float(value) for value in a]
    bx, by, bz, bw = [float(value) for value in b]
    result = np.asarray([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=float)
    return result / max(float(np.linalg.norm(result)), 1.0e-12)


def quaternion_rotation_xyzw(values):
    x, y, z, w = np.asarray(values, dtype=float)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


class RobustnessFaultInjector(Node):
    def __init__(self):
        super().__init__("robustness_v3_fault_injector")
        self.declare_parameter("profile_path", "")
        self.declare_parameter("profile", "nominal")
        self.declare_parameter(
            "frontend_scan_request_input_topic",
            "/robustness/raw/frontend_scan_request",
        )
        self.declare_parameter(
            "frontend_scan_request_output_topic",
            "/fast_lio/frontend_scan_request",
        )
        for channel in CHANNEL_TYPES:
            self.declare_parameter(
                f"{channel}_input_topic", DEFAULT_INPUTS[channel]
            )
            self.declare_parameter(
                f"{channel}_output_topic", DEFAULT_OUTPUTS[channel]
            )
        profile_path = str(self.get_parameter("profile_path").value)
        profile_name = str(self.get_parameter("profile").value)
        if not profile_path:
            raise ValueError("profile_path is required")
        self.profile: FaultProfile = load_fault_profile(profile_path, profile_name)
        self.specs: Dict[str, List[FaultSpec]] = {
            channel: [
                spec for spec in self.profile.faults if spec.channel == channel
            ]
            for channel in CHANNEL_TYPES
        }
        self.started_ns: Dict[str, int] = {}
        self.affected: Dict[int, int] = {
            id(spec): 0 for spec in self.profile.faults
        }
        self.last_active: Dict[int, bool] = {
            id(spec): False for spec in self.profile.faults
        }
        self.rng: Dict[int, np.random.Generator] = {
            id(spec): np.random.default_rng(self.profile.seed + spec.seed_offset)
            for spec in self.profile.faults
        }
        self.fault_pub = self.create_publisher(FaultState, "/fault/state", 50)
        self._output_publishers = {}
        for channel, message_type in CHANNEL_TYPES.items():
            self._output_publishers[channel] = self.create_publisher(
                message_type,
                str(self.get_parameter(f"{channel}_output_topic").value),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                message_type,
                str(self.get_parameter(f"{channel}_input_topic").value),
                lambda msg, name=channel: self._sensor(name, msg),
                qos_profile_sensor_data,
            )
        self._scan_request_publisher = self.create_publisher(
            FrontendScanRequest,
            str(self.get_parameter("frontend_scan_request_output_topic").value),
            QoSProfile(
                depth=4,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            FrontendScanRequest,
            str(self.get_parameter("frontend_scan_request_input_topic").value),
            self._frontend_scan_request,
            qos_profile_sensor_data,
        )
        self._score_publishers = {}
        for modality in ("lidar", "imu", "gnss", "optical_flow", "vision"):
            output = f"/reliability/{modality}_score"
            self._score_publishers[modality] = self.create_publisher(
                ReliabilityScore, output, qos_profile_sensor_data
            )
            self.create_subscription(
                ReliabilityScore,
                f"/robustness/raw/{modality}_score",
                lambda msg, name=modality: self._score(name, msg),
                qos_profile_sensor_data,
            )
        self.get_logger().info(
            f"Robustness V3 profile={self.profile.name} seed={self.profile.seed} "
            f"faults={len(self.profile.faults)} description={self.profile.description}"
        )

    def _elapsed(self, channel: str, source_ns: int) -> float:
        first = self.started_ns.get(channel)
        if first is None or source_ns < first:
            self.started_ns[channel] = source_ns
            first = source_ns
        return max(0.0, (source_ns - first) * 1.0e-9)

    @staticmethod
    def _active(spec: FaultSpec, elapsed_s: float) -> bool:
        return elapsed_s >= spec.start_s and (
            spec.duration_s <= 0.0 or elapsed_s < spec.start_s + spec.duration_s
        )

    def _publish_state(self, spec: FaultSpec, header, active: bool):
        key = id(spec)
        # Transitions are mandatory; active heartbeats make affected counts
        # visible without flooding nominal replay logs.
        if active:
            self.affected[key] += 1
        transitioned = self.last_active[key] != active
        self.last_active[key] = active
        if not transitioned and (not active or self.affected[key] % 20 != 0):
            return
        state = FaultState()
        state.header = copy.deepcopy(header)
        state.modality = spec.modality
        state.fault_type = spec.fault_type
        state.active = active
        state.magnitude = float(spec.magnitude)
        state.affected_messages = int(self.affected[key])
        state.timestamp_repaired = False
        state.timestamp_repairs = 0
        self.fault_pub.publish(state)

    @staticmethod
    def _select_rows(values: Iterable[float], width: int, indices: np.ndarray):
        source = list(values)
        return [source[width * i + j] for i in indices for j in range(width)]

    def _drop_correspondences(self, msg, spec):
        output = copy.deepcopy(msg)
        count = int(msg.matched_points)
        keep_count = max(0, int(round(count * (1.0 - spec.magnitude))))
        if count <= 0 or keep_count >= count:
            return output
        indices = np.sort(self.rng[id(spec)].choice(
            count, size=keep_count, replace=False
        ))
        output.lidar_points_xyz = self._select_rows(
            msg.lidar_points_xyz, 3, indices
        )
        output.plane_normals_xyz = self._select_rows(
            msg.plane_normals_xyz, 3, indices
        )
        output.plane_points_xyz = self._select_rows(
            msg.plane_points_xyz, 3, indices
        )
        output.residuals = [msg.residuals[i] for i in indices]
        columns = int(msg.jacobian_columns)
        if columns > 0 and len(msg.jacobian) == count * columns:
            output.jacobian = self._select_rows(msg.jacobian, columns, indices)
        output.matched_points = keep_count
        output.correspondences_valid = bool(keep_count >= 3)
        return self._relinearize_native_message(output, count)

    @staticmethod
    def _relinearize_native_message(msg, original_count=None):
        """Keep the diagnostic normal equation consistent with raw rows.

        The manifold backend relinearizes raw geometry, while its observability
        check reads the pose block of the exported normal equation.  Updating
        both prevents a dropout profile from advertising the nominal scan's
        information.  Non-pose calibration blocks are proportionally retained
        because the V3 backend keeps online calibration shadow-only.
        """
        output = copy.deepcopy(msg)
        count = int(output.matched_points)
        if count <= 0:
            output.residuals = []
            output.state_hessian = [0.0] * 144
            output.state_gradient = [0.0] * 12
            return output
        points = np.asarray(output.lidar_points_xyz, dtype=float).reshape(count, 3)
        normals = np.asarray(output.plane_normals_xyz, dtype=float).reshape(count, 3)
        plane_points = np.asarray(output.plane_points_xyz, dtype=float).reshape(count, 3)
        body_rotation = quaternion_rotation_xyzw(output.lidar_to_body_quaternion)
        body_translation = np.asarray(output.lidar_to_body_translation, dtype=float)
        state_rotation = quaternion_rotation_xyzw(output.linearization_quaternion)
        state_position = np.asarray(output.linearization_position, dtype=float)
        body_points = points @ body_rotation.T + body_translation
        world_points = body_points @ state_rotation.T + state_position
        residual = np.sum(normals * (world_points - plane_points), axis=1)
        normals_body = normals @ state_rotation
        pose_jacobian = np.concatenate(
            (normals, np.cross(body_points, normals_body)), axis=1
        )
        original = max(count, int(original_count or count))
        ratio = count / original
        full_hessian = np.asarray(output.state_hessian, dtype=float).reshape(12, 12) * ratio
        full_gradient = np.asarray(output.state_gradient, dtype=float) * ratio
        full_hessian[:6, :6] = pose_jacobian.T @ pose_jacobian
        full_gradient[:6] = pose_jacobian.T @ residual
        output.residuals = residual.tolist()
        output.state_hessian = full_hessian.reshape(-1).tolist()
        output.state_gradient = full_gradient.tolist()
        return output

    @staticmethod
    def _shift_native_stamp(msg, offset_s):
        output = copy.deepcopy(msg)
        shift_stamp(output.header.stamp, offset_s)
        shift_stamp(output.scan_begin_stamp, offset_s)
        shift_stamp(output.scan_end_stamp, offset_s)
        return output

    @staticmethod
    def _shift_scan_request_stamp(msg, offset_s):
        output = copy.deepcopy(msg)
        shift_stamp(output.header.stamp, offset_s)
        shift_stamp(output.scan_begin_stamp, offset_s)
        shift_stamp(output.scan_end_stamp, offset_s)
        return output

    def _frontend_scan_request(self, msg):
        """Shift both halves of the frozen FAST-LIO/backend time contract.

        The replay has no packets or per-point time fields. A coherent contract
        shift therefore moves FrontendScanRequest and NativeLidarFactor scan
        intervals together. A factor_only profile deliberately leaves this
        request unchanged to reproduce an interface mismatch.
        """
        source_ns = stamp_ns(msg.header.stamp)
        if source_ns <= 0:
            return
        elapsed_s = self._elapsed("native_lidar", source_ns)
        output = copy.deepcopy(msg)
        for spec in self.specs["native_lidar"]:
            if (
                spec.fault_type == "time_offset"
                and spec.temporal_scope == "coherent_frontend_contract"
                and self._active(spec, elapsed_s)
            ):
                output = self._shift_scan_request_stamp(output, spec.magnitude)
        self._scan_request_publisher.publish(output)

    @staticmethod
    def _perturb_lidar_extrinsic(msg, rotation_deg, translation_m):
        output = copy.deepcopy(msg)
        half = math.radians(float(rotation_deg)) * 0.5
        yaw = [0.0, 0.0, math.sin(half), math.cos(half)]
        output.lidar_to_body_quaternion = list(quaternion_multiply_xyzw(
            yaw, output.lidar_to_body_quaternion
        ))
        translation = list(output.lidar_to_body_translation)
        translation[0] += float(translation_m)
        output.lidar_to_body_translation = translation
        return RobustnessFaultInjector._relinearize_native_message(output)

    def _drop_visual_tracks(self, msg, spec):
        output = copy.deepcopy(msg)
        count = len(msg.tracks)
        keep_count = max(0, int(round(count * (1.0 - spec.magnitude))))
        if keep_count < count:
            indices = np.sort(self.rng[id(spec)].choice(
                count, size=keep_count, replace=False
            ))
            output.tracks = [copy.deepcopy(msg.tracks[i]) for i in indices]
        output.feature_count = len(output.tracks)
        output.valid_depth_count = sum(track.depth_valid for track in output.tracks)
        output.klt_inlier_ratio = (
            sum(track.klt_inlier for track in output.tracks) / len(output.tracks)
            if output.tracks else 0.0
        )
        output.mean_reprojection_error_px = (
            sum(track.reprojection_error_px for track in output.tracks)
            / len(output.tracks) if output.tracks else 0.0
        )
        output.pnp_valid = bool(output.pnp_valid and len(output.tracks) >= 4)
        return output

    def _bias_visual_reprojection(self, msg, spec):
        output = copy.deepcopy(msg)
        fx = max(float(output.camera_matrix[0]), 1.0)
        fy = max(float(output.camera_matrix[4]), 1.0)
        rng = self.rng[id(spec)]
        errors = []
        for track in output.tracks:
            du, dv = rng.normal(0.0, abs(spec.magnitude), size=2)
            track.current_u += float(du)
            track.current_v += float(dv)
            track.current_x += float(du / fx)
            track.current_y += float(dv / fy)
            track.reprojection_error_px += float(math.hypot(du, dv))
            errors.append(float(track.reprojection_error_px))
        if errors:
            output.mean_reprojection_error_px = float(np.mean(errors))
        return output

    def _apply(self, channel: str, msg, spec: FaultSpec):
        if spec.fault_type == "time_offset":
            if channel == "native_lidar":
                return self._shift_native_stamp(msg, spec.magnitude)
            output = copy.deepcopy(msg)
            shift_stamp(output.header.stamp, spec.magnitude)
            if channel == "vision":
                shift_stamp(output.previous_stamp, spec.magnitude)
            return output
        if channel == "native_lidar":
            if spec.fault_type == "correspondence_dropout":
                return self._drop_correspondences(msg, spec)
            if spec.fault_type == "extrinsic_error":
                return self._perturb_lidar_extrinsic(
                    msg, spec.magnitude, spec.secondary_magnitude
                )
        output = copy.deepcopy(msg)
        if channel == "imu" and spec.fault_type == "bias":
            output.angular_velocity.z += spec.magnitude
            output.linear_acceleration.x += spec.secondary_magnitude
        elif channel == "imu" and spec.fault_type == "saturation":
            limit = abs(spec.magnitude)
            for axis in ("x", "y", "z"):
                value = getattr(output.angular_velocity, axis)
                setattr(output.angular_velocity, axis, max(-limit, min(limit, value)))
        elif channel == "gnss" and spec.fault_type == "jump":
            output = add_gnss_jump(msg, spec.magnitude, spec.secondary_magnitude)
        elif channel == "gnss" and spec.fault_type == "covariance_scale":
            output.position_covariance = [
                value * spec.magnitude for value in output.position_covariance
            ]
        elif channel == "optical_flow" and spec.fault_type == "low_quality":
            output.quality = max(0, min(255, int(spec.magnitude)))
        elif channel == "optical_flow" and spec.fault_type == "scale":
            output.integrated_x *= spec.magnitude
            output.integrated_y *= spec.magnitude
        elif channel == "vision" and spec.fault_type == "track_dropout":
            output = self._drop_visual_tracks(msg, spec)
        elif channel == "vision" and spec.fault_type == "reprojection_bias":
            output = self._bias_visual_reprojection(msg, spec)
        return output

    def _sensor(self, channel, msg):
        source_ns = stamp_ns(msg.header.stamp)
        if source_ns <= 0:
            return
        elapsed = self._elapsed(channel, source_ns)
        output = msg
        drop = False
        for spec in self.specs[channel]:
            active = self._active(spec, elapsed)
            self._publish_state(spec, msg.header, active)
            if not active:
                continue
            if spec.fault_type == "outage":
                drop = True
                continue
            output = self._apply(channel, output, spec)
        if not drop:
            self._output_publishers[channel].publish(output)

    def _score(self, modality, msg):
        source_ns = stamp_ns(msg.header.stamp)
        if source_ns <= 0:
            return
        channel = "native_lidar" if modality == "lidar" else modality
        elapsed = self._elapsed(channel, source_ns)
        floor = max((
            spec.score_floor for spec in self.specs[channel]
            if self._active(spec, elapsed)
        ), default=0.0)
        outage_active = any(
            spec.fault_type == "outage" and self._active(spec, elapsed)
            for spec in self.specs[channel]
        )
        output = copy.deepcopy(msg)
        if floor > 0.0:
            output.degradation_score = max(float(output.degradation_score), floor)
            output.reliability_weight = min(
                float(output.reliability_weight), max(0.0, 1.0 - floor)
            )
            reason = f"robustness_v3:{self.profile.name}"
            if reason not in output.reasons:
                output.reasons.append(reason)
        if outage_active:
            output.valid = False
            evidence = dict(zip(output.evidence_names, output.evidence_values))
            evidence["hard_gate_allowed"] = 0.0
            output.evidence_names = list(evidence)
            output.evidence_values = [float(evidence[name]) for name in output.evidence_names]
        self._score_publishers[modality].publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RobustnessFaultInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
