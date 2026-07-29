"""Online Ultra-Fusion-style fixed-lag fusion backend.

FAST-LIO owns scan deskew, point-to-plane association, and its local map.  The
manifold backend owns navigation-state IMU propagation and joint optimization;
FAST-LIO odometry is only the first-state initializer when native LiDAR factors
are available.  FCU fused local position and Gazebo truth never enter the
estimator.
"""

from bisect import bisect_left, bisect_right
from collections import deque
import copy
from dataclasses import dataclass, replace
import math
import queue
import threading
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from uf_interfaces.msg import ReliabilityScore, SchedulerState

from .imu_preintegration import (
    ImuSample,
    _quat_to_rotvec,
    preintegrate,
    preintegrate_manifold,
)
from .manifold_window import ManifoldSlidingWindowBackend, propagate_state
from .native_lidar import (
    NativeFactorBuffer,
    native_factor_from_message,
    quaternion_xyzw_to_rpy,
    right_perturbation_jacobian_rpy,
    rpy_to_quaternion_xyzw,
    rpy_to_rotation_matrix,
    validate_native_frame_contract,
    with_yaw_reference,
)
from .window import SlidingWindowBackend
from uf_reliability.scoring import (
    gnss_score,
    optical_flow_displacement_frd,
    optical_flow_score,
)
from uf_reliability.flow_rotation_gate import (
    FlowRotationGateConfig,
    OpticalFlowRotationGate,
    interval_mean_absolute_yaw_rate,
)

try:
    from fast_lio.msg import NativeLidarFactor
except ImportError:  # pragma: no cover - unit tests run without the external overlay
    NativeLidarFactor = None


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
MIN_FLOW_QUALITY = 20
MAX_COVARIANCE_INFLATION = 20.0


def enqueue_latest(work_queue, item):
    """Keep only the newest unprocessed native frame when the worker lags."""
    try:
        work_queue.put_nowait(item)
        return 0
    except queue.Full:
        pass
    discarded = 0
    while True:
        try:
            work_queue.get_nowait()
            work_queue.task_done()
            discarded += 1
        except queue.Empty:
            break
    try:
        work_queue.put_nowait(item)
    except queue.Full:
        return -1
    return discarded


@dataclass(frozen=True)
class StationaryImuInitialization:
    valid: bool
    reason: str
    accel_bias: tuple[float, float, float]
    gyro_bias: tuple[float, float, float]
    sample_count: int
    span_s: float
    accel_norm_mps2: float
    accel_residual_rms_mps2: float
    gyro_norm_radps: float
    gyro_residual_rms_radps: float


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def native_frame_odometry(header, factor):
    """Build a keyframe trigger without consuming FAST-LIO odometry."""
    message = Odometry()
    message.header = copy.deepcopy(header)
    message.header.frame_id = factor.map_frame
    message.child_frame_id = factor.state_frame
    position = factor.linearization_pose[:3]
    quaternion = rpy_to_quaternion_xyzw(factor.linearization_pose[3:6])
    message.pose.pose.position.x = float(position[0])
    message.pose.pose.position.y = float(position[1])
    message.pose.pose.position.z = float(position[2])
    message.pose.pose.orientation.x = quaternion[0]
    message.pose.pose.orientation.y = quaternion[1]
    message.pose.pose.orientation.z = quaternion[2]
    message.pose.pose.orientation.w = quaternion[3]
    return message


def native_trigger_order_status(last_stamp_ns, last_sequence, stamp_ns, sequence):
    """Classify a NativeLidarFactor trigger without mutating backend state."""
    if last_stamp_ns is None or last_sequence is None:
        return "accept", 0
    same_stamp = int(stamp_ns) == int(last_stamp_ns)
    same_sequence = int(sequence) == int(last_sequence)
    if same_stamp and same_sequence:
        return "duplicate", 0
    if same_stamp != same_sequence:
        return "sequence_conflict", 0
    if int(stamp_ns) < int(last_stamp_ns):
        return "nonmonotonic", 0
    if int(sequence) < int(last_sequence):
        return "sequence_reset", 0
    return "accept", max(0, int(sequence) - int(last_sequence) - 1)


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


def apply_flow_rotation_gate(decision, gate_result):
    """Apply the low-latency FCU rotation gate after scheduler weighting."""
    gated = copy.deepcopy(decision)
    rotation_weight = max(0.0, min(1.0, float(gate_result.weight)))
    gated["degradation_score"] = max(
        float(gated.get("degradation_score", 0.0)), 1.0 - rotation_weight
    )
    reasons = list(gated.get("reasons", ()))
    if gate_result.phase != "ACTIVE" and gate_result.reason not in reasons:
        reasons.append(gate_result.reason)
    gated["reasons"] = reasons
    if gate_result.hard_disabled or rotation_weight <= 0.0:
        gated["factor_enabled"] = False
        gated["reliability_weight"] = 0.0
        gated["covariance_inflation"] = MAX_COVARIANCE_INFLATION
        return gated
    gated["reliability_weight"] = min(
        float(gated.get("reliability_weight", 1.0)), rotation_weight
    )
    gated["covariance_inflation"] = max(
        float(gated.get("covariance_inflation", 1.0)),
        min(
            MAX_COVARIANCE_INFLATION,
            1.0 / max(0.05, rotation_weight),
        ),
    )
    gated["factor_enabled"] = bool(
        gated.get("factor_enabled", True)
        and gated["reliability_weight"] > 0.0
    )
    return gated


def apply_lidar_anchor_floor(
    decision, minimum_effective_weight=0.10, maximum_inflation=5.0,
):
    """Retain enough LiDAR information while no independent yaw source exists."""
    minimum_effective_weight = float(minimum_effective_weight)
    maximum_inflation = float(maximum_inflation)
    if not 0.0 < minimum_effective_weight <= 1.0:
        raise ValueError("minimum effective LiDAR weight must be in (0, 1]")
    if maximum_inflation < 1.0:
        raise ValueError("maximum LiDAR anchor inflation must be at least one")
    protected = dict(decision)
    original_enabled = bool(protected.get("factor_enabled", True))
    original_weight = float(protected.get("reliability_weight", 1.0))
    original_inflation = float(protected.get("covariance_inflation", 1.0))
    inflation = min(original_inflation, maximum_inflation)
    minimum_weight = min(1.0, minimum_effective_weight * inflation)
    protected["factor_enabled"] = True
    protected["covariance_inflation"] = inflation
    protected["reliability_weight"] = max(original_weight, minimum_weight)
    if (
        not original_enabled
        or protected["reliability_weight"] != original_weight
        or inflation != original_inflation
    ):
        protected["anchor_override"] = True
    return protected


def lidar_bypass_allowed(
        preserve_lio_anchor, lidar_score_fresh,
        imu_score_fresh, imu_factor_enabled):
    """Require explicit opt-in plus a live inertial orientation backup."""
    return bool(
        not preserve_lio_anchor
        and lidar_score_fresh
        and imu_score_fresh
        and imu_factor_enabled
    )


def gnss_jump_rejected(current_position, gnss_position, gate_m=20.0):
    current = np.asarray(current_position, dtype=float)
    measurement = np.asarray(gnss_position, dtype=float)
    if current.shape != (3,) or measurement.shape != (3,):
        raise ValueError("GNSS jump gate expects two 3-vectors")
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(measurement)):
        return True
    return float(np.linalg.norm(current - measurement)) > float(gate_m)


def gnss_temporal_jump_rejected(
    previous_position,
    previous_stamp_s,
    current_position,
    current_stamp_s,
    gate_m=20.0,
    maximum_speed_mps=15.0,
):
    """Reject a GNSS jump using accepted raw fixes, not the fused estimate.

    Comparing a fix with a LiDAR-corrupted state creates a feedback failure:
    once LiDAR drifts, valid GNSS is mislabeled as a jump and can no longer
    recover the estimator. A temporal gate remains independent of that state.
    """
    if previous_position is None:
        return False
    previous = np.asarray(previous_position, dtype=float)
    current = np.asarray(current_position, dtype=float)
    if previous.shape != (3,) or current.shape != (3,):
        raise ValueError("GNSS temporal jump gate expects two 3-vectors")
    if not np.all(np.isfinite(previous)) or not np.all(np.isfinite(current)):
        return True
    previous_stamp_s = float(previous_stamp_s)
    current_stamp_s = float(current_stamp_s)
    if not math.isfinite(previous_stamp_s) or not math.isfinite(current_stamp_s):
        return True
    dt_s = current_stamp_s - previous_stamp_s
    if dt_s <= 0.0:
        return True
    gate_m = float(gate_m)
    maximum_speed_mps = float(maximum_speed_mps)
    if not math.isfinite(gate_m) or gate_m <= 0.0:
        raise ValueError("GNSS temporal jump distance gate must be positive")
    if not math.isfinite(maximum_speed_mps) or maximum_speed_mps <= 0.0:
        raise ValueError("GNSS temporal jump speed gate must be positive")
    allowed_distance = max(gate_m, maximum_speed_mps * dt_s)
    return float(np.linalg.norm(current - previous)) > allowed_distance


def fused_motion_reference(previous_state, dt_s):
    """Propagate a short-horizon gate reference without using current LIO."""
    state = np.asarray(previous_state, dtype=float)
    dt_s = float(dt_s)
    if state.shape != (15,) or not np.all(np.isfinite(state)):
        raise ValueError("previous fused state must be a finite 15-vector")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("propagation interval must be finite and positive")
    delta_position = state[6:9] * dt_s
    return {
        "position": state[:3] + delta_position,
        "delta_position": delta_position,
        "orientation": state[3:6].copy(),
        "yaw": float(state[5]),
    }


def manifold_motion_reference(previous_state, predicted_state):
    """Describe one backend-owned IMU propagation for gating and aiding."""
    previous = np.asarray(previous_state, dtype=float)
    predicted = np.asarray(predicted_state, dtype=float)
    if previous.shape != (15,) or predicted.shape != (15,):
        raise ValueError("manifold motion reference expects two 15-vectors")
    if np.any(~np.isfinite(previous)) or np.any(~np.isfinite(predicted)):
        raise ValueError("manifold motion reference must be finite")
    return {
        "position": predicted[:3].copy(),
        "delta_position": predicted[:3] - previous[:3],
        "orientation": predicted[3:6].copy(),
        "yaw": float(predicted[5]),
    }


def imu_interval_covered(latest_imu_stamp, target_stamp):
    if latest_imu_stamp is None:
        return False
    latest_imu_stamp = float(latest_imu_stamp)
    target_stamp = float(target_stamp)
    return (
        math.isfinite(latest_imu_stamp)
        and math.isfinite(target_stamp)
        and latest_imu_stamp >= target_stamp
    )


def ordered_imu_samples(samples):
    return sorted(samples, key=lambda sample: float(sample.stamp_s))


def imu_interval_status(samples, start_stamp, end_stamp, maximum_gap_s=0.10):
    start_stamp = float(start_stamp)
    end_stamp = float(end_stamp)
    maximum_gap_s = float(maximum_gap_s)
    if (
        not math.isfinite(start_stamp)
        or not math.isfinite(end_stamp)
        or end_stamp <= start_stamp
    ):
        return False, "invalid_interval", 0.0
    if not math.isfinite(maximum_gap_s) or maximum_gap_s <= 0.0:
        raise ValueError("maximum IMU gap must be finite and positive")
    stamps = sorted({
        float(sample.stamp_s)
        for sample in samples
        if math.isfinite(float(sample.stamp_s))
    })
    if len(stamps) < 2:
        return False, "insufficient_samples", 0.0
    start_index = bisect_right(stamps, start_stamp) - 1
    end_index = bisect_left(stamps, end_stamp)
    if start_index < 0 or end_index >= len(stamps):
        return False, "interval_not_covered", 0.0
    interval_stamps = stamps[start_index:end_index + 1]
    gaps = np.diff(interval_stamps)
    largest_gap = float(np.max(gaps)) if len(gaps) else 0.0
    if largest_gap > maximum_gap_s:
        return False, "sample_gap_exceeds_limit", largest_gap
    return True, "ok", largest_gap


def estimate_stationary_imu_bias(
    samples,
    reference_orientation,
    end_stamp_s,
    window_s=1.5,
    minimum_samples=40,
    minimum_span_s=0.8,
    maximum_mean_gyro_radps=0.08,
    maximum_gyro_residual_rms_radps=0.03,
    gravity_mps2=9.81,
    gravity_tolerance_mps2=0.60,
    maximum_accel_residual_rms_mps2=0.40,
):
    """Estimate one startup bias from an observably stationary FCU IMU window.

    The LiDAR orientation supplies only the gravity direction.  FCU EKF pose is
    deliberately not consumed, and the returned bias is used only to seed the
    first backend state before normal between-keyframe preintegration begins.
    """
    end_stamp_s = float(end_stamp_s)
    window_s = float(window_s)
    minimum_span_s = float(minimum_span_s)
    gravity_mps2 = float(gravity_mps2)
    if (
        not math.isfinite(end_stamp_s)
        or not math.isfinite(window_s)
        or window_s <= 0.0
        or not math.isfinite(minimum_span_s)
        or minimum_span_s <= 0.0
        or minimum_span_s > window_s
        or int(minimum_samples) < 2
        or not math.isfinite(gravity_mps2)
        or gravity_mps2 <= 0.0
    ):
        raise ValueError("invalid stationary IMU initialization limits")

    selected_by_stamp = {}
    start_stamp_s = end_stamp_s - window_s
    for sample in samples:
        stamp_s = float(sample.stamp_s)
        acceleration = np.asarray(sample.acceleration, dtype=float)
        angular_velocity = np.asarray(sample.angular_velocity, dtype=float)
        if (
            math.isfinite(stamp_s)
            and start_stamp_s <= stamp_s <= end_stamp_s
            and acceleration.shape == (3,)
            and angular_velocity.shape == (3,)
            and np.all(np.isfinite(acceleration))
            and np.all(np.isfinite(angular_velocity))
        ):
            selected_by_stamp[stamp_s] = (acceleration, angular_velocity)
    selected = sorted(selected_by_stamp.items())

    zero = (0.0, 0.0, 0.0)
    if len(selected) < 2:
        return StationaryImuInitialization(
            False, "insufficient_observation_span", zero, zero,
            len(selected), 0.0, 0.0, 0.0, 0.0, 0.0,
        )
    span_s = float(selected[-1][0] - selected[0][0])
    if span_s < minimum_span_s:
        return StationaryImuInitialization(
            False, "insufficient_observation_span", zero, zero,
            len(selected), span_s, 0.0, 0.0, 0.0, 0.0,
        )
    if len(selected) < int(minimum_samples):
        return StationaryImuInitialization(
            False, "insufficient_samples", zero, zero,
            len(selected), span_s, 0.0, 0.0, 0.0, 0.0,
        )

    acceleration = np.stack([item[1][0] for item in selected])
    angular_velocity = np.stack([item[1][1] for item in selected])
    mean_acceleration = np.mean(acceleration, axis=0)
    mean_angular_velocity = np.mean(angular_velocity, axis=0)
    accel_norm = float(np.linalg.norm(mean_acceleration))
    gyro_norm = float(np.linalg.norm(mean_angular_velocity))
    accel_residual_rms = float(np.sqrt(np.mean(np.sum(
        (acceleration - mean_acceleration) ** 2, axis=1
    ))))
    gyro_residual_rms = float(np.sqrt(np.mean(np.sum(
        (angular_velocity - mean_angular_velocity) ** 2, axis=1
    ))))

    reason = "ok"
    if gyro_norm > float(maximum_mean_gyro_radps):
        reason = "mean_angular_rate_exceeds_limit"
    elif gyro_residual_rms > float(maximum_gyro_residual_rms_radps):
        reason = "angular_rate_variation_exceeds_limit"
    elif abs(accel_norm - gravity_mps2) > float(gravity_tolerance_mps2):
        reason = "specific_force_not_gravity"
    elif accel_residual_rms > float(maximum_accel_residual_rms_mps2):
        reason = "specific_force_variation_exceeds_limit"

    if reason != "ok":
        return StationaryImuInitialization(
            False, reason, zero, zero, len(selected), span_s,
            accel_norm, accel_residual_rms, gyro_norm, gyro_residual_rms,
        )

    orientation = np.asarray(reference_orientation, dtype=float)
    if orientation.shape != (3,) or np.any(~np.isfinite(orientation)):
        raise ValueError("reference orientation must be a finite RPY vector")
    expected_specific_force = (
        rpy_to_rotation_matrix(orientation).T
        @ np.asarray([0.0, 0.0, gravity_mps2], dtype=float)
    )
    accel_bias = mean_acceleration - expected_specific_force
    return StationaryImuInitialization(
        True,
        "ok",
        tuple(float(value) for value in accel_bias),
        tuple(float(value) for value in mean_angular_velocity),
        len(selected),
        span_s,
        accel_norm,
        accel_residual_rms,
        gyro_norm,
        gyro_residual_rms,
    )


def inflate_manifold_imu_covariance(
    covariance, motion_scale, minimum_bias_variance,
):
    """Inflate a full IMU covariance by congruence while preserving SPD."""
    values = np.asarray(covariance, dtype=float)
    if values.shape == (15,):
        values = np.diag(values)
    elif values.size == 225:
        values = values.reshape(15, 15)
    if values.shape != (15, 15) or np.any(~np.isfinite(values)):
        raise ValueError("manifold IMU covariance must be a finite 15x15 matrix")
    motion_scale = float(motion_scale)
    minimum_bias_variance = float(minimum_bias_variance)
    if (
        not math.isfinite(motion_scale)
        or motion_scale < 1.0
        or not math.isfinite(minimum_bias_variance)
        or minimum_bias_variance <= 0.0
    ):
        raise ValueError("IMU covariance inflation limits are invalid")
    scale = np.ones(15, dtype=float)
    scale[:9] = math.sqrt(motion_scale)
    inflated = scale[:, None] * values * scale[None, :]
    bias_diagonal = np.diag(inflated)[9:15]
    diagonal_addition = np.maximum(
        minimum_bias_variance - bias_diagonal, 0.0
    )
    inflated[9:15, 9:15] += np.diag(diagonal_addition)
    inflated = 0.5 * (inflated + inflated.T)
    try:
        np.linalg.cholesky(inflated)
    except np.linalg.LinAlgError as error:
        raise ValueError("inflated IMU covariance is not positive definite") from error
    return inflated


def lidar_prediction_innovation(position, yaw, reference):
    """Compare LIO with a short-horizon prediction formed before current LIO."""
    current = np.asarray(position, dtype=float)
    predicted = np.asarray(reference["position"], dtype=float)
    if current.shape != (3,) or predicted.shape != (3,):
        raise ValueError("LiDAR prediction innovation expects two 3-vectors")
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(predicted)):
        raise ValueError("LiDAR prediction innovation must be finite")
    yaw_error = math.atan2(
        math.sin(float(yaw) - float(reference["yaw"])),
        math.cos(float(yaw) - float(reference["yaw"])),
    )
    return {
        "position_m": float(np.linalg.norm(current - predicted)),
        "yaw_rad": abs(float(yaw_error)),
    }


def flow_observation_delta(flow_records, yaw):
    """Aggregate valid MAVLink optical-flow increments into map ENU."""
    delta = np.zeros(2, dtype=float)
    delta_body = np.zeros(3, dtype=float)
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
        delta_body += np.asarray(
            [float(displacement[0]), -float(displacement[1]), 0.0],
            dtype=float,
        )
        qualities.append(float(flow["quality"]))
        distances.append(distance)
    if not qualities:
        return None
    return {
        "delta_position": [float(delta[0]), float(delta[1]), 0.0],
        "delta_body": [float(value) for value in delta_body],
        "quality": float(np.mean(qualities)),
        "distance_m": float(np.mean(distances)),
        "sample_count": len(qualities),
    }


class UnifiedBackendNode(Node):
    def __init__(self):
        super().__init__("unified_backend_fusion")
        self.imu_buffer_lock = threading.Lock()
        defaults = {
            "lio_topic": "/lio/odom",
            "native_lidar_factor_topic": "/fast_lio/native_lidar_factor",
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
        self.declare_parameter("backend_solver_mode", "manifold")
        self.declare_parameter("nonlinear_max_iterations", 4)
        self.declare_parameter("nonlinear_damping", 1.0e-6)
        self.declare_parameter("nonlinear_convergence_threshold", 1.0e-5)
        self.declare_parameter("lidar_huber_delta", 2.5)
        self.declare_parameter("lm_max_trials", 6)
        self.declare_parameter("lm_damping_up", 10.0)
        self.declare_parameter("lm_damping_down", 0.3)
        self.declare_parameter("allow_lio_pose_fallback", False)
        self.declare_parameter("gnss_max_age_s", 2.0)
        self.declare_parameter("flow_max_age_s", 1.0)
        self.declare_parameter("imu_buffer_s", 5.0)
        self.declare_parameter("imu_factor_wait_s", 0.080)
        self.declare_parameter("imu_max_gap_s", 0.10)
        self.declare_parameter("imu_startup_bias_initialization_enabled", True)
        self.declare_parameter("imu_startup_window_s", 1.5)
        self.declare_parameter("imu_startup_minimum_samples", 40)
        self.declare_parameter("imu_startup_minimum_span_s", 0.8)
        self.declare_parameter("imu_startup_maximum_mean_gyro_radps", 0.08)
        self.declare_parameter("imu_startup_maximum_gyro_residual_rms_radps", 0.03)
        self.declare_parameter("imu_startup_gravity_tolerance_mps2", 0.60)
        self.declare_parameter("imu_startup_maximum_accel_residual_rms_mps2", 0.40)
        self.declare_parameter("minimum_flow_quality", MIN_FLOW_QUALITY)
        self.declare_parameter("minimum_flow_distance_m", 0.08)
        self.declare_parameter("maximum_flow_distance_m", 12.0)
        self.declare_parameter("gnss_default_variance_m2", 4.0)
        self.declare_parameter("gnss_jump_gate_m", 20.0)
        self.declare_parameter("gnss_jump_speed_mps", 15.0)
        self.declare_parameter("optical_flow_yaw_coupling_enabled", False)
        self.declare_parameter("flow_rotation_lower_yaw_rate_radps", 0.08)
        self.declare_parameter("flow_rotation_upper_yaw_rate_radps", 0.30)
        self.declare_parameter("flow_rotation_recovery_dwell_s", 0.8)
        self.declare_parameter("flow_rotation_recovery_ramp_s", 1.5)
        self.declare_parameter("flow_rotation_minimum_translation_m", 0.01)
        self.declare_parameter("flow_rotation_recovery_max_base_score", 0.55)
        self.declare_parameter("flow_rotation_imu_max_gap_s", 0.12)
        self.declare_parameter("imu_factor_enabled", True)
        self.declare_parameter("preserve_lio_anchor", True)
        self.declare_parameter("lidar_anchor_minimum_effective_weight", 0.10)
        self.declare_parameter("lidar_anchor_maximum_covariance_inflation", 5.0)
        self.declare_parameter("native_lidar_factor_enabled", True)
        self.declare_parameter("input_trigger_mode", "native_factor")
        self.declare_parameter("native_lidar_factor_tolerance_s", 0.005)
        self.declare_parameter("native_lidar_factor_wait_s", 0.030)
        self.declare_parameter("native_lidar_minimum_matches", 50)
        self.declare_parameter("native_lidar_qos_depth", 32)
        self.declare_parameter("native_worker_queue_size", 1)
        self.declare_parameter("imu_qos_depth", 64)
        self.declare_parameter("imu_covariance_scale", 50.0)
        self.declare_parameter("imu_bias_random_walk_variance", 1.0e-4)
        self.declare_parameter("scheduler_timeout_s", 1.0)
        self.declare_parameter("reliability_mode", "dynamic")
        self.declare_parameter("fixed_lidar_weight", 1.0)
        self.declare_parameter("fixed_gnss_weight", 1.0)
        self.declare_parameter("fixed_imu_weight", 1.0)
        self.declare_parameter("fixed_optical_flow_weight", 1.0)
        self.declare_parameter("fixed_covariance_inflation", 1.0)
        self.declare_parameter("publish_path_length", 2000)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.gnss_max_age_s = float(self.get_parameter("gnss_max_age_s").value)
        self.flow_max_age_s = float(self.get_parameter("flow_max_age_s").value)
        self.imu_buffer_s = float(self.get_parameter("imu_buffer_s").value)
        self.imu_factor_wait_s = float(
            self.get_parameter("imu_factor_wait_s").value
        )
        self.imu_max_gap_s = float(self.get_parameter("imu_max_gap_s").value)
        if self.imu_factor_wait_s < 0.0 or self.imu_max_gap_s <= 0.0:
            raise ValueError("IMU timing limits are invalid")
        self.imu_startup_bias_initialization_enabled = bool(
            self.get_parameter("imu_startup_bias_initialization_enabled").value
        )
        self.imu_startup_window_s = float(
            self.get_parameter("imu_startup_window_s").value
        )
        self.imu_startup_minimum_samples = int(
            self.get_parameter("imu_startup_minimum_samples").value
        )
        self.imu_startup_minimum_span_s = float(
            self.get_parameter("imu_startup_minimum_span_s").value
        )
        self.imu_startup_maximum_mean_gyro_radps = float(
            self.get_parameter("imu_startup_maximum_mean_gyro_radps").value
        )
        self.imu_startup_maximum_gyro_residual_rms_radps = float(
            self.get_parameter(
                "imu_startup_maximum_gyro_residual_rms_radps"
            ).value
        )
        self.imu_startup_gravity_tolerance_mps2 = float(
            self.get_parameter("imu_startup_gravity_tolerance_mps2").value
        )
        self.imu_startup_maximum_accel_residual_rms_mps2 = float(
            self.get_parameter(
                "imu_startup_maximum_accel_residual_rms_mps2"
            ).value
        )
        if (
            self.imu_startup_window_s <= 0.0
            or self.imu_startup_minimum_samples < 2
            or self.imu_startup_minimum_span_s <= 0.0
            or self.imu_startup_minimum_span_s > self.imu_startup_window_s
            or self.imu_startup_maximum_mean_gyro_radps <= 0.0
            or self.imu_startup_maximum_gyro_residual_rms_radps <= 0.0
            or self.imu_startup_gravity_tolerance_mps2 <= 0.0
            or self.imu_startup_maximum_accel_residual_rms_mps2 <= 0.0
        ):
            raise ValueError("stationary IMU initialization limits are invalid")
        self.minimum_flow_quality = int(self.get_parameter("minimum_flow_quality").value)
        self.minimum_flow_distance_m = float(
            self.get_parameter("minimum_flow_distance_m").value)
        self.maximum_flow_distance_m = float(
            self.get_parameter("maximum_flow_distance_m").value)
        self.gnss_default_variance = float(
            self.get_parameter("gnss_default_variance_m2").value)
        self.gnss_jump_gate_m = float(
            self.get_parameter("gnss_jump_gate_m").value)
        self.gnss_jump_speed_mps = float(
            self.get_parameter("gnss_jump_speed_mps").value)
        self.optical_flow_yaw_coupling_enabled = bool(
            self.get_parameter("optical_flow_yaw_coupling_enabled").value)
        self.flow_rotation_recovery_max_base_score = float(
            self.get_parameter("flow_rotation_recovery_max_base_score").value)
        self.flow_rotation_imu_max_gap_s = float(
            self.get_parameter("flow_rotation_imu_max_gap_s").value)
        self.imu_factor_enabled = bool(self.get_parameter("imu_factor_enabled").value)
        self.preserve_lio_anchor = bool(self.get_parameter("preserve_lio_anchor").value)
        self.lidar_anchor_minimum_effective_weight = float(
            self.get_parameter("lidar_anchor_minimum_effective_weight").value
        )
        self.lidar_anchor_maximum_covariance_inflation = float(
            self.get_parameter("lidar_anchor_maximum_covariance_inflation").value
        )
        if not 0.0 < self.lidar_anchor_minimum_effective_weight <= 1.0:
            raise ValueError(
                "lidar_anchor_minimum_effective_weight must be in (0, 1]"
            )
        if self.lidar_anchor_maximum_covariance_inflation < 1.0:
            raise ValueError(
                "lidar_anchor_maximum_covariance_inflation must be at least one"
            )
        native_requested = bool(
            self.get_parameter("native_lidar_factor_enabled").value
        )
        self.native_lidar_enabled = bool(native_requested and NativeLidarFactor is not None)
        requested_trigger_mode = str(
            self.get_parameter("input_trigger_mode").value
        ).lower()
        if requested_trigger_mode not in {"native_factor", "lio_pair"}:
            raise ValueError("input_trigger_mode must be native_factor or lio_pair")
        self.input_trigger_mode = requested_trigger_mode
        if self.input_trigger_mode == "native_factor" and not self.native_lidar_enabled:
            self.input_trigger_mode = "lio_pair"
        self.native_lidar_tolerance_s = float(
            self.get_parameter("native_lidar_factor_tolerance_s").value
        )
        self.native_lidar_wait_s = float(
            self.get_parameter("native_lidar_factor_wait_s").value
        )
        self.native_lidar_minimum_matches = int(
            self.get_parameter("native_lidar_minimum_matches").value
        )
        if self.native_lidar_tolerance_s < 0.0 or self.native_lidar_wait_s < 0.0:
            raise ValueError("native LiDAR timing limits must be non-negative")
        if self.native_lidar_minimum_matches < 1:
            raise ValueError("native LiDAR minimum matches must be positive")
        self.imu_covariance_scale = float(
            self.get_parameter("imu_covariance_scale").value)
        self.imu_bias_random_walk_variance = float(
            self.get_parameter("imu_bias_random_walk_variance").value)
        self.scheduler_timeout_s = float(
            self.get_parameter("scheduler_timeout_s").value)
        self.reliability_mode = str(
            self.get_parameter("reliability_mode").value).lower()
        if self.reliability_mode not in {"dynamic", "fixed"}:
            raise ValueError("reliability_mode must be dynamic or fixed")
        self.fixed_weights = {
            modality: float(self.get_parameter(f"fixed_{modality}_weight").value)
            for modality in ("lidar", "gnss", "imu", "optical_flow")
        }
        if any(
            not math.isfinite(weight) or not 0.0 <= weight <= 1.0
            for weight in self.fixed_weights.values()
        ):
            raise ValueError("fixed modality weights must be finite in [0, 1]")
        self.fixed_covariance_inflation = float(
            self.get_parameter("fixed_covariance_inflation").value)
        if (
            not math.isfinite(self.fixed_covariance_inflation)
            or self.fixed_covariance_inflation < 1.0
        ):
            raise ValueError("fixed_covariance_inflation must be at least one")
        self.max_path = int(self.get_parameter("publish_path_length").value)
        self.backend_solver_mode = str(
            self.get_parameter("backend_solver_mode").value
        ).lower()
        if self.backend_solver_mode not in {"manifold", "linear"}:
            raise ValueError("backend_solver_mode must be manifold or linear")
        self.allow_lio_pose_fallback = bool(
            self.get_parameter("allow_lio_pose_fallback").value
        )
        self.native_lidar_qos_depth = int(
            self.get_parameter("native_lidar_qos_depth").value
        )
        self.native_worker_queue_size = int(
            self.get_parameter("native_worker_queue_size").value
        )
        self.imu_qos_depth = int(self.get_parameter("imu_qos_depth").value)
        if (
            self.native_lidar_qos_depth < 1
            or self.native_worker_queue_size < 1
            or self.imu_qos_depth < 2
        ):
            raise ValueError("sensor QoS depths must be positive")
        self.native_lidar_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.native_lidar_qos_depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.imu_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.imu_qos_depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        window_size = max(2, int(self.get_parameter("window_size").value))
        if self.backend_solver_mode == "manifold":
            self.backend = ManifoldSlidingWindowBackend(
                max_states=window_size,
                max_iterations=int(
                    self.get_parameter("nonlinear_max_iterations").value
                ),
                damping=float(self.get_parameter("nonlinear_damping").value),
                convergence_threshold=float(
                    self.get_parameter("nonlinear_convergence_threshold").value
                ),
                lidar_huber_delta=float(
                    self.get_parameter("lidar_huber_delta").value
                ),
                lm_max_trials=int(self.get_parameter("lm_max_trials").value),
                lm_damping_up=float(
                    self.get_parameter("lm_damping_up").value
                ),
                lm_damping_down=float(
                    self.get_parameter("lm_damping_down").value
                ),
            )
        else:
            self.backend = SlidingWindowBackend(max_states=window_size)
        self.path = Path()
        self.path.poses = []
        self.imu_buffer = deque(maxlen=10000)
        self.flow_buffer = deque(maxlen=3000)
        self.flow_rotation_gate = OpticalFlowRotationGate(
            FlowRotationGateConfig(
                lower_yaw_rate_radps=float(self.get_parameter(
                    "flow_rotation_lower_yaw_rate_radps").value),
                upper_yaw_rate_radps=float(self.get_parameter(
                    "flow_rotation_upper_yaw_rate_radps").value),
                recovery_dwell_s=float(self.get_parameter(
                    "flow_rotation_recovery_dwell_s").value),
                recovery_ramp_s=float(self.get_parameter(
                    "flow_rotation_recovery_ramp_s").value),
                minimum_translation_m=float(self.get_parameter(
                    "flow_rotation_minimum_translation_m").value),
            )
        )
        self.native_lidar_buffer = NativeFactorBuffer(max_size=128)
        self.native_work_queue = queue.Queue(maxsize=self.native_worker_queue_size)
        self.native_worker_stop = threading.Event()
        self.native_worker_thread = None
        self.pending_lio = deque(maxlen=32)
        self.pending_imu_lio = deque(maxlen=64)
        self.latest_gnss = None
        self.last_gnss_admitted = None
        self.projector = None
        self.lio_origin = None
        self.last_lio_stamp = None
        self.last_lio_position = None
        self.last_lio_yaw = 0.0
        self.last_imu_arrival_stamp = None
        self.imu_max_positive_arrival_gap_s = 0.0
        self.flow_clock_offset_s = None
        self.scheduler = {}
        self.scheduler_arrival = None
        self.scheduler_health = "UNAVAILABLE"
        self.scores = {}
        self.counts = {
            "lio": 0, "published": 0, "lidar_factors": 0,
            "lidar_disabled": 0, "gnss_factors": 0, "gnss_jump_rejected": 0,
            "flow_factor_attempts": 0, "flow_factors": 0,
            "flow_disabled_quality": 0,
            "flow_disabled_rotation": 0,
            "imu_factors": 0, "imu_invalid": 0, "optimization_errors": 0,
            "lidar_anchor_overrides": 0, "imu_residual_updates": 0,
            "imu_residual_errors": 0,
            "native_lidar_received": 0, "native_lidar_invalid": 0,
            "native_lidar_factors": 0, "native_lidar_hard_disabled": 0,
            "native_lidar_pose_fallbacks": 0, "native_lidar_pair_timeouts": 0,
            "native_lidar_relinearized": 0,
            "native_lidar_condensed_fallbacks": 0,
            "native_trigger_only_frames": 0,
            "native_trigger_duplicates": 0,
            "native_trigger_nonmonotonic": 0,
            "native_trigger_sequence_conflicts": 0,
            "native_trigger_sequence_gaps": 0,
            "native_trigger_waiting_for_initial_factor": 0,
            "native_worker_queue_overflow": 0,
            "native_worker_queue_discarded": 0,
            "native_worker_errors": 0,
            "lio_pose_inputs_ignored": 0,
            "imu_propagated_initializations": 0,
            "imu_pair_timeouts": 0,
            "imu_received": 0,
            "imu_nonmonotonic_arrivals": 0,
            "imu_startup_waits": 0,
            "imu_startup_bias_accepted": 0,
            "imu_startup_bias_rejected": 0,
        }
        self.imu_invalid_reasons = {}
        self.last_reason = "waiting_for_lio"
        self.last_callback_ms = 0.0
        self.last_imu_reason = "unavailable"
        self.last_imu_startup_reason = "not_attempted"
        self.last_imu_startup_sample_count = 0
        self.last_imu_startup_span_s = 0.0
        self.last_imu_startup_accel_bias = np.zeros(3, dtype=float)
        self.last_imu_startup_gyro_bias = np.zeros(3, dtype=float)
        self.last_imu_preintegration_residual_mahalanobis = -1.0
        self.last_imu_residual_error = "none"
        self.last_exception = "none"
        self.last_flow_reason = "unavailable"
        self.last_flow_factor_type = "unavailable"
        self.last_flow_rotation_phase = "unavailable"
        self.last_flow_rotation_weight = 0.0
        self.last_flow_yaw_rate_abs_radps = -1.0
        self.last_lidar_prediction_position_innovation_m = -1.0
        self.last_lidar_prediction_yaw_innovation_rad = -1.0
        self.last_lidar_source = "unavailable"
        self.last_native_sequence = -1
        self.last_native_input_stamp_ns = None
        self.last_native_input_sequence = None
        self.last_native_matches = 0
        self.last_native_stamp_error_ms = -1.0
        self.last_output = None
        self.backend_solve_count = 0
        self.backend_solve_ms_total = 0.0
        self.backend_solve_ms_max = 0.0

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostic_topic").value), 10)
        if self.input_trigger_mode == "lio_pair":
            self.create_subscription(
                Odometry, str(self.get_parameter("lio_topic").value),
                self._lio, 20)
        if self.native_lidar_enabled:
            self.create_subscription(
                NativeLidarFactor,
                str(self.get_parameter("native_lidar_factor_topic").value),
                self._native_lidar,
                self.native_lidar_qos,
            )
        if self.input_trigger_mode == "lio_pair" and (
            self.native_lidar_enabled or self.backend_solver_mode == "manifold"
        ):
            self.create_timer(0.010, self._drain_pending_inputs)
        self.create_subscription(
            NavSatFix, str(self.get_parameter("gnss_topic").value),
            self._gnss, qos_profile_sensor_data)
        self.create_subscription(
            OpticalFlowRad, str(self.get_parameter("flow_topic").value),
            self._flow, qos_profile_sensor_data)
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value),
            self._imu, self.imu_qos)
        self.create_subscription(
            SchedulerState, str(self.get_parameter("scheduler_topic").value),
            self._scheduler, 20)
        for modality in ("lidar", "gnss", "imu", "optical_flow"):
            self.create_subscription(
                ReliabilityScore, f"/reliability/{modality}_score",
                lambda msg, name=modality: self._score(name, msg),
                qos_profile_sensor_data,
            )
        if self.input_trigger_mode == "native_factor":
            self.native_worker_thread = threading.Thread(
                target=self._native_worker_loop,
                name="uf-native-factor-worker",
                daemon=True,
            )
            self.native_worker_thread.start()
        self.create_timer(1.0, self._diagnostics)
        if native_requested and NativeLidarFactor is None:
            self.get_logger().warning(
                "FAST-LIO NativeLidarFactor is unavailable; using lio_pair trigger mode. "
                "Source the patched FAST-LIO overlay before launching the backend."
            )
        self.get_logger().info(
            f"Unified backend active: solver={self.backend_solver_mode}; "
            f"reliability_mode={self.reliability_mode}; native LiDAR + GNSS/flow; "
            f"input_trigger={self.input_trigger_mode}; "
            f"native_lidar={'on' if self.native_lidar_enabled else 'fallback'}; "
            f"preserve_lio_anchor={'on' if self.preserve_lio_anchor else 'off'}; "
            f"lio_pose_fallback={'on' if self.allow_lio_pose_fallback else 'off'}; "
            f"IMU preintegration={'on' if self.imu_factor_enabled else 'off'}")

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
        if self.reliability_mode == "fixed":
            weight = self.fixed_weights.get(modality, 1.0)
            return scheduler_decision(
                weight,
                default_enabled and weight > 0.0,
                self.fixed_covariance_inflation,
            )
        now = time.monotonic()
        # LIO is the local estimator anchor. A missing/stale diagnostic must
        # not silently remove its pose factor and leave rotation unobservable.
        score_item = self.scores.get(modality)
        score_fresh = self._score_is_fresh(modality, now)
        if modality == "lidar" and not score_fresh:
            return scheduler_decision(1.0, default_enabled, 1.0)
        if self.scheduler_arrival is not None and now - self.scheduler_arrival <= self.scheduler_timeout_s:
            item = self.scheduler.get(modality)
            if item is not None:
                decision = scheduler_decision(item[0], item[1], item[2])
                return self._protect_lidar_anchor(
                    modality, decision, now, score_fresh
                )
        item = self.scores.get(modality)
        if item is not None and now - item["stamp_mono"] <= self.scheduler_timeout_s:
            decision = scheduler_decision(item["weight"], item["valid"], 1.0)
            return self._protect_lidar_anchor(
                modality, decision, now, score_fresh
            )
        return scheduler_decision(1.0, default_enabled, 1.0)

    def _score_is_fresh(self, modality, now):
        item = self.scores.get(modality)
        return bool(
            item is not None
            and item["valid"]
            and now - item["stamp_mono"] <= self.scheduler_timeout_s
        )

    def _imu_backup_ready(self, now):
        score_fresh = self._score_is_fresh("imu", now)
        scheduler_fresh = (
            self.scheduler_arrival is not None
            and now - self.scheduler_arrival <= self.scheduler_timeout_s
        )
        if scheduler_fresh:
            item = self.scheduler.get("imu")
            factor_enabled = bool(
                item is not None and item[1] and float(item[0]) > 0.0
            )
        else:
            item = self.scores.get("imu")
            factor_enabled = bool(
                score_fresh and item is not None and item["weight"] > 0.0
            )
        return score_fresh, factor_enabled

    def _protect_lidar_anchor(self, modality, decision, now, score_fresh):
        if modality != "lidar":
            return decision
        if self.preserve_lio_anchor:
            return apply_lidar_anchor_floor(
                decision,
                self.lidar_anchor_minimum_effective_weight,
                self.lidar_anchor_maximum_covariance_inflation,
            )
        if decision["factor_enabled"]:
            return decision
        imu_score_fresh, imu_factor_enabled = self._imu_backup_ready(now)
        if lidar_bypass_allowed(
            self.preserve_lio_anchor, score_fresh,
            imu_score_fresh, imu_factor_enabled,
        ):
            return decision
        decision["factor_enabled"] = True
        decision["reliability_weight"] = max(0.05, decision["reliability_weight"])
        decision["anchor_override"] = True
        decision["covariance_inflation"] = max(
            1.0,
            min(MAX_COVARIANCE_INFLATION, decision["covariance_inflation"]),
        )
        return decision

    def _imu_snapshot(self):
        with self.imu_buffer_lock:
            return list(self.imu_buffer)

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
        with self.imu_buffer_lock:
            if self.last_imu_arrival_stamp is not None:
                arrival_delta = stamp - self.last_imu_arrival_stamp
                if arrival_delta <= 0.0:
                    self.counts["imu_nonmonotonic_arrivals"] += 1
                else:
                    self.imu_max_positive_arrival_gap_s = max(
                        self.imu_max_positive_arrival_gap_s, arrival_delta
                    )
            self.last_imu_arrival_stamp = stamp
            self.counts["imu_received"] += 1
            self.imu_buffer.append(sample)
            if self.last_lio_stamp is not None:
                cutoff = self.last_lio_stamp - self.imu_buffer_s
                self.imu_buffer = deque(
                    (
                        item for item in self.imu_buffer
                        if item.stamp_s >= cutoff
                    ),
                    maxlen=10000,
                )

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
        position_enu = np.asarray(self.projector.project(*values), dtype=float)
        stamp_s = stamp_seconds(msg.header.stamp)
        if stamp_s <= 0.0:
            stamp_s = self._now_s()
        temporal_jump = False
        if self.last_gnss_admitted is not None:
            temporal_jump = gnss_temporal_jump_rejected(
                self.last_gnss_admitted["position_enu"],
                self.last_gnss_admitted["stamp_s"],
                position_enu,
                stamp_s,
                self.gnss_jump_gate_m,
                self.gnss_jump_speed_mps,
            )
        if not temporal_jump:
            self.last_gnss_admitted = {
                "stamp_s": stamp_s,
                "position_enu": position_enu.copy(),
            }
        self.latest_gnss = {
            "stamp_s": stamp_s,
            "position_enu": position_enu,
            "covariance": covariance,
            "status": int(msg.status.status),
            "temporal_jump": temporal_jump,
        }

    def _imu_factor(
        self, previous_stamp, current_stamp, previous_orientation,
        previous_index, current_index,
    ):
        samples = ordered_imu_samples(self._imu_snapshot())
        if not self.imu_factor_enabled or len(samples) < 2:
            self.last_imu_reason = (
                "disabled" if not self.imu_factor_enabled else "insufficient_samples"
            )
            return None
        stamps = [sample.stamp_s for sample in samples]
        start = max(0, bisect_left(stamps, previous_stamp) - 1)
        end = min(len(samples), bisect_right(stamps, current_stamp) + 1)
        result = preintegrate(
            samples[start:end], previous_stamp, current_stamp,
            max_gap_s=self.imu_max_gap_s,
        )
        self.last_imu_reason = result.reason
        if not result.valid:
            self._record_imu_invalid(result.reason)
            return None
        map_rotation = rpy_to_rotation_matrix(previous_orientation)
        delta_position = map_rotation @ np.asarray(result.delta_position, dtype=float)
        delta_velocity = map_rotation @ np.asarray(result.delta_velocity, dtype=float)
        right_to_rpy = np.linalg.inv(
            right_perturbation_jacobian_rpy(previous_orientation)
        )
        delta_rotation = right_to_rpy @ _quat_to_rotvec(
            np.asarray(result.delta_quaternion)
        )
        position_accel_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_position_accel_bias).reshape(3, 3)
        position_gyro_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_position_gyro_bias).reshape(3, 3)
        velocity_accel_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_velocity_accel_bias).reshape(3, 3)
        velocity_gyro_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_velocity_gyro_bias).reshape(3, 3)
        rotation_gyro_jacobian = right_to_rpy @ np.asarray(
            result.jacobian_delta_rotation_gyro_bias).reshape(3, 3)
        nominal_covariance = np.maximum(
            np.asarray(result.covariance, dtype=float), 1.0e-6
        )
        optimization_covariance = np.maximum(
            nominal_covariance * self.imu_covariance_scale, 1.0e-6
        )
        bias_random_walk_covariance = np.full(
            6, self.imu_bias_random_walk_variance
        )
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
            covariance=optimization_covariance,
            bias_random_walk_covariance=bias_random_walk_covariance,
            decision=decision,
        )
        self.counts["imu_factors"] += 1
        return np.concatenate((nominal_covariance, bias_random_walk_covariance))

    def _manifold_imu_measurement(
        self, previous_stamp, current_stamp, previous_state,
    ):
        samples = ordered_imu_samples(self._imu_snapshot())
        if not self.imu_factor_enabled or len(samples) < 2:
            self.last_imu_reason = (
                "disabled" if not self.imu_factor_enabled else "insufficient_samples"
            )
            return None
        stamps = [sample.stamp_s for sample in samples]
        start = max(0, bisect_left(stamps, previous_stamp) - 1)
        end = min(len(samples), bisect_right(stamps, current_stamp) + 1)
        result = preintegrate_manifold(
            samples[start:end],
            previous_stamp,
            current_stamp,
            accel_bias=np.asarray(previous_state[9:12], dtype=float),
            gyro_bias=np.asarray(previous_state[12:15], dtype=float),
            max_gap_s=self.imu_max_gap_s,
        )
        self.last_imu_reason = result.reason
        if not result.valid:
            self._record_imu_invalid(result.reason)
            return None
        covariance = inflate_manifold_imu_covariance(
            result.covariance,
            self.imu_covariance_scale,
            self.imu_bias_random_walk_variance,
        )
        return replace(
            result,
            covariance=tuple(float(value) for value in covariance.ravel()),
        )

    def _add_manifold_imu_factor(self, previous_index, current_index, measurement):
        if measurement is None:
            return None
        self.backend.add_imu_preintegrated(
            previous_index,
            current_index,
            measurement,
            decision=self._decision("imu", default_enabled=True),
        )
        self.counts["imu_factors"] += 1
        return np.asarray(measurement.covariance, dtype=float)

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
        if bool(self.latest_gnss.get("temporal_jump", False)):
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
        observation = flow_observation_delta(records, previous_yaw)
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
        quality_or_distance_invalid = (
            observation["quality"] < self.minimum_flow_quality
            or not self.minimum_flow_distance_m <= observation["distance_m"] <= self.maximum_flow_distance_m
        )
        imu_yaw_samples = sorted([
            (sample.stamp_s, float(sample.angular_velocity[2]))
            for sample in self._imu_snapshot()
        ])
        yaw_rate = interval_mean_absolute_yaw_rate(
            imu_yaw_samples,
            previous_stamp,
            current_stamp,
            self.flow_rotation_imu_max_gap_s,
        )
        translation_norm = float(np.linalg.norm(
            np.asarray(observation["delta_body"], dtype=float)[:2]
        ))
        rotation_gate = self.flow_rotation_gate.update(
            current_stamp,
            yaw_rate,
            translation_norm,
            (
                not quality_or_distance_invalid
                and score <= self.flow_rotation_recovery_max_base_score
            ),
        )
        decision["evidence"].update({
            "fcu_yaw_rate_abs_radps": rotation_gate.yaw_rate_abs_radps,
            "rotation_gate_weight": rotation_gate.weight,
            "rotation_gate_phase_code": OpticalFlowRotationGate.PHASE_CODES[
                rotation_gate.phase
            ],
            "rotation_gate_translation_ready": (
                1.0 if rotation_gate.translation_ready else 0.0
            ),
        })
        decision = apply_flow_rotation_gate(decision, rotation_gate)
        self.last_flow_rotation_phase = rotation_gate.phase
        self.last_flow_rotation_weight = rotation_gate.weight
        self.last_flow_yaw_rate_abs_radps = rotation_gate.yaw_rate_abs_radps
        if quality_or_distance_invalid:
            decision["factor_enabled"] = False
            decision["reliability_weight"] = 0.0
            decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
            self.counts["flow_disabled_quality"] += 1
            self.last_flow_reason = "quality_or_distance_gate"
        elif rotation_gate.hard_disabled:
            self.counts["flow_disabled_rotation"] += 1
            self.last_flow_reason = rotation_gate.reason
        elif rotation_gate.phase != "ACTIVE":
            self.last_flow_reason = rotation_gate.reason
        else:
            self.last_flow_reason = "accepted"
        if self.optical_flow_yaw_coupling_enabled:
            self.backend.add_optical_flow_body(
                previous_index, current_index, observation["delta_body"],
                previous_yaw,
                covariance=[0.10 ** 2, 0.10 ** 2, 1.0], decision=decision,
            )
            self.last_flow_factor_type = "body_yaw_linearized"
        else:
            self.backend.add_optical_flow(
                previous_index, current_index, observation["delta_position"],
                covariance=[0.10 ** 2, 0.10 ** 2, 1.0], decision=decision,
            )
            self.last_flow_factor_type = "map_translation"
        self.counts["flow_factor_attempts"] += 1
        if (
            bool(decision.get("factor_enabled", True))
            and float(decision.get("reliability_weight", 1.0)) > 0.0
        ):
            self.counts["flow_factors"] += 1

    def _native_lidar(self, msg):
        self.counts["native_lidar_received"] += 1
        try:
            factor = native_factor_from_message(msg)
        except (ValueError, TypeError) as error:
            self.counts["native_lidar_invalid"] += 1
            self.last_reason = f"invalid_native_lidar:{type(error).__name__}"
            self.last_exception = f"{type(error).__name__}:{error}"
            return
        try:
            validate_native_frame_contract(factor, self.map_frame, self.body_frame)
        except ValueError as error:
            self.counts["native_lidar_invalid"] += 1
            self.last_reason = "invalid_native_lidar_frame_contract"
            self.last_exception = f"{type(error).__name__}:{error}"
            return
        if self.input_trigger_mode == "native_factor":
            self._dispatch_native_frame(msg.header, factor)
            return
        self.native_lidar_buffer.push(factor)
        self._drain_pending_inputs()

    def _dispatch_native_frame(self, header, factor):
        stamp_ns = int(factor.stamp_ns)
        sequence = int(factor.scan_sequence)
        status, gap = native_trigger_order_status(
            self.last_native_input_stamp_ns,
            self.last_native_input_sequence,
            stamp_ns,
            sequence,
        )
        if status == "duplicate":
            self.counts["native_trigger_duplicates"] += 1
            return
        if status in {"sequence_conflict", "sequence_reset"}:
            self.counts["native_trigger_sequence_conflicts"] += 1
            self.last_reason = f"native_trigger_{status}"
            return
        if status == "nonmonotonic":
            self.counts["native_trigger_nonmonotonic"] += 1
            self.last_reason = "nonmonotonic_native_trigger"
            return
        self.counts["native_trigger_sequence_gaps"] += gap
        self.last_native_input_stamp_ns = stamp_ns
        self.last_native_input_sequence = sequence
        discarded = enqueue_latest(
            self.native_work_queue, (copy.deepcopy(header), factor)
        )
        if discarded < 0:
            self.counts["native_worker_queue_overflow"] += 1
            self.last_reason = "native_worker_queue_enqueue_failed"
        elif discarded:
            self.counts["native_worker_queue_overflow"] += 1
            self.counts["native_worker_queue_discarded"] += discarded

    def _native_worker_loop(self):
        while not self.native_worker_stop.is_set():
            try:
                header, factor = self.native_work_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self._process_native_worker_frame(header, factor)
            except Exception as error:  # one bad packet must not kill the worker
                self.counts["native_worker_errors"] += 1
                self.last_reason = f"native_worker_error:{type(error).__name__}"
                self.last_exception = f"{type(error).__name__}:{error}"
            finally:
                self.native_work_queue.task_done()

    def _process_native_worker_frame(self, header, factor):
        message = native_frame_odometry(header, factor)
        stamp = factor.stamp_s
        if (
            self.backend_solver_mode == "manifold"
            and self.imu_factor_enabled
            and self.last_lio_stamp is not None
        ):
            deadline = time.monotonic() + self.imu_factor_wait_s
            covered = False
            while not self.native_worker_stop.is_set():
                covered, _, _ = imu_interval_status(
                    self._imu_snapshot(),
                    self.last_lio_stamp,
                    stamp,
                    self.imu_max_gap_s,
                )
                if covered or time.monotonic() >= deadline:
                    break
                self.native_worker_stop.wait(0.002)
            if not covered:
                self.counts["imu_pair_timeouts"] += 1
        if not self.native_worker_stop.is_set():
            self._process_lio(message, factor)

    def stop_native_worker(self):
        self.native_worker_stop.set()
        if self.native_worker_thread is not None:
            self.native_worker_thread.join(timeout=2.0)

    def _lio(self, msg):
        if self.input_trigger_mode == "native_factor":
            self.counts["lio_pose_inputs_ignored"] += 1
            return
        if not self.native_lidar_enabled:
            self._dispatch_lio(msg, None)
            return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self._now_s()
        factor = self.native_lidar_buffer.pop_nearest(
            stamp, self.native_lidar_tolerance_s
        )
        if factor is not None:
            self._dispatch_lio(msg, factor)
            return
        self.pending_lio.append((time.monotonic(), msg))

    def _dispatch_lio(self, msg, native_factor):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self._now_s()
        imu_ready = True
        if self.last_lio_stamp is not None:
            imu_ready, _, _ = imu_interval_status(
                self._imu_snapshot(),
                self.last_lio_stamp,
                stamp,
                self.imu_max_gap_s,
            )
        needs_imu_wait = (
            self.backend_solver_mode == "manifold"
            and self.imu_factor_enabled
            and self.last_lio_stamp is not None
            and not imu_ready
        )
        if needs_imu_wait:
            self.pending_imu_lio.append(
                (time.monotonic(), msg, native_factor)
            )
            return
        self._process_lio(msg, native_factor)

    def _drain_pending_inputs(self):
        self._drain_pending_lio()
        self._drain_pending_imu_lio()

    def _drain_pending_lio(self):
        if not self.pending_lio:
            return
        now = time.monotonic()
        while self.pending_lio:
            arrival, msg = self.pending_lio[0]
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                stamp = self._now_s()
            factor = self.native_lidar_buffer.pop_nearest(
                stamp, self.native_lidar_tolerance_s
            )
            if factor is None and now - arrival < self.native_lidar_wait_s:
                break
            self.pending_lio.popleft()
            if factor is None:
                self.counts["native_lidar_pair_timeouts"] += 1
            self._dispatch_lio(msg, factor)

    def _drain_pending_imu_lio(self):
        if not self.pending_imu_lio:
            return
        now = time.monotonic()
        while self.pending_imu_lio:
            arrival, msg, native_factor = self.pending_imu_lio[0]
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                stamp = self._now_s()
            covered, _, _ = imu_interval_status(
                self._imu_snapshot(),
                self.last_lio_stamp,
                stamp,
                self.imu_max_gap_s,
            )
            timed_out = now - arrival >= self.imu_factor_wait_s
            if not covered and not timed_out:
                break
            self.pending_imu_lio.popleft()
            if not covered:
                self.counts["imu_pair_timeouts"] += 1
            self._process_lio(msg, native_factor)

    def _process_lio(self, msg, native_factor):
        started = time.perf_counter_ns()
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self._now_s()
        if self.last_lio_stamp is not None and stamp <= self.last_lio_stamp:
            self.last_reason = "nonmonotonic_lio_stamp"
            return
        if (
            self.backend.state_count == 0
            and native_factor is not None
            and not native_factor.correspondences_valid
        ):
            self.counts["native_trigger_waiting_for_initial_factor"] += 1
            self.last_reason = "waiting_for_initial_native_lidar_factor"
            return
        pose = msg.pose.pose
        position = np.asarray(
            [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
            dtype=float,
        )
        orientation = quaternion_xyzw_to_rpy([
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ])
        orientation[2] = unwrap_yaw(
            self.last_lio_yaw if self.last_lio_stamp is not None else None,
            float(orientation[2]),
        )
        yaw = float(orientation[2])
        previous_state = (
            self.backend.state(-1) if self.backend.state_count > 0 else None
        )
        manifold_measurement = None
        reference = None
        if previous_state is None:
            initial_state = np.zeros(15, dtype=float)
            initial_state[:3] = position
            initial_state[3:6] = orientation
            if (
                self.backend_solver_mode == "manifold"
                and self.imu_startup_bias_initialization_enabled
            ):
                startup = estimate_stationary_imu_bias(
                    self._imu_snapshot(),
                    orientation,
                    stamp,
                    window_s=self.imu_startup_window_s,
                    minimum_samples=self.imu_startup_minimum_samples,
                    minimum_span_s=self.imu_startup_minimum_span_s,
                    maximum_mean_gyro_radps=(
                        self.imu_startup_maximum_mean_gyro_radps
                    ),
                    maximum_gyro_residual_rms_radps=(
                        self.imu_startup_maximum_gyro_residual_rms_radps
                    ),
                    gravity_tolerance_mps2=(
                        self.imu_startup_gravity_tolerance_mps2
                    ),
                    maximum_accel_residual_rms_mps2=(
                        self.imu_startup_maximum_accel_residual_rms_mps2
                    ),
                )
                self.last_imu_startup_reason = startup.reason
                self.last_imu_startup_sample_count = startup.sample_count
                self.last_imu_startup_span_s = startup.span_s
                if startup.reason == "insufficient_observation_span":
                    self.counts["imu_startup_waits"] += 1
                    self.last_reason = "waiting_for_stationary_imu_window"
                    return
                if startup.valid:
                    initial_state[9:12] = startup.accel_bias
                    initial_state[12:15] = startup.gyro_bias
                    self.last_imu_startup_accel_bias = np.asarray(
                        startup.accel_bias, dtype=float
                    )
                    self.last_imu_startup_gyro_bias = np.asarray(
                        startup.gyro_bias, dtype=float
                    )
                    self.counts["imu_startup_bias_accepted"] += 1
                else:
                    self.counts["imu_startup_bias_rejected"] += 1
            elif self.backend_solver_mode == "manifold":
                self.last_imu_startup_reason = "disabled"
        elif self.backend_solver_mode == "manifold":
            manifold_measurement = self._manifold_imu_measurement(
                self.last_lio_stamp, stamp, previous_state,
            )
            if manifold_measurement is not None:
                initial_state = propagate_state(previous_state, manifold_measurement)
                self.counts["imu_propagated_initializations"] += 1
            else:
                initial_state = previous_state.copy()
                initial_state[:3] += previous_state[6:9] * (
                    stamp - self.last_lio_stamp
                )
            reference = manifold_motion_reference(previous_state, initial_state)
        else:
            initial_state = previous_state.copy()
            initial_state[:3] = position
            initial_state[3:6] = orientation
            reference = fused_motion_reference(
                previous_state, stamp - self.last_lio_stamp,
            )
        current_index = self.backend.add_state(initial_state)
        if reference is not None:
            innovation = lidar_prediction_innovation(position, yaw, reference)
            self.last_lidar_prediction_position_innovation_m = innovation[
                "position_m"
            ]
            self.last_lidar_prediction_yaw_innovation_rad = innovation["yaw_rad"]
        if self.last_lio_stamp is None:
            self.lio_origin = position.copy()
            prior_covariance = np.full(15, 1.0e-4)
            if self.backend_solver_mode == "manifold":
                startup_bias_accepted = (
                    self.counts["imu_startup_bias_accepted"] > 0
                )
                prior_covariance = np.asarray(
                    [1.0e-4] * 6
                    + [1.0] * 3
                    + [0.10 ** 2 if startup_bias_accepted else 0.50 ** 2] * 3
                    + [0.01 ** 2 if startup_bias_accepted else 0.05 ** 2] * 3,
                    dtype=float,
                )
            self.backend.add_prior(
                current_index,
                initial_state.copy(),
                covariance=prior_covariance,
            )
        lidar_decision = self._decision("lidar", default_enabled=True)
        if lidar_decision.get("anchor_override", False):
            self.counts["lidar_anchor_overrides"] += 1
        lidar_factor_added = False
        if native_factor is not None:
            native_factor = with_yaw_reference(native_factor, yaw)
            self.last_native_sequence = int(native_factor.scan_sequence)
            self.last_native_matches = int(native_factor.matched_points)
            self.last_native_stamp_error_ms = abs(
                native_factor.stamp_s - stamp
            ) * 1000.0
        if native_factor is not None and not native_factor.correspondences_valid:
            lidar_decision["factor_enabled"] = False
            lidar_decision["reliability_weight"] = 0.0
            lidar_decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
            self.counts["native_trigger_only_frames"] += 1
            self.last_lidar_source = "native_frame_without_correspondences"
        if native_factor is not None and native_factor.correspondences_valid:
            if native_factor.matched_points < self.native_lidar_minimum_matches:
                lidar_decision["factor_enabled"] = False
                lidar_decision["reliability_weight"] = 0.0
                lidar_decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
                self.counts["native_lidar_hard_disabled"] += 1
            raw_correspondences_available = all(
                value is not None
                for value in (
                    native_factor.lidar_points,
                    native_factor.plane_normals,
                    native_factor.plane_points,
                    native_factor.lidar_to_body_rotation,
                    native_factor.lidar_to_body_translation,
                )
            )
            if self.backend_solver_mode == "manifold" and raw_correspondences_available:
                self.backend.add_native_lidar_correspondences(
                    current_index, native_factor, decision=lidar_decision
                )
                self.counts["native_lidar_relinearized"] += 1
                self.last_lidar_source = "native_point_to_plane_relinearized"
                lidar_factor_added = True
            elif self.backend_solver_mode == "manifold" and all(
                value is not None
                for value in (
                    native_factor.pose_hessian_right,
                    native_factor.pose_gradient_right,
                )
            ):
                self.backend.add_native_lidar_normal(
                    current_index,
                    native_factor.linearization_pose,
                    native_factor.pose_hessian_right,
                    native_factor.pose_gradient_right,
                    native_factor.measurement_variance,
                    native_factor.matched_points,
                    native_factor.residual_squared,
                    decision=lidar_decision,
                )
                self.counts["native_lidar_condensed_fallbacks"] += 1
                self.last_lidar_source = "native_point_to_plane_condensed"
                lidar_factor_added = True
            elif self.backend_solver_mode == "linear":
                self.backend.add_native_lidar_normal(
                    current_index,
                    native_factor.linearization_pose,
                    native_factor.pose_hessian,
                    native_factor.pose_gradient,
                    native_factor.measurement_variance,
                    native_factor.matched_points,
                    native_factor.residual_squared,
                    decision=lidar_decision,
                )
                self.last_lidar_source = "native_point_to_plane_linear"
                lidar_factor_added = True
            else:
                self.last_lidar_source = "native_factor_incompatible"
            if lidar_factor_added:
                self.counts["native_lidar_factors"] += 1
        if native_factor is None and (
            self.backend_solver_mode == "linear" or self.allow_lio_pose_fallback
        ):
            self.backend.add_lidar_pose(
                current_index, position, orientation,
                covariance=[0.05 ** 2] * 3 + [0.03 ** 2] * 3,
                decision=lidar_decision,
            )
            self.counts["native_lidar_pose_fallbacks"] += 1
            self.last_lidar_source = "lio_pose_fallback"
            lidar_factor_added = True
        elif native_factor is None:
            self.counts["lio_pose_inputs_ignored"] += 1
            self.last_lidar_source = "native_factor_missing_lio_pose_ignored"
        if lidar_factor_added:
            self.counts["lidar_factors"] += 1
        if lidar_factor_added and not lidar_decision["factor_enabled"]:
            self.counts["lidar_disabled"] += 1
        imu_diagnostic_covariance = None
        if self.last_lio_stamp is not None:
            previous_index = current_index - 1
            self._gnss_factor(stamp, reference["position"], current_index)
            self._flow_factor(
                self.last_lio_stamp, stamp, reference["yaw"],
                previous_index, current_index, reference["delta_position"],
            )
            if self.backend_solver_mode == "manifold":
                imu_diagnostic_covariance = self._add_manifold_imu_factor(
                    previous_index, current_index, manifold_measurement
                )
            else:
                imu_diagnostic_covariance = self._imu_factor(
                    self.last_lio_stamp, stamp, reference["orientation"],
                    previous_index, current_index,
                )
        try:
            self.backend.optimize()
            solve_ms = float(getattr(self.backend, "last_solve_ms", 0.0))
            self.backend_solve_count += 1
            self.backend_solve_ms_total += solve_ms
            self.backend_solve_ms_max = max(self.backend_solve_ms_max, solve_ms)
            if imu_diagnostic_covariance is not None:
                try:
                    residual = self.backend.latest_factor_residual(
                        "imu_preintegrated", covariance=imu_diagnostic_covariance
                    )
                    if residual is not None:
                        self.last_imu_preintegration_residual_mahalanobis = (
                            residual.mahalanobis_squared
                        )
                        self.counts["imu_residual_updates"] += 1
                        self.last_imu_residual_error = "none"
                    else:
                        self.last_imu_preintegration_residual_mahalanobis = -1.0
                        self.last_imu_residual_error = "factor_not_found"
                except (ValueError, IndexError) as error:
                    self.last_imu_preintegration_residual_mahalanobis = -1.0
                    self.last_imu_residual_error = (
                        f"{type(error).__name__}:{error}"
                    )
                    self.counts["imu_residual_errors"] += 1
            else:
                self.last_imu_preintegration_residual_mahalanobis = -1.0
            estimate = self.backend.state(current_index)
            self._publish(msg.header, estimate)
            self.counts["lio"] += 1
            self.last_reason = "ok"
        except (np.linalg.LinAlgError, ValueError, IndexError) as error:
            self.counts["optimization_errors"] += 1
            self.last_reason = f"optimization_error:{type(error).__name__}"
            self.last_exception = f"{type(error).__name__}:{error}"
        self.last_lio_stamp = stamp
        self.last_lio_position = position
        self.last_lio_yaw = yaw
        self.last_callback_ms = (time.perf_counter_ns() - started) * 1.0e-6

    def _publish(self, header, state):
        orientation = np.asarray(state[3:6], dtype=float)
        qx, qy, qz, qw = rpy_to_quaternion_xyzw(orientation)
        body_velocity = rpy_to_rotation_matrix(orientation).T @ np.asarray(
            state[6:9], dtype=float
        )
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
        output.twist.twist.linear.x = float(body_velocity[0])
        output.twist.twist.linear.y = float(body_velocity[1])
        output.twist.twist.linear.z = float(body_velocity[2])
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

    def _record_imu_invalid(self, reason):
        reason = str(reason)
        self.counts["imu_invalid"] += 1
        self.imu_invalid_reasons[reason] = (
            self.imu_invalid_reasons.get(reason, 0) + 1
        )

    def log_final_summary(self):
        average_solve_ms = (
            self.backend_solve_ms_total / self.backend_solve_count
            if self.backend_solve_count > 0 else 0.0
        )
        ownership = (
            "native_relinearized="
            f"{self.counts['native_lidar_relinearized']};"
            "native_condensed="
            f"{self.counts['native_lidar_condensed_fallbacks']};"
            "lio_pose_fallbacks="
            f"{self.counts['native_lidar_pose_fallbacks']};"
            "lio_pose_ignored="
            f"{self.counts['lio_pose_inputs_ignored']};"
            "imu_propagated="
            f"{self.counts['imu_propagated_initializations']};"
            f"imu_factors={self.counts['imu_factors']};"
            f"imu_pair_timeouts={self.counts['imu_pair_timeouts']}"
        )
        invalid_reasons = ",".join(
            f"{name}:{count}"
            for name, count in sorted(self.imu_invalid_reasons.items())
        ) or "none"
        print(
            "Unified backend final summary: "
            f"solver={self.backend_solver_mode};"
            f"input_trigger={self.input_trigger_mode};"
            f"published={self.counts['published']};"
            f"optimization_errors={self.counts['optimization_errors']};"
            f"solve_mean_ms={average_solve_ms:.3f};"
            f"solve_max_ms={self.backend_solve_ms_max:.3f};{ownership};"
            f"imu_invalid={self.counts['imu_invalid']};"
            f"imu_invalid_reasons={invalid_reasons};"
            f"imu_received={self.counts['imu_received']};"
            f"imu_startup_reason={self.last_imu_startup_reason};"
            "imu_startup_samples="
            f"{self.last_imu_startup_sample_count};"
            f"imu_startup_span_s={self.last_imu_startup_span_s:.6f};"
            "imu_startup_accel_bias="
            f"{','.join(f'{value:.9g}' for value in self.last_imu_startup_accel_bias)};"
            "imu_startup_gyro_bias="
            f"{','.join(f'{value:.9g}' for value in self.last_imu_startup_gyro_bias)};"
            "imu_startup_bias_accepted="
            f"{self.counts['imu_startup_bias_accepted']};"
            "imu_startup_bias_rejected="
            f"{self.counts['imu_startup_bias_rejected']};"
            "imu_nonmonotonic_arrivals="
            f"{self.counts['imu_nonmonotonic_arrivals']};"
            "imu_max_positive_arrival_gap_s="
            f"{self.imu_max_positive_arrival_gap_s:.6f};"
            f"native_pair_timeouts={self.counts['native_lidar_pair_timeouts']};"
            f"native_received={self.counts['native_lidar_received']};"
            f"native_queue_overflow={self.counts['native_worker_queue_overflow']};"
            "native_queue_discarded="
            f"{self.counts['native_worker_queue_discarded']}",
            flush=True,
        )

    def _diagnostics(self):
        average_solve_ms = (
            self.backend_solve_ms_total / self.backend_solve_count
            if self.backend_solve_count else 0.0
        )
        diagnostic = DiagnosticStatus()
        diagnostic.name = "unified_backend_fusion"
        diagnostic.hardware_id = "companion_computer"
        healthy = self.last_reason == "ok" and self.counts["optimization_errors"] == 0
        diagnostic.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        diagnostic.message = self.last_reason
        diagnostic.values = [
            self._key("scheduler_health", self.scheduler_health),
            self._key("reliability_mode", self.reliability_mode),
            self._key("backend_solver_mode", self.backend_solver_mode),
            self._key("window_states", self.backend.state_count),
            self._key("window_factors", self.backend.factor_count),
            self._key(
                "nonlinear_iterations",
                getattr(self.backend, "last_iterations", 1),
            ),
            self._key(
                "lm_rejected_steps",
                getattr(self.backend, "last_rejected_steps", 0),
            ),
            self._key(
                "lm_damping",
                f"{getattr(self.backend, 'last_damping', 0.0):.9g}",
            ),
            self._key(
                "backend_solve_ms",
                f"{getattr(self.backend, 'last_solve_ms', 0.0):.3f}",
            ),
            self._key(
                "backend_solve_mean_ms",
                f"{average_solve_ms:.3f}",
            ),
            self._key(
                "backend_solve_max_ms", f"{self.backend_solve_ms_max:.3f}",
            ),
            self._key(
                "backend_cost",
                f"{getattr(self.backend, 'last_cost', 0.0):.9g}",
            ),
            self._key("callback_ms", f"{self.last_callback_ms:.3f}"),
            self._key("last_imu_reason", self.last_imu_reason),
            self._key("imu_startup_reason", self.last_imu_startup_reason),
            self._key(
                "imu_startup_sample_count", self.last_imu_startup_sample_count
            ),
            self._key(
                "imu_startup_span_s", f"{self.last_imu_startup_span_s:.9g}"
            ),
            self._key(
                "imu_startup_accel_bias",
                ",".join(
                    f"{value:.9g}" for value in self.last_imu_startup_accel_bias
                ),
            ),
            self._key(
                "imu_startup_gyro_bias",
                ",".join(
                    f"{value:.9g}" for value in self.last_imu_startup_gyro_bias
                ),
            ),
            self._key(
                "imu_max_positive_arrival_gap_s",
                f"{self.imu_max_positive_arrival_gap_s:.9g}",
            ),
            self._key("last_imu_residual_error", self.last_imu_residual_error),
            self._key("last_exception", self.last_exception),
            self._key(
                "imu_preintegration_residual_mahalanobis",
                f"{self.last_imu_preintegration_residual_mahalanobis:.9g}",
            ),
            self._key(
                "lidar_prediction_position_innovation_m",
                f"{self.last_lidar_prediction_position_innovation_m:.9g}",
            ),
            self._key(
                "lidar_prediction_yaw_innovation_rad",
                f"{self.last_lidar_prediction_yaw_innovation_rad:.9g}",
            ),
            self._key("lidar_factor_source", self.last_lidar_source),
            self._key("native_lidar_sequence", self.last_native_sequence),
            self._key("native_lidar_matches", self.last_native_matches),
            self._key(
                "native_lidar_stamp_error_ms",
                f"{self.last_native_stamp_error_ms:.9g}",
            ),
            self._key("pending_lio_messages", len(self.pending_lio)),
            self._key("pending_imu_lio_messages", len(self.pending_imu_lio)),
            self._key("pending_native_worker_frames", self.native_work_queue.qsize()),
            self._key("last_flow_reason", self.last_flow_reason),
            self._key("last_flow_factor_type", self.last_flow_factor_type),
            self._key("flow_rotation_phase", self.last_flow_rotation_phase),
            self._key(
                "flow_rotation_weight", f"{self.last_flow_rotation_weight:.9g}"
            ),
            self._key(
                "flow_yaw_rate_abs_radps",
                f"{self.last_flow_yaw_rate_abs_radps:.9g}",
            ),
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
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.stop_native_worker()
        node.log_final_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
