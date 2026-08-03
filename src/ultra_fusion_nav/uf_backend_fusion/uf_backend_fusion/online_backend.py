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
from builtin_interfaces.msg import Time as RosTime
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
from uf_interfaces.msg import ReliabilityScore, RelocalizationResult, SchedulerState

from .imu_preintegration import (
    ImuSample,
    _quat_to_rotvec,
    preintegrate,
    preintegrate_manifold,
)
from .manifold_window import ManifoldSlidingWindowBackend, propagate_state
from .scan_prediction import (
    ScanPrediction,
    build_scan_prediction,
    consume_cached_prediction,
    prediction_reusable,
    scan_request_ready,
    scan_request_stale,
)
from .spatiotemporal_calibration import (
    OnlineSpatiotemporalCalibrator,
    effective_time_offset,
)
from .native_lidar import (
    NativeFactorBuffer,
    lidar_pose_observability,
    native_factor_from_message,
    quaternion_xyzw_to_rpy,
    matrix_to_pose_vector,
    pose_vector_to_matrix,
    right_perturbation_jacobian_rpy,
    rpy_to_quaternion_xyzw,
    rpy_to_rotation_matrix,
    transform_native_factor_map,
    validate_native_frame_contract,
    with_yaw_reference,
)
from .window import SlidingWindowBackend
from uf_reliability.scoring import (
    gnss_score,
    optical_flow_displacement_frd,
    optical_flow_los_prediction_flu,
    optical_flow_los_rate_apm,
    optical_flow_score,
)
from uf_reliability.flow_rotation_gate import (
    FlowRotationGateConfig,
    OpticalFlowRotationGate,
    interval_mean_absolute_yaw_rate,
    interval_mean_vector,
)

try:
    from fast_lio.msg import (
        BackendDeskewTrajectory,
        BackendStateSeed,
        FrontendScanRequest,
        NativeLidarFactor,
    )
except ImportError:  # pragma: no cover - unit tests run without the external overlay
    BackendStateSeed = None
    BackendDeskewTrajectory = None
    FrontendScanRequest = None
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


@dataclass(frozen=True)
class OptimizationIntegrity:
    valid: bool
    reason: str
    translation_correction_m: float
    rotation_correction_rad: float
    velocity_correction_mps: float
    accel_bias_correction_mps2: float
    gyro_bias_correction_radps: float
    initial_cost: float
    final_cost: float
    latest_information_rank: int
    latest_information_condition: float


def validate_optimized_state(
    initial_state,
    estimate,
    latest_information,
    initial_cost,
    final_cost,
    *,
    maximum_translation_correction_m,
    maximum_rotation_correction_rad,
    maximum_velocity_correction_mps,
    maximum_accel_bias_correction_mps2,
    maximum_gyro_bias_correction_radps,
    maximum_information_condition,
    information_rank_tolerance,
):
    """Gate one optimized keyframe before state publication or map insertion."""
    initial = np.asarray(initial_state, dtype=float)
    optimized = np.asarray(estimate, dtype=float)
    information = np.asarray(latest_information, dtype=float)
    if (
        initial.shape != (15,)
        or optimized.shape != (15,)
        or information.shape != (15, 15)
        or np.any(~np.isfinite(initial))
        or np.any(~np.isfinite(optimized))
        or np.any(~np.isfinite(information))
        or not math.isfinite(float(initial_cost))
        or not math.isfinite(float(final_cost))
        or float(initial_cost) < 0.0
        or float(final_cost) < 0.0
    ):
        return OptimizationIntegrity(
            False, "nonfinite_optimization_result",
            math.inf, math.inf, math.inf, math.inf, math.inf,
            float(initial_cost), float(final_cost), 0, math.inf,
        )

    relative_pose = np.linalg.inv(pose_vector_to_matrix(initial[:6])) @ (
        pose_vector_to_matrix(optimized[:6])
    )
    relative = matrix_to_pose_vector(relative_pose)
    translation = float(np.linalg.norm(optimized[:3] - initial[:3]))
    rotation = float(np.linalg.norm(relative[3:6]))
    velocity = float(np.linalg.norm(optimized[6:9] - initial[6:9]))
    accel_bias = float(np.linalg.norm(optimized[9:12] - initial[9:12]))
    gyro_bias = float(np.linalg.norm(optimized[12:15] - initial[12:15]))

    symmetric_information = 0.5 * (information + information.T)
    eigenvalues = np.linalg.eigvalsh(symmetric_information)
    scale = max(1.0, float(np.max(np.abs(eigenvalues))))
    if float(np.min(eigenvalues)) < -information_rank_tolerance * scale:
        rank = 0
        condition = math.inf
        reason = "indefinite_latest_information"
    else:
        active = eigenvalues > information_rank_tolerance * scale
        rank = int(np.count_nonzero(active))
        condition = (
            float(eigenvalues[-1] / eigenvalues[active][0])
            if rank > 0 else math.inf
        )
        reason = "ok"

    limits = (
        (translation, maximum_translation_correction_m, "translation_correction"),
        (rotation, maximum_rotation_correction_rad, "rotation_correction"),
        (velocity, maximum_velocity_correction_mps, "velocity_correction"),
        (accel_bias, maximum_accel_bias_correction_mps2, "accel_bias_correction"),
        (gyro_bias, maximum_gyro_bias_correction_radps, "gyro_bias_correction"),
    )
    cost_tolerance = 1.0e-10 * max(1.0, abs(float(initial_cost)))
    if reason == "ok" and float(final_cost) > float(initial_cost) + cost_tolerance:
        reason = "optimization_cost_increased"
    if reason == "ok":
        for value, limit, name in limits:
            if value > limit:
                reason = f"excessive_{name}"
                break
    if reason == "ok" and (
        rank < 1 or condition > maximum_information_condition
    ):
        reason = "ill_conditioned_latest_information"

    return OptimizationIntegrity(
        reason == "ok", reason, translation, rotation, velocity,
        accel_bias, gyro_bias, float(initial_cost), float(final_cost),
        rank, condition,
    )


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def ros_time_from_seconds(stamp_s):
    stamp_s = float(stamp_s)
    if not math.isfinite(stamp_s) or stamp_s < 0.0:
        raise ValueError("ROS timestamp must be finite and non-negative")
    seconds = int(math.floor(stamp_s))
    nanoseconds = int(round((stamp_s - seconds) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return RosTime(sec=seconds, nanosec=nanoseconds)


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


def select_gnss_observation(
    observations, state_stamp_s, maximum_age_s, future_tolerance_s,
):
    """Consume at most one GNSS fix for a state and never reuse a fix.

    The newest eligible fix is paired with the state. Older eligible fixes are
    superseded, while future fixes remain queued for a later state.
    """
    state_stamp_s = float(state_stamp_s)
    maximum_age_s = float(maximum_age_s)
    future_tolerance_s = float(future_tolerance_s)
    if (
        not math.isfinite(state_stamp_s)
        or not math.isfinite(maximum_age_s)
        or maximum_age_s <= 0.0
        or not math.isfinite(future_tolerance_s)
        or future_tolerance_s < 0.0
    ):
        raise ValueError("invalid GNSS observation selection limits")

    ordered = sorted(observations, key=lambda item: float(item["stamp_s"]))
    observations.clear()
    eligible = []
    stale_count = 0
    for observation in ordered:
        observation_stamp = float(observation["stamp_s"])
        age_s = state_stamp_s - observation_stamp
        if age_s > maximum_age_s:
            stale_count += 1
        elif observation_stamp <= state_stamp_s + future_tolerance_s:
            eligible.append(observation)
        else:
            observations.append(observation)
    if not eligible:
        return None, stale_count, 0
    return eligible[-1], stale_count, len(eligible) - 1


def gnss_covariance_diagonal(
    covariance, covariance_type, default_variance, minimum_variance=0.04,
):
    values = np.asarray(covariance, dtype=float)
    if values.size < 9:
        raise ValueError("GNSS covariance must contain nine entries")
    default_variance = float(default_variance)
    minimum_variance = float(minimum_variance)
    if (
        not math.isfinite(default_variance)
        or default_variance <= 0.0
        or not math.isfinite(minimum_variance)
        or minimum_variance <= 0.0
    ):
        raise ValueError("GNSS covariance limits must be positive")
    diagonal = values[[0, 4, 8]]
    if (
        int(covariance_type) == NavSatFix.COVARIANCE_TYPE_UNKNOWN
        or np.any(~np.isfinite(diagonal))
        or np.any(diagonal <= 0.0)
    ):
        return np.full(3, default_variance, dtype=float)
    return np.maximum(diagonal, minimum_variance)


def covariance_update_due(last_stamp_s, current_stamp_s, update_period_s):
    current_stamp_s = float(current_stamp_s)
    update_period_s = float(update_period_s)
    if not math.isfinite(current_stamp_s) or not math.isfinite(update_period_s):
        raise ValueError("covariance update timing must be finite")
    if update_period_s <= 0.0:
        raise ValueError("covariance update period must be positive")
    if last_stamp_s is None:
        return True
    last_stamp_s = float(last_stamp_s)
    return current_stamp_s < last_stamp_s or (
        current_stamp_s - last_stamp_s >= update_period_s
    )


def path_sample_due(
    previous_position, previous_orientation, position, orientation,
    minimum_translation_m, minimum_rotation_rad,
):
    if previous_position is None or previous_orientation is None:
        return True
    previous_position = np.asarray(previous_position, dtype=float)
    previous_orientation = np.asarray(previous_orientation, dtype=float)
    position = np.asarray(position, dtype=float)
    orientation = np.asarray(orientation, dtype=float)
    if any(value.shape != (3,) for value in (
        previous_position, previous_orientation, position, orientation,
    )):
        raise ValueError("path sampling expects finite 3-vectors")
    if any(np.any(~np.isfinite(value)) for value in (
        previous_position, previous_orientation, position, orientation,
    )):
        raise ValueError("path sampling expects finite values")
    translation = float(np.linalg.norm(position - previous_position))
    rotation_delta = np.asarray([
        math.atan2(math.sin(delta), math.cos(delta))
        for delta in orientation - previous_orientation
    ])
    return bool(
        translation >= float(minimum_translation_m)
        or np.linalg.norm(rotation_delta) >= float(minimum_rotation_rad)
    )


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


def flow_los_observation(flow_records):
    """Aggregate APM-compensated flow LOS rates using exposure duration."""
    weighted_rates = []
    total_duration_s = 0.0
    weighted_distance = 0.0
    for flow in flow_records:
        try:
            integration_s = float(flow.get("integration_time_s", 0.0))
            distance_m = float(flow["distance_m"])
        except (KeyError, TypeError, ValueError):
            continue
        if (
            not math.isfinite(integration_s)
            or integration_s <= 0.0
            or not math.isfinite(distance_m)
            or distance_m <= 0.0
        ):
            continue
        rate = optical_flow_los_rate_apm(
            flow["integrated_x"], flow["integrated_y"],
            flow["integrated_xgyro"], flow["integrated_ygyro"],
            integration_s,
        )
        if rate is None:
            continue
        weighted_rates.append((rate, integration_s))
        total_duration_s += integration_s
        weighted_distance += distance_m * integration_s
    if not weighted_rates or total_duration_s <= 0.0:
        return None
    measurement = tuple(
        sum(rate[component] * duration for rate, duration in weighted_rates)
        / total_duration_s
        for component in range(2)
    )
    return {
        "measurement_radps": measurement,
        "distance_m": weighted_distance / total_duration_s,
        "integration_s": total_duration_s,
        "sample_count": len(weighted_rates),
    }


def _flow_record_is_future(item, current_stamp):
    try:
        stamp = float(item["stamp_s"])
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(stamp) and stamp > float(current_stamp)


def select_flow_records(flow_records, previous_stamp, current_stamp, max_age_s):
    """Select one flow interval without consuming future samples.

    The normal association interval is ``(previous_stamp, current_stamp]``.
    If it is empty, a recent late sample is used once as a bounded fallback.
    All samples at or before ``current_stamp`` are consumed so a late packet
    cannot be applied repeatedly to later intervals.
    """
    previous_stamp = float(previous_stamp)
    current_stamp = float(current_stamp)
    max_age_s = float(max_age_s)
    if (
        not math.isfinite(previous_stamp)
        or not math.isfinite(current_stamp)
        or not math.isfinite(max_age_s)
        or current_stamp <= previous_stamp
        or max_age_s < 0.0
    ):
        raise ValueError("invalid flow interval or age limit")

    records = list(flow_records)
    strict = []
    recent = []
    cutoff = current_stamp - max_age_s
    for item in records:
        try:
            stamp = float(item["stamp_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(stamp):
            continue
        if previous_stamp < stamp <= current_stamp:
            strict.append(item)
        elif cutoff < stamp <= current_stamp:
            recent.append(item)

    if strict:
        return strict, [item for item in records if _flow_record_is_future(item, current_stamp)], False
    if recent:
        return recent, [item for item in records if _flow_record_is_future(item, current_stamp)], True
    return [], [item for item in records if _flow_record_is_future(item, current_stamp)], False


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
            "relocalization_result_topic": "/relocalization/result",
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
        self.declare_parameter("gnss_future_tolerance_s", 0.05)
        self.declare_parameter("flow_max_age_s", 1.0)
        self.declare_parameter("maximum_sensor_clock_skew_s", 5.0)
        self.declare_parameter("imu_buffer_s", 5.0)
        self.declare_parameter("imu_factor_wait_s", 0.080)
        self.declare_parameter("imu_nominal_gap_s", 0.10)
        self.declare_parameter("imu_max_gap_s", 0.30)
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
        self.declare_parameter("optical_flow_yaw_coupling_enabled", True)
        self.declare_parameter("flow_rotation_lower_yaw_rate_radps", 0.08)
        self.declare_parameter("flow_rotation_upper_yaw_rate_radps", 0.30)
        self.declare_parameter("flow_rotation_recovery_dwell_s", 0.8)
        self.declare_parameter("flow_rotation_recovery_ramp_s", 1.5)
        self.declare_parameter("flow_rotation_minimum_translation_m", 0.01)
        self.declare_parameter("flow_rotation_recovery_max_base_score", 0.55)
        self.declare_parameter("flow_rotation_imu_max_gap_s", 0.12)
        self.declare_parameter("flow_rotation_allow_compensated", True)
        self.declare_parameter("flow_los_diagnostics_enabled", True)
        # ROS FLU body coordinates. Replace this simulation mount with the
        # measured optical-flow-to-IMU lever arm on the aircraft.
        self.declare_parameter("flow_sensor_offset_body_m", [0.0, 0.0, -0.35])
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
        self.declare_parameter("imu_reintegration_accel_bias_threshold", 0.05)
        self.declare_parameter("imu_reintegration_gyro_bias_threshold", 0.005)
        self.declare_parameter("marginal_rank_tolerance", 1.0e-9)
        self.declare_parameter("marginal_covariance_update_period_s", 1.0)
        self.declare_parameter("online_calibration_enabled", True)
        self.declare_parameter("calibration_window_s", 5.0)
        self.declare_parameter("calibration_minimum_pairs", 8)
        self.declare_parameter("calibration_time_offset_range_s", 0.10)
        self.declare_parameter("calibration_time_offset_step_s", 0.005)
        self.declare_parameter("calibration_minimum_correlation", 0.70)
        self.declare_parameter("calibration_minimum_correlation_margin", 0.05)
        self.declare_parameter("calibration_minimum_excitation_eigenvalue", 1.0e-4)
        self.declare_parameter("calibration_maximum_rotation_residual_rad", 0.08)
        self.declare_parameter("calibration_sharp_turn_rate_radps", 1.5)
        self.declare_parameter("calibration_solve_period_s", 1.0)
        self.declare_parameter("scheduler_timeout_s", 1.0)
        self.declare_parameter("reliability_mode", "dynamic")
        self.declare_parameter("fixed_lidar_weight", 1.0)
        self.declare_parameter("fixed_gnss_weight", 1.0)
        self.declare_parameter("fixed_imu_weight", 1.0)
        self.declare_parameter("fixed_optical_flow_weight", 1.0)
        self.declare_parameter("fixed_covariance_inflation", 1.0)
        self.declare_parameter("publish_path_length", 2000)
        self.declare_parameter("path_publish_period_s", 0.5)
        self.declare_parameter("path_minimum_translation_m", 0.05)
        self.declare_parameter("path_minimum_rotation_rad", 0.02)
        self.declare_parameter("relocalization_enabled", True)
        self.declare_parameter("transactional_update_enabled", True)
        self.declare_parameter("optimization_max_translation_correction_m", 1.0)
        self.declare_parameter("optimization_max_rotation_correction_rad", 0.50)
        self.declare_parameter("optimization_max_velocity_correction_mps", 5.0)
        self.declare_parameter("optimization_max_accel_bias_correction_mps2", 1.5)
        self.declare_parameter("optimization_max_gyro_bias_correction_radps", 0.30)
        self.declare_parameter("optimization_max_information_condition", 1.0e12)
        self.declare_parameter("frontend_state_seed_enabled", False)
        self.declare_parameter(
            "frontend_state_seed_topic", "/fusion/unified/frontend_state_seed"
        )
        self.declare_parameter("frontend_scan_prediction_enabled", False)
        self.declare_parameter(
            "frontend_scan_request_topic", "/fast_lio/frontend_scan_request"
        )
        self.declare_parameter(
            "backend_deskew_trajectory_topic",
            "/fusion/unified/backend_deskew_trajectory",
        )
        self.declare_parameter("scan_prediction_maximum_begin_gap_s", 0.02)
        self.declare_parameter("scan_prediction_timestamp_tolerance_s", 0.002)
        self.declare_parameter("scan_prediction_state_tolerance", 1.0e-8)
        self.declare_parameter("scan_prediction_cache_size", 8)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.gnss_max_age_s = float(self.get_parameter("gnss_max_age_s").value)
        self.gnss_future_tolerance_s = float(
            self.get_parameter("gnss_future_tolerance_s").value
        )
        self.maximum_sensor_clock_skew_s = max(
            0.0,
            float(self.get_parameter("maximum_sensor_clock_skew_s").value),
        )
        if self.gnss_max_age_s <= 0.0 or self.gnss_future_tolerance_s < 0.0:
            raise ValueError("GNSS timing limits are invalid")
        self.flow_max_age_s = float(self.get_parameter("flow_max_age_s").value)
        self.imu_buffer_s = float(self.get_parameter("imu_buffer_s").value)
        self.imu_factor_wait_s = float(
            self.get_parameter("imu_factor_wait_s").value
        )
        self.imu_nominal_gap_s = float(
            self.get_parameter("imu_nominal_gap_s").value
        )
        self.imu_max_gap_s = float(self.get_parameter("imu_max_gap_s").value)
        if (
            self.imu_factor_wait_s < 0.0
            or self.imu_nominal_gap_s <= 0.0
            or self.imu_max_gap_s < self.imu_nominal_gap_s
        ):
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
        self.flow_rotation_allow_compensated = bool(
            self.get_parameter("flow_rotation_allow_compensated").value)
        self.flow_rotation_recovery_max_base_score = float(
            self.get_parameter("flow_rotation_recovery_max_base_score").value)
        self.flow_rotation_imu_max_gap_s = float(
            self.get_parameter("flow_rotation_imu_max_gap_s").value)
        self.flow_los_diagnostics_enabled = bool(
            self.get_parameter("flow_los_diagnostics_enabled").value)
        flow_sensor_offset = tuple(
            float(value) for value in self.get_parameter(
                "flow_sensor_offset_body_m").value
        )
        if len(flow_sensor_offset) != 3 or not all(
            math.isfinite(value) for value in flow_sensor_offset
        ):
            raise ValueError("flow_sensor_offset_body_m must be a finite 3-vector")
        self.flow_sensor_offset_body_m = np.asarray(
            flow_sensor_offset, dtype=float
        )
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
        self.imu_reintegration_accel_bias_threshold = float(
            self.get_parameter("imu_reintegration_accel_bias_threshold").value)
        self.imu_reintegration_gyro_bias_threshold = float(
            self.get_parameter("imu_reintegration_gyro_bias_threshold").value)
        self.marginal_rank_tolerance = float(
            self.get_parameter("marginal_rank_tolerance").value)
        self.marginal_covariance_update_period_s = float(
            self.get_parameter("marginal_covariance_update_period_s").value
        )
        if (
            not math.isfinite(self.imu_reintegration_accel_bias_threshold)
            or self.imu_reintegration_accel_bias_threshold <= 0.0
            or not math.isfinite(self.imu_reintegration_gyro_bias_threshold)
            or self.imu_reintegration_gyro_bias_threshold <= 0.0
            or not math.isfinite(self.marginal_rank_tolerance)
            or self.marginal_rank_tolerance <= 0.0
            or not math.isfinite(self.marginal_covariance_update_period_s)
            or self.marginal_covariance_update_period_s <= 0.0
        ):
            raise ValueError("IMU reintegration and marginalization limits are invalid")
        self.online_calibration_enabled = bool(
            self.get_parameter("online_calibration_enabled").value
        )
        calibration_kwargs = {
            "window_s": float(self.get_parameter("calibration_window_s").value),
            "minimum_pairs": int(self.get_parameter("calibration_minimum_pairs").value),
            "time_offset_range_s": float(
                self.get_parameter("calibration_time_offset_range_s").value
            ),
            "time_offset_step_s": float(
                self.get_parameter("calibration_time_offset_step_s").value
            ),
            "minimum_correlation": float(
                self.get_parameter("calibration_minimum_correlation").value
            ),
            "minimum_correlation_margin": float(
                self.get_parameter("calibration_minimum_correlation_margin").value
            ),
            "minimum_excitation_eigenvalue": float(
                self.get_parameter("calibration_minimum_excitation_eigenvalue").value
            ),
            "maximum_rotation_residual_rad": float(
                self.get_parameter("calibration_maximum_rotation_residual_rad").value
            ),
            "sharp_turn_rate_radps": float(
                self.get_parameter("calibration_sharp_turn_rate_radps").value
            ),
            "solve_period_s": float(
                self.get_parameter("calibration_solve_period_s").value
            ),
        }
        self.calibrator = OnlineSpatiotemporalCalibrator(**calibration_kwargs)
        self.last_calibration_update = self.calibrator.last_update
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
        self.path_publish_period_s = float(
            self.get_parameter("path_publish_period_s").value
        )
        self.path_minimum_translation_m = float(
            self.get_parameter("path_minimum_translation_m").value
        )
        self.path_minimum_rotation_rad = float(
            self.get_parameter("path_minimum_rotation_rad").value
        )
        if (
            self.path_publish_period_s <= 0.0
            or self.path_minimum_translation_m < 0.0
            or self.path_minimum_rotation_rad < 0.0
        ):
            raise ValueError("path publication limits are invalid")
        self.relocalization_enabled = bool(
            self.get_parameter("relocalization_enabled").value
        )
        self.backend_solver_mode = str(
            self.get_parameter("backend_solver_mode").value
        ).lower()
        if self.backend_solver_mode not in {"manifold", "linear"}:
            raise ValueError("backend_solver_mode must be manifold or linear")
        self.allow_lio_pose_fallback = bool(
            self.get_parameter("allow_lio_pose_fallback").value
        )
        self.transactional_update_enabled = bool(
            self.get_parameter("transactional_update_enabled").value
        )
        self.optimization_integrity_limits = {
            "maximum_translation_correction_m": float(self.get_parameter(
                "optimization_max_translation_correction_m").value),
            "maximum_rotation_correction_rad": float(self.get_parameter(
                "optimization_max_rotation_correction_rad").value),
            "maximum_velocity_correction_mps": float(self.get_parameter(
                "optimization_max_velocity_correction_mps").value),
            "maximum_accel_bias_correction_mps2": float(self.get_parameter(
                "optimization_max_accel_bias_correction_mps2").value),
            "maximum_gyro_bias_correction_radps": float(self.get_parameter(
                "optimization_max_gyro_bias_correction_radps").value),
            "maximum_information_condition": float(self.get_parameter(
                "optimization_max_information_condition").value),
            "information_rank_tolerance": self.marginal_rank_tolerance,
        }
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in self.optimization_integrity_limits.values()
        ):
            raise ValueError("optimization integrity limits must be positive")
        if self.transactional_update_enabled and self.backend_solver_mode != "manifold":
            raise ValueError("transactional updates require the manifold backend")
        self.frontend_state_seed_enabled = bool(
            self.get_parameter("frontend_state_seed_enabled").value
        )
        if self.frontend_state_seed_enabled and BackendStateSeed is None:
            raise ValueError(
                "frontend state seed requires the patched FAST-LIO message overlay"
            )
        self.frontend_scan_prediction_enabled = bool(
            self.get_parameter("frontend_scan_prediction_enabled").value
        )
        self.scan_prediction_maximum_begin_gap_s = float(
            self.get_parameter("scan_prediction_maximum_begin_gap_s").value
        )
        self.scan_prediction_timestamp_tolerance_s = float(
            self.get_parameter("scan_prediction_timestamp_tolerance_s").value
        )
        self.scan_prediction_state_tolerance = float(
            self.get_parameter("scan_prediction_state_tolerance").value
        )
        self.scan_prediction_cache_size = int(
            self.get_parameter("scan_prediction_cache_size").value
        )
        if self.frontend_scan_prediction_enabled:
            if self.backend_solver_mode != "manifold":
                raise ValueError("front-end scan prediction requires manifold backend")
            if not self.native_lidar_enabled or self.input_trigger_mode != "native_factor":
                raise ValueError(
                    "front-end scan prediction requires native-factor triggering"
                )
            if FrontendScanRequest is None or BackendDeskewTrajectory is None:
                raise ValueError(
                    "front-end scan prediction requires patched FAST-LIO messages"
                )
            if self.input_trigger_mode != "native_factor":
                raise ValueError(
                    "front-end scan prediction requires native-factor triggering"
                )
        if (
            self.scan_prediction_maximum_begin_gap_s < 0.0
            or self.scan_prediction_timestamp_tolerance_s <= 0.0
            or self.scan_prediction_state_tolerance <= 0.0
            or self.scan_prediction_cache_size < 1
        ):
            raise ValueError("scan prediction limits are invalid")
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
        self.frontend_scan_request_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.backend_trajectory_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=QoSReliabilityPolicy.RELIABLE,
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
                marginal_rank_tolerance=self.marginal_rank_tolerance,
            )
        else:
            self.backend = SlidingWindowBackend(max_states=window_size)
        self.path = Path()
        self.path.poses = []
        self.imu_buffer = deque(maxlen=10000)
        self.flow_buffer = deque(maxlen=3000)
        self.flow_buffer_lock = threading.Lock()
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
                allow_compensated_rotation=self.flow_rotation_allow_compensated,
            )
        )
        self.native_lidar_buffer = NativeFactorBuffer(max_size=128)
        self.relocalization_lock = threading.Lock()
        self.pending_relocalization = None
        self.map_from_lio = np.eye(4, dtype=float)
        self.native_work_queue = queue.Queue(maxsize=self.native_worker_queue_size)
        self.native_worker_stop = threading.Event()
        self.native_worker_thread = None
        self.pending_lio = deque(maxlen=32)
        self.pending_imu_lio = deque(maxlen=64)
        self.gnss_lock = threading.Lock()
        self.gnss_buffer = deque(maxlen=512)
        self.latest_gnss = None
        self.last_gnss_admitted = None
        self.projector = None
        self.lio_origin = None
        self.last_lio_stamp = None
        self.last_lio_position = None
        self.last_lio_yaw = 0.0
        self.last_imu_arrival_stamp = None
        self.imu_max_positive_arrival_gap_s = 0.0
        self.scheduler = {}
        self.scheduler_arrival = None
        self.scheduler_health = "UNAVAILABLE"
        self.scores = {}
        self.counts = {
            "lio": 0, "published": 0, "lidar_factors": 0,
            "lidar_disabled": 0, "gnss_factors": 0, "gnss_jump_rejected": 0,
            "gnss_received": 0, "gnss_consumed": 0,
            "gnss_duplicates": 0, "gnss_out_of_order": 0,
            "gnss_stale_discarded": 0, "gnss_superseded": 0,
            "flow_received": 0,
            "flow_factor_attempts": 0, "flow_factors": 0,
            "flow_clock_mismatch": 0,
            "flow_disabled_quality": 0,
            "flow_disabled_rotation": 0,
            "flow_los_diagnostic_samples": 0,
            "flow_los_diagnostic_invalid": 0,
            "imu_factors": 0, "imu_invalid": 0, "optimization_errors": 0,
            "imu_reintegrations": 0,
            "calibration_updates": 0, "calibration_accepted": 0,
            "calibration_frozen": 0,
            "lidar_anchor_overrides": 0, "imu_residual_updates": 0,
            "imu_residual_errors": 0,
            "native_lidar_received": 0, "native_lidar_invalid": 0,
            "native_lidar_factors": 0, "native_lidar_hard_disabled": 0,
            "native_lidar_pose_fallbacks": 0, "native_lidar_pair_timeouts": 0,
            "native_lidar_relinearized": 0,
            "native_lidar_condensed_fallbacks": 0,
            "native_lidar_directionally_degenerate": 0,
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
            "relocalization_resets": 0,
            "relocalization_rejections": 0,
            "marginal_covariance_updates": 0,
            "marginal_covariance_errors": 0,
            "marginal_covariance_reuses": 0,
            "path_samples": 0,
            "path_messages": 0,
            "optimization_rejected": 0,
            "optimization_rollbacks": 0,
            "frontend_state_seeds": 0,
            "scan_prediction_requests": 0,
            "scan_prediction_published": 0,
            "scan_prediction_rejected": 0,
            "scan_prediction_cache_hits": 0,
            "scan_prediction_cache_misses": 0,
            "scan_prediction_reuse_rejected": 0,
            "scan_prediction_deferred": 0,
            "scan_prediction_deferred_released": 0,
            "scan_prediction_duplicate_requests": 0,
            "scan_prediction_stale_requests": 0,
            "native_consumed_without_state_commit": 0,
        }
        self.imu_invalid_reasons = {}
        self.last_reason = "waiting_for_lio"
        self.last_scan_prediction_reason = "none"
        self.last_covariance_source = "fixed_startup"
        self.last_state_covariance = None
        self.last_covariance_stamp_s = None
        self.last_path_sample_position = None
        self.last_path_sample_orientation = None
        self.last_path_publish_stamp_s = None
        self.last_callback_ms = 0.0
        self.phase_timing = {
            name: {"count": 0, "total_ms": 0.0, "max_ms": 0.0, "last_ms": 0.0}
            for name in (
                "prepare", "pre_state", "snapshot", "add_state", "lidar_factor",
                "aux_factors", "optimize", "reintegrate", "post_optimize", "publish",
            )
        }
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
        self.last_flow_los_diagnostic = None
        self.flow_los_residual_no_lever_norms = deque(maxlen=5000)
        self.flow_los_residual_norms = deque(maxlen=5000)
        self.flow_los_lever_arm_norms = deque(maxlen=5000)
        self.last_lidar_prediction_position_innovation_m = -1.0
        self.last_lidar_prediction_yaw_innovation_rad = -1.0
        self.last_lidar_source = "unavailable"
        self.last_native_sequence = -1
        self.last_native_input_stamp_ns = None
        self.last_native_input_sequence = None
        self.last_native_matches = 0
        self.last_native_stamp_error_ms = -1.0
        self.last_native_effective_rank = 0
        self.last_native_translation_rank = 0
        self.last_native_rotation_rank = 0
        self.last_native_condition_number = math.inf
        self.last_native_characteristic_range_m = 0.0
        self.last_native_normalized_eigenvalues = np.zeros(6, dtype=float)
        self.last_output = None
        self.backend_solve_count = 0
        self.backend_solve_ms_total = 0.0
        self.backend_solve_ms_max = 0.0
        self.last_optimization_integrity = OptimizationIntegrity(
            False, "not_evaluated", 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0, math.inf
        )
        self.frontend_state_seed_sequence = 0
        self.state_reset_counter = 0
        self.scan_prediction_cache = deque(
            maxlen=self.scan_prediction_cache_size
        )
        self.scan_prediction_by_sequence = {}
        self.last_native_consumed_sequence = -1
        self.pending_scan_requests = {}
        self.pending_scan_request_lock = threading.Lock()
        self.scan_prediction_pub = None
        self.deskew_trajectory_pub = None
        self.scheduler_estimator_support = 0.0
        self.active_transaction_snapshot = None

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostic_topic").value), 10)
        self.frontend_state_seed_pub = None
        if self.frontend_state_seed_enabled:
            self.frontend_state_seed_pub = self.create_publisher(
                BackendStateSeed,
                str(self.get_parameter("frontend_state_seed_topic").value),
                self.native_lidar_qos,
            )
        if self.frontend_scan_prediction_enabled:
            self.deskew_trajectory_pub = self.create_publisher(
                BackendDeskewTrajectory,
                str(self.get_parameter("backend_deskew_trajectory_topic").value),
                self.backend_trajectory_qos,
            )
            self.create_subscription(
                FrontendScanRequest,
                str(self.get_parameter("frontend_scan_request_topic").value),
                self._scan_request,
                self.frontend_scan_request_qos,
            )
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
        if self.relocalization_enabled:
            self.create_subscription(
                RelocalizationResult,
                str(self.get_parameter("relocalization_result_topic").value),
                self._relocalization_result, 10)
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

    @staticmethod
    def _age_s(now_s, received_s):
        if received_s is None or received_s > now_s:
            return math.inf
        return now_s - received_s

    def _score(self, modality, msg):
        self.scores[modality] = {
            "weight": float(msg.reliability_weight) if msg.valid else 0.0,
            "valid": bool(msg.valid),
            "received_ros_s": stamp_seconds(msg.header.stamp),
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
        self.scheduler_estimator_support = float(msg.estimator_support)
        self.scheduler_arrival = stamp_seconds(msg.header.stamp)

    @staticmethod
    def _pose_msg_to_matrix(pose):
        orientation = quaternion_xyzw_to_rpy([
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ])
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rpy_to_rotation_matrix(orientation)
        transform[:3, 3] = [
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z),
        ]
        if np.any(~np.isfinite(transform)):
            raise ValueError("relocalization pose is non-finite")
        return transform

    def _relocalization_result(self, msg):
        if int(msg.state) != int(RelocalizationResult.SUCCESS) or not bool(msg.accepted):
            if int(msg.state) == int(RelocalizationResult.FAILED):
                self.counts["relocalization_rejections"] += 1
            return
        if self.backend_solver_mode != "manifold":
            self.counts["relocalization_rejections"] += 1
            self.last_reason = "relocalization_requires_manifold_backend"
            return
        try:
            alignment = self._pose_msg_to_matrix(msg.map_from_lio)
            recovered_pose = self._pose_msg_to_matrix(msg.pose.pose)
            source_pose = self._pose_msg_to_matrix(msg.source_lio_pose)
            reconstructed = alignment @ source_pose
            position_error = float(np.linalg.norm(
                reconstructed[:3, 3] - recovered_pose[:3, 3]
            ))
            rotation_error = float(np.linalg.norm(
                matrix_to_pose_vector(
                    np.linalg.inv(reconstructed) @ recovered_pose
                )[3:6]
            ))
            if position_error > 0.05 or rotation_error > 0.05:
                raise ValueError("relocalization pose and map alignment disagree")
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                raise ValueError("relocalization result timestamp is invalid")
            with self.relocalization_lock:
                self.pending_relocalization = (
                    stamp, alignment, recovered_pose, source_pose,
                    np.asarray(msg.pose.covariance, dtype=float),
                )
            self.last_reason = "relocalization_pending_window_reset"
        except (ValueError, IndexError, TypeError) as error:
            self.counts["relocalization_rejections"] += 1
            self.last_reason = f"relocalization_invalid_result:{type(error).__name__}"
            self.last_exception = f"{type(error).__name__}:{error}"

    def _apply_pending_relocalization(self, stamp):
        with self.relocalization_lock:
            pending = self.pending_relocalization
            if pending is None or float(pending[0]) >= float(stamp):
                return False
            self.pending_relocalization = None
        result_stamp, alignment, recovered_pose, _, covariance = pending
        recovered = np.zeros(15, dtype=float)
        if self.backend.state_count > 0:
            recovered[6:15] = self.backend.state(-1)[6:15]
        recovered[:6] = matrix_to_pose_vector(recovered_pose)
        prior_variance = np.ones(15, dtype=float) * 1.0
        if covariance.size >= 36 and np.all(np.isfinite(covariance[:36])):
            prior_variance[:3] = np.maximum(
                1.0e-4, [covariance[0], covariance[7], covariance[14]]
            )
            prior_variance[3:6] = np.maximum(
                1.0e-4, [covariance[21], covariance[28], covariance[35]]
            )
        self.map_from_lio = alignment
        self.backend.reset(recovered, covariance=prior_variance)
        self.last_lio_stamp = float(result_stamp)
        self.last_lio_position = recovered[:3].copy()
        self.last_lio_yaw = float(recovered[5])
        self.path.poses = []
        self.counts["relocalization_resets"] += 1
        self.state_reset_counter += 1
        self.last_reason = "relocalization_window_reset_applied"
        return True

    def _decision(self, modality, default_enabled=False):
        if self.reliability_mode == "fixed":
            weight = self.fixed_weights.get(modality, 1.0)
            return scheduler_decision(
                weight,
                default_enabled and weight > 0.0,
                self.fixed_covariance_inflation,
            )
        now = self._now_s()
        # LIO is the local estimator anchor. A missing/stale diagnostic must
        # not silently remove its pose factor and leave rotation unobservable.
        score_item = self.scores.get(modality)
        score_fresh = self._score_is_fresh(modality, now)
        if modality == "lidar" and not score_fresh:
            return scheduler_decision(1.0, default_enabled, 1.0)
        if self._age_s(now, self.scheduler_arrival) <= self.scheduler_timeout_s:
            item = self.scheduler.get(modality)
            if item is not None:
                decision = scheduler_decision(item[0], item[1], item[2])
                return self._protect_lidar_anchor(
                    modality, decision, now, score_fresh
                )
        item = self.scores.get(modality)
        if item is not None and self._age_s(
            now, item["received_ros_s"]
        ) <= self.scheduler_timeout_s:
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
            and self._age_s(now, item["received_ros_s"])
            <= self.scheduler_timeout_s
        )

    def _imu_backup_ready(self, now):
        score_fresh = self._score_is_fresh("imu", now)
        scheduler_fresh = (
            self._age_s(now, self.scheduler_arrival)
            <= self.scheduler_timeout_s
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
            self.counts["imu_invalid"] += 1
            self.imu_invalid_reasons["missing_timestamp"] = (
                self.imu_invalid_reasons.get("missing_timestamp", 0) + 1
            )
            return
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
        now_s = self._now_s()
        if stamp <= 0.0:
            self.counts["flow_clock_mismatch"] += 1
            self.last_flow_reason = "missing_sensor_timestamp"
            return None
        if (
            now_s > 0.0
            and abs(stamp - now_s) > self.maximum_sensor_clock_skew_s
        ):
            self.counts["flow_clock_mismatch"] += 1
            self.last_flow_reason = "sensor_clock_domain_mismatch"
            return None
        return stamp

    def _flow(self, msg):
        self.counts["flow_received"] += 1
        stamp = self._flow_stamp(stamp_seconds(msg.header.stamp))
        if stamp is None:
            return
        with self.flow_buffer_lock:
            self.flow_buffer.append({
                "stamp_s": stamp,
                "integrated_x": float(msg.integrated_x),
                "integrated_y": float(msg.integrated_y),
                "integrated_xgyro": float(msg.integrated_xgyro),
                "integrated_ygyro": float(msg.integrated_ygyro),
                "integration_time_s": float(msg.integration_time_us) * 1.0e-6,
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
        covariance = gnss_covariance_diagonal(
            msg.position_covariance,
            msg.position_covariance_type,
            self.gnss_default_variance,
        ).tolist()
        position_enu = np.asarray(self.projector.project(*values), dtype=float)
        stamp_s = stamp_seconds(msg.header.stamp)
        if stamp_s <= 0.0:
            return
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
        observation = {
            "stamp_s": stamp_s,
            "position_enu": position_enu,
            "covariance": covariance,
            "status": int(msg.status.status),
            "temporal_jump": temporal_jump,
        }
        with self.gnss_lock:
            self.counts["gnss_received"] += 1
            if any(
                abs(float(item["stamp_s"]) - stamp_s) <= 1.0e-9
                for item in self.gnss_buffer
            ):
                self.counts["gnss_duplicates"] += 1
                return
            if self.gnss_buffer and stamp_s < float(self.gnss_buffer[-1]["stamp_s"]):
                self.counts["gnss_out_of_order"] += 1
            ordered = sorted(
                [*self.gnss_buffer, observation],
                key=lambda item: float(item["stamp_s"]),
            )
            self.gnss_buffer.clear()
            self.gnss_buffer.extend(ordered[-self.gnss_buffer.maxlen:])
            self.latest_gnss = observation

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
        calibration_offset = effective_time_offset(
            self.last_calibration_update,
            self.online_calibration_enabled,
        )
        imu_previous_stamp = float(previous_stamp) + calibration_offset
        imu_current_stamp = float(current_stamp) + calibration_offset
        stamps = [sample.stamp_s for sample in samples]
        start = max(0, bisect_left(stamps, imu_previous_stamp) - 1)
        end = min(len(samples), bisect_right(stamps, imu_current_stamp) + 1)
        result = preintegrate_manifold(
            samples[start:end],
            imu_previous_stamp,
            imu_current_stamp,
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

    def _record_phase_timing(self, name, started_ns):
        elapsed_ms = (time.perf_counter_ns() - started_ns) * 1.0e-6
        timing = self.phase_timing[name]
        timing["count"] += 1
        timing["total_ms"] += elapsed_ms
        timing["max_ms"] = max(timing["max_ms"], elapsed_ms)
        timing["last_ms"] = elapsed_ms
        return elapsed_ms

    def _phase_mean_ms(self, name):
        timing = self.phase_timing[name]
        return timing["total_ms"] / max(1, timing["count"])

    def _manifold_imu_bias_changed(self, previous_index, measurement):
        """Check whether the optimized start bias invalidates this delta."""
        if measurement is None or self.backend.state_count <= previous_index:
            return False
        optimized = self.backend.state(previous_index)
        accel_delta = np.linalg.norm(
            optimized[9:12]
            - np.asarray(measurement.accel_bias_linearization, dtype=float)
        )
        gyro_delta = np.linalg.norm(
            optimized[12:15]
            - np.asarray(measurement.gyro_bias_linearization, dtype=float)
        )
        return bool(
            accel_delta > self.imu_reintegration_accel_bias_threshold
            or gyro_delta > self.imu_reintegration_gyro_bias_threshold
        )

    def _gnss_factor(self, stamp, position, index):
        if self.projector is None or self.lio_origin is None:
            return
        with self.gnss_lock:
            observation, stale_count, superseded_count = select_gnss_observation(
                self.gnss_buffer,
                stamp,
                self.gnss_max_age_s,
                self.gnss_future_tolerance_s,
            )
            self.counts["gnss_stale_discarded"] += stale_count
            self.counts["gnss_superseded"] += superseded_count
            if observation is not None:
                self.counts["gnss_consumed"] += 1
        if observation is None:
            return
        gnss_position = np.asarray(self.lio_origin) + np.asarray(
            observation["position_enu"], dtype=float)
        covariance = np.asarray(observation["covariance"], dtype=float)
        current = np.asarray(position, dtype=float)
        innovation = current - gnss_position
        mahalanobis = float(np.sum(innovation * innovation / covariance))
        score, _, _ = gnss_score(
            1.0 if observation["status"] >= 0 else 0.0,
            float(np.sum(covariance)), mahalanobis,
        )
        decision = self._decision("gnss", default_enabled=True)
        decision["degradation_score"] = float(score)
        if bool(observation.get("temporal_jump", False)):
            self.counts["gnss_jump_rejected"] += 1
            return
        self.backend.add_gnss(index, gnss_position, covariance=covariance, decision=decision)
        self.counts["gnss_factors"] += 1

    def _flow_los_diagnostic(self, records, previous_state,
                             previous_stamp, current_stamp):
        """Evaluate APM LOS residuals without adding a new optimization factor."""
        if not self.flow_los_diagnostics_enabled:
            self.last_flow_los_diagnostic = None
            return None
        observation = flow_los_observation(records)
        if observation is None or previous_state is None:
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        state = np.asarray(previous_state, dtype=float)
        if state.shape != (15,) or np.any(~np.isfinite(state)):
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        angular_samples = [
            (sample.stamp_s, sample.angular_velocity)
            for sample in self._imu_snapshot()
        ]
        angular_velocity = interval_mean_vector(
            angular_samples,
            previous_stamp,
            current_stamp,
            self.flow_rotation_imu_max_gap_s,
        )
        if angular_velocity is None:
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        rotation_body_to_map = rpy_to_rotation_matrix(state[3:6])
        velocity_body = rotation_body_to_map.T @ state[6:9]
        angular_velocity = tuple(
            float(angular_velocity[index]) - float(state[12 + index])
            for index in range(3)
        )
        prediction_without_lever = optical_flow_los_prediction_flu(
            velocity_body,
            angular_velocity,
            (0.0, 0.0, 0.0),
            observation["distance_m"],
        )
        prediction_with_lever = optical_flow_los_prediction_flu(
            velocity_body,
            angular_velocity,
            self.flow_sensor_offset_body_m,
            observation["distance_m"],
        )
        if prediction_without_lever is None or prediction_with_lever is None:
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        measurement = observation["measurement_radps"]
        residual_without_lever = tuple(
            float(measurement[index]) - float(prediction_without_lever[index])
            for index in range(2)
        )
        residual = tuple(
            float(measurement[index]) - float(prediction_with_lever[index])
            for index in range(2)
        )
        lever_delta = tuple(
            float(prediction_with_lever[index])
            - float(prediction_without_lever[index])
            for index in range(2)
        )
        residual_norm = float(np.linalg.norm(residual))
        residual_without_lever_norm = float(
            np.linalg.norm(residual_without_lever)
        )
        lever_norm = float(np.linalg.norm(lever_delta))
        diagnostic = {
            "measurement_radps": tuple(float(value) for value in measurement),
            "prediction_without_lever_radps": tuple(
                float(value) for value in prediction_without_lever
            ),
            "prediction_with_lever_radps": tuple(
                float(value) for value in prediction_with_lever
            ),
            "residual_radps": residual,
            "residual_norm_radps": residual_norm,
            "residual_without_lever_radps": residual_without_lever,
            "residual_without_lever_norm_radps": residual_without_lever_norm,
            "lever_delta_radps": lever_delta,
            "lever_delta_norm_radps": lever_norm,
            "distance_m": float(observation["distance_m"]),
            "integration_s": float(observation["integration_s"]),
            "sample_count": int(observation["sample_count"]),
            "angular_velocity_body_flu": tuple(float(value) for value in angular_velocity),
            "velocity_body_flu": tuple(float(value) for value in velocity_body),
            "sensor_offset_body_m": tuple(
                float(value) for value in self.flow_sensor_offset_body_m
            ),
            "state_source": "previous_optimized_state",
        }
        self.last_flow_los_diagnostic = diagnostic
        self.counts["flow_los_diagnostic_samples"] += 1
        self.flow_los_residual_no_lever_norms.append(residual_without_lever_norm)
        self.flow_los_residual_norms.append(residual_norm)
        self.flow_los_lever_arm_norms.append(lever_norm)
        return diagnostic

    def _flow_factor(self, previous_stamp, current_stamp, previous_yaw,
                     previous_index, current_index, lio_delta,
                     previous_state=None):
        self.counts["flow_factor_attempts"] += 1
        with self.flow_buffer_lock:
            records, remaining, delayed = select_flow_records(
                self.flow_buffer,
                previous_stamp,
                current_stamp,
                self.flow_max_age_s,
            )
            self.flow_buffer = deque(remaining, maxlen=3000)
        if not records:
            self.last_flow_reason = "no_samples"
            return
        observation = flow_observation_delta(records, previous_yaw)
        if observation is None:
            self.last_flow_reason = "no_valid_observation"
            return
        flow_displacement = observation["delta_position"]
        los_diagnostic = self._flow_los_diagnostic(
            records, previous_state, previous_stamp, current_stamp,
        )
        score, evidence, reasons = optical_flow_score(
            observation["delta_position"],
            [float(lio_delta[0]), float(lio_delta[1])],
            observation["quality"], observation["distance_m"],
        )
        decision = self._decision("optical_flow", default_enabled=True)
        decision["degradation_score"] = float(score)
        decision["evidence"] = evidence
        decision["reasons"] = list(reasons)
        if los_diagnostic is None:
            decision["evidence"]["flow_los_diagnostic_valid"] = 0.0
        else:
            decision["evidence"].update({
                "flow_los_diagnostic_valid": 1.0,
                "flow_los_measurement_x_radps": los_diagnostic[
                    "measurement_radps"][0],
                "flow_los_measurement_y_radps": los_diagnostic[
                    "measurement_radps"][1],
                "flow_los_prediction_x_radps": los_diagnostic[
                    "prediction_with_lever_radps"][0],
                "flow_los_prediction_y_radps": los_diagnostic[
                    "prediction_with_lever_radps"][1],
                "flow_los_residual_norm_radps": los_diagnostic[
                    "residual_norm_radps"],
                "flow_los_residual_without_lever_norm_radps": los_diagnostic[
                    "residual_without_lever_norm_radps"],
                "flow_los_lever_delta_norm_radps": los_diagnostic[
                    "lever_delta_norm_radps"],
                "flow_los_distance_m": los_diagnostic["distance_m"],
                "flow_los_integration_s": los_diagnostic["integration_s"],
            })
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
            flow_displacement is not None and not quality_or_distance_invalid,
            rotation_compensated=flow_displacement is not None,
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
                if self.native_worker_stop.is_set() or not rclpy.ok():
                    self.active_transaction_snapshot = None
                    self.last_reason = "shutdown"
                    break
                if self.active_transaction_snapshot is not None:
                    self.backend.restore(self.active_transaction_snapshot)
                    self.active_transaction_snapshot = None
                    self.counts["optimization_rollbacks"] += 1
                self.counts["native_worker_errors"] += 1
                self.last_reason = f"native_worker_error:{type(error).__name__}"
                self.last_exception = f"{type(error).__name__}:{error}"
                self._consume_native_sequence(
                    int(factor.scan_sequence), state_committed=False
                )
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
            deadline = self._now_s() + self.imu_factor_wait_s
            covered = False
            while not self.native_worker_stop.is_set():
                covered, _, _ = imu_interval_status(
                    self._imu_snapshot(),
                    self.last_lio_stamp,
                    stamp,
                    self.imu_max_gap_s,
                )
                if covered or self._now_s() >= deadline:
                    break
                self.native_worker_stop.wait(0.002)
            if not covered:
                self.counts["imu_pair_timeouts"] += 1
        if not self.native_worker_stop.is_set():
            self._process_lio(message, factor)
            state_committed = (
                self.last_lio_stamp is not None
                and abs(float(self.last_lio_stamp) - float(factor.stamp_s))
                <= self.scan_prediction_timestamp_tolerance_s
                and self.last_reason == "ok"
            )
            self._consume_native_sequence(
                int(factor.scan_sequence), state_committed=state_committed
            )

    def _consume_native_sequence(self, sequence, state_committed):
        """Release the next scan after this factor reaches a terminal outcome."""
        if not state_committed:
            self.counts["native_consumed_without_state_commit"] += 1
        self.last_native_consumed_sequence = max(
            self.last_native_consumed_sequence, int(sequence)
        )
        self._release_pending_scan_requests()

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
            self.last_reason = "missing_lio_timestamp"
            return
        factor = self.native_lidar_buffer.pop_nearest(
            stamp, self.native_lidar_tolerance_s
        )
        if factor is not None:
            self._dispatch_lio(msg, factor)
            return
        self.pending_lio.append((self._now_s(), msg))

    def _dispatch_lio(self, msg, native_factor):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            self.last_reason = "missing_lio_timestamp"
            return
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
                (self._now_s(), msg, native_factor)
            )
            return
        self._process_lio(msg, native_factor)

    def _drain_pending_inputs(self):
        self._drain_pending_lio()
        self._drain_pending_imu_lio()

    def _drain_pending_lio(self):
        if not self.pending_lio:
            return
        now = self._now_s()
        while self.pending_lio:
            arrival, msg = self.pending_lio[0]
            if arrival > now:
                self.pending_lio.clear()
                return
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                self.pending_lio.popleft()
                self.last_reason = "missing_lio_timestamp"
                continue
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
        now = self._now_s()
        while self.pending_imu_lio:
            arrival, msg, native_factor = self.pending_imu_lio[0]
            if arrival > now:
                self.pending_imu_lio.clear()
                return
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                self.pending_imu_lio.popleft()
                self.last_reason = "missing_lio_timestamp"
                continue
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
            self.last_reason = "missing_lio_timestamp"
            return
        self._apply_pending_relocalization(stamp)
        if native_factor is not None:
            native_factor = transform_native_factor_map(
                native_factor, self.map_from_lio
            )
            msg = native_frame_odometry(msg.header, native_factor)
            stamp = native_factor.stamp_s
        elif not np.allclose(self.map_from_lio, np.eye(4), atol=1.0e-12):
            local_pose = np.concatenate((
                np.asarray([
                    float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    float(msg.pose.pose.position.z),
                ]),
                quaternion_xyzw_to_rpy([
                    float(msg.pose.pose.orientation.x),
                    float(msg.pose.pose.orientation.y),
                    float(msg.pose.pose.orientation.z),
                    float(msg.pose.pose.orientation.w),
                ]),
            ))
            aligned_pose = matrix_to_pose_vector(
                self.map_from_lio @ pose_vector_to_matrix(local_pose)
            )
            msg = copy.deepcopy(msg)
            msg.pose.pose.position.x = float(aligned_pose[0])
            msg.pose.pose.position.y = float(aligned_pose[1])
            msg.pose.pose.position.z = float(aligned_pose[2])
            qx, qy, qz, qw = rpy_to_quaternion_xyzw(aligned_pose[3:6])
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
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
        scan_prediction = None
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
        elif (
            self.frontend_scan_prediction_enabled
            and native_factor is not None
        ):
            if native_factor.scan_end_s <= native_factor.scan_begin_s:
                self.counts["optimization_rejected"] += 1
                self.last_reason = "scan_prediction_missing_exact_interval"
                return
            scan_prediction, prediction_reason = consume_cached_prediction(
                self.scan_prediction_by_sequence,
                sequence=native_factor.scan_sequence,
                previous_stamp_s=self.last_lio_stamp,
                scan_end_s=native_factor.scan_end_s,
                current_previous_state=previous_state,
                timestamp_tolerance_s=self.scan_prediction_timestamp_tolerance_s,
                state_tolerance=self.scan_prediction_state_tolerance,
            )
            if scan_prediction is None:
                if prediction_reason == "cache_miss":
                    self.counts["scan_prediction_cache_misses"] += 1
                else:
                    self.counts["scan_prediction_reuse_rejected"] += 1
                self.counts["optimization_rejected"] += 1
                self.last_reason = f"scan_prediction_not_reusable:{prediction_reason}"
                return
            self.counts["scan_prediction_cache_hits"] += 1
            self.scan_prediction_cache.append(scan_prediction)
            initial_state = scan_prediction.end_state.copy()
            manifold_measurement = scan_prediction.measurement
            self.counts["imu_propagated_initializations"] += 1
            reference = manifold_motion_reference(previous_state, initial_state)
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
        self._record_phase_timing("pre_state", started)
        snapshot_started = time.perf_counter_ns()
        transaction_snapshot = (
            self.backend.snapshot() if self.transactional_update_enabled else None
        )
        self._record_phase_timing("snapshot", snapshot_started)
        self.active_transaction_snapshot = transaction_snapshot
        add_state_started = time.perf_counter_ns()
        current_index = self.backend.add_state(initial_state)
        self._record_phase_timing("add_state", add_state_started)
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
        lidar_factor_started = time.perf_counter_ns()
        lidar_decision = self._decision("lidar", default_enabled=True)
        if lidar_decision.get("anchor_override", False):
            self.counts["lidar_anchor_overrides"] += 1
        lidar_factor_added = False
        if native_factor is not None:
            native_factor = with_yaw_reference(native_factor, yaw)
            if (
                self.online_calibration_enabled
                and native_factor.correspondences_valid
                and native_factor.lidar_to_body_rotation is not None
            ):
                try:
                    calibration_update = self.calibrator.update(
                        native_factor.stamp_s,
                        rpy_to_rotation_matrix(native_factor.linearization_pose[3:6]),
                        native_factor.lidar_to_body_rotation,
                        self._imu_snapshot(),
                    )
                    self.last_calibration_update = calibration_update
                    if calibration_update.reason != "update_throttled":
                        self.counts["calibration_updates"] += 1
                    if calibration_update.accepted:
                        self.counts["calibration_accepted"] += 1
                    if calibration_update.reason == "sharp_turn_frozen":
                        self.counts["calibration_frozen"] += 1
                    if calibration_update.locked:
                        native_factor = replace(
                            native_factor,
                            lidar_to_body_rotation=(
                                calibration_update.lidar_to_body_rotation.copy()
                            ),
                        )
                except (ValueError, np.linalg.LinAlgError) as error:
                    self.last_exception = (
                        f"calibration_{type(error).__name__}:{error}"
                    )
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
            observability = lidar_pose_observability(native_factor)
            self.last_native_effective_rank = observability.effective_rank
            self.last_native_translation_rank = observability.translation_rank
            self.last_native_rotation_rank = observability.rotation_rank
            self.last_native_condition_number = observability.condition_number
            self.last_native_characteristic_range_m = (
                observability.characteristic_range_m
            )
            self.last_native_normalized_eigenvalues = np.asarray(
                observability.normalized_eigenvalues, dtype=float
            )
            if observability.effective_rank < 6:
                self.counts["native_lidar_directionally_degenerate"] += 1
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
        self._record_phase_timing("lidar_factor", lidar_factor_started)
        imu_diagnostic_covariance = None
        aux_factors_started = time.perf_counter_ns()
        if self.last_lio_stamp is not None:
            previous_index = current_index - 1
            self._gnss_factor(stamp, reference["position"], current_index)
            self._flow_factor(
                self.last_lio_stamp, stamp, reference["yaw"],
                previous_index, current_index, reference["delta_position"],
                previous_state=previous_state,
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
        self._record_phase_timing("aux_factors", aux_factors_started)
        self._record_phase_timing("prepare", started)
        post_optimize_started = time.perf_counter_ns()
        try:
            optimize_started = time.perf_counter_ns()
            self.backend.optimize()
            self._record_phase_timing("optimize", optimize_started)
            solve_ms = float(getattr(self.backend, "last_solve_ms", 0.0))
            self.backend_solve_count += 1
            self.backend_solve_ms_total += solve_ms
            self.backend_solve_ms_max = max(self.backend_solve_ms_max, solve_ms)
            # A preintegrated delta is linearized at the start-state bias. The
            # first nonlinear solve can move that bias enough to invalidate the
            # delta, especially after a long or dynamic interval. Recompute it
            # once at the optimized start bias, replace the active factor, and
            # solve again. This keeps the window's IMU factor consistent without
            # feeding the FCU's fused pose back into the estimator.
            if (
                self.backend_solver_mode == "manifold"
                and manifold_measurement is not None
                and self.last_lio_stamp is not None
                and self._manifold_imu_bias_changed(
                    current_index - 1, manifold_measurement
                )
            ):
                updated_measurement = self._manifold_imu_measurement(
                    self.last_lio_stamp,
                    stamp,
                    self.backend.state(current_index - 1),
                )
                if updated_measurement is not None and self.backend.replace_imu_preintegrated(
                    current_index - 1, current_index, updated_measurement
                ):
                    manifold_measurement = updated_measurement
                    imu_diagnostic_covariance = np.asarray(
                        updated_measurement.covariance, dtype=float
                    )
                    self.counts["imu_reintegrations"] += 1
                    reintegration_started = time.perf_counter_ns()
                    self.backend.optimize()
                    self._record_phase_timing("reintegrate", reintegration_started)
                    second_solve_ms = float(
                        getattr(self.backend, "last_solve_ms", 0.0)
                    )
                    self.backend_solve_count += 1
                    self.backend_solve_ms_total += second_solve_ms
                    self.backend_solve_ms_max = max(
                        self.backend_solve_ms_max, second_solve_ms
                    )
            post_optimize_started = time.perf_counter_ns()
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
            if self.transactional_update_enabled:
                self.last_optimization_integrity = validate_optimized_state(
                    initial_state,
                    estimate,
                    self.backend.latest_state_information(),
                    self.backend.last_initial_cost,
                    self.backend.last_cost,
                    **self.optimization_integrity_limits,
                )
                if not self.last_optimization_integrity.valid:
                    self.backend.restore(transaction_snapshot)
                    self.active_transaction_snapshot = None
                    self.counts["optimization_rejected"] += 1
                    self.counts["optimization_rollbacks"] += 1
                    self.last_reason = (
                        "optimization_rejected:"
                        f"{self.last_optimization_integrity.reason}"
                    )
                    self.last_callback_ms = (
                        time.perf_counter_ns() - started
                    ) * 1.0e-6
                    return
            publish_started = time.perf_counter_ns()
            self._publish(msg.header, estimate)
            self._record_phase_timing("publish", publish_started)
            self.active_transaction_snapshot = None
            self.counts["lio"] += 1
            self.last_reason = "ok"
        except (np.linalg.LinAlgError, ValueError, IndexError) as error:
            if transaction_snapshot is not None:
                self.backend.restore(transaction_snapshot)
                self.active_transaction_snapshot = None
                self.counts["optimization_rollbacks"] += 1
            self.counts["optimization_errors"] += 1
            self.last_reason = f"optimization_error:{type(error).__name__}"
            self.last_exception = f"{type(error).__name__}:{error}"
        self._record_phase_timing("post_optimize", post_optimize_started)
        self.last_lio_stamp = stamp
        self.last_lio_position = position
        self.last_lio_yaw = yaw
        self.last_callback_ms = (time.perf_counter_ns() - started) * 1.0e-6

    def _publish_frontend_state_seed(self, header, state, covariance):
        if self.frontend_state_seed_pub is None:
            return
        if stamp_seconds(header.stamp) <= 0.0:
            raise ValueError("frontend state seed requires a source timestamp")
        state = np.asarray(state, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if state.shape != (15,) or covariance.shape != (15, 15):
            raise ValueError("frontend state seed has an invalid state dimension")
        if np.any(~np.isfinite(state)) or np.any(~np.isfinite(covariance)):
            raise ValueError("frontend state seed must be finite")
        orientation = rpy_to_quaternion_xyzw(state[3:6])
        message = BackendStateSeed()
        message.header = copy.deepcopy(header)
        message.header.frame_id = self.map_frame
        message.map_frame = self.map_frame
        message.body_frame = self.body_frame
        message.sequence = int(self.frontend_state_seed_sequence)
        message.reset_counter = int(self.state_reset_counter)
        support = (
            1.0 if self.reliability_mode == "fixed"
            else self.scheduler_estimator_support
        )
        message.quality = int(round(100.0 * min(1.0, max(0.0, support))))
        message.valid = True
        message.position = [float(value) for value in state[:3]]
        message.orientation_xyzw = [float(value) for value in orientation]
        message.velocity_map = [float(value) for value in state[6:9]]
        message.accel_bias = [float(value) for value in state[9:12]]
        message.gyro_bias = [float(value) for value in state[12:15]]
        message.covariance = [float(value) for value in covariance.ravel()]
        self.frontend_state_seed_pub.publish(message)
        self.frontend_state_seed_sequence += 1
        self.counts["frontend_state_seeds"] += 1

    @staticmethod
    def _state_orientation_xyzw(state):
        return rpy_to_quaternion_xyzw(np.asarray(state[3:6], dtype=float))

    def _publish_scan_request(self, factor):
        if self.scan_prediction_pub is None:
            return
        request = FrontendScanRequest()
        request.header.stamp = ros_time_from_seconds(factor.stamp_s)
        request.header.frame_id = self.map_frame
        request.scan_sequence = int(factor.scan_sequence)
        request.scan_begin_stamp = ros_time_from_seconds(float(factor.scan_begin_s))
        request.scan_end_stamp = ros_time_from_seconds(float(factor.scan_end_s))
        request.map_frame = self.map_frame
        request.body_frame = self.body_frame
        request.sensor_frame = str(factor.sensor_frame)
        request.point_count = int(factor.candidate_points)
        self.scan_prediction_pub.publish(request)

    def _publish_deskew_trajectory(self, prediction, covariance, reason=None):
        if self.deskew_trajectory_pub is None:
            return
        message = BackendDeskewTrajectory()
        message.header.stamp = ros_time_from_seconds(prediction.scan_end_s)
        message.header.frame_id = self.map_frame
        message.scan_sequence = int(prediction.sequence)
        message.reset_counter = int(self.state_reset_counter)
        message.quality = (
            int(round(100.0 * prediction.quality)) if prediction.valid else 0
        )
        message.valid = bool(prediction.valid)
        message.reason = str(prediction.reason if reason is None else reason)
        message.map_frame = self.map_frame
        message.body_frame = self.body_frame
        message.scan_begin_stamp = ros_time_from_seconds(prediction.scan_begin_s)
        message.scan_end_stamp = ros_time_from_seconds(prediction.scan_end_s)
        message.begin_position = [float(value) for value in prediction.begin_state[:3]]
        message.begin_orientation_xyzw = [
            float(value) for value in self._state_orientation_xyzw(prediction.begin_state)
        ]
        message.end_position = [float(value) for value in prediction.end_state[:3]]
        message.end_orientation_xyzw = [
            float(value) for value in self._state_orientation_xyzw(prediction.end_state)
        ]
        message.end_velocity_map = [float(value) for value in prediction.end_state[6:9]]
        message.accel_bias = [float(value) for value in prediction.end_state[9:12]]
        message.gyro_bias = [float(value) for value in prediction.end_state[12:15]]
        matrix = np.asarray(covariance, dtype=float)
        if matrix.shape != (15, 15) or np.any(~np.isfinite(matrix)):
            matrix = np.eye(15, dtype=float)
        message.end_covariance = [float(value) for value in matrix.ravel()]
        self.deskew_trajectory_pub.publish(message)
        self.counts["scan_prediction_published"] += 1

    def _scan_request(self, message):
        """Serialize trajectory N after native factor N-1 is consumed."""
        if not self.frontend_scan_prediction_enabled:
            return
        self.counts["scan_prediction_requests"] += 1
        sequence = int(message.scan_sequence)
        if scan_request_stale(self.last_native_consumed_sequence, sequence):
            self.counts["scan_prediction_stale_requests"] += 1
            return
        with self.pending_scan_request_lock:
            if not scan_request_ready(
                self.last_native_consumed_sequence, sequence
            ):
                if sequence in self.pending_scan_requests:
                    self.counts["scan_prediction_duplicate_requests"] += 1
                self.pending_scan_requests[sequence] = copy.deepcopy(message)
                self.counts["scan_prediction_deferred"] += 1
                return
        if sequence in self.scan_prediction_by_sequence:
            self.counts["scan_prediction_duplicate_requests"] += 1
        self._produce_scan_prediction(message)

    def _release_pending_scan_requests(self):
        ready = []
        with self.pending_scan_request_lock:
            for sequence in sorted(self.pending_scan_requests):
                if scan_request_ready(
                    self.last_native_consumed_sequence, sequence
                ):
                    ready.append(self.pending_scan_requests.pop(sequence))
        for message in ready:
            self.counts["scan_prediction_deferred_released"] += 1
            self._produce_scan_prediction(message)

    def _produce_scan_prediction(self, message):
        """Produce one trajectory against the latest committed backend state."""
        sequence = int(message.scan_sequence)
        begin_s = stamp_seconds(message.scan_begin_stamp)
        end_s = stamp_seconds(message.scan_end_stamp)
        if self.last_lio_stamp is None or self.backend.state_count == 0:
            self.counts["scan_prediction_rejected"] += 1
            self.last_scan_prediction_reason = "backend_state_unavailable"
            return
        previous_state = self.backend.state(-1)
        cached = self.scan_prediction_by_sequence.get(sequence)
        if cached is not None:
            reusable, _cached_reason = prediction_reusable(
                cached,
                sequence=sequence,
                previous_stamp_s=self.last_lio_stamp,
                scan_end_s=end_s,
                current_previous_state=previous_state,
                timestamp_tolerance_s=self.scan_prediction_timestamp_tolerance_s,
                state_tolerance=self.scan_prediction_state_tolerance,
            )
            begin_matches = (
                abs(float(cached.scan_begin_s) - begin_s)
                <= self.scan_prediction_timestamp_tolerance_s
            )
            if reusable and begin_matches:
                self.counts["scan_prediction_cache_hits"] += 1
                self._publish_deskew_trajectory(
                    cached, self.last_state_covariance, reason="cached"
                )
                return
            self.scan_prediction_by_sequence.pop(sequence, None)
            self.counts["scan_prediction_reuse_rejected"] += 1
        prediction = build_scan_prediction(
            sequence,
            self.last_lio_stamp,
            begin_s,
            end_s,
            previous_state,
            ordered_imu_samples(self._imu_snapshot()),
            maximum_begin_gap_s=self.scan_prediction_maximum_begin_gap_s,
            nominal_imu_gap_s=self.imu_nominal_gap_s,
            maximum_imu_gap_s=self.imu_max_gap_s,
        )
        self.scan_prediction_by_sequence[sequence] = prediction
        while len(self.scan_prediction_by_sequence) > self.scan_prediction_cache_size:
            self.scan_prediction_by_sequence.pop(next(iter(self.scan_prediction_by_sequence)))
        if not prediction.valid:
            self.counts["scan_prediction_rejected"] += 1
            self.last_scan_prediction_reason = prediction.reason
            self._publish_deskew_trajectory(
                prediction, self.last_state_covariance, reason=prediction.reason
            )
            return
        self.last_scan_prediction_reason = "ok"
        self._publish_deskew_trajectory(prediction, self.last_state_covariance)

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
        state_covariance = None
        if self.backend_solver_mode == "manifold":
            output_stamp_s = stamp_seconds(header.stamp)
            if output_stamp_s <= 0.0:
                raise ValueError("backend output requires a source timestamp")
            update_due = covariance_update_due(
                self.last_covariance_stamp_s,
                output_stamp_s,
                self.marginal_covariance_update_period_s,
            )
            if update_due:
                try:
                    candidate = self.backend.marginal_covariance(-1)
                    if candidate.shape != (15, 15) or np.any(~np.isfinite(candidate)):
                        raise ValueError("optimizer returned an invalid covariance")
                    self.last_state_covariance = candidate
                    self.last_covariance_stamp_s = output_stamp_s
                    self.counts["marginal_covariance_updates"] += 1
                    self.last_covariance_source = "window_marginal"
                except (ValueError, IndexError, np.linalg.LinAlgError) as error:
                    self.counts["marginal_covariance_errors"] += 1
                    self.last_covariance_source = "fixed_fallback"
                    self.last_exception = f"covariance_{type(error).__name__}:{error}"
            elif self.last_state_covariance is not None:
                self.counts["marginal_covariance_reuses"] += 1
                self.last_covariance_source = "window_marginal_cached"
            state_covariance = self.last_state_covariance
        if state_covariance is None:
            output.pose.covariance[0] = 0.05 ** 2
            output.pose.covariance[7] = 0.05 ** 2
            output.pose.covariance[14] = 0.10 ** 2
            output.pose.covariance[35] = 0.03 ** 2
            output.twist.covariance[0] = 0.25
            output.twist.covariance[7] = 0.25
            output.twist.covariance[14] = 0.50
            state_covariance = np.diag(np.asarray(
                [0.05 ** 2, 0.05 ** 2, 0.10 ** 2]
                + [0.03 ** 2] * 3
                + [0.50 ** 2, 0.50 ** 2, 0.75 ** 2]
                + [0.50 ** 2] * 3
                + [0.05 ** 2] * 3,
                dtype=float,
            ))
        else:
            pose_covariance = 0.5 * (
                state_covariance[:6, :6] + state_covariance[:6, :6].T
            )
            output.pose.covariance = [
                float(value) for value in pose_covariance.ravel()
            ]
            map_to_body = rpy_to_rotation_matrix(orientation).T
            velocity_covariance = (
                map_to_body @ state_covariance[6:9, 6:9] @ map_to_body.T
            )
            for row in range(3):
                for column in range(3):
                    output.twist.covariance[row * 6 + column] = float(
                        velocity_covariance[row, column]
                    )
            # Angular velocity is not part of the optimized state. Mark it as
            # unobserved instead of publishing a misleading small covariance.
            output.twist.covariance[21] = 1.0e6
            output.twist.covariance[28] = 1.0e6
            output.twist.covariance[35] = 1.0e6
        self.odom_pub.publish(output)
        self._publish_frontend_state_seed(header, state, state_covariance)
        self.last_output = output
        if path_sample_due(
            self.last_path_sample_position,
            self.last_path_sample_orientation,
            state[:3],
            orientation,
            self.path_minimum_translation_m,
            self.path_minimum_rotation_rad,
        ):
            pose = PoseStamped()
            pose.header = copy.deepcopy(output.header)
            pose.pose = copy.deepcopy(output.pose.pose)
            self.path.poses.append(pose)
            if len(self.path.poses) > self.max_path:
                self.path.poses = self.path.poses[-self.max_path:]
            self.last_path_sample_position = np.asarray(state[:3], dtype=float).copy()
            self.last_path_sample_orientation = orientation.copy()
            self.counts["path_samples"] += 1
        output_stamp_s = stamp_seconds(output.header.stamp)
        if output_stamp_s <= 0.0:
            raise ValueError("backend path requires a source timestamp")
        if covariance_update_due(
            self.last_path_publish_stamp_s,
            output_stamp_s,
            self.path_publish_period_s,
        ):
            self.path.header = copy.deepcopy(output.header)
            self.path_pub.publish(self.path)
            self.last_path_publish_stamp_s = output_stamp_s
            self.counts["path_messages"] += 1
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
        flow_los_residual_mean = (
            float(np.mean(self.flow_los_residual_norms))
            if self.flow_los_residual_norms else -1.0
        )
        flow_los_residual_no_lever_mean = (
            float(np.mean(self.flow_los_residual_no_lever_norms))
            if self.flow_los_residual_no_lever_norms else -1.0
        )
        flow_los_residual_p95 = (
            float(np.percentile(self.flow_los_residual_norms, 95))
            if self.flow_los_residual_norms else -1.0
        )
        flow_los_residual_no_lever_p95 = (
            float(np.percentile(self.flow_los_residual_no_lever_norms, 95))
            if self.flow_los_residual_no_lever_norms else -1.0
        )
        flow_los_lever_mean = (
            float(np.mean(self.flow_los_lever_arm_norms))
            if self.flow_los_lever_arm_norms else -1.0
        )
        flow_los_lever_p95 = (
            float(np.percentile(self.flow_los_lever_arm_norms, 95))
            if self.flow_los_lever_arm_norms else -1.0
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
            f"imu_pair_timeouts={self.counts['imu_pair_timeouts']};"
            f"imu_reintegrations={self.counts['imu_reintegrations']};"
            f"calibration_accepted={self.counts['calibration_accepted']};"
            f"calibration_time_offset_s={self.last_calibration_update.time_offset_s:.6f}"
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
            f"last_reason={self.last_reason};"
            f"optimization_errors={self.counts['optimization_errors']};"
            f"optimization_rejected={self.counts['optimization_rejected']};"
            f"optimization_rollbacks={self.counts['optimization_rollbacks']};"
            f"frontend_state_seeds={self.counts['frontend_state_seeds']};"
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
            f"{self.counts['native_worker_queue_discarded']};"
            f"native_worker_errors={self.counts['native_worker_errors']};"
            f"native_rank={self.last_native_effective_rank};"
            f"native_translation_rank={self.last_native_translation_rank};"
            f"native_rotation_rank={self.last_native_rotation_rank};"
            "native_condition="
            f"{self.last_native_condition_number:.9g};"
            "native_degenerate_frames="
            f"{self.counts['native_lidar_directionally_degenerate']};"
            f"scan_requests={self.counts['scan_prediction_requests']};"
            f"scan_predictions={self.counts['scan_prediction_published']};"
            f"scan_rejected={self.counts['scan_prediction_rejected']};"
            f"scan_last_reason={self.last_scan_prediction_reason};"
            f"scan_deferred={self.counts['scan_prediction_deferred']};"
            "scan_deferred_released="
            f"{self.counts['scan_prediction_deferred_released']};"
            "scan_duplicate_requests="
            f"{self.counts['scan_prediction_duplicate_requests']};"
            "scan_stale_requests="
            f"{self.counts['scan_prediction_stale_requests']};"
            "native_consumed_without_state_commit="
            f"{self.counts['native_consumed_without_state_commit']};"
            f"scan_cache_hits={self.counts['scan_prediction_cache_hits']};"
            f"scan_cache_misses={self.counts['scan_prediction_cache_misses']};"
            "scan_reuse_rejected="
            f"{self.counts['scan_prediction_reuse_rejected']};"
            f"gnss_received={self.counts['gnss_received']};"
            f"gnss_consumed={self.counts['gnss_consumed']};"
            f"gnss_factors={self.counts['gnss_factors']};"
            f"gnss_duplicates={self.counts['gnss_duplicates']};"
            f"gnss_stale={self.counts['gnss_stale_discarded']};"
            f"flow_received={self.counts['flow_received']};"
            f"flow_attempts={self.counts['flow_factor_attempts']};"
            f"flow_factors={self.counts['flow_factors']};"
            f"flow_disabled_quality={self.counts['flow_disabled_quality']};"
            f"flow_disabled_rotation={self.counts['flow_disabled_rotation']};"
            f"flow_clock_mismatch={self.counts['flow_clock_mismatch']};"
            f"flow_last_reason={self.last_flow_reason};"
            f"flow_rotation_phase={self.last_flow_rotation_phase};"
            f"flow_los_samples={self.counts['flow_los_diagnostic_samples']};"
            f"flow_los_invalid={self.counts['flow_los_diagnostic_invalid']};"
            f"flow_los_residual_mean_radps={flow_los_residual_mean:.9g};"
            f"flow_los_residual_p95_radps={flow_los_residual_p95:.9g};"
            "flow_los_residual_no_lever_mean_radps="
            f"{flow_los_residual_no_lever_mean:.9g};"
            "flow_los_residual_no_lever_p95_radps="
            f"{flow_los_residual_no_lever_p95:.9g};"
            f"flow_los_lever_mean_radps={flow_los_lever_mean:.9g};"
            f"flow_los_lever_p95_radps={flow_los_lever_p95:.9g};"
            f"prepare_mean_ms={self._phase_mean_ms('prepare'):.3f};"
            f"prepare_max_ms={self.phase_timing['prepare']['max_ms']:.3f};"
            f"pre_state_mean_ms={self._phase_mean_ms('pre_state'):.3f};"
            f"snapshot_mean_ms={self._phase_mean_ms('snapshot'):.3f};"
            f"add_state_mean_ms={self._phase_mean_ms('add_state'):.3f};"
            f"lidar_factor_mean_ms={self._phase_mean_ms('lidar_factor'):.3f};"
            f"aux_factors_mean_ms={self._phase_mean_ms('aux_factors'):.3f};"
            f"optimize_mean_ms={self._phase_mean_ms('optimize'):.3f};"
            f"optimize_max_ms={self.phase_timing['optimize']['max_ms']:.3f};"
            f"reintegrate_mean_ms={self._phase_mean_ms('reintegrate'):.3f};"
            f"reintegrate_max_ms={self.phase_timing['reintegrate']['max_ms']:.3f};"
            f"post_optimize_mean_ms={self._phase_mean_ms('post_optimize'):.3f};"
            f"post_optimize_max_ms={self.phase_timing['post_optimize']['max_ms']:.3f};"
            f"publish_mean_ms={self._phase_mean_ms('publish'):.3f};"
            f"publish_max_ms={self.phase_timing['publish']['max_ms']:.3f};"
            f"covariance_source={self.last_covariance_source};"
            "marginal_covariance_errors="
            f"{self.counts['marginal_covariance_errors']}",
            flush=True,
        )

    def _diagnostics(self):
        average_solve_ms = (
            self.backend_solve_ms_total / self.backend_solve_count
            if self.backend_solve_count else 0.0
        )
        flow_los_residual_p95 = (
            float(np.percentile(self.flow_los_residual_norms, 95))
            if self.flow_los_residual_norms else -1.0
        )
        flow_los_residual_no_lever_p95 = (
            float(np.percentile(self.flow_los_residual_no_lever_norms, 95))
            if self.flow_los_residual_no_lever_norms else -1.0
        )
        flow_los_lever_p95 = (
            float(np.percentile(self.flow_los_lever_arm_norms, 95))
            if self.flow_los_lever_arm_norms else -1.0
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
                "backend_solve_wall_ms",
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
                "backend_marginalization_ms",
                f"{getattr(self.backend, 'last_marginalization_ms', 0.0):.3f}",
            ),
            self._key(
                "backend_marginalization_wall_ms",
                f"{getattr(self.backend, 'last_marginalization_ms', 0.0):.3f}",
            ),
            self._key(
                "backend_cost",
                f"{getattr(self.backend, 'last_cost', 0.0):.9g}",
            ),
            self._key("callback_ms", f"{self.last_callback_ms:.3f}"),
            self._key("callback_wall_ms", f"{self.last_callback_ms:.3f}"),
            self._key("algorithm_clock", "ros_sim_time"),
            self._key("covariance_source", self.last_covariance_source),
            self._key(
                "optimization_integrity_reason",
                self.last_optimization_integrity.reason,
            ),
            self._key(
                "optimization_translation_correction_m",
                f"{self.last_optimization_integrity.translation_correction_m:.9g}",
            ),
            self._key(
                "optimization_rotation_correction_rad",
                f"{self.last_optimization_integrity.rotation_correction_rad:.9g}",
            ),
            self._key(
                "optimization_velocity_correction_mps",
                f"{self.last_optimization_integrity.velocity_correction_mps:.9g}",
            ),
            self._key(
                "optimization_information_rank",
                self.last_optimization_integrity.latest_information_rank,
            ),
            self._key(
                "optimization_initial_cost",
                f"{self.last_optimization_integrity.initial_cost:.9g}",
            ),
            self._key(
                "optimization_final_cost",
                f"{self.last_optimization_integrity.final_cost:.9g}",
            ),
            self._key(
                "optimization_information_condition",
                f"{self.last_optimization_integrity.latest_information_condition:.9g}",
            ),
            self._key(
                "frontend_state_seed_enabled", self.frontend_state_seed_enabled
            ),
            self._key("state_reset_counter", self.state_reset_counter),
            self._key("last_imu_reason", self.last_imu_reason),
            self._key("calibration_reason", self.last_calibration_update.reason),
            self._key(
                "calibration_time_offset_s",
                f"{self.last_calibration_update.time_offset_s:.9g}",
            ),
            self._key(
                "calibration_time_correlation",
                f"{self.last_calibration_update.time_correlation:.9g}",
            ),
            self._key(
                "calibration_time_margin",
                f"{self.last_calibration_update.time_margin:.9g}",
            ),
            self._key(
                "calibration_rotation_residual_rad",
                f"{self.last_calibration_update.rotation_residual_rad:.9g}",
            ),
            self._key("calibration_locked", self.last_calibration_update.locked),
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
            self._key("native_lidar_effective_rank", self.last_native_effective_rank),
            self._key("native_lidar_translation_rank", self.last_native_translation_rank),
            self._key("native_lidar_rotation_rank", self.last_native_rotation_rank),
            self._key(
                "native_lidar_condition_number",
                f"{self.last_native_condition_number:.9g}",
            ),
            self._key(
                "native_lidar_characteristic_range_m",
                f"{self.last_native_characteristic_range_m:.9g}",
            ),
            self._key(
                "native_lidar_normalized_eigenvalues",
                ",".join(
                    f"{value:.6g}"
                    for value in self.last_native_normalized_eigenvalues
                ),
            ),
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
            self._key(
                "flow_los_diagnostic_valid",
                0 if self.last_flow_los_diagnostic is None else 1,
            ),
            self._key(
                "flow_los_residual_p95_radps",
                f"{flow_los_residual_p95:.9g}",
            ),
            self._key(
                "flow_los_residual_no_lever_p95_radps",
                f"{flow_los_residual_no_lever_p95:.9g}",
            ),
            self._key(
                "flow_los_lever_p95_radps",
                f"{flow_los_lever_p95:.9g}",
            ),
            self._key(
                "flow_los_sensor_offset_body_m",
                ",".join(
                    f"{value:.6g}" for value in self.flow_sensor_offset_body_m
                ),
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
