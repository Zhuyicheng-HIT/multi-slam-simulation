"""Read-only IMU propagation from the last optimized navigation state."""

from dataclasses import dataclass
import math

import numpy as np

from .imu_preintegration import ManifoldPreintegratedImu
from .manifold_window import propagate_state
from .native_lidar import rpy_to_rotation_matrix


STATE_SIZE = 15


@dataclass(frozen=True)
class OptimizationAnchor:
    """Immutable state owned by the optimizer, safe to copy across callbacks."""

    stamp_s: float
    state: tuple[float, ...]
    covariance: tuple[float, ...]
    generation: int
    reset_counter: int


@dataclass(frozen=True)
class PropagatedState:
    """A publication-only state derived from one optimization anchor."""

    stamp_s: float
    state: tuple[float, ...]
    covariance: tuple[float, ...]
    anchor_generation: int
    anchor_reset_counter: int


def _skew(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.asarray([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ], dtype=float)


def _quaternion_wxyz_to_rotation(quaternion):
    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm([w, x, y, z]))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("preintegrated rotation quaternion is invalid")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def _finite_symmetric_psd(matrix, *, reject_indefinite=False):
    value = np.asarray(matrix, dtype=float)
    if value.size == STATE_SIZE * STATE_SIZE:
        value = value.reshape(STATE_SIZE, STATE_SIZE)
    if value.shape != (STATE_SIZE, STATE_SIZE) or np.any(~np.isfinite(value)):
        raise ValueError("state covariance must be a finite 15x15 matrix")
    value = 0.5 * (value + value.T)
    eigenvalues, eigenvectors = np.linalg.eigh(value)
    scale = max(1.0, float(np.max(np.abs(eigenvalues))))
    if reject_indefinite and float(np.min(eigenvalues)) < -1.0e-9 * scale:
        raise ValueError("state covariance is not positive semidefinite")
    eigenvalues = np.maximum(eigenvalues, 0.0)
    projected = (eigenvectors * eigenvalues) @ eigenvectors.T
    return 0.5 * (projected + projected.T)


def make_optimization_anchor(
    stamp_s, state, covariance, generation, reset_counter=0
):
    """Validate and deep-copy one optimizer commit into an immutable anchor."""
    stamp_s = float(stamp_s)
    generation = int(generation)
    reset_counter = int(reset_counter)
    state = np.asarray(state, dtype=float)
    if not math.isfinite(stamp_s) or stamp_s <= 0.0:
        raise ValueError("optimization anchor timestamp must be positive")
    if generation <= 0:
        raise ValueError("optimization anchor generation must be positive")
    if reset_counter < 0:
        raise ValueError("optimization anchor reset counter must be nonnegative")
    if state.shape != (STATE_SIZE,) or np.any(~np.isfinite(state)):
        raise ValueError("optimization anchor state must be a finite 15-vector")
    covariance = _finite_symmetric_psd(covariance, reject_indefinite=True)
    return OptimizationAnchor(
        stamp_s,
        tuple(float(value) for value in state),
        tuple(float(value) for value in covariance.ravel()),
        generation,
        reset_counter,
    )


def live_propagation_admission(
    now_s,
    latest_imu_stamp_s,
    target_stamp_s,
    anchor_stamp_s,
    last_output_stamp_s,
    latest_lidar_activity_s,
    lidar_silence_timeout_s,
    maximum_output_age_s,
    minimum_output_interval_s,
    maximum_imu_age_s,
):
    """Gate publication-only propagation without advancing the factor graph."""
    required = (
        now_s,
        latest_imu_stamp_s,
        target_stamp_s,
        anchor_stamp_s,
        lidar_silence_timeout_s,
        maximum_output_age_s,
        minimum_output_interval_s,
        maximum_imu_age_s,
    )
    if any(value is None or not math.isfinite(float(value)) for value in required):
        return False, "invalid_time"
    now_s = float(now_s)
    latest_imu_stamp_s = float(latest_imu_stamp_s)
    target_stamp_s = float(target_stamp_s)
    anchor_stamp_s = float(anchor_stamp_s)
    if min(now_s, latest_imu_stamp_s, target_stamp_s, anchor_stamp_s) <= 0.0:
        return False, "clock_unavailable"
    if (
        float(lidar_silence_timeout_s) < 0.0
        or float(maximum_output_age_s) <= 0.0
        or float(minimum_output_interval_s) <= 0.0
        or float(maximum_imu_age_s) <= 0.0
    ):
        raise ValueError("live propagation timing limits are invalid")
    if latest_lidar_activity_s is None:
        return False, "lidar_activity_unavailable"
    latest_lidar_activity_s = float(latest_lidar_activity_s)
    if (
        not math.isfinite(latest_lidar_activity_s)
        or latest_lidar_activity_s > now_s
    ):
        return False, "lidar_clock_invalid"
    output_age_s = math.inf
    if last_output_stamp_s is not None:
        last_output_stamp_s = float(last_output_stamp_s)
        if not math.isfinite(last_output_stamp_s):
            return False, "last_output_invalid"
        output_age_s = now_s - last_output_stamp_s
    if (
        now_s - latest_lidar_activity_s <= float(lidar_silence_timeout_s)
        and output_age_s <= float(maximum_output_age_s)
    ):
        return False, "lidar_recent"
    imu_age_s = now_s - latest_imu_stamp_s
    if imu_age_s < -0.05:
        return False, "imu_from_future"
    if imu_age_s > float(maximum_imu_age_s):
        return False, "imu_stale"
    if target_stamp_s <= anchor_stamp_s + float(minimum_output_interval_s):
        return False, "imu_not_advanced"
    if last_output_stamp_s is not None:
        if target_stamp_s <= (
            last_output_stamp_s + float(minimum_output_interval_s)
        ):
            return False, "output_not_advanced"
    return True, "ready"


def auxiliary_keyframe_admission(
    now_s,
    latest_imu_stamp_s,
    last_state_stamp_s,
    latest_native_arrival_s,
    lidar_silence_timeout_s,
    minimum_state_interval_s,
    maximum_imu_age_s,
):
    """Admit a common-window state when IMU time advanced past the anchor.

    Native-factor arrival is deliberately not an admission condition. A
    degraded LiDAR may continue publishing trigger packets while contributing
    no accepted factor; those packets must not block healthy asynchronous
    factors from advancing the common window.
    """
    required = (
        now_s,
        latest_imu_stamp_s,
        last_state_stamp_s,
        lidar_silence_timeout_s,
        minimum_state_interval_s,
        maximum_imu_age_s,
    )
    if any(value is None or not math.isfinite(float(value)) for value in required):
        return False, "invalid_time"
    now_s = float(now_s)
    latest_imu_stamp_s = float(latest_imu_stamp_s)
    last_state_stamp_s = float(last_state_stamp_s)
    if min(now_s, latest_imu_stamp_s, last_state_stamp_s) <= 0.0:
        return False, "clock_unavailable"
    if (
        float(lidar_silence_timeout_s) < 0.0
        or float(minimum_state_interval_s) <= 0.0
        or float(maximum_imu_age_s) <= 0.0
    ):
        raise ValueError("auxiliary keyframe timing limits are invalid")
    imu_age_s = now_s - latest_imu_stamp_s
    if imu_age_s < -0.05:
        return False, "imu_from_future"
    if imu_age_s > float(maximum_imu_age_s):
        return False, "imu_stale"
    if latest_imu_stamp_s <= last_state_stamp_s + float(minimum_state_interval_s):
        return False, "imu_not_advanced"
    return True, "ready"


def backend_state_transition(state, measurement):
    """Return F in backend error order [p,dtheta,v,ba,bg]."""
    state = np.asarray(state, dtype=float)
    if state.shape != (STATE_SIZE,) or np.any(~np.isfinite(state)):
        raise ValueError("state transition requires a finite 15-vector")
    if not isinstance(measurement, ManifoldPreintegratedImu) or not measurement.valid:
        raise ValueError("state transition requires valid manifold preintegration")
    rotation = rpy_to_rotation_matrix(state[3:6])
    delta_rotation = _quaternion_wxyz_to_rotation(measurement.delta_quaternion)
    delta_position = np.asarray(measurement.delta_position, dtype=float)
    delta_velocity = np.asarray(measurement.delta_velocity, dtype=float)
    position_accel = np.asarray(
        measurement.jacobian_delta_position_accel_bias, dtype=float
    ).reshape(3, 3)
    position_gyro = np.asarray(
        measurement.jacobian_delta_position_gyro_bias, dtype=float
    ).reshape(3, 3)
    velocity_accel = np.asarray(
        measurement.jacobian_delta_velocity_accel_bias, dtype=float
    ).reshape(3, 3)
    velocity_gyro = np.asarray(
        measurement.jacobian_delta_velocity_gyro_bias, dtype=float
    ).reshape(3, 3)
    rotation_gyro = np.asarray(
        measurement.jacobian_delta_rotation_gyro_bias, dtype=float
    ).reshape(3, 3)

    transition = np.eye(STATE_SIZE, dtype=float)
    transition[0:3, 3:6] = -rotation @ _skew(delta_position)
    transition[0:3, 6:9] = np.eye(3, dtype=float) * float(measurement.dt_s)
    transition[0:3, 9:12] = rotation @ position_accel
    transition[0:3, 12:15] = rotation @ position_gyro
    transition[3:6, 3:6] = delta_rotation.T
    transition[3:6, 12:15] = rotation_gyro
    transition[6:9, 3:6] = -rotation @ _skew(delta_velocity)
    transition[6:9, 9:12] = rotation @ velocity_accel
    transition[6:9, 12:15] = rotation @ velocity_gyro
    return transition


def backend_process_covariance(state, measurement):
    """Map Q from [dp,dv,dtheta,dba,dbg] to [p,dtheta,v,ba,bg]."""
    state = np.asarray(state, dtype=float)
    if state.shape != (STATE_SIZE,) or np.any(~np.isfinite(state)):
        raise ValueError("process covariance requires a finite 15-vector")
    if not isinstance(measurement, ManifoldPreintegratedImu) or not measurement.valid:
        raise ValueError("process covariance requires valid manifold preintegration")
    preintegrated = _finite_symmetric_psd(
        np.asarray(measurement.covariance, dtype=float).reshape(STATE_SIZE, STATE_SIZE),
        reject_indefinite=True,
    )
    rotation = rpy_to_rotation_matrix(state[3:6])
    ordering = np.zeros((STATE_SIZE, STATE_SIZE), dtype=float)
    ordering[0:3, 0:3] = rotation
    ordering[3:6, 6:9] = np.eye(3, dtype=float)
    ordering[6:9, 3:6] = rotation
    ordering[9:12, 9:12] = np.eye(3, dtype=float)
    ordering[12:15, 12:15] = np.eye(3, dtype=float)
    return _finite_symmetric_psd(ordering @ preintegrated @ ordering.T)


def state_covariance_to_odometry_covariances(state, covariance):
    """Map the backend right-local error covariance into ROS odometry axes."""
    state = np.asarray(state, dtype=float)
    covariance = _finite_symmetric_psd(covariance)
    if state.shape != (STATE_SIZE,) or np.any(~np.isfinite(state)):
        raise ValueError("odometry covariance mapping requires a finite state")
    rotation = rpy_to_rotation_matrix(state[3:6])
    pose_jacobian = np.zeros((6, STATE_SIZE), dtype=float)
    pose_jacobian[0:3, 0:3] = np.eye(3, dtype=float)
    # The backend uses a body/right-local rotation perturbation while ROS pose
    # covariance uses fixed axes in the odometry header frame.
    pose_jacobian[3:6, 3:6] = rotation
    pose_covariance = pose_jacobian @ covariance @ pose_jacobian.T

    body_velocity = rotation.T @ state[6:9]
    velocity_jacobian = np.zeros((3, STATE_SIZE), dtype=float)
    velocity_jacobian[:, 3:6] = _skew(body_velocity)
    velocity_jacobian[:, 6:9] = rotation.T
    velocity_covariance = velocity_jacobian @ covariance @ velocity_jacobian.T
    return (
        0.5 * (pose_covariance + pose_covariance.T),
        0.5 * (velocity_covariance + velocity_covariance.T),
    )


def propagate_optimization_anchor(anchor, target_stamp_s, measurement):
    """Apply x'=f(x,u), P'=F P F^T+Q without creating a graph state."""
    if not isinstance(anchor, OptimizationAnchor):
        raise ValueError("live propagation requires an optimization anchor")
    target_stamp_s = float(target_stamp_s)
    if (
        not math.isfinite(target_stamp_s)
        or target_stamp_s <= anchor.stamp_s
        or abs(
            float(measurement.dt_s) - (target_stamp_s - anchor.stamp_s)
        ) > 1.0e-6
    ):
        raise ValueError("live propagation interval does not match preintegration")
    state = np.asarray(anchor.state, dtype=float)
    accel_bias = np.asarray(measurement.accel_bias_linearization, dtype=float)
    gyro_bias = np.asarray(measurement.gyro_bias_linearization, dtype=float)
    if (
        not np.allclose(accel_bias, state[9:12], rtol=0.0, atol=1.0e-9)
        or not np.allclose(gyro_bias, state[12:15], rtol=0.0, atol=1.0e-9)
    ):
        raise ValueError("live preintegration bias does not match its anchor")
    propagated = propagate_state(state, measurement)
    transition = backend_state_transition(state, measurement)
    process = backend_process_covariance(state, measurement)
    anchor_covariance = np.asarray(anchor.covariance, dtype=float).reshape(
        STATE_SIZE, STATE_SIZE
    )
    covariance = _finite_symmetric_psd(
        transition @ anchor_covariance @ transition.T + process
    )
    return PropagatedState(
        target_stamp_s,
        tuple(float(value) for value in propagated),
        tuple(float(value) for value in covariance.ravel()),
        int(anchor.generation),
        int(anchor.reset_counter),
    )
