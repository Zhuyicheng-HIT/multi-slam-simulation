"""Nonlinear SO(3) fixed-lag backend for the Ultra-Fusion reproduction."""

from collections import defaultdict, deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Sequence

import numpy as np

try:
    from uf_backend_core_cpp import (
        imu_preintegrated_cost as cpp_imu_preintegrated_cost,
        imu_preintegrated_normal as cpp_imu_preintegrated_normal,
        lidar_point_plane_cost as cpp_lidar_point_plane_cost,
        lidar_point_plane_normal as cpp_lidar_point_plane_normal,
        lidar_point_plane_normal_subspace as cpp_lidar_point_plane_normal_subspace,
        marginal_prior_cost as cpp_marginal_prior_cost,
        marginal_prior_normal as cpp_marginal_prior_normal,
        state_plus_batch as cpp_state_plus_batch,
        visual_reprojection_cost as cpp_visual_reprojection_cost,
        visual_reprojection_normal as cpp_visual_reprojection_normal,
    )
    CPP_MATH_CORE_AVAILABLE = True
except ImportError:
    cpp_imu_preintegrated_cost = None
    cpp_imu_preintegrated_normal = None
    cpp_lidar_point_plane_cost = None
    cpp_lidar_point_plane_normal = None
    cpp_lidar_point_plane_normal_subspace = None
    cpp_marginal_prior_cost = None
    cpp_marginal_prior_normal = None
    cpp_state_plus_batch = None
    cpp_visual_reprojection_cost = None
    cpp_visual_reprojection_normal = None
    CPP_MATH_CORE_AVAILABLE = False

try:
    from uf_backend_core_cpp import (
        lidar_point_plane_normal_axis_scaled as cpp_lidar_point_plane_normal_axis_scaled,
    )
    CPP_LIDAR_AXIS_SCALED_CORE_AVAILABLE = callable(
        cpp_lidar_point_plane_normal_axis_scaled
    )
except ImportError:
    cpp_lidar_point_plane_normal_axis_scaled = None
    CPP_LIDAR_AXIS_SCALED_CORE_AVAILABLE = False

try:
    from uf_backend_core_cpp import (
        imu_preintegrated_graph_normal as cpp_imu_preintegrated_graph_normal,
    )
    CPP_IMU_GRAPH_CORE_AVAILABLE = True
except ImportError:
    cpp_imu_preintegrated_graph_normal = None
    CPP_IMU_GRAPH_CORE_AVAILABLE = False

try:
    from uf_backend_core_cpp import (
        lidar_point_plane_graph_normal as cpp_lidar_point_plane_graph_normal,
    )
    CPP_LIDAR_GRAPH_CORE_AVAILABLE = True
except ImportError:
    cpp_lidar_point_plane_graph_normal = None
    CPP_LIDAR_GRAPH_CORE_AVAILABLE = False

try:
    from uf_backend_core_cpp import (
        lidar_point_plane_graph_normal_axis_scaled as cpp_lidar_point_plane_graph_normal_axis_scaled,
    )
    CPP_LIDAR_AXIS_SCALED_GRAPH_CORE_AVAILABLE = callable(
        cpp_lidar_point_plane_graph_normal_axis_scaled
    )
except ImportError:
    cpp_lidar_point_plane_graph_normal_axis_scaled = None
    CPP_LIDAR_AXIS_SCALED_GRAPH_CORE_AVAILABLE = False

try:
    from uf_backend_core_cpp import (
        rgbd_depth_cost as cpp_rgbd_depth_cost,
        rgbd_depth_normal as cpp_rgbd_depth_normal,
        rgbd_direct_cost as cpp_rgbd_direct_cost,
        rgbd_direct_normal as cpp_rgbd_direct_normal,
    )
    CPP_RGBD_DEPTH_CORE_AVAILABLE = True
except ImportError:
    cpp_rgbd_depth_cost = None
    cpp_rgbd_depth_normal = None
    cpp_rgbd_direct_cost = None
    cpp_rgbd_direct_normal = None
    CPP_RGBD_DEPTH_CORE_AVAILABLE = False

from .imu_preintegration import ManifoldPreintegratedImu
from .manifold import (
    STATE_SIZE,
    numerical_state_jacobian,
    skew,
    so3_exp,
    so3_left_jacobian_inverse,
    so3_log,
    so3_right_jacobian,
    so3_right_jacobian_inverse,
    state_local,
    state_plus,
)
from .native_lidar import (
    NativeLidarPoseNormal,
    point_plane_residual,
    point_plane_residual_jacobian,
    rpy_to_rotation_matrix,
)
from .range_facet import (
    RangeFacetObservation,
    range_facet_prediction_jacobian,
)
from .window import FactorRecord, FactorResidual, _scheduler_values
from .visual_reprojection import (
    RgbdDepthTrackBatch,
    RgbdDirectTrackBatch,
    VisualTrackBatch,
    rgbd_depth_residual_jacobians,
    rgbd_direct_residual_jacobians,
    visual_reprojection_residual_jacobians,
)


POSITION = slice(0, 3)
ROTATION = slice(3, 6)
VELOCITY = slice(6, 9)
ACCEL_BIAS = slice(9, 12)
GYRO_BIAS = slice(12, 15)


@dataclass
class ManifoldBackendSnapshot:
    states: list
    factors: list
    last_initial_cost: float
    last_cost: float
    last_iterations: int
    last_solve_ms: float
    last_rejected_steps: int
    last_hessian: object
    last_marginalization_ms: float
    lm_damping: float


def _positive_diagonal(covariance, dimension):
    values = np.asarray(covariance, dtype=float)
    if values.ndim == 0:
        values = np.full(dimension, float(values))
    elif values.ndim == 2:
        values = np.diag(values)
    if values.shape != (dimension,):
        raise ValueError("factor covariance has the wrong dimension")
    if np.any(~np.isfinite(values)) or np.any(values <= 0.0):
        raise ValueError("factor covariance must be finite and positive")
    return values


def _positive_covariance(covariance, dimension):
    values = np.asarray(covariance, dtype=float)
    if values.ndim == 0:
        values = np.eye(dimension, dtype=float) * float(values)
    elif values.ndim == 1 and values.shape == (dimension,):
        values = np.diag(values)
    elif values.ndim == 1 and values.size == dimension * dimension:
        values = values.reshape(dimension, dimension)
    if values.shape != (dimension, dimension):
        raise ValueError("factor covariance has the wrong dimension")
    if np.any(~np.isfinite(values)):
        raise ValueError("factor covariance must be finite")
    values = 0.5 * (values + values.T)
    try:
        np.linalg.cholesky(values)
    except np.linalg.LinAlgError as error:
        raise ValueError(
            "factor covariance must be positive definite") from error
    return values


def _covariance_information(covariance):
    covariance = np.asarray(covariance, dtype=float)
    cholesky = np.linalg.cholesky(covariance)
    inverse_cholesky = np.linalg.solve(cholesky, np.eye(covariance.shape[0]))
    return inverse_cholesky.T @ inverse_cholesky


def lidar_translation_subspace_normal(hessian, gradient, scale):
    """Reweight the rotation-conditioned translation normal subspace."""
    hessian = np.asarray(hessian, dtype=float)
    gradient = np.asarray(gradient, dtype=float)
    scale = np.asarray(scale, dtype=float)
    if hessian.shape != (6, 6) or gradient.shape != (6,):
        raise ValueError("LiDAR pose normal must be 6-dimensional")
    if scale.shape != (3, 3) or np.any(~np.isfinite(scale)):
        raise ValueError("LiDAR subspace scale must be a finite 3x3 matrix")
    scale = 0.5 * (scale + scale.T)
    values, vectors = np.linalg.eigh(scale)
    if np.any(values < -1.0e-9) or np.any(values > 1.0 + 1.0e-9):
        raise ValueError("LiDAR subspace scale must be PSD and bounded")
    root = (vectors * np.sqrt(np.clip(values, 0.0, 1.0))) @ vectors.T
    translation = slice(0, 3)
    rotation = slice(3, 6)
    h_rr = 0.5 * (
        hessian[rotation, rotation] + hessian[rotation, rotation].T
    )
    coupling = hessian[translation, rotation]
    conditional_map = coupling @ np.linalg.pinv(h_rr, rcond=1.0e-12)
    schur = hessian[translation, translation] - conditional_map @ coupling.T
    schur = 0.5 * (schur + schur.T)
    conditional_gradient = (
        gradient[translation] - conditional_map @ gradient[rotation]
    )
    result_hessian = hessian.copy()
    result_gradient = gradient.copy()
    result_hessian[translation, translation] = (
        root @ schur @ root + conditional_map @ h_rr @ conditional_map.T
    )
    result_gradient[translation] = (
        scale @ conditional_gradient
        + conditional_map @ gradient[rotation]
    )
    return 0.5 * (result_hessian + result_hessian.T), result_gradient


def huber_loss_and_weight(standardized_residual, delta):
    """Return Huber loss and IRLS weight in measurement-sigma units."""
    residual = np.asarray(standardized_residual, dtype=float)
    delta = float(delta)
    if np.any(~np.isfinite(residual)) or not math.isfinite(
            delta) or delta < 0.0:
        raise ValueError("Huber residual and threshold must be finite")
    absolute = np.abs(residual)
    if delta == 0.0:
        return 0.5 * residual ** 2, np.ones_like(residual)
    quadratic = absolute <= delta
    loss = np.where(
        quadratic,
        0.5 * residual ** 2,
        delta * (absolute - 0.5 * delta),
    )
    weight = np.ones_like(residual)
    weight[~quadratic] = delta / absolute[~quadratic]
    return loss, weight


def _quaternion_wxyz_to_rotation(quaternion):
    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm([w, x, y, z]))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("preintegrated quaternion must have positive norm")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def propagate_state(
    state: Sequence[float],
    measurement: ManifoldPreintegratedImu,
    gravity: Sequence[float] = (0.0, 0.0, -9.81),
) -> np.ndarray:
    state = np.asarray(state, dtype=float)
    if state.shape != (STATE_SIZE,) or not measurement.valid:
        raise ValueError(
            "state propagation requires a valid state and IMU interval")
    gravity = np.asarray(gravity, dtype=float)
    rotation = rpy_to_rotation_matrix(state[ROTATION])
    dt_s = float(measurement.dt_s)
    delta_position = np.asarray(measurement.delta_position, dtype=float)
    delta_velocity = np.asarray(measurement.delta_velocity, dtype=float)
    delta_rotation = _quaternion_wxyz_to_rotation(measurement.delta_quaternion)
    propagated = state.copy()
    propagated[POSITION] = (
        state[POSITION]
        + state[VELOCITY] * dt_s
        + 0.5 * gravity * dt_s * dt_s
        + rotation @ delta_position
    )
    propagated[VELOCITY] = (
        state[VELOCITY] + gravity * dt_s + rotation @ delta_velocity
    )
    rotation_increment = so3_log(delta_rotation)
    propagated = state_plus(
        propagated,
        np.concatenate((np.zeros(3), rotation_increment, np.zeros(9))),
    )
    return propagated


def imu_residual(states, previous, current, measurement, gravity):
    state_i = states[previous]
    state_j = states[current]
    rotation_i = rpy_to_rotation_matrix(state_i[ROTATION])
    rotation_j = rpy_to_rotation_matrix(state_j[ROTATION])
    dt_s = float(measurement.dt_s)
    gravity = np.asarray(gravity, dtype=float)
    delta_position = np.asarray(measurement.delta_position, dtype=float)
    delta_velocity = np.asarray(measurement.delta_velocity, dtype=float)
    delta_rotation = _quaternion_wxyz_to_rotation(measurement.delta_quaternion)
    accel_delta = state_i[ACCEL_BIAS] - np.asarray(
        measurement.accel_bias_linearization, dtype=float
    )
    gyro_delta = state_i[GYRO_BIAS] - np.asarray(
        measurement.gyro_bias_linearization, dtype=float
    )
    position_accel = np.asarray(
        measurement.jacobian_delta_position_accel_bias
    ).reshape(3, 3)
    position_gyro = np.asarray(
        measurement.jacobian_delta_position_gyro_bias
    ).reshape(3, 3)
    velocity_accel = np.asarray(
        measurement.jacobian_delta_velocity_accel_bias
    ).reshape(3, 3)
    velocity_gyro = np.asarray(
        measurement.jacobian_delta_velocity_gyro_bias
    ).reshape(3, 3)
    rotation_gyro = np.asarray(
        measurement.jacobian_delta_rotation_gyro_bias
    ).reshape(3, 3)
    corrected_position = (
        delta_position +
        position_accel @ accel_delta +
        position_gyro @ gyro_delta)
    corrected_velocity = (
        delta_velocity +
        velocity_accel @ accel_delta +
        velocity_gyro @ gyro_delta)
    corrected_rotation = delta_rotation @ so3_exp(rotation_gyro @ gyro_delta)
    residual_position = (
        rotation_i.T
        @ (
            state_j[POSITION]
            - state_i[POSITION]
            - state_i[VELOCITY] * dt_s
            - 0.5 * gravity * dt_s * dt_s
        )
        - corrected_position
    )
    residual_velocity = (
        rotation_i.T
        @ (state_j[VELOCITY] - state_i[VELOCITY] - gravity * dt_s)
        - corrected_velocity
    )
    residual_rotation = so3_log(
        corrected_rotation.T @ rotation_i.T @ rotation_j
    )
    return np.concatenate((
        residual_position,
        residual_velocity,
        residual_rotation,
        state_j[ACCEL_BIAS] - state_i[ACCEL_BIAS],
        state_j[GYRO_BIAS] - state_i[GYRO_BIAS],
    ))


def imu_residual_jacobians(states, previous, current, measurement, gravity):
    """Analytic right-local Jacobians for the standard IMU preintegration factor."""
    state_i = states[previous]
    state_j = states[current]
    rotation_i = rpy_to_rotation_matrix(state_i[ROTATION])
    rotation_j = rpy_to_rotation_matrix(state_j[ROTATION])
    rotation_i_transpose = rotation_i.T
    dt_s = float(measurement.dt_s)
    gravity = np.asarray(gravity, dtype=float)
    delta_position = np.asarray(measurement.delta_position, dtype=float)
    delta_velocity = np.asarray(measurement.delta_velocity, dtype=float)
    delta_rotation = _quaternion_wxyz_to_rotation(measurement.delta_quaternion)
    accel_delta = state_i[ACCEL_BIAS] - np.asarray(
        measurement.accel_bias_linearization, dtype=float
    )
    gyro_delta = state_i[GYRO_BIAS] - np.asarray(
        measurement.gyro_bias_linearization, dtype=float
    )
    position_accel = np.asarray(
        measurement.jacobian_delta_position_accel_bias
    ).reshape(3, 3)
    position_gyro = np.asarray(
        measurement.jacobian_delta_position_gyro_bias
    ).reshape(3, 3)
    velocity_accel = np.asarray(
        measurement.jacobian_delta_velocity_accel_bias
    ).reshape(3, 3)
    velocity_gyro = np.asarray(
        measurement.jacobian_delta_velocity_gyro_bias
    ).reshape(3, 3)
    rotation_gyro = np.asarray(
        measurement.jacobian_delta_rotation_gyro_bias
    ).reshape(3, 3)
    corrected_position = (
        delta_position +
        position_accel @ accel_delta +
        position_gyro @ gyro_delta)
    corrected_velocity = (
        delta_velocity +
        velocity_accel @ accel_delta +
        velocity_gyro @ gyro_delta)
    rotation_bias_vector = rotation_gyro @ gyro_delta
    corrected_rotation = delta_rotation @ so3_exp(rotation_bias_vector)
    position_delta_world = (
        state_j[POSITION]
        - state_i[POSITION]
        - state_i[VELOCITY] * dt_s
        - 0.5 * gravity * dt_s * dt_s
    )
    velocity_delta_world = (
        state_j[VELOCITY] - state_i[VELOCITY] - gravity * dt_s
    )
    position_delta_body = rotation_i_transpose @ position_delta_world
    velocity_delta_body = rotation_i_transpose @ velocity_delta_world
    rotation_residual = so3_log(
        corrected_rotation.T @ rotation_i_transpose @ rotation_j
    )
    residual = np.concatenate((
        position_delta_body - corrected_position,
        velocity_delta_body - corrected_velocity,
        rotation_residual,
        state_j[ACCEL_BIAS] - state_i[ACCEL_BIAS],
        state_j[GYRO_BIAS] - state_i[GYRO_BIAS],
    ))

    jacobian_i = np.zeros((15, STATE_SIZE), dtype=float)
    jacobian_j = np.zeros((15, STATE_SIZE), dtype=float)
    jacobian_i[0:3, POSITION] = -rotation_i_transpose
    jacobian_i[0:3, ROTATION] = skew(position_delta_body)
    jacobian_i[0:3, VELOCITY] = -rotation_i_transpose * dt_s
    jacobian_i[0:3, ACCEL_BIAS] = -position_accel
    jacobian_i[0:3, GYRO_BIAS] = -position_gyro
    jacobian_j[0:3, POSITION] = rotation_i_transpose

    jacobian_i[3:6, ROTATION] = skew(velocity_delta_body)
    jacobian_i[3:6, VELOCITY] = -rotation_i_transpose
    jacobian_i[3:6, ACCEL_BIAS] = -velocity_accel
    jacobian_i[3:6, GYRO_BIAS] = -velocity_gyro
    jacobian_j[3:6, VELOCITY] = rotation_i_transpose

    left_inverse = so3_left_jacobian_inverse(rotation_residual)
    right_inverse = so3_right_jacobian_inverse(rotation_residual)
    jacobian_i[6:9, ROTATION] = -left_inverse @ corrected_rotation.T
    jacobian_i[6:9, GYRO_BIAS] = (
        -left_inverse
        @ so3_right_jacobian(rotation_bias_vector)
        @ rotation_gyro
    )
    jacobian_j[6:9, ROTATION] = right_inverse

    jacobian_i[9:12, ACCEL_BIAS] = -np.eye(3)
    jacobian_j[9:12, ACCEL_BIAS] = np.eye(3)
    jacobian_i[12:15, GYRO_BIAS] = -np.eye(3)
    jacobian_j[12:15, GYRO_BIAS] = np.eye(3)
    return residual, jacobian_i, jacobian_j


def _cpp_imu_factor_arguments(factor, states, gravity):
    previous, current = factor["indices"]
    measurement = factor["measurement"]
    return (
        states[previous],
        states[current],
        gravity,
        float(measurement.dt_s),
        np.asarray(measurement.delta_position, dtype=float),
        np.asarray(measurement.delta_velocity, dtype=float),
        np.asarray(measurement.delta_quaternion, dtype=float),
        np.asarray(measurement.accel_bias_linearization, dtype=float),
        np.asarray(measurement.gyro_bias_linearization, dtype=float),
        np.asarray(
            measurement.jacobian_delta_position_accel_bias,
            dtype=float,
        ).reshape(3, 3),
        np.asarray(
            measurement.jacobian_delta_position_gyro_bias,
            dtype=float,
        ).reshape(3, 3),
        np.asarray(
            measurement.jacobian_delta_velocity_accel_bias,
            dtype=float,
        ).reshape(3, 3),
        np.asarray(
            measurement.jacobian_delta_velocity_gyro_bias,
            dtype=float,
        ).reshape(3, 3),
        np.asarray(
            measurement.jacobian_delta_rotation_gyro_bias,
            dtype=float,
        ).reshape(3, 3),
        factor["information_matrix"],
        factor["effective_weight"],
    )


def _cpp_imu_graph_arguments(factors, states, gravity):
    measurements = [factor["measurement"] for factor in factors]
    return (
        np.asarray(states, dtype=float),
        np.asarray([factor["indices"] for factor in factors], dtype=np.int32),
        gravity,
        np.asarray([measurement.dt_s for measurement in measurements]),
        np.asarray([measurement.delta_position for measurement in measurements]),
        np.asarray([measurement.delta_velocity for measurement in measurements]),
        np.asarray([measurement.delta_quaternion for measurement in measurements]),
        np.asarray([
            measurement.accel_bias_linearization
            for measurement in measurements
        ]),
        np.asarray([
            measurement.gyro_bias_linearization
            for measurement in measurements
        ]),
        np.asarray([
            measurement.jacobian_delta_position_accel_bias
            for measurement in measurements
        ]).reshape((-1, 9)),
        np.asarray([
            measurement.jacobian_delta_position_gyro_bias
            for measurement in measurements
        ]).reshape((-1, 9)),
        np.asarray([
            measurement.jacobian_delta_velocity_accel_bias
            for measurement in measurements
        ]).reshape((-1, 9)),
        np.asarray([
            measurement.jacobian_delta_velocity_gyro_bias
            for measurement in measurements
        ]).reshape((-1, 9)),
        np.asarray([
            measurement.jacobian_delta_rotation_gyro_bias
            for measurement in measurements
        ]).reshape((-1, 9)),
        np.asarray([
            factor["information_matrix"] for factor in factors
        ]).reshape((-1, STATE_SIZE * STATE_SIZE)),
        np.asarray([factor["effective_weight"] for factor in factors]),
    )


def _cpp_lidar_graph_arguments(factors, states):
    measurements = [factor["measurement"] for factor in factors]
    counts = np.asarray(
        [measurement.lidar_points.shape[0] for measurement in measurements],
        dtype=np.int32,
    )
    offsets = np.r_[0, np.cumsum(counts, dtype=np.int32)]
    return (
        np.asarray(states, dtype=float),
        np.asarray([factor["indices"][0] for factor in factors], dtype=np.int32),
        offsets,
        np.concatenate([
            measurement.lidar_points for measurement in measurements
        ]),
        np.concatenate([
            measurement.plane_normals for measurement in measurements
        ]),
        np.concatenate([
            measurement.plane_points for measurement in measurements
        ]),
        np.asarray([
            measurement.lidar_to_body_rotation for measurement in measurements
        ]).reshape((-1, 9)),
        np.asarray([
            measurement.lidar_to_body_translation for measurement in measurements
        ]),
        np.concatenate([factor["variance"] for factor in factors]),
        np.asarray([factor["effective_weight"] for factor in factors]),
    )


def _cpp_lidar_axis_scaled_graph_arguments(factors, states):
    arguments = _cpp_lidar_graph_arguments(factors, states)
    return (
        *arguments[:8],
        np.asarray([
            factor["translation_information_scale"] for factor in factors
        ], dtype=float),
        *arguments[8:],
    )


def _cpp_visual_factor_arguments(factor, states):
    previous, current = factor["indices"]
    tracks = factor["measurement"]
    return (
        states[previous],
        states[current],
        tracks.anchor_normalized,
        tracks.current_normalized,
        tracks.inverse_depth,
        factor["variance"],
        tracks.rotation_body_camera,
        tracks.translation_body_camera,
        factor["effective_weight"],
        2.5,
        1.0e-4,
    )


def _cpp_rgbd_depth_factor_arguments(factor, states):
    previous, current = factor["indices"]
    tracks = factor["measurement"]
    return (
        states[previous],
        states[current],
        tracks.anchor_normalized,
        tracks.anchor_depth_m,
        tracks.current_depth_m,
        factor["variance"],
        tracks.rotation_body_camera,
        tracks.translation_body_camera,
        factor["effective_weight"],
        2.5,
        1.0e-4,
    )


def _cpp_rgbd_direct_factor_arguments(factor, states):
    previous, current = factor["indices"]
    tracks = factor["measurement"]
    return (
        states[previous],
        states[current],
        tracks.anchor_normalized,
        tracks.current_normalized,
        tracks.anchor_depth_m,
        tracks.current_depth_m,
        tracks.depth_variance_m2,
        tracks.previous_intensity,
        tracks.current_intensity,
        tracks.current_gradient_normalized,
        tracks.photometric_variance,
        tracks.rotation_body_camera,
        tracks.translation_body_camera,
        factor["effective_weight"],
        2.5,
        1.0e-4,
    )


class ManifoldSlidingWindowBackend:
    """Gauss-Newton fixed-lag smoother with right-local SO(3) increments."""

    def __init__(
        self,
        max_states=8,
        damping=1.0e-6,
        max_iterations=4,
        convergence_threshold=1.0e-5,
        gravity=(0.0, 0.0, -9.81),
        lidar_huber_delta=2.5,
        lm_max_trials=6,
        lm_damping_up=10.0,
        lm_damping_down=0.3,
        lm_min_damping=1.0e-12,
        lm_max_damping=1.0e12,
        marginal_rank_tolerance=1.0e-9,
        cpp_math_core_enabled=True,
        profiling_enabled=False,
        profiling_capacity=4096,
    ):
        if max_states < 2:
            raise ValueError("manifold window requires at least two states")
        if damping <= 0.0 or max_iterations < 1 or convergence_threshold <= 0.0:
            raise ValueError("invalid nonlinear solver configuration")
        if (
            not math.isfinite(lidar_huber_delta)
            or lidar_huber_delta < 0.0
            or lm_max_trials < 1
            or not all(math.isfinite(value) for value in (
                lm_damping_up, lm_damping_down,
                lm_min_damping, lm_max_damping,
            ))
            or lm_damping_up <= 1.0
            or not 0.0 < lm_damping_down < 1.0
            or lm_min_damping <= 0.0
            or lm_max_damping < lm_min_damping
            or marginal_rank_tolerance <= 0.0
        ):
            raise ValueError("invalid robust solver configuration")
        self.max_states = int(max_states)
        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.convergence_threshold = float(convergence_threshold)
        self.lidar_huber_delta = float(lidar_huber_delta)
        self.lm_max_trials = int(lm_max_trials)
        self.lm_damping_up = float(lm_damping_up)
        self.lm_damping_down = float(lm_damping_down)
        self.lm_min_damping = float(lm_min_damping)
        self.lm_max_damping = float(lm_max_damping)
        # This tolerance is used only when forming a marginal prior.  The
        # Levenberg-Marquardt damping is deliberately excluded from that
        # prior: damping is a solver device, not an observation.
        self.marginal_rank_tolerance = float(marginal_rank_tolerance)
        self.cpp_math_core_requested = bool(cpp_math_core_enabled)
        self.cpp_math_core_enabled = bool(
            self.cpp_math_core_requested and CPP_MATH_CORE_AVAILABLE
        )
        self._lm_damping = min(
            self.lm_max_damping, max(self.lm_min_damping, self.damping)
        )
        self.gravity = np.asarray(gravity, dtype=float)
        if self.gravity.shape != (3,) or np.any(~np.isfinite(self.gravity)):
            raise ValueError("gravity must be a finite 3-vector")
        self._states = []
        self._factors = []
        self._last_initial_cost = 0.0
        self._last_cost = 0.0
        self._last_iterations = 0
        self._last_iteration_budget = int(max_iterations)
        self._last_solve_ms = 0.0
        self._last_rejected_steps = 0
        self._last_hessian = None
        self._last_marginalization_ms = 0.0
        self.last_marginal_prior_diagnostic = {}
        self.profiling_enabled = bool(profiling_enabled)
        self.profiling_capacity = max(64, int(profiling_capacity))
        self._profile_samples = defaultdict(
            lambda: deque(maxlen=self.profiling_capacity)
        )
        self._profile_samples_lock = threading.Lock()
        self._profile_cycle = None
        self._profile_cycle_counts = None
        self._last_profile_cycle = {}

    def begin_profile_cycle(self):
        """Start one transaction-scoped profile without changing solver work."""
        if not self.profiling_enabled:
            return
        self._profile_cycle = defaultdict(float)
        self._profile_cycle_counts = defaultdict(int)
        self._profile_cycle["marginalization_happened"] = False

    def finish_profile_cycle(self):
        """Return and retain the current transaction profile."""
        if not self.profiling_enabled or self._profile_cycle is None:
            return {}
        result = dict(self._profile_cycle)
        result["stage_call_counts"] = dict(self._profile_cycle_counts)
        self._last_profile_cycle = result
        self._profile_cycle = None
        self._profile_cycle_counts = None
        return dict(result)

    @property
    def last_profile_cycle(self):
        return dict(self._last_profile_cycle)

    def _profile_start(self):
        return time.perf_counter_ns() if self.profiling_enabled else None

    def _profile_stop(self, name, started_ns):
        if started_ns is None:
            return 0.0
        elapsed_ms = (time.perf_counter_ns() - started_ns) * 1.0e-6
        with self._profile_samples_lock:
            self._profile_samples[str(name)].append(elapsed_ms)
        if self._profile_cycle is not None:
            self._profile_cycle[str(name)] += elapsed_ms
            self._profile_cycle_counts[str(name)] += 1
        return elapsed_ms

    def profile_summary(self):
        """Return bounded wall-time percentiles for opt-in runtime profiling."""
        if not self.profiling_enabled:
            return {}
        with self._profile_samples_lock:
            snapshot = {
                name: tuple(samples)
                for name, samples in self._profile_samples.items()
            }
        summary = {}
        for name, samples in snapshot.items():
            if not samples:
                continue
            values = np.fromiter(samples, dtype=float)
            summary[name] = {
                "count": int(values.size),
                "p50_ms": float(np.percentile(values, 50)),
                "p90_ms": float(np.percentile(values, 90)),
                "p95_ms": float(np.percentile(values, 95)),
                "max_ms": float(np.max(values)),
            }
        return summary

    @property
    def state_count(self):
        return len(self._states)

    @property
    def factor_count(self):
        return len(self._factors)

    def state(self, index):
        return self._states[index].copy()

    def states(self):
        return [state.copy() for state in self._states]

    def snapshot(self):
        """Capture all mutable window state before a transactional update."""
        return ManifoldBackendSnapshot(
            states=[state.copy() for state in self._states],
            # Factor payload arrays and measurement objects are immutable once
            # admitted.  A transaction only replaces dictionary fields such
            # as indices/measurement; it never edits those payloads in place.
            # Copy dictionaries to isolate those replacements without cloning
            # every LiDAR correspondence and dense marginal prior each frame.
            factors=[dict(factor) for factor in self._factors],
            last_initial_cost=float(self._last_initial_cost),
            last_cost=float(self._last_cost),
            last_iterations=int(self._last_iterations),
            last_solve_ms=float(self._last_solve_ms),
            last_rejected_steps=int(self._last_rejected_steps),
            last_hessian=(
                None if self._last_hessian is None else self._last_hessian.copy()
            ),
            last_marginalization_ms=float(self._last_marginalization_ms),
            lm_damping=float(self._lm_damping),
        )

    def restore(self, snapshot):
        """Restore a snapshot after an invalid solve or failed publication."""
        if not isinstance(snapshot, ManifoldBackendSnapshot):
            raise TypeError("snapshot has the wrong backend type")
        self._states = [state.copy() for state in snapshot.states]
        self._factors = [dict(factor) for factor in snapshot.factors]
        self._last_initial_cost = float(snapshot.last_initial_cost)
        self._last_cost = float(snapshot.last_cost)
        self._last_iterations = int(snapshot.last_iterations)
        self._last_solve_ms = float(snapshot.last_solve_ms)
        self._last_rejected_steps = int(snapshot.last_rejected_steps)
        self._last_hessian = (
            None if snapshot.last_hessian is None else snapshot.last_hessian.copy())
        self._last_marginalization_ms = float(
            snapshot.last_marginalization_ms
        )
        self._lm_damping = float(snapshot.lm_damping)

    def latest_state_information(self):
        """Return the undamped information block for the newest state."""
        if not self._states:
            raise IndexError("cannot inspect an empty window")
        hessian = self._last_hessian
        expected = len(self._states) * STATE_SIZE
        if hessian is None or hessian.shape != (expected, expected):
            hessian, _, _ = self._normal()
        block = np.asarray(hessian[-STATE_SIZE:, -STATE_SIZE:], dtype=float)
        return 0.5 * (block + block.T)

    @property
    def last_initial_cost(self):
        return self._last_initial_cost

    @property
    def last_cost(self):
        return self._last_cost

    @property
    def last_iterations(self):
        return self._last_iterations

    @property
    def last_iteration_budget(self):
        return self._last_iteration_budget

    @property
    def last_solve_ms(self):
        return self._last_solve_ms

    @property
    def last_rejected_steps(self):
        return self._last_rejected_steps

    @property
    def last_damping(self):
        return self._lm_damping

    @property
    def last_marginalization_ms(self):
        return self._last_marginalization_ms

    def marginal_covariance(self, index=-1, unobservable_variance=1.0e6):
        """Return one state's covariance from the undamped window Hessian.

        Solver damping is deliberately excluded because it is not sensor
        information. Rank-deficient directions receive a large finite
        variance so downstream consumers do not mistake a nullspace for high
        confidence.
        """
        if not self._states:
            raise IndexError("cannot compute covariance for an empty window")
        index = int(index)
        if index < 0:
            index += len(self._states)
        if index < 0 or index >= len(self._states):
            raise IndexError("covariance state is outside the active window")
        if (
            not math.isfinite(unobservable_variance)
            or unobservable_variance <= 0.0
        ):
            raise ValueError(
                "unobservable variance must be finite and positive")

        hessian = self._last_hessian
        if hessian is None or hessian.shape[0] != len(
                self._states) * STATE_SIZE:
            hessian, _, _ = self._normal()
        hessian = 0.5 * (hessian + hessian.T)
        eigenvalues, eigenvectors = np.linalg.eigh(hessian)
        scale = max(1.0, float(np.max(np.abs(eigenvalues))))
        active = eigenvalues > self.marginal_rank_tolerance * scale
        inverse_eigenvalues = np.full(
            eigenvalues.shape, float(unobservable_variance), dtype=float
        )
        inverse_eigenvalues[active] = 1.0 / eigenvalues[active]
        covariance = (eigenvectors * inverse_eigenvalues) @ eigenvectors.T
        start = index * STATE_SIZE
        block = covariance[start:start + STATE_SIZE, start:start + STATE_SIZE]
        return 0.5 * (block + block.T)

    def add_state(self, initial=None):
        state = np.zeros(STATE_SIZE, dtype=float)
        if initial is not None:
            state[:] = np.asarray(initial, dtype=float)
        if np.any(~np.isfinite(state)):
            raise ValueError("manifold state must be finite")
        self._states.append(state)
        self._last_hessian = None
        self._marginalize_if_needed()
        return len(self._states) - 1

    def reset(self, initial, covariance=1.0):
        """Start a new fixed-lag epoch after an accepted global relocalization."""
        state = np.asarray(initial, dtype=float)
        if state.shape != (STATE_SIZE,) or np.any(~np.isfinite(state)):
            raise ValueError("reset state must be a finite 15-vector")
        self._states = []
        self._factors = []
        self._last_initial_cost = 0.0
        self._last_cost = 0.0
        self._last_iterations = 0
        self._last_solve_ms = 0.0
        self._last_rejected_steps = 0
        self._last_hessian = None
        self._last_marginalization_ms = 0.0
        self.last_marginal_prior_diagnostic = {}
        self._lm_damping = min(
            self.lm_max_damping, max(self.lm_min_damping, self.damping)
        )
        index = self.add_state(state)
        self.add_prior(index, state, covariance=covariance)
        return index

    def _append(self, name, indices, residual_dimension, decision, **values):
        for index in indices:
            if index < 0 or index >= len(self._states):
                raise IndexError("factor state is outside the active window")
        enabled, reliability_weight, inflation = _scheduler_values(decision)
        self._factors.append({
            "name": name,
            "indices": tuple(int(index) for index in indices),
            "residual_dimension": int(residual_dimension),
            "enabled": bool(enabled and reliability_weight > 0.0),
            "reliability_weight": reliability_weight,
            "covariance_inflation": inflation,
            "effective_weight": reliability_weight / inflation,
            **values,
        })
        self._last_hessian = None

    def add_prior(self, index, state, covariance=1.0):
        state = np.asarray(state, dtype=float)
        if state.shape != (STATE_SIZE,):
            raise ValueError("prior state must be a 15-vector")
        self._append(
            "prior", (index,), STATE_SIZE, None,
            measurement=state.copy(),
            variance=_positive_diagonal(covariance, STATE_SIZE),
        )

    def add_imu_preintegrated(
            self,
            previous,
            current,
            measurement,
            decision=None):
        if not isinstance(
                measurement,
                ManifoldPreintegratedImu) or not measurement.valid:
            raise ValueError(
                "IMU factor requires valid manifold preintegration")
        covariance = _positive_covariance(measurement.covariance, 15)
        self._append(
            "imu_preintegrated", (previous, current), 15, decision,
            measurement=measurement,
            covariance=covariance,
            information_matrix=_covariance_information(covariance),
        )

    def replace_imu_preintegrated(self, previous, current, measurement):
        """Replace the active interval's preintegration after a bias update."""
        if not isinstance(
                measurement,
                ManifoldPreintegratedImu) or not measurement.valid:
            raise ValueError(
                "IMU factor requires valid manifold preintegration")
        covariance = _positive_covariance(measurement.covariance, 15)
        for factor in reversed(self._factors):
            if (
                factor["name"] == "imu_preintegrated"
                and factor["indices"] == (int(previous), int(current))
            ):
                factor["measurement"] = measurement
                factor["covariance"] = covariance
                factor["information_matrix"] = _covariance_information(
                    covariance)
                return True
        return False

    def add_native_lidar_correspondences(
            self, index, factor, decision=None,
            axis_information_scale=None):
        if not isinstance(factor, NativeLidarPoseNormal):
            raise ValueError("native LiDAR factor has the wrong type")
        if factor.lidar_points is None:
            raise ValueError("native LiDAR factor lacks raw correspondences")
        translation_information_scale = (
            np.ones(3, dtype=float)
            if axis_information_scale is None
            else np.asarray(axis_information_scale, dtype=float)
        )
        if (
            translation_information_scale.shape != (3,)
            or np.any(~np.isfinite(translation_information_scale))
            or np.any(translation_information_scale < 0.0)
            or np.any(translation_information_scale > 1.0)
        ):
            raise ValueError(
                "LiDAR axis information scale must be a finite 3-vector "
                "within [0, 1]"
            )
        variance = np.full(
            factor.matched_points, factor.measurement_variance, dtype=float
        )
        self._append(
            "lidar_point_plane",
            (index,
             ),
            factor.matched_points,
            decision,
            measurement=factor,
            variance=variance,
            translation_information_scale=(
                translation_information_scale.copy()
            ),
            translation_subspace_scale=np.eye(3, dtype=float),
        )

    def set_lidar_subspace_scale(self, scale):
        """Apply one translation subspace scale to active raw LiDAR factors.

        Marginal priors are deliberately excluded: only factors retaining raw
        point-plane correspondences can be reweighted without reconstructing a
        prior.  The transform is applied during the next normal assembly.
        """
        scale = np.asarray(scale, dtype=float)
        if scale.shape != (3, 3) or np.any(~np.isfinite(scale)):
            raise ValueError("LiDAR subspace scale must be a finite 3x3 matrix")
        scale = 0.5 * (scale + scale.T)
        eigenvalues = np.linalg.eigvalsh(scale)
        if np.any(eigenvalues < -1.0e-9) or np.any(eigenvalues > 1.0 + 1.0e-9):
            raise ValueError("LiDAR subspace scale must be PSD and bounded")
        for factor in self._factors:
            if factor["name"] == "lidar_point_plane":
                factor["translation_subspace_scale"] = scale.copy()
        self._last_hessian = None

    def add_native_lidar_normal(
        self,
        index,
        linearization_pose,
        pose_hessian,
        pose_gradient,
        measurement_variance,
        residual_dimension,
        residual_squared,
        decision=None,
    ):
        self._append(
            "lidar_point_plane_condensed", (index,), residual_dimension, decision,
            linearization=np.asarray(linearization_pose, dtype=float),
            normal_hessian=np.asarray(pose_hessian, dtype=float),
            normal_gradient=np.asarray(pose_gradient, dtype=float),
            residual_squared=float(residual_squared),
            measurement_variance=float(measurement_variance),
        )

    def add_lidar_pose(
            self,
            index,
            position,
            rotation,
            covariance=1.0,
            decision=None):
        measurement = np.zeros(STATE_SIZE, dtype=float)
        measurement[:3] = position
        measurement[3:6] = rotation
        self._append(
            "lidar_pose", (index,), 6, decision,
            measurement=measurement,
            variance=_positive_diagonal(covariance, 6),
        )

    def add_gnss(self, index, position, covariance=1.0, decision=None):
        self._append(
            "gnss", (index,), 3, decision,
            measurement=np.asarray(position, dtype=float),
            variance=_positive_diagonal(covariance, 3),
        )

    def add_barometer_local_z(
            self, index, height_m, variance_m2, decision=None):
        height_m = float(height_m)
        if not math.isfinite(height_m):
            raise ValueError("barometer height must be finite")
        self._append(
            "barometer_local_z", (index,), 1, decision,
            measurement=height_m,
            variance=_positive_diagonal(variance_m2, 1),
        )

    def add_optical_flow(
            self,
            previous,
            current,
            delta_position,
            covariance=1.0,
            decision=None):
        measurement = np.asarray(delta_position, dtype=float)
        if measurement.shape not in ((2,), (3,)):
            raise ValueError("optical flow must be a planar 2- or 3-vector")
        if np.any(~np.isfinite(measurement)):
            raise ValueError("optical flow measurement must be finite")
        self._append(
            "optical_flow", (previous, current), 2, decision,
            measurement=measurement[:2].copy(),
            variance=_positive_diagonal(covariance, 2),
        )

    def add_optical_flow_body(
        self, previous, current, delta_body, linearization_yaw=None,
        covariance=1.0, decision=None,
    ):
        measurement = np.asarray(delta_body, dtype=float).copy()
        if measurement.shape == (2,):
            measurement = np.r_[measurement, 0.0]
        if measurement.shape != (3,) or np.any(~np.isfinite(measurement)):
            raise ValueError("body optical flow must be a finite 2- or 3-vector")
        measurement[2] = 0.0
        self._append(
            "optical_flow_body", (previous, current), 2, decision,
            measurement=measurement,
            variance=_positive_diagonal(covariance, 2),
        )

    def add_optical_flow_range_body(
            self,
            previous,
            current,
            delta_body,
            range_observation,
            range_variance_m2,
            linearization_yaw=None,
            covariance=1.0,
            decision=None):
        """Add one MTF packet as planar flow plus one RangeFacet row.

        The range row is part of this factor record, so a flow/range packet
        is consumed once and cannot accidentally count as two independent
        measurements from the same sensor source.
        """
        if not isinstance(range_observation, RangeFacetObservation):
            raise ValueError("range observation has the wrong type")
        measurement = np.asarray(delta_body, dtype=float).copy()
        if measurement.shape == (2,):
            measurement = np.r_[measurement, 0.0]
        if measurement.shape != (3,) or np.any(~np.isfinite(measurement)):
            raise ValueError("body optical flow must be a finite 2- or 3-vector")
        measurement[2] = 0.0
        range_variance_m2 = float(range_variance_m2)
        if not math.isfinite(range_variance_m2) or range_variance_m2 <= 0.0:
            raise ValueError("range variance must be finite and positive")
        self._append(
            "optical_flow_range_body", (previous, current), 3, decision,
            measurement={
                "delta_body": measurement,
                "range_observation": range_observation,
                "range_variance_m2": range_variance_m2,
            },
            variance=_positive_diagonal(covariance, 2),
        )

    def add_visual_reprojection(
            self,
            previous,
            current,
            tracks,
            decision=None):
        if not isinstance(tracks, VisualTrackBatch):
            raise ValueError("visual reprojection factor has the wrong type")
        self._append(
            "visual_reprojection", (previous, current),
            tracks.track_count * 2, decision,
            measurement=tracks,
            variance=tracks.variance.copy(),
        )

    def add_rgbd_depth(
            self, previous, current, tracks, decision=None):
        if not isinstance(tracks, RgbdDepthTrackBatch):
            raise ValueError("RGB-D depth factor has the wrong type")
        self._append(
            "rgbd_depth", (previous, current), tracks.track_count, decision,
            measurement=tracks,
            variance=tracks.variance_m2.copy(),
        )

    def add_rgbd_direct(self, previous, current, tracks, decision=None):
        if not isinstance(tracks, RgbdDirectTrackBatch):
            raise ValueError("RGB-D direct factor has the wrong type")
        self._append(
            "rgbd_direct", (previous, current), tracks.track_count * 2,
            decision, measurement=tracks,
            variance=np.column_stack((
                tracks.depth_variance_m2,
                tracks.photometric_variance,
            )).reshape(-1),
        )

    def add_legacy_visual_odometry(
            self, previous, current, delta_body, delta_rotation,
            covariance=1.0, decision=None):
        """Explicit A/B-only RTAB-style relative SE(3) increment."""
        measurement = np.r_[
            np.asarray(delta_body, dtype=float),
            np.asarray(delta_rotation, dtype=float),
        ]
        if measurement.shape != (6,) or np.any(~np.isfinite(measurement)):
            raise ValueError(
                "legacy visual increment must be a finite 6-vector")
        self._append(
            "legacy_visual_odometry", (previous, current), 6, decision,
            measurement=measurement,
            variance=_positive_diagonal(covariance, 6),
        )

    def _residual(self, factor, states):
        name = factor["name"]
        indices = factor["indices"]
        if name == "prior":
            return state_local(factor["measurement"], states[indices[0]])
        if name == "imu_preintegrated":
            return imu_residual(
                states,
                indices[0],
                indices[1],
                factor["measurement"],
                self.gravity)
        if name == "lidar_pose":
            return state_local(factor["measurement"], states[indices[0]])[:6]
        if name == "gnss":
            return states[indices[0]][POSITION] - factor["measurement"]
        if name == "barometer_local_z":
            return np.asarray([
                states[indices[0]][POSITION][2] - factor["measurement"]
            ])
        if name == "optical_flow":
            return (
                states[indices[1]][POSITION]
                - states[indices[0]][POSITION]
            )[:2] - factor["measurement"]
        if name == "optical_flow_body":
            return (
                states[indices[1]][POSITION]
                - states[indices[0]][POSITION]
                - rpy_to_rotation_matrix(states[indices[0]][ROTATION])
                @ factor["measurement"]
            )[:2]
        if name == "optical_flow_range_body":
            previous, current = indices
            flow = factor["measurement"]["delta_body"]
            flow_residual = (
                states[current][POSITION]
                - states[previous][POSITION]
                - rpy_to_rotation_matrix(states[previous][ROTATION]) @ flow
            )[:2]
            try:
                predicted, _, _ = range_facet_prediction_jacobian(
                    factor["measurement"]["range_observation"],
                    states[current][POSITION],
                    rpy_to_rotation_matrix(states[current][ROTATION]),
                )
                range_residual = predicted - float(
                    factor["measurement"]["range_observation"].measured_range_m
                )
            except (ValueError, TypeError, FloatingPointError):
                range_residual = 0.0
            return np.r_[flow_residual, range_residual]
        if name == "visual_reprojection":
            return visual_reprojection_residual_jacobians(
                states[indices[0]], states[indices[1]], factor["measurement"]
            )[0]
        if name == "rgbd_depth":
            return rgbd_depth_residual_jacobians(
                states[indices[0]], states[indices[1]], factor["measurement"]
            )[0]
        if name == "rgbd_direct":
            values = rgbd_direct_residual_jacobians(
                states[indices[0]], states[indices[1]], factor["measurement"]
            )
            return np.column_stack((values[0], values[1])).reshape(-1)
        if name == "legacy_visual_odometry":
            previous, current = indices
            previous_rotation = rpy_to_rotation_matrix(
                states[previous][ROTATION])
            current_rotation = rpy_to_rotation_matrix(
                states[current][ROTATION])
            translation = previous_rotation.T @ (
                states[current][POSITION] - states[previous][POSITION]
            )
            rotation = so3_log(previous_rotation.T @ current_rotation)
            return np.r_[translation, rotation] - factor["measurement"]
        raise ValueError(f"factor {name} has no residual form")

    def _factor_normal(self, factor, states, hessian=None, gradient=None):
        dimension = len(states) * STATE_SIZE
        if hessian is None:
            hessian = np.zeros((dimension, dimension), dtype=float)
        if gradient is None:
            gradient = np.zeros(dimension, dtype=float)
        if hessian.shape != (dimension, dimension) or gradient.shape != (dimension,):
            raise ValueError("normal-equation accumulator has the wrong dimension")
        if not factor["enabled"]:
            return hessian, gradient, 0.0
        name = factor["name"]
        if name == "marginal_prior":
            if self.cpp_math_core_enabled:
                references = np.asarray(factor["references"], dtype=float)
                local_states = np.asarray(
                    [states[index] for index in factor["indices"]],
                    dtype=float,
                )
                (
                    transformed_hessian,
                    transformed_gradient,
                    cost,
                ) = cpp_marginal_prior_normal(
                    references,
                    local_states,
                    factor["normal_hessian"],
                    factor["normal_gradient"],
                )
                indices = np.concatenate([
                    np.arange(index * STATE_SIZE, (index + 1) * STATE_SIZE)
                    for index in factor["indices"]
                ])
                hessian[np.ix_(indices, indices)] += transformed_hessian
                gradient[indices] += transformed_gradient
                return hessian, gradient, float(cost)
            local = np.concatenate([
                state_local(reference, states[index])
                for reference, index in zip(factor["references"], factor["indices"])
            ])
            local_hessian = factor["normal_hessian"]
            local_gradient = local_hessian @ local + factor["normal_gradient"]
            indices = np.concatenate([
                np.arange(index * STATE_SIZE, (index + 1) * STATE_SIZE)
                for index in factor["indices"]
            ])
            # The stored prior is expressed in the tangent space at its
            # references.  Re-linearize the SO(3) local coordinates before
            # applying it at the current state; treating this Jacobian as an
            # identity causes yaw/roll information to become inconsistent as
            # the fixed-lag window moves.
            # The prior Jacobian is block diagonal and differs from identity
            # only in each state's 3x3 rotation block.  Forming a full dense
            # (15*N)^2 Jacobian and multiplying J.T@H@J dominated the fixed-lag
            # runtime.  Apply the same left/right block transforms directly;
            # this is algebraically identical and retains every cross block.
            transformed_hessian = local_hessian.copy()
            transformed_gradient = local_gradient.copy()
            rotation_jacobians = []
            for block, (reference, index) in enumerate(zip(
                    factor["references"], factor["indices"])):
                state_local_rotation = state_local(
                    reference, states[index])[3:6]
                rotation_jacobian = so3_right_jacobian_inverse(
                    state_local_rotation
                )
                rotation_jacobians.append(rotation_jacobian)
                rotation = slice(
                    block * STATE_SIZE + 3,
                    block * STATE_SIZE + 6,
                )
                transformed_hessian[rotation, :] = (
                    rotation_jacobian.T
                    @ transformed_hessian[rotation, :]
                )
                transformed_gradient[rotation] = (
                    rotation_jacobian.T @ transformed_gradient[rotation]
                )
            for block, rotation_jacobian in enumerate(rotation_jacobians):
                rotation = slice(
                    block * STATE_SIZE + 3,
                    block * STATE_SIZE + 6,
                )
                transformed_hessian[:, rotation] = (
                    transformed_hessian[:, rotation] @ rotation_jacobian
                )
            hessian[np.ix_(indices, indices)] += transformed_hessian
            gradient[indices] += transformed_gradient
            cost = float(
                0.5 * local @ local_hessian @ local
                + factor["normal_gradient"] @ local
            )
            return hessian, gradient, cost
        if name == "lidar_point_plane":
            index = factor["indices"][0]
            translation_information_scale = factor[
                "translation_information_scale"
            ]
            subspace_scale = factor.get(
                "translation_subspace_scale", np.eye(3, dtype=float)
            )
            subspace_scaled = not np.allclose(
                subspace_scale, np.eye(3), atol=1.0e-12, rtol=0.0
            )
            axis_scaled = not np.allclose(
                translation_information_scale, 1.0, atol=0.0, rtol=0.0
            )
            if (
                self.cpp_math_core_enabled
                and (
                    not subspace_scaled
                    or cpp_lidar_point_plane_normal_subspace is not None
                )
                and (
                    not axis_scaled
                    or CPP_LIDAR_AXIS_SCALED_CORE_AVAILABLE
                )
            ):
                measurement = factor["measurement"]
                if subspace_scaled:
                    kernel = cpp_lidar_point_plane_normal_subspace
                    kernel_arguments = (
                        states[index][:6],
                        measurement.lidar_points,
                        measurement.plane_normals,
                        measurement.plane_points,
                        measurement.lidar_to_body_rotation,
                        measurement.lidar_to_body_translation,
                        translation_information_scale,
                        subspace_scale,
                        factor["variance"],
                        factor["effective_weight"],
                        self.lidar_huber_delta,
                    )
                elif axis_scaled:
                    kernel = cpp_lidar_point_plane_normal_axis_scaled
                    kernel_arguments = (
                        states[index][:6],
                        measurement.lidar_points,
                        measurement.plane_normals,
                        measurement.plane_points,
                        measurement.lidar_to_body_rotation,
                        measurement.lidar_to_body_translation,
                        translation_information_scale,
                        factor["variance"],
                        factor["effective_weight"],
                        self.lidar_huber_delta,
                    )
                else:
                    kernel = cpp_lidar_point_plane_normal
                    kernel_arguments = (
                        states[index][:6],
                        measurement.lidar_points,
                        measurement.plane_normals,
                        measurement.plane_points,
                        measurement.lidar_to_body_rotation,
                        measurement.lidar_to_body_translation,
                        factor["variance"],
                        factor["effective_weight"],
                        self.lidar_huber_delta,
                    )
                local_hessian, local_gradient, cost = kernel(
                    *kernel_arguments
                )
                start = index * STATE_SIZE
                pose = slice(start, start + 6)
                hessian[pose, pose] += local_hessian
                gradient[pose] += local_gradient
                return hessian, gradient, float(cost)
            residual, pose_jacobian = point_plane_residual_jacobian(
                factor["measurement"], states[index][:6]
            )
            pose_jacobian = pose_jacobian.copy()
            pose_jacobian[:, :3] *= np.sqrt(
                translation_information_scale
            )[None, :]
            standardized = residual / np.sqrt(factor["variance"])
            loss, robust_weight = huber_loss_and_weight(
                standardized, self.lidar_huber_delta
            )
            information = (
                factor["effective_weight"]
                * robust_weight
                / factor["variance"]
            )
            start = index * STATE_SIZE
            pose = slice(start, start + 6)
            local_hessian = (
                pose_jacobian.T @ (information[:, None] * pose_jacobian)
            )
            local_gradient = pose_jacobian.T @ (information * residual)
            if subspace_scaled:
                local_hessian, local_gradient = (
                    lidar_translation_subspace_normal(
                        local_hessian, local_gradient, subspace_scale
                    )
                )
            hessian[pose, pose] += local_hessian
            gradient[pose] += local_gradient
            return (
                hessian,
                gradient,
                factor["effective_weight"] * float(np.sum(loss)),
            )
        if name == "lidar_point_plane_condensed":
            index = factor["indices"][0]
            reference = np.zeros(STATE_SIZE)
            reference[:6] = factor["linearization"]
            delta = state_local(reference, states[index])[:6]
            scale = factor["effective_weight"] / factor["measurement_variance"]
            local_hessian = factor["normal_hessian"]
            local_gradient = factor["normal_gradient"] + local_hessian @ delta
            start = index * STATE_SIZE
            hessian[start:start + 6, start:start + 6] += scale * local_hessian
            gradient[start:start + 6] += scale * local_gradient
            cost = scale * (
                factor["residual_squared"]
                + 2.0 * factor["normal_gradient"] @ delta
                + delta @ local_hessian @ delta
            )
            return hessian, gradient, 0.5 * float(cost)

        if name == "gnss":
            index = factor["indices"][0]
            residual = self._residual(factor, states)
            information = factor["effective_weight"] / factor["variance"]
            start = index * STATE_SIZE
            position = slice(start, start + 3)
            hessian[position, position] += np.diag(information)
            gradient[position] += information * residual
            cost = 0.5 * float(np.sum(information * residual ** 2))
            return hessian, gradient, cost

        if name == "optical_flow_range_body":
            previous, current = factor["indices"]
            measurement = factor["measurement"]
            flow = measurement["delta_body"]
            observation = measurement["range_observation"]
            residual = self._residual(factor, states)
            flow_information = factor["effective_weight"] / factor["variance"]
            try:
                _, range_jacobian, _ = range_facet_prediction_jacobian(
                    observation,
                    states[current][POSITION],
                    rpy_to_rotation_matrix(states[current][ROTATION]),
                )
            except (ValueError, TypeError, FloatingPointError):
                range_jacobian = np.zeros(6, dtype=float)
            range_variance = float(measurement["range_variance_m2"])
            range_standardized = residual[2] / math.sqrt(range_variance)
            _, range_robust_weight = huber_loss_and_weight(
                np.asarray([range_standardized]), 2.5
            )
            range_information = (
                factor["effective_weight"]
                * float(range_robust_weight[0])
                / range_variance
            )
            previous_start = previous * STATE_SIZE
            current_start = current * STATE_SIZE
            previous_jacobian = np.zeros((3, STATE_SIZE), dtype=float)
            current_jacobian = np.zeros((3, STATE_SIZE), dtype=float)
            previous_jacobian[:2, :2] = -np.eye(2)
            current_jacobian[:2, :2] = np.eye(2)
            rotation_jacobian = (
                rpy_to_rotation_matrix(states[previous][ROTATION])
                @ skew(flow)
            )[:2, :]
            previous_jacobian[:2, 3:6] = -rotation_jacobian
            current_jacobian[2, :6] = range_jacobian
            information = np.r_[flow_information, range_information]
            weighted_residual = information * residual
            previous_slice = slice(previous_start, previous_start + STATE_SIZE)
            current_slice = slice(current_start, current_start + STATE_SIZE)
            gradient[previous_slice] += previous_jacobian.T @ weighted_residual
            gradient[current_slice] += current_jacobian.T @ weighted_residual
            hessian[previous_slice, previous_slice] += (
                previous_jacobian.T @ (information[:, None] * previous_jacobian)
            )
            hessian[previous_slice, current_slice] += (
                previous_jacobian.T @ (information[:, None] * current_jacobian)
            )
            hessian[current_slice, previous_slice] += (
                current_jacobian.T @ (information[:, None] * previous_jacobian)
            )
            hessian[current_slice, current_slice] += (
                current_jacobian.T @ (information[:, None] * current_jacobian)
            )
            flow_cost = 0.5 * float(np.sum(flow_information * residual[:2] ** 2))
            range_cost = 0.5 * range_information * residual[2] ** 2
            return hessian, gradient, flow_cost + range_cost

        if name == "barometer_local_z":
            index = factor["indices"][0]
            residual = float(self._residual(factor, states)[0])
            information = (
                factor["effective_weight"] / float(factor["variance"][0])
            )
            z_index = index * STATE_SIZE + 2
            hessian[z_index, z_index] += information
            gradient[z_index] += information * residual
            return hessian, gradient, 0.5 * information * residual * residual

        if name == "optical_flow":
            previous, current = factor["indices"]
            residual = self._residual(factor, states)
            information = factor["effective_weight"] / factor["variance"]
            information_matrix = np.diag(information)
            previous_position = slice(
                previous * STATE_SIZE, previous * STATE_SIZE + 2
            )
            current_position = slice(
                current * STATE_SIZE, current * STATE_SIZE + 2
            )
            weighted_residual = information * residual
            gradient[previous_position] -= weighted_residual
            gradient[current_position] += weighted_residual
            hessian[previous_position, previous_position] += information_matrix
            hessian[current_position, current_position] += information_matrix
            hessian[previous_position, current_position] -= information_matrix
            hessian[current_position, previous_position] -= information_matrix
            cost = 0.5 * float(np.sum(information * residual ** 2))
            return hessian, gradient, cost

        if name == "optical_flow_body":
            previous, current = factor["indices"]
            residual = self._residual(factor, states)
            information = factor["effective_weight"] / factor["variance"]
            information_matrix = np.diag(information)
            rotation_jacobian = (
                rpy_to_rotation_matrix(states[previous][ROTATION])
                @ skew(factor["measurement"])
            )[:2, :]
            previous_start = previous * STATE_SIZE
            current_start = current * STATE_SIZE
            previous_position = slice(previous_start, previous_start + 2)
            previous_rotation = slice(previous_start + 3, previous_start + 6)
            current_position = slice(current_start, current_start + 2)
            weighted_residual = information * residual
            weighted_rotation = information[:, None] * rotation_jacobian
            gradient[previous_position] -= weighted_residual
            gradient[previous_rotation] += rotation_jacobian.T @ weighted_residual
            gradient[current_position] += weighted_residual
            hessian[previous_position, previous_position] += information_matrix
            hessian[previous_position, previous_rotation] -= weighted_rotation
            hessian[previous_rotation, previous_position] -= weighted_rotation.T
            hessian[previous_rotation, previous_rotation] += (
                rotation_jacobian.T @ weighted_rotation
            )
            hessian[previous_position, current_position] -= information_matrix
            hessian[current_position, previous_position] -= information_matrix
            hessian[previous_rotation, current_position] += weighted_rotation.T
            hessian[current_position, previous_rotation] += weighted_rotation
            hessian[current_position, current_position] += information_matrix
            cost = 0.5 * float(np.sum(information * residual ** 2))
            return hessian, gradient, cost

        if name == "imu_preintegrated" and self.cpp_math_core_enabled:
            previous, current = factor["indices"]
            local_hessian, local_gradient, cost = (
                cpp_imu_preintegrated_normal(
                    *_cpp_imu_factor_arguments(factor, states, self.gravity)
                )
            )
            indices = np.r_[
                np.arange(
                    previous * STATE_SIZE,
                    (previous + 1) * STATE_SIZE,
                ),
                np.arange(
                    current * STATE_SIZE,
                    (current + 1) * STATE_SIZE,
                ),
            ]
            hessian[np.ix_(indices, indices)] += local_hessian
            gradient[indices] += local_gradient
            return hessian, gradient, float(cost)

        if name == "visual_reprojection" and self.cpp_math_core_enabled:
            previous, current = factor["indices"]
            local_hessian, local_gradient, cost = (
                cpp_visual_reprojection_normal(
                    *_cpp_visual_factor_arguments(factor, states)
                )
            )
            indices = np.r_[
                np.arange(
                    previous * STATE_SIZE,
                    (previous + 1) * STATE_SIZE,
                ),
                np.arange(
                    current * STATE_SIZE,
                    (current + 1) * STATE_SIZE,
                ),
            ]
            hessian[np.ix_(indices, indices)] += local_hessian
            gradient[indices] += local_gradient
            return hessian, gradient, float(cost)

        jacobians = {}
        if name == "visual_reprojection":
            previous, current = factor["indices"]
            residual, previous_jacobian, current_jacobian, valid = (
                visual_reprojection_residual_jacobians(
                    states[previous], states[current], factor["measurement"]
                )
            )
            if residual.size == 0:
                return hessian, gradient, 0.0
            jacobians[previous] = previous_jacobian
            jacobians[current] = current_jacobian
            variance = factor["variance"].reshape(-1, 2)[valid].reshape(-1)
            standardized = residual / np.sqrt(variance)
            loss, robust_weight = huber_loss_and_weight(standardized, 2.5)
            information = factor["effective_weight"] * robust_weight / variance
            for first_index, first_jacobian in jacobians.items():
                first = slice(
                    first_index * STATE_SIZE,
                    (first_index + 1) * STATE_SIZE)
                gradient[first] += first_jacobian.T @ (information * residual)
                for second_index, second_jacobian in jacobians.items():
                    second = slice(
                        second_index * STATE_SIZE,
                        (second_index + 1) * STATE_SIZE)
                    hessian[first, second] += first_jacobian.T @ (
                        information[:, None] * second_jacobian
                    )
            return hessian, gradient, factor["effective_weight"] * \
                float(np.sum(loss))
        if (
                name == "rgbd_depth"
                and self.cpp_math_core_enabled
                and CPP_RGBD_DEPTH_CORE_AVAILABLE):
            previous, current = factor["indices"]
            local_hessian, local_gradient, cost = cpp_rgbd_depth_normal(
                *_cpp_rgbd_depth_factor_arguments(factor, states)
            )
            indices = np.r_[
                np.arange(previous * STATE_SIZE, (previous + 1) * STATE_SIZE),
                np.arange(current * STATE_SIZE, (current + 1) * STATE_SIZE),
            ]
            hessian[np.ix_(indices, indices)] += local_hessian
            gradient[indices] += local_gradient
            return hessian, gradient, float(cost)
        if name == "rgbd_depth":
            previous, current = factor["indices"]
            residual, previous_jacobian, current_jacobian, valid = (
                rgbd_depth_residual_jacobians(
                    states[previous], states[current], factor["measurement"]
                )
            )
            if residual.size == 0:
                return hessian, gradient, 0.0
            variance = factor["variance"][valid]
            standardized = residual / np.sqrt(variance)
            loss, robust_weight = huber_loss_and_weight(standardized, 2.5)
            information = factor["effective_weight"] * robust_weight / variance
            jacobians = {
                previous: previous_jacobian,
                current: current_jacobian,
            }
            for first_index, first_jacobian in jacobians.items():
                first = slice(
                    first_index * STATE_SIZE, (first_index + 1) * STATE_SIZE
                )
                gradient[first] += first_jacobian.T @ (information * residual)
                for second_index, second_jacobian in jacobians.items():
                    second = slice(
                        second_index * STATE_SIZE,
                        (second_index + 1) * STATE_SIZE,
                    )
                    hessian[first, second] += first_jacobian.T @ (
                        information[:, None] * second_jacobian
                    )
            return hessian, gradient, factor["effective_weight"] * float(
                np.sum(loss)
            )
        if (
                name == "rgbd_direct"
                and self.cpp_math_core_enabled
                and CPP_RGBD_DEPTH_CORE_AVAILABLE):
            previous, current = factor["indices"]
            local_hessian, local_gradient, cost = cpp_rgbd_direct_normal(
                *_cpp_rgbd_direct_factor_arguments(factor, states)
            )
            indices = np.r_[
                np.arange(previous * STATE_SIZE, (previous + 1) * STATE_SIZE),
                np.arange(current * STATE_SIZE, (current + 1) * STATE_SIZE),
            ]
            hessian[np.ix_(indices, indices)] += local_hessian
            gradient[indices] += local_gradient
            return hessian, gradient, float(cost)
        if name == "rgbd_direct":
            previous, current = factor["indices"]
            values = rgbd_direct_residual_jacobians(
                states[previous], states[current], factor["measurement"]
            )
            if values[0].size == 0:
                return hessian, gradient, 0.0
            valid = values[6]
            residual = np.column_stack((values[0], values[1])).reshape(-1)
            variance = np.column_stack((
                factor["measurement"].depth_variance_m2[valid],
                factor["measurement"].photometric_variance[valid],
            )).reshape(-1)
            previous_jacobian = np.stack(
                (values[2], values[4]), axis=1
            ).reshape(-1, STATE_SIZE)
            current_jacobian = np.stack(
                (values[3], values[5]), axis=1
            ).reshape(-1, STATE_SIZE)
            standardized = residual / np.sqrt(variance)
            loss, robust_weight = huber_loss_and_weight(standardized, 2.5)
            information = factor["effective_weight"] * robust_weight / variance
            jacobians = {
                previous: previous_jacobian,
                current: current_jacobian,
            }
            for first_index, first_jacobian in jacobians.items():
                first = slice(
                    first_index * STATE_SIZE, (first_index + 1) * STATE_SIZE
                )
                gradient[first] += first_jacobian.T @ (information * residual)
                for second_index, second_jacobian in jacobians.items():
                    second = slice(
                        second_index * STATE_SIZE,
                        (second_index + 1) * STATE_SIZE,
                    )
                    hessian[first, second] += first_jacobian.T @ (
                        information[:, None] * second_jacobian
                    )
            return hessian, gradient, factor["effective_weight"] * float(
                np.sum(loss)
            )
        if name == "imu_preintegrated":
            previous, current = factor["indices"]
            residual, previous_jacobian, current_jacobian = (
                imu_residual_jacobians(
                    states, previous, current, factor["measurement"], self.gravity
                )
            )
            jacobians[previous] = previous_jacobian
            jacobians[current] = current_jacobian
        else:
            residual = self._residual(factor, states)
            for index in factor["indices"]:
                jacobians[index] = numerical_state_jacobian(
                    lambda values: self._residual(factor, values),
                    states,
                    index,
                )
        if "information_matrix" in factor:
            information = (
                factor["effective_weight"] * factor["information_matrix"]
            )
            weighted_residual = information @ residual
            for first_index, first_jacobian in jacobians.items():
                first = slice(
                    first_index * STATE_SIZE, (first_index + 1) * STATE_SIZE
                )
                gradient[first] += first_jacobian.T @ weighted_residual
                for second_index, second_jacobian in jacobians.items():
                    second = slice(
                        second_index * STATE_SIZE,
                        (second_index + 1) * STATE_SIZE,
                    )
                    hessian[first, second] += (
                        first_jacobian.T @ information @ second_jacobian
                    )
            cost = 0.5 * float(residual @ weighted_residual)
        else:
            information = factor["effective_weight"] / factor["variance"]
            for first_index, first_jacobian in jacobians.items():
                first = slice(
                    first_index * STATE_SIZE, (first_index + 1) * STATE_SIZE
                )
                gradient[first] += first_jacobian.T @ (information * residual)
                for second_index, second_jacobian in jacobians.items():
                    second = slice(
                        second_index * STATE_SIZE,
                        (second_index + 1) * STATE_SIZE,
                    )
                    hessian[first, second] += (
                        first_jacobian.T
                        @ (information[:, None] * second_jacobian)
                    )
            cost = 0.5 * float(np.sum(information * residual ** 2))
        return hessian, gradient, cost

    def _factor_cost(self, factor, states):
        if not factor["enabled"]:
            return 0.0
        name = factor["name"]
        if name == "marginal_prior":
            if self.cpp_math_core_enabled:
                return float(cpp_marginal_prior_cost(
                    np.asarray(factor["references"], dtype=float),
                    np.asarray(
                        [states[index] for index in factor["indices"]],
                        dtype=float,
                    ),
                    factor["normal_hessian"],
                    factor["normal_gradient"],
                ))
            local = np.concatenate([
                state_local(reference, states[index])
                for reference, index in zip(
                    factor["references"], factor["indices"]
                )
            ])
            return float(
                0.5 * local @ factor["normal_hessian"] @ local
                + factor["normal_gradient"] @ local
            )
        if name == "lidar_point_plane":
            index = factor["indices"][0]
            if self.cpp_math_core_enabled:
                measurement = factor["measurement"]
                return float(cpp_lidar_point_plane_cost(
                    states[index][:6],
                    measurement.lidar_points,
                    measurement.plane_normals,
                    measurement.plane_points,
                    measurement.lidar_to_body_rotation,
                    measurement.lidar_to_body_translation,
                    factor["variance"],
                    factor["effective_weight"],
                    self.lidar_huber_delta,
                ))
            residual = point_plane_residual(
                factor["measurement"], states[index][:6]
            )
            standardized = residual / np.sqrt(factor["variance"])
            loss, _ = huber_loss_and_weight(
                standardized, self.lidar_huber_delta
            )
            return factor["effective_weight"] * float(np.sum(loss))
        if name == "lidar_point_plane_condensed":
            index = factor["indices"][0]
            reference = np.zeros(STATE_SIZE)
            reference[:6] = factor["linearization"]
            delta = state_local(reference, states[index])[:6]
            scale = factor["effective_weight"] / factor["measurement_variance"]
            cost = scale * (
                factor["residual_squared"]
                + 2.0 * factor["normal_gradient"] @ delta
                + delta @ factor["normal_hessian"] @ delta
            )
            return 0.5 * float(cost)
        if name == "visual_reprojection" and self.cpp_math_core_enabled:
            return float(cpp_visual_reprojection_cost(
                *_cpp_visual_factor_arguments(factor, states)
            ))
        if name == "visual_reprojection":
            previous, current = factor["indices"]
            residual, _, _, valid = visual_reprojection_residual_jacobians(
                states[previous], states[current], factor["measurement"]
            )
            if residual.size == 0:
                return 0.0
            variance = factor["variance"].reshape(-1, 2)[valid].reshape(-1)
            standardized = residual / np.sqrt(variance)
            loss, _ = huber_loss_and_weight(standardized, 2.5)
            return factor["effective_weight"] * float(np.sum(loss))
        if (
                name == "rgbd_depth"
                and self.cpp_math_core_enabled
                and CPP_RGBD_DEPTH_CORE_AVAILABLE):
            return float(cpp_rgbd_depth_cost(
                *_cpp_rgbd_depth_factor_arguments(factor, states)
            ))
        if name == "rgbd_depth":
            previous, current = factor["indices"]
            residual, _, _, valid = rgbd_depth_residual_jacobians(
                states[previous], states[current], factor["measurement"]
            )
            if residual.size == 0:
                return 0.0
            variance = factor["variance"][valid]
            standardized = residual / np.sqrt(variance)
            loss, _ = huber_loss_and_weight(standardized, 2.5)
            return factor["effective_weight"] * float(np.sum(loss))
        if (
                name == "rgbd_direct"
                and self.cpp_math_core_enabled
                and CPP_RGBD_DEPTH_CORE_AVAILABLE):
            return float(cpp_rgbd_direct_cost(
                *_cpp_rgbd_direct_factor_arguments(factor, states)
            ))
        if name == "rgbd_direct":
            previous, current = factor["indices"]
            values = rgbd_direct_residual_jacobians(
                states[previous], states[current], factor["measurement"]
            )
            if values[0].size == 0:
                return 0.0
            valid = values[6]
            residual = np.column_stack((values[0], values[1])).reshape(-1)
            variance = np.column_stack((
                factor["measurement"].depth_variance_m2[valid],
                factor["measurement"].photometric_variance[valid],
            )).reshape(-1)
            loss, _ = huber_loss_and_weight(
                residual / np.sqrt(variance), 2.5
            )
            return factor["effective_weight"] * float(np.sum(loss))
        if name == "imu_preintegrated" and self.cpp_math_core_enabled:
            return float(cpp_imu_preintegrated_cost(
                *_cpp_imu_factor_arguments(factor, states, self.gravity)
            ))

        residual = self._residual(factor, states)
        if "information_matrix" in factor:
            information = (
                factor["effective_weight"] * factor["information_matrix"]
            )
            return 0.5 * float(residual @ information @ residual)
        information = factor["effective_weight"] / factor["variance"]
        return 0.5 * float(np.sum(information * residual ** 2))

    def _cost(self, factors=None, states=None):
        states = self._states if states is None else states
        factors = self._factors if factors is None else factors
        return sum(self._factor_cost(factor, states) for factor in factors)

    def _normal(self, factors=None, states=None):
        normal_started = self._profile_start()
        states = self._states if states is None else states
        factors = self._factors if factors is None else factors
        dimension = len(states) * STATE_SIZE
        hessian = np.zeros((dimension, dimension))
        gradient = np.zeros(dimension)
        cost = 0.0
        batched_factor_ids = set()
        if self.cpp_math_core_enabled and CPP_IMU_GRAPH_CORE_AVAILABLE:
            imu_factors = [
                factor for factor in factors
                if factor["name"] == "imu_preintegrated" and factor["enabled"]
            ]
            if imu_factors:
                factor_started = self._profile_start()
                imu_hessian, imu_gradient, imu_cost = (
                    cpp_imu_preintegrated_graph_normal(
                        *_cpp_imu_graph_arguments(
                            imu_factors, states, self.gravity
                        )
                    )
                )
                hessian += imu_hessian
                gradient += imu_gradient
                cost += float(imu_cost)
                batched_factor_ids.update(id(factor) for factor in imu_factors)
                self._profile_stop(
                    "factor_imu_preintegrated", factor_started
                )
        if self.cpp_math_core_enabled and CPP_LIDAR_GRAPH_CORE_AVAILABLE:
            lidar_factors = [
                factor for factor in factors
                if factor["name"] == "lidar_point_plane" and factor["enabled"]
            ]
            if lidar_factors:
                has_subspace_scaled_factor = any(
                    not np.allclose(
                        factor.get(
                            "translation_subspace_scale", np.eye(3)
                        ),
                        np.eye(3),
                        atol=1.0e-12,
                        rtol=0.0,
                    )
                    for factor in lidar_factors
                )
                has_axis_scaled_factor = any(
                    not np.allclose(
                        factor["translation_information_scale"],
                        1.0,
                        atol=0.0,
                        rtol=0.0,
                    )
                    for factor in lidar_factors
                )
                if (
                    not has_subspace_scaled_factor
                    and (
                    not has_axis_scaled_factor
                    or CPP_LIDAR_AXIS_SCALED_GRAPH_CORE_AVAILABLE
                    )
                ):
                    factor_started = self._profile_start()
                    if has_axis_scaled_factor:
                        lidar_hessian, lidar_gradient, lidar_cost = (
                            cpp_lidar_point_plane_graph_normal_axis_scaled(
                                *_cpp_lidar_axis_scaled_graph_arguments(
                                    lidar_factors, states
                                ),
                                self.lidar_huber_delta,
                            )
                        )
                    else:
                        lidar_hessian, lidar_gradient, lidar_cost = (
                            cpp_lidar_point_plane_graph_normal(
                                *_cpp_lidar_graph_arguments(
                                    lidar_factors, states
                                ),
                                self.lidar_huber_delta,
                            )
                        )
                    hessian += lidar_hessian
                    gradient += lidar_gradient
                    cost += float(lidar_cost)
                    batched_factor_ids.update(
                        id(factor) for factor in lidar_factors
                    )
                    self._profile_stop(
                        "factor_lidar_point_plane", factor_started
                    )
        for factor in factors:
            if id(factor) in batched_factor_ids:
                continue
            factor_started = self._profile_start()
            _, _, factor_cost = self._factor_normal(
                factor, states, hessian, gradient
            )
            self._profile_stop(
                f"factor_{factor['name']}", factor_started
            )
            assembly_started = self._profile_start()
            cost += factor_cost
            self._profile_stop("graph_assembly", assembly_started)
        self._profile_stop("factor_graph_linearization", normal_started)
        return hessian, gradient, cost

    def optimize(self, max_iterations=None):
        if not self._states:
            return []
        iteration_budget = (
            self.max_iterations
            if max_iterations is None
            else int(max_iterations)
        )
        if iteration_budget < 1:
            raise ValueError("max_iterations must be positive")
        self._last_iteration_budget = iteration_budget
        automatic_profile_cycle = (
            self.profiling_enabled and self._profile_cycle is None
        )
        if automatic_profile_cycle:
            self.begin_profile_cycle()
        started = time.perf_counter()
        profile_started = self._profile_start()
        accepted_iterations = 0
        rejected_steps = 0
        states = self.states()
        hessian, gradient, current_cost = self._normal(states=states)
        self._last_initial_cost = float(current_cost)
        damping = self._lm_damping
        for _ in range(iteration_budget):
            if float(np.max(np.abs(gradient))) < 1.0e-10:
                break
            diagonal_scale = np.maximum(np.abs(np.diag(hessian)), 1.0)
            accepted = False
            converged = False
            for trial_index in range(self.lm_max_trials):
                system = hessian + damping * np.diag(diagonal_scale)
                solve_started = self._profile_start()
                try:
                    increment = np.linalg.solve(system, -gradient)
                except np.linalg.LinAlgError:
                    increment = np.linalg.lstsq(
                        system, -gradient, rcond=None)[0]
                self._profile_stop("linear_solve", solve_started)
                if np.any(~np.isfinite(increment)):
                    damping = min(
                        self.lm_max_damping,
                        damping * self.lm_damping_up)
                    rejected_steps += 1
                    continue
                update_started = self._profile_start()
                if self.cpp_math_core_enabled:
                    candidate_matrix = cpp_state_plus_batch(
                        np.asarray(states, dtype=float),
                        increment.reshape((-1, STATE_SIZE)),
                    )
                    candidate = [row.copy() for row in candidate_matrix]
                else:
                    candidate = []
                    for index, state in enumerate(states):
                        local = increment[
                            index * STATE_SIZE:(index + 1) * STATE_SIZE
                        ]
                        candidate.append(state_plus(state, local))
                self._profile_stop("state_update", update_started)
                candidate_hessian = None
                candidate_gradient = None
                if trial_index == 0:
                    # The first trial normally succeeds, so retain its full
                    # linearization and avoid a second residual traversal.
                    (
                        candidate_hessian,
                        candidate_gradient,
                        candidate_cost,
                    ) = self._normal(states=candidate)
                else:
                    # Damping-only retries need the actual objective value but
                    # no Hessian or Jacobians until the step is accepted.
                    candidate_cost = self._cost(states=candidate)
                predicted = float(
                    -gradient @ increment
                    - 0.5 * increment @ hessian @ increment
                )
                actual = float(current_cost - candidate_cost)
                tolerance = 1.0e-12 * max(1.0, abs(current_cost))
                step_norm = float(np.max(np.abs(increment)))
                ratio = actual / predicted if predicted > 0.0 else -math.inf
                if (
                    math.isfinite(candidate_cost)
                    and (
                        (actual > tolerance and ratio > 1.0e-4)
                        or (
                            step_norm < self.convergence_threshold
                            and candidate_cost <= current_cost + tolerance
                        )
                    )
                ):
                    if candidate_hessian is None:
                        candidate_hessian, candidate_gradient, _ = self._normal(
                            states=candidate
                        )
                    states = candidate
                    hessian = candidate_hessian
                    gradient = candidate_gradient
                    current_cost = candidate_cost
                    damping = max(
                        self.lm_min_damping, damping * self.lm_damping_down
                    )
                    accepted = True
                    converged = step_norm < self.convergence_threshold
                    accepted_iterations += 1
                    break
                damping = min(
                    self.lm_max_damping,
                    damping * self.lm_damping_up)
                rejected_steps += 1
            if not accepted or converged:
                break
        self._states = states
        self._last_cost = current_cost
        self._last_iterations = accepted_iterations
        self._last_rejected_steps = rejected_steps
        self._lm_damping = damping
        self._last_hessian = hessian.copy()
        self._last_solve_ms = (time.perf_counter() - started) * 1000.0
        self._profile_stop("optimize_total", profile_started)
        if automatic_profile_cycle:
            self.finish_profile_cycle()
        return self.states()

    def latest_factor_residual(self, name, covariance=None):
        for factor in reversed(self._factors):
            if factor["name"] != name:
                continue
            if name == "lidar_point_plane":
                index = factor["indices"][0]
                residual, _ = point_plane_residual_jacobian(
                    factor["measurement"], self._states[index][:6]
                )
                return FactorResidual(
                    name=name,
                    state_indices=factor["indices"],
                    residual_dimension=factor["residual_dimension"],
                    enabled=factor["enabled"],
                    mahalanobis_squared=float(
                        np.sum(residual ** 2 / factor["variance"])
                    ),
                )
            if name in {"lidar_point_plane_condensed", "marginal_prior"}:
                _, _, cost = self._factor_normal(factor, self._states)
                return FactorResidual(
                    name=name,
                    state_indices=factor["indices"],
                    residual_dimension=factor["residual_dimension"],
                    enabled=factor["enabled"],
                    mahalanobis_squared=max(0.0, 2.0 * cost),
                )
            residual = self._residual(factor, self._states)
            if covariance is not None:
                covariance_matrix = _positive_covariance(
                    covariance, residual.size
                )
                mahalanobis_squared = float(
                    residual @ np.linalg.solve(covariance_matrix, residual)
                )
            elif "information_matrix" in factor:
                mahalanobis_squared = float(
                    residual @ factor["information_matrix"] @ residual
                )
            else:
                mahalanobis_squared = float(
                    np.sum(residual ** 2 / factor["variance"])
                )
            return FactorResidual(
                name=name,
                state_indices=factor["indices"],
                residual_dimension=residual.size,
                enabled=factor["enabled"],
                mahalanobis_squared=mahalanobis_squared,
            )
        return None

    def latest_factor_rmse(self, name):
        """Return unweighted residual RMSE for runtime diagnostics only."""
        for factor in reversed(self._factors):
            if factor["name"] != name or not factor["enabled"]:
                continue
            residual = np.asarray(
                self._residual(factor, self._states), dtype=float
            )
            if residual.size == 0 or np.any(~np.isfinite(residual)):
                return None
            return float(np.sqrt(np.mean(residual ** 2))), int(residual.size)
        return None

    def factor_summary(self):
        return [
            FactorRecord(
                name=factor["name"],
                state_indices=factor["indices"],
                residual_dimension=factor["residual_dimension"],
                enabled=factor["enabled"],
                reliability_weight=factor["reliability_weight"],
                covariance_inflation=factor["covariance_inflation"],
                effective_weight=factor["effective_weight"],
            )
            for factor in self._factors
        ]

    def marginal_prior_translation_diagnostic(self, subspace_scale):
        """Project the live marginal prior onto the current LiDAR subspace."""
        scale = np.asarray(subspace_scale, dtype=float)
        if scale.shape != (3, 3) or np.any(~np.isfinite(scale)):
            raise ValueError("subspace scale must be a finite 3x3 matrix")
        scale = 0.5 * (scale + scale.T)
        eigenvalues, eigenvectors = np.linalg.eigh(scale)
        weak = eigenvalues < 1.0 - 1.0e-6
        weak_projector = (
            eigenvectors[:, weak] @ eigenvectors[:, weak].T
            if np.any(weak)
            else np.zeros((3, 3), dtype=float)
        )
        for factor in reversed(self._factors):
            if factor["name"] != "marginal_prior" or not factor["enabled"]:
                continue
            prior = np.asarray(factor["normal_hessian"], dtype=float)
            position_information = np.zeros((3, 3), dtype=float)
            for block in range(len(factor["indices"])):
                start = block * STATE_SIZE
                position_information += prior[
                    start:start + 3, start:start + 3
                ]
            position_information = 0.5 * (
                position_information + position_information.T
            )
            total_trace = float(np.trace(position_information))
            weak_trace = float(np.trace(
                weak_projector @ position_information
            ))
            return {
                "active": True,
                "position_information_trace": total_trace,
                "current_weak_position_information_trace": weak_trace,
                "current_strong_position_information_trace": (
                    total_trace - weak_trace
                ),
                "current_weak_position_information_fraction": (
                    weak_trace / total_trace if total_trace > 0.0 else 0.0
                ),
                "current_weak_mode_count": int(np.count_nonzero(weak)),
                "historical_source_factor_counts": dict(factor.get(
                    "marginal_source_factor_counts", {}
                )),
                "historical_lidar_pre_schur_translation_trace": float(
                    factor.get("marginal_lidar_translation_trace", 0.0)
                ),
                "historical_lidar_attenuated_weak_trace": float(
                    factor.get("marginal_lidar_weak_translation_trace", 0.0)
                ),
            }
        return {
            "active": False,
            "position_information_trace": 0.0,
            "current_weak_position_information_trace": 0.0,
            "current_strong_position_information_trace": 0.0,
            "current_weak_position_information_fraction": 0.0,
            "current_weak_mode_count": int(np.count_nonzero(weak)),
            "historical_source_factor_counts": {},
            "historical_lidar_pre_schur_translation_trace": 0.0,
            "historical_lidar_attenuated_weak_trace": 0.0,
        }

    def _marginalize_if_needed(self):
        started = time.perf_counter()
        profile_started = self._profile_start()
        marginalized = False
        while len(self._states) > self.max_states:
            marginalized = True
            eliminated = [
                factor for factor in self._factors if 0 in factor["indices"]
            ]
            source_counts = defaultdict(int)
            lidar_translation_trace = 0.0
            lidar_weak_translation_trace = 0.0
            lidar_weak_mode_count = 0
            for factor in eliminated:
                if factor["name"] == "marginal_prior":
                    for name, count in factor.get(
                        "marginal_source_factor_counts", {}
                    ).items():
                        source_counts[name] += int(count)
                    lidar_translation_trace += float(factor.get(
                        "marginal_lidar_translation_trace", 0.0
                    ))
                    lidar_weak_translation_trace += float(factor.get(
                        "marginal_lidar_weak_translation_trace", 0.0
                    ))
                    lidar_weak_mode_count += int(factor.get(
                        "marginal_lidar_weak_mode_count", 0
                    ))
                    continue
                source_counts[factor["name"]] += 1
                if factor["name"] != "lidar_point_plane":
                    continue
                local_hessian = np.zeros((len(self._states) * STATE_SIZE,) * 2)
                local_gradient = np.zeros(len(self._states) * STATE_SIZE)
                self._factor_normal(
                    factor, self._states, local_hessian, local_gradient
                )
                block = local_hessian[:3, :3]
                lidar_translation_trace += float(np.trace(block))
                scale = factor.get(
                    "translation_subspace_scale", np.eye(3)
                )
                weak_projector = np.eye(3) - np.asarray(scale, dtype=float)
                lidar_weak_translation_trace += float(
                    np.trace(weak_projector @ block)
                )
                lidar_weak_mode_count += int(
                    np.count_nonzero(
                        np.linalg.eigvalsh(weak_projector) > 1.0e-6
                    )
                )
            retained = [
                factor for factor in self._factors if 0 not in factor["indices"]]
            references = [state.copy() for state in self._states[1:]]
            if eliminated:
                hessian, gradient, _ = self._normal(eliminated, self._states)
                first = slice(0, STATE_SIZE)
                rest = slice(STATE_SIZE, hessian.shape[0])
                eliminated_hessian = 0.5 * (
                    hessian[first, first] + hessian[first, first].T
                )
                cross = hessian[first, rest]
                eigenvalues, eigenvectors = np.linalg.eigh(eliminated_hessian)
                scale = max(1.0, float(np.max(np.abs(eigenvalues))))
                active = eigenvalues > self.marginal_rank_tolerance * scale
                if np.any(active):
                    inverse = (
                        eigenvectors[:, active]
                        @ np.diag(1.0 / eigenvalues[active])
                        @ eigenvectors[:, active].T
                    )
                    solved_cross = inverse @ cross
                    solved_gradient = inverse @ gradient[first]
                else:
                    solved_cross = np.zeros_like(cross)
                    solved_gradient = np.zeros_like(gradient[first])
                schur_hessian = hessian[rest, rest] - cross.T @ solved_cross
                schur_gradient = gradient[rest] - cross.T @ solved_gradient
                schur_hessian = 0.5 * (schur_hessian + schur_hessian.T)
                # Preserve a bounded, positive-semidefinite prior even when
                # the eliminated block was rank deficient.  Components of the
                # linear term in discarded null directions have no finite
                # quadratic model and must be discarded with those directions.
                try:
                    # A globally anchored, observable window is normally SPD.
                    # Cholesky is substantially cheaper than a full dense
                    # eigendecomposition and does not alter the prior.
                    np.linalg.cholesky(schur_hessian)
                except np.linalg.LinAlgError:
                    eigenvalues, eigenvectors = np.linalg.eigh(schur_hessian)
                    scale = max(1.0, float(np.max(np.abs(eigenvalues))))
                    active = eigenvalues > self.marginal_rank_tolerance * scale
                    if np.any(active):
                        basis = eigenvectors[:, active]
                        schur_hessian = (
                            basis * eigenvalues[active]
                        ) @ basis.T
                        schur_gradient = basis @ (basis.T @ schur_gradient)
                    else:
                        schur_hessian = np.zeros_like(schur_hessian)
                        schur_gradient = np.zeros_like(schur_gradient)
            else:
                schur_hessian = None
                schur_gradient = None
            self._states = self._states[1:]
            self._last_hessian = None
            for factor in retained:
                factor["indices"] = tuple(
                    index - 1 for index in factor["indices"])
            self._factors = retained
            if schur_hessian is not None and schur_hessian.size:
                self.last_marginal_prior_diagnostic = {
                    "source_factor_counts": dict(source_counts),
                    "lidar_translation_trace": lidar_translation_trace,
                    "lidar_weak_translation_trace": (
                        lidar_weak_translation_trace
                    ),
                    "lidar_weak_mode_count": lidar_weak_mode_count,
                }
                self._factors.append({
                    "name": "marginal_prior",
                    "indices": tuple(range(len(self._states))),
                    "residual_dimension": schur_hessian.shape[0],
                    "enabled": True,
                    "reliability_weight": 1.0,
                    "covariance_inflation": 1.0,
                    "effective_weight": 1.0,
                    "normal_hessian": schur_hessian,
                    "normal_gradient": schur_gradient,
                    "references": references,
                    "marginal_source_factor_counts": dict(source_counts),
                    "marginal_lidar_translation_trace": (
                        lidar_translation_trace
                    ),
                    "marginal_lidar_weak_translation_trace": (
                        lidar_weak_translation_trace
                    ),
                    "marginal_lidar_weak_mode_count": (
                        lidar_weak_mode_count
                    ),
                })
        self._last_marginalization_ms = (
            (time.perf_counter() - started) * 1000.0 if marginalized else 0.0
        )
        if marginalized:
            if self._profile_cycle is not None:
                self._profile_cycle["marginalization_happened"] = True
            self._profile_stop("marginalization", profile_started)
