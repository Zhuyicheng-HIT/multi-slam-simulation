"""Nonlinear SO(3) fixed-lag backend for the Ultra-Fusion reproduction."""

from collections import defaultdict, deque
from dataclasses import dataclass
import math
import time
from typing import Mapping, Sequence

import numpy as np

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
    point_plane_residual_jacobian,
    rpy_to_rotation_matrix,
)
from .window import FactorRecord, FactorResidual, _scheduler_values
from .visual_reprojection import (
    VisualTrackBatch,
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
        self._last_solve_ms = 0.0
        self._last_rejected_steps = 0
        self._last_hessian = None
        self._last_marginalization_ms = 0.0
        self.profiling_enabled = bool(profiling_enabled)
        self.profiling_capacity = max(64, int(profiling_capacity))
        self._profile_samples = defaultdict(
            lambda: deque(maxlen=self.profiling_capacity)
        )

    def _profile_start(self):
        return time.perf_counter_ns() if self.profiling_enabled else None

    def _profile_stop(self, name, started_ns):
        if started_ns is None:
            return 0.0
        elapsed_ms = (time.perf_counter_ns() - started_ns) * 1.0e-6
        self._profile_samples[str(name)].append(elapsed_ms)
        return elapsed_ms

    def profile_summary(self):
        """Return bounded wall-time percentiles for opt-in runtime profiling."""
        if not self.profiling_enabled:
            return {}
        summary = {}
        for name, samples in self._profile_samples.items():
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

    def add_native_lidar_correspondences(self, index, factor, decision=None):
        if not isinstance(factor, NativeLidarPoseNormal):
            raise ValueError("native LiDAR factor has the wrong type")
        if factor.lidar_points is None:
            raise ValueError("native LiDAR factor lacks raw correspondences")
        self._append(
            "lidar_point_plane",
            (index,
             ),
            factor.matched_points,
            decision,
            measurement=factor,
            variance=np.full(
                factor.matched_points,
                factor.measurement_variance),
        )

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

    def add_optical_flow(
            self,
            previous,
            current,
            delta_position,
            covariance=1.0,
            decision=None):
        self._append(
            "optical_flow", (previous, current), 3, decision,
            measurement=np.asarray(delta_position, dtype=float),
            variance=_positive_diagonal(covariance, 3),
        )

    def add_optical_flow_body(
        self, previous, current, delta_body, linearization_yaw=None,
        covariance=1.0, decision=None,
    ):
        self._append(
            "optical_flow_body", (previous, current), 3, decision,
            measurement=np.asarray(delta_body, dtype=float),
            variance=_positive_diagonal(covariance, 3),
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
        if name == "optical_flow":
            return (
                states[indices[1]][POSITION]
                - states[indices[0]][POSITION]
                - factor["measurement"]
            )
        if name == "optical_flow_body":
            return (
                states[indices[1]][POSITION]
                - states[indices[0]][POSITION]
                - rpy_to_rotation_matrix(states[indices[0]][ROTATION])
                @ factor["measurement"]
            )
        if name == "visual_reprojection":
            return visual_reprojection_residual_jacobians(
                states[indices[0]], states[indices[1]], factor["measurement"]
            )[0]
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

    def _factor_normal(self, factor, states):
        dimension = len(states) * STATE_SIZE
        hessian = np.zeros((dimension, dimension), dtype=float)
        gradient = np.zeros(dimension, dtype=float)
        if not factor["enabled"]:
            return hessian, gradient, 0.0
        name = factor["name"]
        if name == "marginal_prior":
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
            residual, pose_jacobian = point_plane_residual_jacobian(
                factor["measurement"], states[index][:6]
            )
            jacobian = np.zeros((residual.size, STATE_SIZE))
            jacobian[:, :6] = pose_jacobian
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
            block = jacobian.T @ (information[:, None] * jacobian)
            vector = jacobian.T @ (information * residual)
            hessian[start:start + STATE_SIZE,
                    start:start + STATE_SIZE] += block
            gradient[start:start + STATE_SIZE] += vector
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
            if name == "gnss":
                jacobians[factor["indices"][0]] = np.pad(
                    np.eye(3), ((0, 0), (0, STATE_SIZE - 3))
                )
            elif name == "optical_flow":
                minus = np.zeros((3, STATE_SIZE))
                plus = np.zeros((3, STATE_SIZE))
                minus[:, POSITION] = -np.eye(3)
                plus[:, POSITION] = np.eye(3)
                jacobians[factor["indices"][0]] = minus
                jacobians[factor["indices"][1]] = plus
            elif name == "optical_flow_body":
                previous, current = factor["indices"]
                previous_jacobian = np.zeros((3, STATE_SIZE))
                current_jacobian = np.zeros((3, STATE_SIZE))
                rotation = rpy_to_rotation_matrix(states[previous][ROTATION])
                previous_jacobian[:, POSITION] = -np.eye(3)
                previous_jacobian[:, ROTATION] = rotation @ skew(
                    factor["measurement"]
                )
                current_jacobian[:, POSITION] = np.eye(3)
                jacobians[previous] = previous_jacobian
                jacobians[current] = current_jacobian
            else:
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

    def _normal(self, factors=None, states=None):
        normal_started = self._profile_start()
        states = self._states if states is None else states
        factors = self._factors if factors is None else factors
        dimension = len(states) * STATE_SIZE
        hessian = np.zeros((dimension, dimension))
        gradient = np.zeros(dimension)
        cost = 0.0
        for factor in factors:
            factor_started = self._profile_start()
            factor_hessian, factor_gradient, factor_cost = self._factor_normal(
                factor, states
            )
            self._profile_stop(
                f"factor_{factor['name']}", factor_started
            )
            hessian += factor_hessian
            gradient += factor_gradient
            cost += factor_cost
        self._profile_stop("factor_graph_linearization", normal_started)
        return hessian, gradient, cost

    def optimize(self):
        if not self._states:
            return []
        started = time.perf_counter()
        profile_started = self._profile_start()
        accepted_iterations = 0
        rejected_steps = 0
        states = self.states()
        hessian, gradient, current_cost = self._normal(states=states)
        self._last_initial_cost = float(current_cost)
        damping = self._lm_damping
        for _ in range(self.max_iterations):
            if float(np.max(np.abs(gradient))) < 1.0e-10:
                break
            diagonal_scale = np.maximum(np.abs(np.diag(hessian)), 1.0)
            accepted = False
            converged = False
            for _ in range(self.lm_max_trials):
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
                candidate = []
                for index, state in enumerate(states):
                    local = increment[
                        index * STATE_SIZE:(index + 1) * STATE_SIZE
                    ]
                    candidate.append(state_plus(state, local))
                self._profile_stop("state_update", update_started)
                candidate_hessian, candidate_gradient, candidate_cost = self._normal(
                    states=candidate)
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

    def _marginalize_if_needed(self):
        started = time.perf_counter()
        profile_started = self._profile_start()
        marginalized = False
        while len(self._states) > self.max_states:
            marginalized = True
            eliminated = [
                factor for factor in self._factors if 0 in factor["indices"]
            ]
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
                })
        self._last_marginalization_ms = (
            (time.perf_counter() - started) * 1000.0 if marginalized else 0.0
        )
        if marginalized:
            self._profile_stop("marginalization", profile_started)
