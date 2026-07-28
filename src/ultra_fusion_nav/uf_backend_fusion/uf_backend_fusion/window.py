"""A bounded tangent-space sliding-window backend prototype.

The first backend increment deliberately uses linear residual blocks in a local
tangent space. It exercises the factor contract and scheduler weights before a
full SE(3) marginalization implementation is introduced.  Native FAST-LIO
point-to-plane rows can enter as a condensed pose normal equation, avoiding a
second pose anchor made from the same LiDAR/IMU estimate.
"""

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence
import warnings

import numpy as np

try:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        from scipy.sparse import csc_matrix
        from scipy.sparse.linalg import spsolve
    SPARSE_SOLVER_AVAILABLE = True
except Exception:  # pragma: no cover - exercised on minimal ROS images
    csc_matrix = None
    spsolve = None
    SPARSE_SOLVER_AVAILABLE = False


STATE_SIZE = 15
POSITION = slice(0, 3)
ROTATION = slice(3, 6)
VELOCITY = slice(6, 9)
ACCEL_BIAS = slice(9, 12)
GYRO_BIAS = slice(12, 15)


@dataclass(frozen=True)
class FactorRecord:
    name: str
    state_indices: tuple[int, ...]
    residual_dimension: int
    enabled: bool
    reliability_weight: float
    covariance_inflation: float
    effective_weight: float


@dataclass(frozen=True)
class FactorResidual:
    name: str
    state_indices: tuple[int, ...]
    residual_dimension: int
    enabled: bool
    mahalanobis_squared: float


def _covariance_diagonal(covariance, dimension: int) -> np.ndarray:
    values = np.asarray(covariance, dtype=float)
    if values.ndim == 0:
        diagonal = np.full(dimension, float(values), dtype=float)
    elif values.ndim == 1:
        if values.size != dimension:
            raise ValueError("covariance vector dimension does not match residual")
        diagonal = values
    elif values.ndim == 2:
        if values.shape != (dimension, dimension):
            raise ValueError("covariance matrix dimension does not match residual")
        diagonal = np.diag(values)
    else:
        raise ValueError("covariance must be scalar, vector, or square matrix")
    if np.any(~np.isfinite(diagonal)) or np.any(diagonal <= 0.0):
        raise ValueError("covariance diagonal must be finite and positive")
    return diagonal


def _scheduler_values(decision: Mapping[str, object] | None) -> tuple[bool, float, float]:
    if decision is None:
        return True, 1.0, 1.0
    enabled = bool(decision.get("factor_enabled", decision.get("enabled", True)))
    reliability_weight = float(
        decision.get("reliability_weight", decision.get("weight", 1.0))
    )
    covariance_inflation = float(decision.get("covariance_inflation", 1.0))
    if not np.isfinite(reliability_weight) or reliability_weight < 0.0:
        raise ValueError("reliability weight must be finite and non-negative")
    if not np.isfinite(covariance_inflation) or covariance_inflation < 1.0:
        raise ValueError("covariance inflation must be finite and at least one")
    return enabled, reliability_weight, covariance_inflation


class SlidingWindowBackend:
    """Solve a bounded weighted linearized factor window.

    Each state is ordered as ``{p(3), theta(3), v(3), ba(3), bg(3)}``. The
    current factor blocks are linear tangent-space constraints; callers provide
    the local linearization measurements. The scheduler decision is applied as
    ``sqrt(s / inflation)`` to every residual row, matching the factor-level
    weighting contract without silently feeding one modality into another.
    """

    def __init__(self, max_states: int = 10, damping: float = 1.0e-8,
                 solver: str = "auto", carry_marginal_prior: bool = True,
                 marginal_prior_variance: float = 1.0e-2):
        if max_states < 1:
            raise ValueError("max_states must be positive")
        if damping <= 0.0:
            raise ValueError("damping must be positive")
        if marginal_prior_variance <= 0.0:
            raise ValueError("marginal_prior_variance must be positive")
        self.max_states = int(max_states)
        self.damping = float(damping)
        requested_solver = str(solver).lower()
        if requested_solver not in {"auto", "dense", "sparse"}:
            raise ValueError("solver must be auto, dense, or sparse")
        if requested_solver == "sparse" and not SPARSE_SOLVER_AVAILABLE:
            raise RuntimeError("sparse solver requested but scipy is unavailable")
        self.solver = (
            "sparse" if requested_solver == "auto" and SPARSE_SOLVER_AVAILABLE
            else ("dense" if requested_solver == "auto" else requested_solver)
        )
        self.carry_marginal_prior = bool(carry_marginal_prior)
        self.marginal_prior_variance = float(marginal_prior_variance)
        self._states: list[np.ndarray] = []
        self._factors: list[dict[str, object]] = []
        self._last_cost = 0.0

    @property
    def state_count(self) -> int:
        return len(self._states)

    @property
    def factor_count(self) -> int:
        return len(self._factors)

    def add_state(self, initial: Sequence[float] | None = None) -> int:
        value = np.zeros(STATE_SIZE, dtype=float)
        if initial is not None:
            value[:] = np.asarray(initial, dtype=float)
            if value.shape != (STATE_SIZE,):
                raise ValueError(f"state must have {STATE_SIZE} elements")
        self._states.append(value)
        self._marginalize_if_needed()
        return len(self._states) - 1

    def state(self, index: int) -> np.ndarray:
        return self._states[index].copy()

    def states(self) -> list[np.ndarray]:
        return [state.copy() for state in self._states]

    def _check_index(self, index: int) -> None:
        if index < 0 or index >= len(self._states):
            raise IndexError(f"state index {index} is outside the active window")

    def _append_factor(
        self,
        name: str,
        blocks: Iterable[tuple[int, np.ndarray]],
        measurement: Sequence[float],
        covariance,
        decision: Mapping[str, object] | None,
    ) -> None:
        measurement_array = np.asarray(measurement, dtype=float)
        if measurement_array.ndim != 1 or np.any(~np.isfinite(measurement_array)):
            raise ValueError("factor measurement must be a finite vector")
        dimension = int(measurement_array.size)
        diagonal = _covariance_diagonal(covariance, dimension)
        enabled, reliability_weight, covariance_inflation = _scheduler_values(decision)
        effective_weight = reliability_weight / covariance_inflation
        records = []
        for index, block in blocks:
            self._check_index(index)
            matrix = np.asarray(block, dtype=float)
            if matrix.shape != (dimension, STATE_SIZE):
                raise ValueError("factor block has the wrong shape")
            if np.any(~np.isfinite(matrix)):
                raise ValueError("factor block must be finite")
            records.append((int(index), matrix))
        self._factors.append({
            "name": name,
            "kind": "linear",
            "blocks": records,
            "measurement": measurement_array,
            "variance": diagonal,
            "residual_dimension": dimension,
            "enabled": enabled and effective_weight > 0.0,
            "reliability_weight": reliability_weight,
            "covariance_inflation": covariance_inflation,
            "effective_weight": effective_weight,
        })

    @staticmethod
    def _block(dimension: int, columns: slice) -> np.ndarray:
        block = np.zeros((dimension, STATE_SIZE), dtype=float)
        width = columns.stop - columns.start
        if width != dimension:
            raise ValueError("slice width must equal factor dimension")
        block[:, columns] = np.eye(dimension)
        return block

    def _ensure_unique_lidar_factor(self, index: int) -> None:
        """One scan state may contain a pose proxy or native LiDAR, never both."""
        for factor in self._factors:
            if factor["name"] not in {"lidar_pose", "lidar_point_plane"}:
                continue
            if any(state_index == index for state_index, _ in factor["blocks"]):
                raise ValueError(
                    f"state {index} already has a LiDAR factor; duplicate information is forbidden"
                )

    def add_prior(self, index: int, state: Sequence[float], covariance=1.0) -> None:
        block = np.eye(STATE_SIZE, dtype=float)
        self._append_factor("prior", [(index, block)], state, covariance, None)

    def add_lidar_pose(
        self, index: int, position: Sequence[float], rotation: Sequence[float],
        covariance=1.0, decision: Mapping[str, object] | None = None,
    ) -> None:
        self._check_index(index)
        self._ensure_unique_lidar_factor(index)
        block = np.zeros((6, STATE_SIZE), dtype=float)
        block[:3, POSITION] = np.eye(3)
        block[3:, ROTATION] = np.eye(3)
        self._append_factor(
            "lidar_pose", [(index, block)],
            np.concatenate((np.asarray(position, dtype=float), np.asarray(rotation, dtype=float))),
            covariance, decision)

    def add_native_lidar_normal(
        self,
        index: int,
        linearization_pose: Sequence[float],
        pose_hessian: Sequence[float],
        pose_gradient: Sequence[float],
        measurement_variance: float,
        residual_dimension: int,
        residual_squared: float,
        decision: Mapping[str, object] | None = None,
    ) -> None:
        """Add a condensed point-to-plane factor at a pose linearization.

        For ``r(x) = r0 + J (x - x0)``, FAST-LIO exports ``H=J.T J`` and
        ``g=J.T r0``.  The resulting normal equation is
        ``H x = H x0 - g``.  The caller has already transformed FAST-LIO's
        right-perturbation rotation columns into this backend's pose coordinates.
        """
        self._check_index(index)
        self._ensure_unique_lidar_factor(index)
        linearization = np.asarray(linearization_pose, dtype=float)
        hessian = np.asarray(pose_hessian, dtype=float)
        gradient = np.asarray(pose_gradient, dtype=float)
        if linearization.shape != (6,) or np.any(~np.isfinite(linearization)):
            raise ValueError("native LiDAR linearization pose must be a finite 6-vector")
        if hessian.shape == (36,):
            hessian = hessian.reshape(6, 6)
        if hessian.shape != (6, 6) or np.any(~np.isfinite(hessian)):
            raise ValueError("native LiDAR pose Hessian must be a finite 6x6 matrix")
        if gradient.shape != (6,) or np.any(~np.isfinite(gradient)):
            raise ValueError("native LiDAR pose gradient must be a finite 6-vector")
        symmetry_error = float(np.linalg.norm(hessian - hessian.T))
        if symmetry_error > 1.0e-7 * max(1.0, float(np.linalg.norm(hessian))):
            raise ValueError("native LiDAR pose Hessian must be symmetric")
        hessian = 0.5 * (hessian + hessian.T)
        minimum_eigenvalue = float(np.linalg.eigvalsh(hessian)[0])
        if minimum_eigenvalue < -1.0e-7 * max(1.0, float(np.linalg.norm(hessian))):
            raise ValueError("native LiDAR pose Hessian must be positive semidefinite")
        variance = float(measurement_variance)
        residual_squared = float(residual_squared)
        residual_dimension = int(residual_dimension)
        if not np.isfinite(variance) or variance <= 0.0:
            raise ValueError("native LiDAR measurement variance must be positive")
        if not np.isfinite(residual_squared) or residual_squared < 0.0:
            raise ValueError("native LiDAR residual norm must be finite and non-negative")
        if residual_dimension < 1:
            raise ValueError("native LiDAR factor must contain at least one residual")
        enabled, reliability_weight, covariance_inflation = _scheduler_values(decision)
        effective_weight = reliability_weight / covariance_inflation
        self._factors.append({
            "name": "lidar_point_plane",
            "kind": "normal",
            "blocks": [(int(index), None)],
            "normal_hessian": hessian,
            "normal_rhs": hessian @ linearization - gradient,
            "normal_gradient": gradient,
            "linearization": linearization,
            "residual_squared": residual_squared,
            "measurement_variance": variance,
            "residual_dimension": residual_dimension,
            "enabled": enabled and effective_weight > 0.0,
            "reliability_weight": reliability_weight,
            "covariance_inflation": covariance_inflation,
            "effective_weight": effective_weight,
        })

    def add_gnss(
        self, index: int, position: Sequence[float], covariance=1.0,
        decision: Mapping[str, object] | None = None,
    ) -> None:
        self._append_factor(
            "gnss", [(index, self._block(3, POSITION))], position, covariance, decision)

    def add_optical_flow(
        self, previous: int, current: int, delta_position: Sequence[float], covariance=1.0,
        decision: Mapping[str, object] | None = None,
    ) -> None:
        plus = self._block(3, POSITION)
        minus = -plus
        self._append_factor(
            "optical_flow", [(current, plus), (previous, minus)],
            delta_position, covariance, decision)

    def add_optical_flow_body(
        self,
        previous: int,
        current: int,
        delta_body: Sequence[float],
        linearization_yaw: float,
        covariance=1.0,
        decision: Mapping[str, object] | None = None,
    ) -> None:
        """Add a body-frame optical-flow displacement with yaw sensitivity.

        With ``d_map = Rz(yaw) d_body``, the first-order residual is
        ``p_k-p_{k-1}-d_map-dRz*d_yaw``.  Keeping the yaw derivative makes
        optical flow useful for heading correction while LiDAR is degraded.
        """
        delta_body = np.asarray(delta_body, dtype=float)
        if delta_body.shape != (3,) or np.any(~np.isfinite(delta_body)):
            raise ValueError("optical-flow body displacement must be a finite 3-vector")
        linearization_yaw = float(linearization_yaw)
        if not np.isfinite(linearization_yaw):
            raise ValueError("optical-flow yaw linearization must be finite")
        cosine, sine = np.cos(linearization_yaw), np.sin(linearization_yaw)
        rotation = np.asarray([
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        yaw_derivative = np.asarray([
            [-sine, -cosine, 0.0],
            [cosine, -sine, 0.0],
            [0.0, 0.0, 0.0],
        ], dtype=float) @ delta_body
        plus = self._block(3, POSITION)
        minus = -plus
        previous_block = minus.copy()
        previous_block[:, ROTATION] = 0.0
        previous_block[:, ROTATION.start + 2] = -yaw_derivative
        measurement = rotation @ delta_body - yaw_derivative * linearization_yaw
        self._append_factor(
            "optical_flow_body",
            [(current, plus), (previous, previous_block)],
            measurement,
            covariance,
            decision,
        )

    def add_imu_delta(
        self, previous: int, current: int, delta_position: Sequence[float],
        delta_velocity: Sequence[float], covariance=1.0,
        decision: Mapping[str, object] | None = None,
    ) -> None:
        block = np.zeros((6, STATE_SIZE), dtype=float)
        block[:3, POSITION] = np.eye(3)
        block[3:, VELOCITY] = np.eye(3)
        negative = -block
        self._append_factor(
            "imu", [(current, block), (previous, negative)],
            np.concatenate((np.asarray(delta_position, dtype=float), np.asarray(delta_velocity, dtype=float))),
            covariance, decision)

    def add_bias_aware_imu(
        self,
        previous: int,
        current: int,
        dt_s: float,
        delta_position: Sequence[float],
        delta_velocity: Sequence[float],
        delta_rotation: Sequence[float],
        position_accel_bias_jacobian: Sequence[float],
        position_gyro_bias_jacobian: Sequence[float],
        velocity_accel_bias_jacobian: Sequence[float],
        velocity_gyro_bias_jacobian: Sequence[float],
        rotation_gyro_bias_jacobian: Sequence[float],
        gravity: Sequence[float] = (0.0, 0.0, -9.81),
        covariance=1.0,
        bias_random_walk_covariance=1.0,
        decision: Mapping[str, object] | None = None,
    ) -> None:
        """Add a first-order local tangent IMU factor.

        The residual is the standard position/velocity/rotation preintegration
        contract with a first-order correction from the previous state's
        accelerometer and gyro biases, followed by bias random-walk rows. The
        factor is linear in the current state and therefore remains compatible
        with this bounded backend; a manifold relinearization is a later gate.
        """
        dt_s = float(dt_s)
        if not np.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("IMU interval must be finite and positive")
        gravity = np.asarray(gravity, dtype=float)
        if gravity.shape != (3,) or np.any(~np.isfinite(gravity)):
            raise ValueError("gravity must be a finite 3-vector")
        jacobians = [
            np.asarray(value, dtype=float).reshape(3, 3)
            for value in (
                position_accel_bias_jacobian,
                position_gyro_bias_jacobian,
                velocity_accel_bias_jacobian,
                velocity_gyro_bias_jacobian,
                rotation_gyro_bias_jacobian,
            )
        ]
        if any(np.any(~np.isfinite(value)) for value in jacobians):
            raise ValueError("IMU bias Jacobians must be finite 3x3 matrices")
        position_accel, position_gyro, velocity_accel, velocity_gyro, rotation_gyro = jacobians
        previous_block = np.zeros((15, STATE_SIZE), dtype=float)
        current_block = np.zeros((15, STATE_SIZE), dtype=float)
        previous_block[0:3, POSITION] = -np.eye(3)
        previous_block[0:3, VELOCITY] = -dt_s * np.eye(3)
        previous_block[0:3, ACCEL_BIAS] = -position_accel
        previous_block[0:3, GYRO_BIAS] = -position_gyro
        current_block[0:3, POSITION] = np.eye(3)
        previous_block[3:6, VELOCITY] = -np.eye(3)
        previous_block[3:6, ACCEL_BIAS] = -velocity_accel
        previous_block[3:6, GYRO_BIAS] = -velocity_gyro
        current_block[3:6, VELOCITY] = np.eye(3)
        previous_block[6:9, ROTATION] = -np.eye(3)
        previous_block[6:9, GYRO_BIAS] = -rotation_gyro
        current_block[6:9, ROTATION] = np.eye(3)
        previous_block[9:12, ACCEL_BIAS] = -np.eye(3)
        current_block[9:12, ACCEL_BIAS] = np.eye(3)
        previous_block[12:15, GYRO_BIAS] = -np.eye(3)
        current_block[12:15, GYRO_BIAS] = np.eye(3)
        delta_position = np.asarray(delta_position, dtype=float)
        delta_velocity = np.asarray(delta_velocity, dtype=float)
        delta_rotation = np.asarray(delta_rotation, dtype=float)
        measurement = np.concatenate((
            delta_position + 0.5 * gravity * dt_s * dt_s,
            delta_velocity + gravity * dt_s,
            delta_rotation,
            np.zeros(6, dtype=float),
        ))
        base_covariance = np.asarray(covariance, dtype=float)
        if base_covariance.ndim == 0:
            base_covariance = np.full(9, float(base_covariance), dtype=float)
        if base_covariance.shape == (15,):
            full_covariance = base_covariance
        elif base_covariance.shape == (9,):
            bias_covariance = np.asarray(bias_random_walk_covariance, dtype=float)
            if bias_covariance.ndim == 0:
                bias_covariance = np.full(6, float(bias_covariance), dtype=float)
            if bias_covariance.shape != (6,):
                raise ValueError("bias random-walk covariance must be scalar or 6-vector")
            full_covariance = np.concatenate((base_covariance, bias_covariance))
        else:
            raise ValueError("IMU covariance must contain 9 or 15 diagonal entries")
        self._append_factor(
            "imu_preintegrated", [(previous, previous_block), (current, current_block)],
            measurement, full_covariance, decision)

    def add_rgbd_pose(
        self, index: int, position: Sequence[float], rotation: Sequence[float],
        covariance=1.0, decision: Mapping[str, object] | None = None,
    ) -> None:
        block = np.zeros((6, STATE_SIZE), dtype=float)
        block[:3, POSITION] = np.eye(3)
        block[3:, ROTATION] = np.eye(3)
        self._append_factor(
            "rgbd", [(index, block)],
            np.concatenate((np.asarray(position, dtype=float), np.asarray(rotation, dtype=float))),
            covariance, decision)

    def _factor_normal_contribution(self, factor, state_count):
        """Return one factor's canonical ``H x = b`` contribution."""
        dimension = int(state_count) * STATE_SIZE
        hessian = np.zeros((dimension, dimension), dtype=float)
        rhs = np.zeros(dimension, dtype=float)
        if not bool(factor["enabled"]):
            return hessian, rhs
        if factor["kind"] == "normal":
            index = int(factor["blocks"][0][0])
            start = index * STATE_SIZE
            indices = np.arange(start, start + 6)
            scale = float(factor["effective_weight"]) / float(
                factor["measurement_variance"]
            )
            hessian[np.ix_(indices, indices)] += (
                scale * np.asarray(factor["normal_hessian"], dtype=float)
            )
            rhs[indices] += scale * np.asarray(factor["normal_rhs"], dtype=float)
            return hessian, rhs
        if factor["kind"] == "marginal_normal":
            indices = np.concatenate([
                np.arange(index * STATE_SIZE, (index + 1) * STATE_SIZE)
                for index, _ in factor["blocks"]
            ])
            hessian[np.ix_(indices, indices)] += np.asarray(
                factor["normal_hessian"], dtype=float
            )
            rhs[indices] += np.asarray(factor["normal_rhs"], dtype=float)
            return hessian, rhs
        measurement = np.asarray(factor["measurement"], dtype=float)
        matrix = np.zeros((measurement.size, dimension), dtype=float)
        for index, block in factor["blocks"]:
            start = index * STATE_SIZE
            matrix[:, start:start + STATE_SIZE] += np.asarray(block, dtype=float)
        information = float(factor["effective_weight"]) / np.asarray(
            factor["variance"], dtype=float
        )
        hessian += matrix.T @ (information[:, None] * matrix)
        rhs += matrix.T @ (information * measurement)
        return hessian, rhs

    def optimize(self) -> list[np.ndarray]:
        if not self._states:
            return []
        dimension = len(self._states) * STATE_SIZE
        hessian = np.eye(dimension, dtype=float) * self.damping
        gradient = np.zeros(dimension, dtype=float)
        cost = 0.0
        state_vector = np.concatenate(self._states)
        for factor in self._factors:
            if not bool(factor["enabled"]):
                continue
            if factor["kind"] == "marginal_normal":
                indices = np.concatenate([
                    np.arange(index * STATE_SIZE, (index + 1) * STATE_SIZE)
                    for index, _ in factor["blocks"]
                ])
                local_hessian = np.asarray(
                    factor["normal_hessian"], dtype=float
                )
                local_rhs = np.asarray(factor["normal_rhs"], dtype=float)
                hessian[np.ix_(indices, indices)] += local_hessian
                gradient[indices] += local_rhs
                local_state = state_vector[indices]
                cost += max(
                    0.0,
                    float(local_state @ local_hessian @ local_state)
                    - 2.0 * float(local_rhs @ local_state),
                )
                continue
            if factor["kind"] == "normal":
                index = int(factor["blocks"][0][0])
                start = index * STATE_SIZE
                pose_slice = slice(start, start + 6)
                scale = float(factor["effective_weight"]) / float(
                    factor["measurement_variance"]
                )
                local_hessian = np.asarray(factor["normal_hessian"], dtype=float)
                local_rhs = np.asarray(factor["normal_rhs"], dtype=float)
                hessian[pose_slice, pose_slice] += scale * local_hessian
                gradient[pose_slice] += scale * local_rhs
                delta = self._states[index][:6] - np.asarray(
                    factor["linearization"], dtype=float
                )
                quadratic = (
                    float(factor["residual_squared"])
                    + 2.0 * float(np.asarray(factor["normal_gradient"]) @ delta)
                    + float(delta @ local_hessian @ delta)
                )
                cost += scale * max(0.0, quadratic)
                continue
            measurement = factor["measurement"]
            variance = factor["variance"]
            weight = float(factor["effective_weight"])
            rows = int(measurement.size)
            matrix = np.zeros((rows, dimension), dtype=float)
            for index, block in factor["blocks"]:
                start = index * STATE_SIZE
                matrix[:, start:start + STATE_SIZE] += block
            information = weight / variance
            hessian += matrix.T @ (information[:, None] * matrix)
            gradient += matrix.T @ (information * measurement)
            residual = matrix @ state_vector - measurement
            cost += float(np.sum(information * residual * residual))
        if self.solver == "sparse":
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", Warning)
                solution = np.asarray(
                    spsolve(csc_matrix(hessian), gradient), dtype=float
                )
        else:
            solution = np.linalg.solve(hessian, gradient)
        if solution.shape != (dimension,) or np.any(~np.isfinite(solution)):
            raise np.linalg.LinAlgError("sliding-window solution is non-finite")
        self._states = [
            solution[index * STATE_SIZE:(index + 1) * STATE_SIZE].copy()
            for index in range(len(self._states))
        ]
        self._last_cost = cost
        return self.states()

    @property
    def last_cost(self) -> float:
        return self._last_cost

    def factor_summary(self) -> list[FactorRecord]:
        return [
            FactorRecord(
                name=str(factor["name"]),
                state_indices=tuple(index for index, _ in factor["blocks"]),
                residual_dimension=int(factor["residual_dimension"]),
                enabled=bool(factor["enabled"]),
                reliability_weight=float(factor["reliability_weight"]),
                covariance_inflation=float(factor["covariance_inflation"]),
                effective_weight=float(factor["effective_weight"]),
            )
            for factor in self._factors
        ]

    def latest_factor_residual(self, name: str, covariance=None) -> FactorResidual | None:
        """Evaluate the newest named factor against the optimized states.

        The caller may provide a nominal covariance distinct from the tuned
        optimization covariance. Scheduler weight and covariance inflation are
        never applied, keeping the evidence independent from the decision it
        will influence on the next scheduling cycle.
        """
        for factor in reversed(self._factors):
            if factor["name"] != name:
                continue
            if factor["kind"] == "marginal_normal":
                indices = [index for index, _ in factor["blocks"]]
                local_state = np.concatenate([self._states[index] for index in indices])
                hessian = np.asarray(factor["normal_hessian"], dtype=float)
                rhs = np.asarray(factor["normal_rhs"], dtype=float)
                quadratic = max(
                    0.0,
                    float(local_state @ hessian @ local_state)
                    - 2.0 * float(rhs @ local_state),
                )
                return FactorResidual(
                    name=str(factor["name"]),
                    state_indices=tuple(indices),
                    residual_dimension=int(factor["residual_dimension"]),
                    enabled=bool(factor["enabled"]),
                    mahalanobis_squared=quadratic,
                )
            if factor["kind"] == "normal":
                index = int(factor["blocks"][0][0])
                delta = self._states[index][:6] - np.asarray(
                    factor["linearization"], dtype=float
                )
                quadratic = (
                    float(factor["residual_squared"])
                    + 2.0 * float(np.asarray(factor["normal_gradient"]) @ delta)
                    + float(delta @ np.asarray(factor["normal_hessian"]) @ delta)
                )
                if covariance is None:
                    variance = float(factor["measurement_variance"])
                else:
                    supplied = np.asarray(covariance, dtype=float)
                    if supplied.ndim != 0 or not np.isfinite(supplied) or supplied <= 0.0:
                        raise ValueError(
                            "condensed native LiDAR residual accepts only scalar covariance"
                        )
                    variance = float(supplied)
                mahalanobis_squared = max(0.0, quadratic) / variance
                return FactorResidual(
                    name=str(factor["name"]),
                    state_indices=(index,),
                    residual_dimension=int(factor["residual_dimension"]),
                    enabled=bool(factor["enabled"]),
                    mahalanobis_squared=float(mahalanobis_squared),
                )
            residual = -np.asarray(factor["measurement"], dtype=float).copy()
            for index, block in factor["blocks"]:
                residual += np.asarray(block, dtype=float) @ self._states[index]
            variance = (
                np.asarray(factor["variance"], dtype=float)
                if covariance is None
                else _covariance_diagonal(covariance, int(residual.size))
            )
            mahalanobis_squared = float(np.sum(residual * residual / variance))
            if not np.isfinite(mahalanobis_squared):
                raise ValueError("factor residual must be finite")
            return FactorResidual(
                name=str(factor["name"]),
                state_indices=tuple(index for index, _ in factor["blocks"]),
                residual_dimension=int(residual.size),
                enabled=bool(factor["enabled"]),
                mahalanobis_squared=mahalanobis_squared,
            )
        return None

    def _marginalize_if_needed(self) -> None:
        excess = len(self._states) - self.max_states
        if excess <= 0:
            return
        state_count = len(self._states)
        removed_dimension = excess * STATE_SIZE
        retained_dimension = (state_count - excess) * STATE_SIZE
        eliminated_factors = [
            factor for factor in self._factors
            if any(index < excess for index, _ in factor["blocks"])
        ]
        schur_hessian = None
        schur_rhs = None
        if self.carry_marginal_prior and eliminated_factors:
            joint_hessian = np.zeros(
                (state_count * STATE_SIZE, state_count * STATE_SIZE), dtype=float
            )
            joint_rhs = np.zeros(state_count * STATE_SIZE, dtype=float)
            for factor in eliminated_factors:
                factor_hessian, factor_rhs = self._factor_normal_contribution(
                    factor, state_count
                )
                joint_hessian += factor_hessian
                joint_rhs += factor_rhs
            h_oo = joint_hessian[:removed_dimension, :removed_dimension]
            h_or = joint_hessian[:removed_dimension, removed_dimension:]
            h_ro = joint_hessian[removed_dimension:, :removed_dimension]
            h_rr = joint_hessian[removed_dimension:, removed_dimension:]
            b_o = joint_rhs[:removed_dimension]
            b_r = joint_rhs[removed_dimension:]
            regularized_h_oo = h_oo + self.damping * np.eye(removed_dimension)
            try:
                solved_cross = np.linalg.solve(regularized_h_oo, h_or)
                solved_rhs = np.linalg.solve(regularized_h_oo, b_o)
            except np.linalg.LinAlgError:
                inverse = np.linalg.pinv(regularized_h_oo)
                solved_cross = inverse @ h_or
                solved_rhs = inverse @ b_o
            schur_hessian = h_rr - h_ro @ solved_cross
            schur_hessian = 0.5 * (schur_hessian + schur_hessian.T)
            schur_rhs = b_r - h_ro @ solved_rhs

        self._states = self._states[excess:]
        retained = []
        for factor in self._factors:
            blocks = factor["blocks"]
            if any(index < excess for index, _ in blocks):
                continue
            factor["blocks"] = [(index - excess, block) for index, block in blocks]
            retained.append(factor)
        self._factors = retained
        if schur_hessian is None or schur_rhs is None:
            return
        active_states = []
        for index in range(retained_dimension // STATE_SIZE):
            block = slice(index * STATE_SIZE, (index + 1) * STATE_SIZE)
            if (
                np.linalg.norm(schur_hessian[block, :]) > 1.0e-12
                or np.linalg.norm(schur_rhs[block]) > 1.0e-12
            ):
                active_states.append(index)
        if not active_states:
            return
        active_indices = np.concatenate([
            np.arange(index * STATE_SIZE, (index + 1) * STATE_SIZE)
            for index in active_states
        ])
        self._factors.append({
            "name": "marginal_prior",
            "kind": "marginal_normal",
            "blocks": [(index, None) for index in active_states],
            "normal_hessian": schur_hessian[np.ix_(active_indices, active_indices)],
            "normal_rhs": schur_rhs[active_indices],
            "residual_dimension": int(active_indices.size),
            "enabled": True,
            "reliability_weight": 1.0,
            "covariance_inflation": 1.0,
            "effective_weight": 1.0,
        })
