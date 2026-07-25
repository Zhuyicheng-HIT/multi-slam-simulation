"""A bounded tangent-space sliding-window backend prototype.

The first backend increment deliberately uses linear residual blocks in a local
tangent space. It exercises the factor contract and scheduler weights before a
full SE(3), IMU preintegration, and marginalization implementation is introduced.
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
                 solver: str = "auto"):
        if max_states < 1:
            raise ValueError("max_states must be positive")
        if damping <= 0.0:
            raise ValueError("damping must be positive")
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
            "blocks": records,
            "measurement": measurement_array,
            "variance": diagonal,
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

    def add_prior(self, index: int, state: Sequence[float], covariance=1.0) -> None:
        block = np.eye(STATE_SIZE, dtype=float)
        self._append_factor("prior", [(index, block)], state, covariance, None)

    def add_lidar_pose(
        self, index: int, position: Sequence[float], rotation: Sequence[float],
        covariance=1.0, decision: Mapping[str, object] | None = None,
    ) -> None:
        block = np.zeros((6, STATE_SIZE), dtype=float)
        block[:3, POSITION] = np.eye(3)
        block[3:, ROTATION] = np.eye(3)
        self._append_factor(
            "lidar_pose", [(index, block)],
            np.concatenate((np.asarray(position, dtype=float), np.asarray(rotation, dtype=float))),
            covariance, decision)

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

    def optimize(self) -> list[np.ndarray]:
        if not self._states:
            return []
        dimension = len(self._states) * STATE_SIZE
        hessian = np.eye(dimension, dtype=float) * self.damping
        gradient = np.zeros(dimension, dtype=float)
        cost = 0.0
        for factor in self._factors:
            if not bool(factor["enabled"]):
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
            residual = matrix @ np.concatenate(self._states) - measurement
            cost += float(np.sum(information * residual * residual))
        if self.solver == "sparse":
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", Warning)
                solution = np.asarray(
                    spsolve(csc_matrix(hessian), gradient), dtype=float
                )
        else:
            solution = np.linalg.solve(hessian, gradient)
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
                residual_dimension=int(factor["measurement"].size),
                enabled=bool(factor["enabled"]),
                reliability_weight=float(factor["reliability_weight"]),
                covariance_inflation=float(factor["covariance_inflation"]),
                effective_weight=float(factor["effective_weight"]),
            )
            for factor in self._factors
        ]

    def _marginalize_if_needed(self) -> None:
        excess = len(self._states) - self.max_states
        if excess <= 0:
            return
        self._states = self._states[excess:]
        retained = []
        for factor in self._factors:
            blocks = factor["blocks"]
            if any(index < excess for index, _ in blocks):
                continue
            factor["blocks"] = [(index - excess, block) for index, block in blocks]
            retained.append(factor)
        self._factors = retained
