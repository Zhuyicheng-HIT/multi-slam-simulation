"""SO(3) local-coordinate helpers for the nonlinear fixed-lag backend."""

import math
from typing import Sequence

import numpy as np

from .native_lidar import rpy_to_rotation_matrix


STATE_SIZE = 15


def skew(value: Sequence[float]) -> np.ndarray:
    x, y, z = np.asarray(value, dtype=float)
    return np.asarray([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def so3_exp(rotation_vector: Sequence[float]) -> np.ndarray:
    vector = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    matrix = skew(vector)
    if angle <= 1.0e-8:
        return np.eye(3) + matrix + 0.5 * matrix @ matrix
    return (
        np.eye(3)
        + math.sin(angle) / angle * matrix
        + (1.0 - math.cos(angle)) / (angle * angle) * matrix @ matrix
    )


def so3_log(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=float)
    cosine = float(np.clip(0.5 * (np.trace(rotation) - 1.0), -1.0, 1.0))
    angle = math.acos(cosine)
    vector = np.asarray([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    if angle <= 1.0e-8:
        return 0.5 * vector
    if math.pi - angle <= 1.0e-5:
        eigenvalues, eigenvectors = np.linalg.eigh(0.5 * (rotation + np.eye(3)))
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        if float(axis @ vector) < 0.0:
            axis = -axis
        return axis * angle
    return vector * (0.5 * angle / math.sin(angle))


def so3_left_jacobian_inverse(rotation_vector: Sequence[float]) -> np.ndarray:
    vector = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    matrix = skew(vector)
    if angle <= 1.0e-6:
        return np.eye(3) - 0.5 * matrix + (1.0 / 12.0) * matrix @ matrix
    coefficient = (
        1.0 - 0.5 * angle / math.tan(0.5 * angle)
    ) / (angle * angle)
    return np.eye(3) - 0.5 * matrix + coefficient * matrix @ matrix


def so3_right_jacobian_inverse(rotation_vector: Sequence[float]) -> np.ndarray:
    return so3_left_jacobian_inverse(-np.asarray(rotation_vector, dtype=float))


def so3_right_jacobian(rotation_vector: Sequence[float]) -> np.ndarray:
    vector = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    matrix = skew(vector)
    if angle <= 1.0e-6:
        return np.eye(3) - 0.5 * matrix + (1.0 / 6.0) * matrix @ matrix
    return (
        np.eye(3)
        - (1.0 - math.cos(angle)) / (angle * angle) * matrix
        + (angle - math.sin(angle)) / (angle ** 3) * matrix @ matrix
    )


def rotation_matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=float)
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-7:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.asarray([roll, pitch, yaw], dtype=float)


def state_plus(state: Sequence[float], increment: Sequence[float]) -> np.ndarray:
    state = np.asarray(state, dtype=float)
    increment = np.asarray(increment, dtype=float)
    if state.shape != (STATE_SIZE,) or increment.shape != (STATE_SIZE,):
        raise ValueError("manifold state and increment must be 15-vectors")
    updated = state.copy()
    updated[:3] += increment[:3]
    rotation = rpy_to_rotation_matrix(state[3:6]) @ so3_exp(increment[3:6])
    updated[3:6] = rotation_matrix_to_rpy(rotation)
    updated[6:] += increment[6:]
    if np.any(~np.isfinite(updated)):
        raise ValueError("manifold state update produced non-finite values")
    return updated


def state_local(reference: Sequence[float], state: Sequence[float]) -> np.ndarray:
    reference = np.asarray(reference, dtype=float)
    state = np.asarray(state, dtype=float)
    if reference.shape != (STATE_SIZE,) or state.shape != (STATE_SIZE,):
        raise ValueError("manifold states must be 15-vectors")
    local = state - reference
    local[3:6] = so3_log(
        rpy_to_rotation_matrix(reference[3:6]).T
        @ rpy_to_rotation_matrix(state[3:6])
    )
    return local


def numerical_state_jacobian(function, states, state_index, epsilon=1.0e-6):
    baseline = np.asarray(function(states), dtype=float)
    jacobian = np.zeros((baseline.size, STATE_SIZE), dtype=float)
    for column in range(STATE_SIZE):
        delta = np.zeros(STATE_SIZE, dtype=float)
        delta[column] = epsilon
        plus_states = [value.copy() for value in states]
        minus_states = [value.copy() for value in states]
        plus_states[state_index] = state_plus(plus_states[state_index], delta)
        minus_states[state_index] = state_plus(minus_states[state_index], -delta)
        jacobian[:, column] = (
            np.asarray(function(plus_states), dtype=float)
            - np.asarray(function(minus_states), dtype=float)
        ) / (2.0 * epsilon)
    return jacobian
