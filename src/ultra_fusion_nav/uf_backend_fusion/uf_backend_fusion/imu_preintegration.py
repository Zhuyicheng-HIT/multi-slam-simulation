"""Testable IMU preintegration primitives for both backend implementations.

The manifold delta contains body specific force in the interval start frame;
global gravity is applied exactly once by the propagation and residual models.
First-order bias Jacobians are propagated with the midpoint SO(3) integration.
The original tangent-space result remains available for controlled rollback.
"""

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ImuSample:
    stamp_s: float
    acceleration: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


@dataclass(frozen=True)
class PreintegratedImu:
    valid: bool
    reason: str
    dt_s: float
    delta_position: tuple[float, float, float]
    delta_velocity: tuple[float, float, float]
    delta_quaternion: tuple[float, float, float, float]
    covariance: tuple[float, ...]
    sample_count: int
    max_gap_s: float
    jacobian_delta_position_accel_bias: tuple[float, ...] = (0.0,) * 9
    jacobian_delta_position_gyro_bias: tuple[float, ...] = (0.0,) * 9
    jacobian_delta_velocity_accel_bias: tuple[float, ...] = (0.0,) * 9
    jacobian_delta_velocity_gyro_bias: tuple[float, ...] = (0.0,) * 9
    jacobian_delta_rotation_gyro_bias: tuple[float, ...] = (0.0,) * 9


@dataclass(frozen=True)
class ManifoldPreintegratedImu:
    valid: bool
    reason: str
    dt_s: float
    delta_position: tuple[float, float, float]
    delta_velocity: tuple[float, float, float]
    delta_quaternion: tuple[float, float, float, float]
    covariance: tuple[float, ...]
    sample_count: int
    max_gap_s: float
    accel_bias_linearization: tuple[float, float, float]
    gyro_bias_linearization: tuple[float, float, float]
    jacobian_delta_position_accel_bias: tuple[float, ...]
    jacobian_delta_position_gyro_bias: tuple[float, ...]
    jacobian_delta_velocity_accel_bias: tuple[float, ...]
    jacobian_delta_velocity_gyro_bias: tuple[float, ...]
    jacobian_delta_rotation_gyro_bias: tuple[float, ...]


def _quat_normalize(value: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return value / norm


def _quat_multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def _quat_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle <= 1.0e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    axis = rotvec / angle
    half = 0.5 * angle
    return np.concatenate(([math.cos(half)], axis * math.sin(half)))


def _rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.concatenate(([0.0], vector))
    conjugate = np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])
    return _quat_multiply(_quat_multiply(quaternion, pure), conjugate)[1:]


def _quat_conjugate(quaternion: np.ndarray) -> np.ndarray:
    return np.array(
        [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]],
        dtype=float,
    )


def _quat_to_rotvec(quaternion: np.ndarray) -> np.ndarray:
    quaternion = _quat_normalize(np.asarray(quaternion, dtype=float))
    if quaternion[0] < 0.0:
        quaternion = -quaternion
    scalar = float(np.clip(quaternion[0], -1.0, 1.0))
    vector = quaternion[1:]
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        return 2.0 * vector
    return vector * (2.0 * math.atan2(norm, scalar) / norm)


def _skew(value: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(value, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def _so3_exp(rotvec: np.ndarray) -> np.ndarray:
    rotvec = np.asarray(rotvec, dtype=float)
    angle = float(np.linalg.norm(rotvec))
    skew = _skew(rotvec)
    if angle <= 1.0e-8:
        return np.eye(3) + skew + 0.5 * (skew @ skew)
    angle_squared = angle * angle
    return (
        np.eye(3)
        + math.sin(angle) / angle * skew
        + (1.0 - math.cos(angle)) / angle_squared * (skew @ skew)
    )


def _so3_right_jacobian(rotvec: np.ndarray) -> np.ndarray:
    rotvec = np.asarray(rotvec, dtype=float)
    angle = float(np.linalg.norm(rotvec))
    skew = _skew(rotvec)
    if angle <= 1.0e-6:
        return np.eye(3) - 0.5 * skew + (1.0 / 6.0) * (skew @ skew)
    angle_squared = angle * angle
    return (
        np.eye(3)
        - (1.0 - math.cos(angle)) / angle_squared * skew
        + (angle - math.sin(angle)) / (angle_squared * angle) * (skew @ skew)
    )


def _rotmat_to_quat(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to a normalized wxyz quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(max(1.0e-15, trace + 1.0))
        quaternion = np.array([
            0.25 * scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
        ])
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2.0 * math.sqrt(max(1.0e-15, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]))
            quaternion = np.array([
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            ])
        elif index == 1:
            scale = 2.0 * math.sqrt(max(1.0e-15, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]))
            quaternion = np.array([
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            ])
        else:
            scale = 2.0 * math.sqrt(max(1.0e-15, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]))
            quaternion = np.array([
                (rotation[1, 0] - rotation[0, 1]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
            ])
    return _quat_normalize(quaternion)


def _vee(value: np.ndarray) -> np.ndarray:
    return np.array([value[2, 1], value[0, 2], value[1, 0]], dtype=float)


def _interpolate(first: ImuSample, second: ImuSample, stamp_s: float) -> ImuSample:
    span = second.stamp_s - first.stamp_s
    ratio = 0.0 if span <= 0.0 else (stamp_s - first.stamp_s) / span
    accel = np.asarray(first.acceleration) + ratio * (
        np.asarray(second.acceleration) - np.asarray(first.acceleration)
    )
    gyro = np.asarray(first.angular_velocity) + ratio * (
        np.asarray(second.angular_velocity) - np.asarray(first.angular_velocity)
    )
    return ImuSample(float(stamp_s), tuple(accel), tuple(gyro))


def _sample_at(samples: Sequence[ImuSample], stamp_s: float) -> ImuSample | None:
    stamps = [sample.stamp_s for sample in samples]
    index = bisect_left(stamps, stamp_s)
    if index < len(samples) and abs(stamps[index] - stamp_s) <= 1.0e-9:
        return samples[index]
    if index == 0 or index >= len(samples):
        return None
    return _interpolate(samples[index - 1], samples[index], stamp_s)


def _invalid(reason: str, start_s: float, end_s: float, sample_count: int = 0,
             max_gap_s: float = 0.0) -> PreintegratedImu:
    return PreintegratedImu(
        False, reason, max(0.0, end_s - start_s),
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0),
        (1.0,) * 9, sample_count, max_gap_s,
    )


def _integrate_interval_with_jacobians(
    interval_samples: Sequence[ImuSample], gravity_vector: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Midpoint integration with first-order constant-bias recurrences."""
    delta_position = np.zeros(3, dtype=float)
    delta_velocity = np.zeros(3, dtype=float)
    rotation = np.eye(3, dtype=float)
    position_accel_jacobian = np.zeros((3, 3), dtype=float)
    position_gyro_jacobian = np.zeros((3, 3), dtype=float)
    velocity_accel_jacobian = np.zeros((3, 3), dtype=float)
    velocity_gyro_jacobian = np.zeros((3, 3), dtype=float)
    rotation_gyro_derivative = np.zeros((3, 3, 3), dtype=float)
    for first, second in zip(interval_samples[:-1], interval_samples[1:]):
        dt = second.stamp_s - first.stamp_s
        accel = 0.5 * (np.asarray(first.acceleration) + np.asarray(second.acceleration))
        gyro = 0.5 * (np.asarray(first.angular_velocity) + np.asarray(second.angular_velocity))
        world_acceleration = rotation @ accel + gravity_vector
        accel_bias_derivative = -rotation
        gyro_bias_accel_derivative = np.stack(
            [rotation_gyro_derivative[:, :, axis] @ accel for axis in range(3)],
            axis=1,
        )
        delta_position += delta_velocity * dt + 0.5 * world_acceleration * dt * dt
        delta_velocity += world_acceleration * dt
        position_accel_jacobian += velocity_accel_jacobian * dt + 0.5 * accel_bias_derivative * dt * dt
        position_gyro_jacobian += velocity_gyro_jacobian * dt + 0.5 * gyro_bias_accel_derivative * dt * dt
        velocity_accel_jacobian += accel_bias_derivative * dt
        velocity_gyro_jacobian += gyro_bias_accel_derivative * dt
        rotation_vector = gyro * dt
        increment = _so3_exp(rotation_vector)
        right_jacobian = _so3_right_jacobian(rotation_vector)
        next_rotation = rotation @ increment
        next_derivative = np.zeros_like(rotation_gyro_derivative)
        for axis in range(3):
            bias_rotation_increment = right_jacobian @ (-np.eye(3)[:, axis] * dt)
            next_derivative[:, :, axis] = (
                rotation_gyro_derivative[:, :, axis] @ increment
                + next_rotation @ _skew(bias_rotation_increment)
            )
        rotation = next_rotation
        rotation_gyro_derivative = next_derivative
    rotation_jacobian = np.stack(
        [_vee(rotation.T @ rotation_gyro_derivative[:, :, axis]) for axis in range(3)],
        axis=1,
    )
    return (
        delta_position,
        delta_velocity,
        _rotmat_to_quat(rotation),
        position_accel_jacobian,
        position_gyro_jacobian,
        velocity_accel_jacobian,
        velocity_gyro_jacobian,
        rotation_jacobian,
    )


def _propagate_manifold_covariance(
    interval_samples: Sequence[ImuSample],
    accel_noise_density: float,
    gyro_noise_density: float,
    accel_bias_random_walk: float,
    gyro_bias_random_walk: float,
) -> np.ndarray:
    """Propagate the coupled [dp,dv,dtheta,dba,dbg] covariance.

    The discrete model follows the same right-local rotation convention as the
    manifold residual. Continuous white acceleration/gyro noise is integrated
    over each sample interval; bias random walks remain explicit states.
    """
    noise_values = np.asarray([
        accel_noise_density,
        gyro_noise_density,
        accel_bias_random_walk,
        gyro_bias_random_walk,
    ], dtype=float)
    if np.any(~np.isfinite(noise_values)) or np.any(noise_values < 0.0):
        raise ValueError("IMU noise densities must be finite and non-negative")

    covariance = np.zeros((15, 15), dtype=float)
    rotation = np.eye(3, dtype=float)
    identity = np.eye(3, dtype=float)
    accel_variance = float(accel_noise_density) ** 2
    gyro_variance = float(gyro_noise_density) ** 2
    accel_bias_variance = float(accel_bias_random_walk) ** 2
    gyro_bias_variance = float(gyro_bias_random_walk) ** 2
    for first, second in zip(interval_samples[:-1], interval_samples[1:]):
        dt = float(second.stamp_s - first.stamp_s)
        acceleration = 0.5 * (
            np.asarray(first.acceleration, dtype=float)
            + np.asarray(second.acceleration, dtype=float)
        )
        angular_velocity = 0.5 * (
            np.asarray(first.angular_velocity, dtype=float)
            + np.asarray(second.angular_velocity, dtype=float)
        )
        rotation_vector = angular_velocity * dt
        increment = _so3_exp(rotation_vector)
        right_jacobian = _so3_right_jacobian(rotation_vector)

        transition = np.eye(15, dtype=float)
        transition[0:3, 3:6] = identity * dt
        transition[0:3, 6:9] = (
            -0.5 * rotation @ _skew(acceleration) * dt * dt
        )
        transition[0:3, 9:12] = -0.5 * rotation * dt * dt
        transition[3:6, 6:9] = -rotation @ _skew(acceleration) * dt
        transition[3:6, 9:12] = -rotation * dt
        transition[6:9, 6:9] = increment.T
        transition[6:9, 12:15] = -right_jacobian * dt

        process = np.zeros((15, 15), dtype=float)
        rotated_accel_covariance = accel_variance * (rotation @ rotation.T)
        process[0:3, 0:3] = rotated_accel_covariance * dt ** 3 / 3.0
        process[0:3, 3:6] = rotated_accel_covariance * dt * dt / 2.0
        process[3:6, 0:3] = process[0:3, 3:6].T
        process[3:6, 3:6] = rotated_accel_covariance * dt
        process[6:9, 6:9] = (
            gyro_variance * dt * right_jacobian @ right_jacobian.T
        )
        process[9:12, 9:12] = accel_bias_variance * dt * identity
        process[12:15, 12:15] = gyro_bias_variance * dt * identity

        covariance = transition @ covariance @ transition.T + process
        covariance = 0.5 * (covariance + covariance.T)
        rotation = rotation @ increment

    # A tiny floor protects the Cholesky whitening from round-off without
    # masking the propagated cross-correlation structure.
    covariance += np.eye(15, dtype=float) * 1.0e-12
    return covariance


def preintegrate(
    samples: Sequence[ImuSample],
    start_s: float,
    end_s: float,
    gravity: Sequence[float] = (0.0, 0.0, -9.81),
    max_gap_s: float = 0.10,
    accel_noise_density: float = 0.10,
    gyro_noise_density: float = 0.02,
) -> PreintegratedImu:
    """Midpoint preintegrate a bounded IMU interval.

    ``acceleration`` is specific force in the body frame. The returned delta
    is expressed in the start frame, with gravity added after rotation. This
    is a data-quality primitive, not a replacement for bias-aware preintegration.
    """
    start_s, end_s = float(start_s), float(end_s)
    if not math.isfinite(start_s) or not math.isfinite(end_s) or end_s <= start_s:
        return _invalid("invalid_interval", start_s, end_s)
    ordered = sorted(samples, key=lambda sample: sample.stamp_s)
    deduplicated: list[ImuSample] = []
    for sample in ordered:
        if deduplicated and abs(sample.stamp_s - deduplicated[-1].stamp_s) <= 1.0e-9:
            previous = deduplicated[-1]
            deduplicated[-1] = ImuSample(
                previous.stamp_s,
                tuple(
                    (np.asarray(previous.acceleration) + np.asarray(sample.acceleration))
                    / 2.0
                ),
                tuple(
                    (np.asarray(previous.angular_velocity) + np.asarray(sample.angular_velocity))
                    / 2.0
                ),
            )
        else:
            deduplicated.append(sample)
    ordered = deduplicated
    if len(ordered) < 2:
        return _invalid("insufficient_samples", start_s, end_s, len(ordered))
    if any(
        not math.isfinite(sample.stamp_s)
        or not np.all(np.isfinite(sample.acceleration))
        or not np.all(np.isfinite(sample.angular_velocity))
        for sample in ordered
    ):
        return _invalid("nonfinite_sample", start_s, end_s)
    before = _sample_at(ordered, start_s)
    after = _sample_at(ordered, end_s)
    if before is None or after is None:
        return _invalid("interval_not_covered", start_s, end_s)
    interior = [sample for sample in ordered if start_s < sample.stamp_s < end_s]
    interval_samples = [before, *interior, after]
    gaps = np.diff([sample.stamp_s for sample in interval_samples])
    largest_gap = float(np.max(gaps)) if len(gaps) else 0.0
    if len(gaps) == 0 or np.any(gaps <= 0.0):
        return _invalid("nonincreasing_samples", start_s, end_s, len(interior), largest_gap)
    if largest_gap > float(max_gap_s):
        return _invalid("sample_gap_exceeds_limit", start_s, end_s, len(interior), largest_gap)

    gravity_vector = np.asarray(gravity, dtype=float)
    if gravity_vector.shape != (3,) or not np.all(np.isfinite(gravity_vector)):
        return _invalid("invalid_gravity", start_s, end_s, len(interior), largest_gap)
    (
        delta_position,
        delta_velocity,
        quaternion,
        accel_position_jacobian,
        gyro_position_jacobian,
        accel_velocity_jacobian,
        gyro_velocity_jacobian,
        gyro_rotation_jacobian,
    ) = _integrate_interval_with_jacobians(interval_samples, gravity_vector)
    dt_s = end_s - start_s
    position_sigma = max(1.0e-6, float(accel_noise_density) * dt_s * dt_s)
    velocity_sigma = max(1.0e-6, float(accel_noise_density) * dt_s)
    rotation_sigma = max(1.0e-6, float(gyro_noise_density) * math.sqrt(dt_s))
    covariance = tuple(
        [position_sigma * position_sigma] * 3
        + [velocity_sigma * velocity_sigma] * 3
        + [rotation_sigma * rotation_sigma] * 3
    )
    return PreintegratedImu(
        True, "ok", dt_s,
        tuple(float(value) for value in delta_position),
        tuple(float(value) for value in delta_velocity),
        tuple(float(value) for value in quaternion),
        covariance, len(interior), largest_gap,
        tuple(float(value) for value in accel_position_jacobian.ravel()),
        tuple(float(value) for value in gyro_position_jacobian.ravel()),
        tuple(float(value) for value in accel_velocity_jacobian.ravel()),
        tuple(float(value) for value in gyro_velocity_jacobian.ravel()),
        tuple(float(value) for value in gyro_rotation_jacobian.ravel()),
    )


def preintegrate_manifold(
    samples: Sequence[ImuSample],
    start_s: float,
    end_s: float,
    accel_bias: Sequence[float] = (0.0, 0.0, 0.0),
    gyro_bias: Sequence[float] = (0.0, 0.0, 0.0),
    max_gap_s: float = 0.10,
    accel_noise_density: float = 0.10,
    gyro_noise_density: float = 0.02,
    accel_bias_random_walk: float = 0.001,
    gyro_bias_random_walk: float = 0.0001,
) -> ManifoldPreintegratedImu:
    """Preintegrate body specific force without embedding global gravity.

    The returned deltas are expressed in the start IMU frame. Gravity is
    applied exactly once by the propagation and residual equations.
    """
    accel_bias = np.asarray(accel_bias, dtype=float)
    gyro_bias = np.asarray(gyro_bias, dtype=float)
    if (
        accel_bias.shape != (3,)
        or gyro_bias.shape != (3,)
        or np.any(~np.isfinite(accel_bias))
        or np.any(~np.isfinite(gyro_bias))
    ):
        raise ValueError("IMU bias linearization must contain finite 3-vectors")
    start_s, end_s = float(start_s), float(end_s)
    ordered = sorted(samples, key=lambda sample: sample.stamp_s)
    deduplicated = []
    for sample in ordered:
        if deduplicated and abs(sample.stamp_s - deduplicated[-1].stamp_s) <= 1.0e-9:
            previous = deduplicated[-1]
            deduplicated[-1] = ImuSample(
                previous.stamp_s,
                tuple(0.5 * (np.asarray(previous.acceleration) + np.asarray(sample.acceleration))),
                tuple(0.5 * (np.asarray(previous.angular_velocity) + np.asarray(sample.angular_velocity))),
            )
        else:
            deduplicated.append(sample)
    invalid_reason = None
    if not math.isfinite(start_s) or not math.isfinite(end_s) or end_s <= start_s:
        invalid_reason = "invalid_interval"
    elif len(deduplicated) < 2:
        invalid_reason = "insufficient_samples"
    elif any(
        not math.isfinite(sample.stamp_s)
        or not np.all(np.isfinite(sample.acceleration))
        or not np.all(np.isfinite(sample.angular_velocity))
        for sample in deduplicated
    ):
        invalid_reason = "nonfinite_sample"
    before = None if invalid_reason else _sample_at(deduplicated, start_s)
    after = None if invalid_reason else _sample_at(deduplicated, end_s)
    if invalid_reason is None and (before is None or after is None):
        invalid_reason = "interval_not_covered"
    interior = (
        [] if invalid_reason else [
            sample for sample in deduplicated if start_s < sample.stamp_s < end_s
        ]
    )
    interval_samples = [] if invalid_reason else [before, *interior, after]
    gaps = np.diff([sample.stamp_s for sample in interval_samples])
    largest_gap = float(np.max(gaps)) if len(gaps) else 0.0
    if invalid_reason is None and (len(gaps) == 0 or np.any(gaps <= 0.0)):
        invalid_reason = "nonincreasing_samples"
    if invalid_reason is None and largest_gap > float(max_gap_s):
        invalid_reason = "sample_gap_exceeds_limit"
    if invalid_reason is not None:
        return ManifoldPreintegratedImu(
            False, invalid_reason, max(0.0, end_s - start_s),
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0), tuple(np.eye(15).ravel()),
            len(interior), largest_gap,
            tuple(accel_bias), tuple(gyro_bias),
            (0.0,) * 9, (0.0,) * 9, (0.0,) * 9, (0.0,) * 9,
            (0.0,) * 9,
        )
    corrected = [
        ImuSample(
            sample.stamp_s,
            tuple(np.asarray(sample.acceleration) - accel_bias),
            tuple(np.asarray(sample.angular_velocity) - gyro_bias),
        )
        for sample in interval_samples
    ]
    (
        delta_position,
        delta_velocity,
        quaternion,
        accel_position_jacobian,
        gyro_position_jacobian,
        accel_velocity_jacobian,
        gyro_velocity_jacobian,
        gyro_rotation_jacobian,
    ) = _integrate_interval_with_jacobians(corrected, np.zeros(3))
    dt_s = end_s - start_s
    covariance = _propagate_manifold_covariance(
        corrected,
        accel_noise_density,
        gyro_noise_density,
        accel_bias_random_walk,
        gyro_bias_random_walk,
    )
    return ManifoldPreintegratedImu(
        True, "ok", dt_s,
        tuple(float(value) for value in delta_position),
        tuple(float(value) for value in delta_velocity),
        tuple(float(value) for value in quaternion),
        tuple(float(value) for value in covariance.ravel()),
        len(interior), largest_gap,
        tuple(float(value) for value in accel_bias),
        tuple(float(value) for value in gyro_bias),
        tuple(float(value) for value in accel_position_jacobian.ravel()),
        tuple(float(value) for value in gyro_position_jacobian.ravel()),
        tuple(float(value) for value in accel_velocity_jacobian.ravel()),
        tuple(float(value) for value in gyro_velocity_jacobian.ravel()),
        tuple(float(value) for value in gyro_rotation_jacobian.ravel()),
    )
