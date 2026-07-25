"""Small, testable IMU preintegration primitive for the Stage 7 data layer.

This module exports a delta in the frame at the start of an interval. It is
deliberately independent of the current linear backend: bias Jacobians and a
full SE(3) factor are a later milestone, so callers must not treat this delta
as final estimator evidence yet.
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
    delta_position = np.zeros(3, dtype=float)
    delta_velocity = np.zeros(3, dtype=float)
    quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    for first, second in zip(interval_samples[:-1], interval_samples[1:]):
        dt = second.stamp_s - first.stamp_s
        accel = 0.5 * (np.asarray(first.acceleration) + np.asarray(second.acceleration))
        gyro = 0.5 * (np.asarray(first.angular_velocity) + np.asarray(second.angular_velocity))
        world_acceleration = _rotate(quaternion, accel) + gravity_vector
        delta_position += delta_velocity * dt + 0.5 * world_acceleration * dt * dt
        delta_velocity += world_acceleration * dt
        quaternion = _quat_normalize(
            _quat_multiply(quaternion, _quat_from_rotvec(gyro * dt))
        )
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
    )
