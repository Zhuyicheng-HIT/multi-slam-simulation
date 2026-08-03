"""Conservative online LiDAR/IMU calibration helpers.

The implementation follows the paper's observable-first calibration order:
estimate a scalar time offset from angular-speed correlation, then solve the
rotation hand-eye relation only when the relative-rotation excitation matrix
has rank in all three axes.  Translation is intentionally not estimated here.
"""

from bisect import bisect_left, bisect_right
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .manifold import so3_exp, so3_log


def _sample_stamp(sample):
    return float(sample.stamp_s)


def _sample_gyro(sample):
    return np.asarray(sample.angular_velocity, dtype=float)


def _interpolate_gyro(samples, stamp_s):
    """Linearly interpolate one gyro vector, returning None outside coverage."""
    if len(samples) < 2:
        return None
    stamps = [_sample_stamp(sample) for sample in samples]
    index = bisect_left(stamps, float(stamp_s))
    if index == 0:
        if abs(stamps[0] - float(stamp_s)) <= 1.0e-9:
            return _sample_gyro(samples[0]).copy()
        return None
    if index >= len(samples):
        if abs(stamps[-1] - float(stamp_s)) <= 1.0e-9:
            return _sample_gyro(samples[-1]).copy()
        return None
    if abs(stamps[index] - float(stamp_s)) <= 1.0e-9:
        # Duplicate source timestamps are possible on a routed FCU stream;
        # average all samples at the exact boundary instead of creating a
        # zero-length interpolation interval.
        end = index
        while end < len(samples) and abs(stamps[end] - float(stamp_s)) <= 1.0e-9:
            end += 1
        return np.mean(
            [_sample_gyro(samples[item]) for item in range(index, end)], axis=0
        )
    left, right = samples[index - 1], samples[index]
    left_stamp, right_stamp = stamps[index - 1], stamps[index]
    span = right_stamp - left_stamp
    if span <= 0.0 or span > 0.2:
        return None
    ratio = (float(stamp_s) - left_stamp) / span
    return (1.0 - ratio) * _sample_gyro(left) + ratio * _sample_gyro(right)


def _integrate_gyro(samples, start_s, end_s, maximum_gap_s=0.12):
    """Integrate gyro samples into ``R_start^T R_end``."""
    start_s, end_s = float(start_s), float(end_s)
    if end_s <= start_s or len(samples) < 2:
        return None
    stamps = [_sample_stamp(sample) for sample in samples]
    if start_s < stamps[0] or end_s > stamps[-1]:
        return None
    indices = range(
        max(0, bisect_left(stamps, start_s) - 1),
        min(len(samples), bisect_right(stamps, end_s) + 1),
    )
    interior = []
    for index in indices:
        sample_stamp = _sample_stamp(samples[index])
        if not (start_s < sample_stamp < end_s):
            continue
        if interior and abs(interior[-1][0] - sample_stamp) <= 1.0e-9:
            continue
        interior.append((sample_stamp, _sample_gyro(samples[index])))
    points = [(start_s, _interpolate_gyro(samples, start_s))]
    points.extend(interior)
    points.append((end_s, _interpolate_gyro(samples, end_s)))
    if any(value is None for _, value in points):
        return None
    rotation = np.eye(3)
    for (left_stamp, left_gyro), (right_stamp, right_gyro) in zip(
        points[:-1], points[1:]
    ):
        dt_s = right_stamp - left_stamp
        if dt_s <= 0.0 or dt_s > float(maximum_gap_s):
            return None
        rotation = rotation @ so3_exp(0.5 * (left_gyro + right_gyro) * dt_s)
    return rotation


@dataclass(frozen=True)
class TimeOffsetCandidate:
    valid: bool
    offset_s: float
    correlation: float
    margin: float
    pair_count: int
    reason: str


@dataclass(frozen=True)
class CalibrationUpdate:
    accepted: bool
    locked: bool
    time_offset_s: float
    lidar_to_body_rotation: np.ndarray
    time_correlation: float
    time_margin: float
    rotation_residual_rad: float
    excitation_eigenvalues: tuple[float, float, float]
    pair_count: int
    reason: str


def effective_time_offset(update, enabled=True):
    """Expose a calibration offset only after all observability locks hold."""
    if not enabled or not bool(update.locked):
        return 0.0
    offset_s = float(update.time_offset_s)
    return offset_s if math.isfinite(offset_s) else 0.0


def estimate_time_offset(
    lidar_rates,
    imu_samples,
    candidate_offsets_s,
    minimum_pairs=8,
):
    """Find the offset maximizing correlation of angular-speed magnitudes."""
    if len(lidar_rates) < minimum_pairs or len(imu_samples) < 2:
        return TimeOffsetCandidate(False, 0.0, -1.0, 0.0, 0, "insufficient_samples")
    scores = []
    for offset_s in candidate_offsets_s:
        lidar_values = []
        imu_values = []
        for stamp_s, lidar_rate in lidar_rates:
            gyro = _interpolate_gyro(imu_samples, stamp_s + float(offset_s))
            if gyro is None:
                continue
            lidar_values.append(float(lidar_rate))
            imu_values.append(float(np.linalg.norm(gyro)))
        if len(lidar_values) < minimum_pairs:
            continue
        lidar_centered = np.asarray(lidar_values) - np.mean(lidar_values)
        imu_centered = np.asarray(imu_values) - np.mean(imu_values)
        denominator = np.linalg.norm(lidar_centered) * np.linalg.norm(imu_centered)
        if denominator <= 1.0e-9:
            continue
        scores.append((float(lidar_centered @ imu_centered / denominator), float(offset_s), len(lidar_values)))
    if not scores:
        return TimeOffsetCandidate(False, 0.0, -1.0, 0.0, 0, "unexcited_angular_speed")
    scores.sort(reverse=True)
    best = scores[0]
    second = scores[1][0] if len(scores) > 1 else -1.0
    return TimeOffsetCandidate(
        True, best[1], best[0], best[0] - second, best[2], "candidate_ready"
    )


class OnlineSpatiotemporalCalibrator:
    """Stateful observable-first LiDAR/IMU calibration worker."""

    def __init__(
        self,
        window_s=5.0,
        minimum_pairs=8,
        time_offset_range_s=0.10,
        time_offset_step_s=0.005,
        minimum_correlation=0.70,
        minimum_correlation_margin=0.05,
        minimum_excitation_eigenvalue=1.0e-4,
        maximum_rotation_residual_rad=0.08,
        sharp_turn_rate_radps=1.5,
        history_length=4,
        lock_count=3,
        stability_tolerance_s=0.008,
        stability_tolerance_rad=0.03,
        update_alpha=0.35,
        solve_period_s=0.0,
    ):
        if (
            window_s <= 0.0
            or minimum_pairs < 3
            or time_offset_step_s <= 0.0
            or solve_period_s < 0.0
        ):
            raise ValueError("invalid calibration window configuration")
        self.window_s = float(window_s)
        self.minimum_pairs = int(minimum_pairs)
        self.candidate_offsets_s = np.arange(
            -float(time_offset_range_s),
            float(time_offset_range_s) + 0.5 * float(time_offset_step_s),
            float(time_offset_step_s),
        )
        self.minimum_correlation = float(minimum_correlation)
        self.minimum_correlation_margin = float(minimum_correlation_margin)
        self.minimum_excitation_eigenvalue = float(minimum_excitation_eigenvalue)
        self.maximum_rotation_residual_rad = float(maximum_rotation_residual_rad)
        self.sharp_turn_rate_radps = float(sharp_turn_rate_radps)
        self.history_length = int(history_length)
        self.lock_count = int(lock_count)
        self.stability_tolerance_s = float(stability_tolerance_s)
        self.stability_tolerance_rad = float(stability_tolerance_rad)
        self.update_alpha = float(update_alpha)
        self.solve_period_s = float(solve_period_s)
        self.last_solve_stamp_s = None
        self.poses = deque()
        self.time_offset_history = deque(maxlen=self.history_length)
        self.rotation_history = deque(maxlen=self.history_length)
        self.time_offset_s = 0.0
        self.lidar_to_body_rotation = np.eye(3)
        self.time_locked = False
        self.rotation_locked = False
        self.last_update = CalibrationUpdate(
            False, False, 0.0, self.lidar_to_body_rotation.copy(),
            -1.0, 0.0, -1.0, (0.0, 0.0, 0.0), 0, "waiting_for_observable_motion",
        )

    def _trim(self, latest_stamp_s):
        cutoff = float(latest_stamp_s) - self.window_s
        while self.poses and self.poses[0][0] < cutoff:
            self.poses.popleft()

    def _lidar_rates(self):
        rates = []
        intervals = []
        for left, right in zip(self.poses, list(self.poses)[1:]):
            dt_s = right[0] - left[0]
            if dt_s <= 0.0 or dt_s > 0.5:
                continue
            relative = left[2].T @ right[2]
            rotation_vector = so3_log(relative)
            intervals.append((left[0], right[0], rotation_vector))
            rates.append((0.5 * (left[0] + right[0]), np.linalg.norm(rotation_vector) / dt_s))
        return rates, intervals

    def update(self, stamp_s, body_rotation, lidar_to_body_rotation, imu_samples):
        stamp_s = float(stamp_s)
        body_rotation = np.asarray(body_rotation, dtype=float)
        extrinsic = np.asarray(lidar_to_body_rotation, dtype=float)
        if body_rotation.shape != (3, 3) or extrinsic.shape != (3, 3):
            raise ValueError("calibration rotations must be 3x3")
        lidar_rotation = body_rotation @ extrinsic
        self.poses.append((stamp_s, body_rotation.copy(), lidar_rotation.copy()))
        self._trim(stamp_s)
        if (
            self.last_solve_stamp_s is not None
            and stamp_s - self.last_solve_stamp_s < self.solve_period_s
        ):
            previous = self.last_update
            return CalibrationUpdate(
                False, previous.locked, previous.time_offset_s,
                previous.lidar_to_body_rotation.copy(),
                previous.time_correlation, previous.time_margin,
                previous.rotation_residual_rad,
                previous.excitation_eigenvalues, previous.pair_count,
                "update_throttled",
            )
        self.last_solve_stamp_s = stamp_s
        imu_samples = tuple(sorted(imu_samples, key=_sample_stamp))
        lidar_rates, intervals = self._lidar_rates()
        if len(intervals) < self.minimum_pairs:
            self.last_update = CalibrationUpdate(
                False, self.time_locked and self.rotation_locked,
                self.time_offset_s, self.lidar_to_body_rotation.copy(),
                -1.0, 0.0, -1.0, (0.0, 0.0, 0.0), len(intervals),
                "insufficient_relative_rotations",
            )
            return self.last_update
        time_candidate = estimate_time_offset(
            lidar_rates, imu_samples, self.candidate_offsets_s, self.minimum_pairs
        )
        sharp_turn = False
        if imu_samples:
            recent = [
                np.linalg.norm(_sample_gyro(sample))
                for sample in imu_samples
                if stamp_s - 0.25 <= _sample_stamp(sample) <= stamp_s + 0.02
            ]
            sharp_turn = bool(recent and max(recent) > self.sharp_turn_rate_radps)
        time_accepted = bool(
            time_candidate.valid
            and time_candidate.correlation >= self.minimum_correlation
            and time_candidate.margin >= self.minimum_correlation_margin
            and not sharp_turn
        )
        if time_accepted:
            self.time_offset_history.append(time_candidate.offset_s)
            recent_offsets = np.asarray(self.time_offset_history, dtype=float)
            stable = len(recent_offsets) >= 2 and np.std(recent_offsets) <= self.stability_tolerance_s
            if stable:
                self.time_offset_s = float(
                    (1.0 - self.update_alpha) * self.time_offset_s
                    + self.update_alpha * np.median(recent_offsets)
                )
            self.time_locked = bool(
                len(recent_offsets) >= self.lock_count
                and np.std(recent_offsets) <= self.stability_tolerance_s
            )
        rotation_offset = time_candidate.offset_s if time_candidate.valid else self.time_offset_s
        imu_rotation_vectors = []
        lidar_rotation_vectors = []
        for start_s, end_s, lidar_vector in intervals:
            imu_rotation = _integrate_gyro(
                imu_samples, start_s + rotation_offset, end_s + rotation_offset
            )
            if imu_rotation is None:
                continue
            imu_rotation_vectors.append(so3_log(imu_rotation))
            lidar_rotation_vectors.append(lidar_vector)
        excitation = np.zeros((3, 3))
        if lidar_rotation_vectors:
            for vector in lidar_rotation_vectors:
                excitation += np.outer(vector, vector)
        eigenvalues = np.linalg.eigvalsh(excitation)
        rotation_accepted = bool(
            len(imu_rotation_vectors) >= self.minimum_pairs
            and eigenvalues[-1] > self.minimum_excitation_eigenvalue
            and eigenvalues[0] > self.minimum_excitation_eigenvalue
        )
        rotation_residual = -1.0
        if rotation_accepted:
            cross_covariance = sum(
                np.outer(imu_vector, lidar_vector)
                for imu_vector, lidar_vector in zip(
                    imu_rotation_vectors, lidar_rotation_vectors
                )
            )
            left, _, right_t = np.linalg.svd(cross_covariance)
            correction = np.eye(3)
            correction[2, 2] = np.linalg.det(left @ right_t)
            candidate_rotation = left @ correction @ right_t
            errors = [
                np.linalg.norm(imu_vector - candidate_rotation @ lidar_vector)
                for imu_vector, lidar_vector in zip(
                    imu_rotation_vectors, lidar_rotation_vectors
                )
            ]
            rotation_residual = float(np.sqrt(np.mean(np.square(errors))))
            rotation_accepted = rotation_residual <= self.maximum_rotation_residual_rad
            if rotation_accepted:
                self.rotation_history.append(candidate_rotation)
                if len(self.rotation_history) >= 2:
                    delta = so3_log(
                        self.rotation_history[-2].T @ self.rotation_history[-1]
                    )
                    stable = np.linalg.norm(delta) <= self.stability_tolerance_rad
                else:
                    stable = True
                if stable:
                    delta = so3_log(
                        self.lidar_to_body_rotation.T @ candidate_rotation
                    )
                    self.lidar_to_body_rotation = self.lidar_to_body_rotation @ so3_exp(
                        self.update_alpha * delta
                    )
                self.rotation_locked = bool(
                    len(self.rotation_history) >= self.lock_count
                    and all(
                        np.linalg.norm(
                            so3_log(self.rotation_history[index - 1].T @ self.rotation_history[index])
                        ) <= self.stability_tolerance_rad
                        for index in range(1, len(self.rotation_history))
                    )
                )
        accepted = time_accepted or rotation_accepted
        reason = "accepted"
        if sharp_turn:
            reason = "sharp_turn_frozen"
        elif not time_accepted and not rotation_accepted:
            reason = "unobservable_or_low_confidence"
        self.last_update = CalibrationUpdate(
            accepted,
            bool(self.time_locked and self.rotation_locked),
            self.time_offset_s,
            self.lidar_to_body_rotation.copy(),
            time_candidate.correlation,
            time_candidate.margin,
            rotation_residual,
            tuple(float(value) for value in eigenvalues),
            len(imu_rotation_vectors),
            reason,
        )
        return self.last_update
