"""Conservative online LiDAR/IMU calibration helpers.

The implementation follows the paper's observable-first calibration order:
estimate a scalar time offset from angular-speed correlation, then solve the
rotation hand-eye relation only when the relative-rotation excitation matrix
has rank in all three axes.  LiDAR motion must come from an isolated
scan-to-scan registration branch; an IMU-propagated backend pose is not an
independent calibration observation. Translation is intentionally fixed.
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


@dataclass(frozen=True)
class LidarMotionSample:
    """One quality-gated scan-to-scan LiDAR relative rotation."""

    start_s: float
    end_s: float
    relative_rotation: np.ndarray
    weight: float = 1.0


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
        for item in lidar_rates:
            stamp_s, lidar_rate = item[:2]
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
        minimum_excitation_ratio=0.05,
        minimum_accumulated_rotation_rad=0.25,
        minimum_rotation_inlier_ratio=0.70,
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
        self.minimum_excitation_ratio = float(minimum_excitation_ratio)
        self.minimum_accumulated_rotation_rad = float(
            minimum_accumulated_rotation_rad
        )
        self.minimum_rotation_inlier_ratio = float(minimum_rotation_inlier_ratio)
        self.maximum_rotation_residual_rad = float(maximum_rotation_residual_rad)
        self.sharp_turn_rate_radps = float(sharp_turn_rate_radps)
        self.history_length = int(history_length)
        self.lock_count = int(lock_count)
        self.stability_tolerance_s = float(stability_tolerance_s)
        self.stability_tolerance_rad = float(stability_tolerance_rad)
        self.update_alpha = float(update_alpha)
        self.solve_period_s = float(solve_period_s)
        self.last_solve_stamp_s = None
        if (
            not 0.0 < self.minimum_excitation_ratio <= 1.0
            or self.minimum_accumulated_rotation_rad <= 0.0
            or not 0.0 < self.minimum_rotation_inlier_ratio <= 1.0
        ):
            raise ValueError("invalid calibration observability limits")
        self.motions = deque()
        self.time_offset_history = deque(maxlen=self.history_length)
        self.rotation_history = deque(maxlen=self.history_length)
        self.time_offset_s = 0.0
        self.lidar_to_body_rotation = np.eye(3)
        self.time_locked = False
        self.rotation_locked = False
        self.initial_rotation_set = False
        # Keep the observability evidence alongside the last update so the
        # runtime diagnostic can distinguish pair starvation from a failed
        # correlation or a rank-deficient rotation solve.
        self.last_time_candidate = TimeOffsetCandidate(
            False, 0.0, -1.0, 0.0, 0, "not_run"
        )
        self.last_excitation_ratio = 0.0
        self.last_accumulated_rotation_rad = 0.0
        self.last_weighted_accumulated_rotation_rad = 0.0
        self.last_unweighted_accumulated_rotation_rad = 0.0
        self.last_imu_accumulated_rotation_rad = 0.0
        self.last_motion_weight_mean = 0.0
        self.last_rotation_inlier_ratio = 0.0
        self.last_update = CalibrationUpdate(
            False, False, 0.0, self.lidar_to_body_rotation.copy(),
            -1.0, 0.0, -1.0, (0.0, 0.0, 0.0), 0, "waiting_for_observable_motion",
        )

    def _trim(self, latest_stamp_s):
        cutoff = float(latest_stamp_s) - self.window_s
        while self.motions and self.motions[0].end_s < cutoff:
            self.motions.popleft()

    def _lidar_rates(self):
        rates = []
        intervals = []
        for motion in self.motions:
            dt_s = motion.end_s - motion.start_s
            if dt_s <= 0.0 or dt_s > 0.5:
                continue
            rotation_vector = so3_log(motion.relative_rotation)
            intervals.append(
                (motion.start_s, motion.end_s, rotation_vector, motion.weight)
            )
            rates.append((
                0.5 * (motion.start_s + motion.end_s),
                np.linalg.norm(rotation_vector) / dt_s,
                motion.weight,
            ))
        return rates, intervals

    def set_initial_rotation(self, lidar_to_body_rotation):
        """Seed the fixed installation rotation before any calibration lock."""
        rotation = np.asarray(lidar_to_body_rotation, dtype=float)
        if rotation.shape != (3, 3) or np.any(~np.isfinite(rotation)):
            raise ValueError("initial calibration rotation must be finite and 3x3")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
            raise ValueError("initial calibration rotation must be orthonormal")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6):
            raise ValueError("initial calibration rotation must have determinant one")
        if self.initial_rotation_set:
            return
        self.lidar_to_body_rotation = rotation.copy()
        self.initial_rotation_set = True
        self.last_update = CalibrationUpdate(
            False, False, self.time_offset_s,
            self.lidar_to_body_rotation.copy(), -1.0, 0.0, -1.0,
            (0.0, 0.0, 0.0), 0, "waiting_for_independent_lidar_motion",
        )

    def update(self, motion, imu_samples):
        if not isinstance(motion, LidarMotionSample):
            raise ValueError("calibration requires a LidarMotionSample")
        start_s = float(motion.start_s)
        stamp_s = float(motion.end_s)
        relative = np.asarray(motion.relative_rotation, dtype=float)
        weight = float(motion.weight)
        if (
            not math.isfinite(start_s)
            or not math.isfinite(stamp_s)
            or stamp_s <= start_s
            or stamp_s - start_s > 0.5
            or relative.shape != (3, 3)
            or np.any(~np.isfinite(relative))
            or not np.allclose(relative.T @ relative, np.eye(3), atol=1.0e-5)
            or not math.isclose(float(np.linalg.det(relative)), 1.0, abs_tol=1.0e-5)
            or not math.isfinite(weight)
            or weight <= 0.0
        ):
            raise ValueError("invalid independent LiDAR motion sample")
        if self.motions and start_s < self.motions[-1].end_s - 1.0e-6:
            raise ValueError("LiDAR calibration motion is nonmonotonic")
        self.motions.append(LidarMotionSample(
            start_s, stamp_s, relative.copy(), min(1.0, weight)
        ))
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
            self.last_time_candidate = TimeOffsetCandidate(
                False, 0.0, -1.0, 0.0, 0, "insufficient_relative_rotations"
            )
            self.last_excitation_ratio = 0.0
            weighted_accumulated_rotation = float(sum(
                interval_weight * np.linalg.norm(rotation_vector)
                for _, _, rotation_vector, interval_weight in intervals
            ))
            physical_accumulated_rotation = float(sum(
                np.linalg.norm(rotation_vector)
                for _, _, rotation_vector, _ in intervals
            ))
            self.last_accumulated_rotation_rad = physical_accumulated_rotation
            self.last_weighted_accumulated_rotation_rad = (
                weighted_accumulated_rotation
            )
            self.last_unweighted_accumulated_rotation_rad = (
                physical_accumulated_rotation
            )
            self.last_imu_accumulated_rotation_rad = 0.0
            self.last_motion_weight_mean = float(np.mean([
                interval_weight
                for _, _, _, interval_weight in intervals
            ])) if intervals else 0.0
            self.last_rotation_inlier_ratio = 0.0
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
        self.last_time_candidate = time_candidate
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
        interval_weights = []
        for start_s, end_s, lidar_vector, interval_weight in intervals:
            imu_rotation = _integrate_gyro(
                imu_samples, start_s + rotation_offset, end_s + rotation_offset
            )
            if imu_rotation is None:
                continue
            imu_rotation_vectors.append(so3_log(imu_rotation))
            lidar_rotation_vectors.append(lidar_vector)
            interval_weights.append(float(interval_weight))
        excitation = np.zeros((3, 3))
        weighted_accumulated_rotation = 0.0
        physical_accumulated_rotation = 0.0
        if lidar_rotation_vectors:
            for vector, interval_weight in zip(
                lidar_rotation_vectors, interval_weights
            ):
                magnitude = float(np.linalg.norm(vector))
                weighted_accumulated_rotation += interval_weight * magnitude
                physical_accumulated_rotation += magnitude
                if magnitude > 1.0e-9:
                    axis = vector / magnitude
                    excitation += interval_weight * np.outer(axis, axis)
        eigenvalues = np.linalg.eigvalsh(excitation)
        excitation_ratio = (
            float(eigenvalues[0] / eigenvalues[-1])
            if eigenvalues[-1] > 1.0e-12 else 0.0
        )
        self.last_excitation_ratio = excitation_ratio
        self.last_accumulated_rotation_rad = physical_accumulated_rotation
        self.last_weighted_accumulated_rotation_rad = (
            weighted_accumulated_rotation
        )
        self.last_unweighted_accumulated_rotation_rad = (
            physical_accumulated_rotation
        )
        self.last_imu_accumulated_rotation_rad = float(sum(
            np.linalg.norm(vector) for vector in imu_rotation_vectors
        ))
        self.last_motion_weight_mean = float(np.mean(interval_weights)) \
            if interval_weights else 0.0
        self.last_rotation_inlier_ratio = 0.0
        rotation_accepted = bool(
            len(imu_rotation_vectors) >= self.minimum_pairs
            and eigenvalues[-1] > self.minimum_excitation_eigenvalue
            and eigenvalues[0] > self.minimum_excitation_eigenvalue
            and excitation_ratio >= self.minimum_excitation_ratio
            and physical_accumulated_rotation
            >= self.minimum_accumulated_rotation_rad
        )
        rotation_residual = -1.0
        if rotation_accepted:
            cross_covariance = sum(
                interval_weight * np.outer(imu_vector, lidar_vector)
                for imu_vector, lidar_vector, interval_weight in zip(
                    imu_rotation_vectors, lidar_rotation_vectors, interval_weights
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
            inliers = np.asarray(errors) <= self.maximum_rotation_residual_rad
            inlier_ratio = float(np.mean(inliers)) if len(inliers) else 0.0
            self.last_rotation_inlier_ratio = inlier_ratio
            rotation_accepted = bool(
                np.count_nonzero(inliers) >= self.minimum_pairs
                and inlier_ratio >= self.minimum_rotation_inlier_ratio
            )
            if rotation_accepted and not np.all(inliers):
                cross_covariance = sum(
                    interval_weight * np.outer(imu_vector, lidar_vector)
                    for imu_vector, lidar_vector, interval_weight, keep in zip(
                        imu_rotation_vectors, lidar_rotation_vectors,
                        interval_weights, inliers
                    )
                    if keep
                )
                left, _, right_t = np.linalg.svd(cross_covariance)
                correction = np.eye(3)
                correction[2, 2] = np.linalg.det(left @ right_t)
                candidate_rotation = left @ correction @ right_t
                errors = [
                    np.linalg.norm(imu_vector - candidate_rotation @ lidar_vector)
                    for imu_vector, lidar_vector, keep in zip(
                        imu_rotation_vectors, lidar_rotation_vectors, inliers
                    )
                    if keep
                ]
            rotation_residual = float(np.sqrt(np.mean(np.square(errors))))
            rotation_accepted = bool(
                rotation_accepted
                and rotation_residual <= self.maximum_rotation_residual_rad
            )
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
