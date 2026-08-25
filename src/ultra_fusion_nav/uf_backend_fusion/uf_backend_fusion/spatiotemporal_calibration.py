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


def _prepare_gyro_interpolation(samples):
    """Sort and deduplicate gyro samples once for repeated interpolation."""
    ordered = sorted(samples, key=_sample_stamp)
    stamps = []
    values = []
    index = 0
    while index < len(ordered):
        stamp_s = _sample_stamp(ordered[index])
        group = [_sample_gyro(ordered[index])]
        index += 1
        while (
            index < len(ordered)
            and abs(_sample_stamp(ordered[index]) - stamp_s) <= 1.0e-9
        ):
            group.append(_sample_gyro(ordered[index]))
            index += 1
        value = group[0] if len(group) == 1 else np.mean(group, axis=0)
        if math.isfinite(stamp_s) and value.shape == (3,) and np.all(np.isfinite(value)):
            stamps.append(float(stamp_s))
            values.append(value)
    return tuple(stamps), tuple(values)


def _interpolate_prepared_gyro(stamps, values, stamp_s):
    if len(stamps) < 2:
        return None
    stamp_s = float(stamp_s)
    index = bisect_left(stamps, stamp_s)
    if index == 0:
        return values[0].copy() if abs(stamps[0] - stamp_s) <= 1.0e-9 else None
    if index >= len(stamps):
        return values[-1].copy() if abs(stamps[-1] - stamp_s) <= 1.0e-9 else None
    if abs(stamps[index] - stamp_s) <= 1.0e-9:
        return values[index].copy()
    left_stamp, right_stamp = stamps[index - 1], stamps[index]
    span = right_stamp - left_stamp
    if span <= 0.0 or span > 0.2:
        return None
    ratio = (stamp_s - left_stamp) / span
    return (1.0 - ratio) * values[index - 1] + ratio * values[index]


def _interpolate_gyro(samples, stamp_s):
    """Linearly interpolate one gyro vector, returning None outside coverage."""
    stamps, values = _prepare_gyro_interpolation(samples)
    return _interpolate_prepared_gyro(stamps, values, stamp_s)


def _integrate_gyro(samples, start_s, end_s, maximum_gap_s=0.12):
    """Integrate gyro samples into ``R_start^T R_end``."""
    stamps, values = _prepare_gyro_interpolation(samples)
    return _integrate_prepared_gyro(
        stamps, values, start_s, end_s, maximum_gap_s
    )


def _integrate_prepared_gyro(
    stamps, values, start_s, end_s, maximum_gap_s=0.12
):
    """Integrate a prepared gyro series without sorting it again."""
    start_s, end_s = float(start_s), float(end_s)
    if end_s <= start_s or len(stamps) < 2 or len(stamps) != len(values):
        return None
    if start_s < stamps[0] or end_s > stamps[-1]:
        return None
    left = bisect_right(stamps, start_s)
    right = bisect_left(stamps, end_s)
    points = [(
        start_s, _interpolate_prepared_gyro(stamps, values, start_s)
    )]
    points.extend(
        (stamps[index], values[index]) for index in range(left, right)
    )
    points.append((
        end_s, _interpolate_prepared_gyro(stamps, values, end_s)
    ))
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


def _prepare_gyro_orientation_trajectory(
    stamps, values, maximum_gap_s=0.12
):
    """Build a cumulative gyro trajectory for repeated interval queries."""
    if len(stamps) < 2 or len(stamps) != len(values):
        return (), ()
    orientations = [np.eye(3)]
    segments = [0]
    for index in range(1, len(stamps)):
        dt_s = float(stamps[index] - stamps[index - 1])
        if dt_s <= 0.0 or dt_s > float(maximum_gap_s):
            orientations.append(np.eye(3))
            segments.append(segments[-1] + 1)
            continue
        orientations.append(
            orientations[-1] @ so3_exp(
                0.5 * (values[index - 1] + values[index]) * dt_s
            )
        )
        segments.append(segments[-1])
    return tuple(orientations), tuple(segments)


def _interpolate_prepared_orientation(
    stamps, values, orientations, segments, stamp_s
):
    if (
        len(stamps) < 2
        or len(stamps) != len(values)
        or len(stamps) != len(orientations)
        or len(stamps) != len(segments)
    ):
        return None
    stamp_s = float(stamp_s)
    index = bisect_left(stamps, stamp_s)
    if index < len(stamps) and abs(stamps[index] - stamp_s) <= 1.0e-9:
        return orientations[index].copy(), segments[index]
    if index == 0 or index >= len(stamps):
        return None
    if segments[index - 1] != segments[index]:
        return None
    left_stamp, right_stamp = stamps[index - 1], stamps[index]
    span = right_stamp - left_stamp
    if span <= 0.0 or span > 0.12:
        return None
    ratio = (stamp_s - left_stamp) / span
    gyro = (1.0 - ratio) * values[index - 1] + ratio * values[index]
    dt_s = stamp_s - left_stamp
    orientation = orientations[index - 1] @ so3_exp(
        0.5 * (values[index - 1] + gyro) * dt_s
    )
    return orientation, segments[index - 1]


def _integrate_prepared_orientation(
    stamps, values, orientations, segments, start_s, end_s
):
    """Return an interval rotation from one cumulative gyro trajectory."""
    if float(end_s) <= float(start_s):
        return None
    start = _interpolate_prepared_orientation(
        stamps, values, orientations, segments, start_s
    )
    end = _interpolate_prepared_orientation(
        stamps, values, orientations, segments, end_s
    )
    if start is None or end is None or start[1] != end[1]:
        return None
    return start[0].T @ end[0]


def _prepare_gyro_vector_integral(stamps, values, maximum_gap_s=0.12):
    """Build a cumulative trapezoidal gyro integral without SO(3) solves."""
    if len(stamps) < 2 or len(stamps) != len(values):
        return (), ()
    integrals = [np.zeros(3)]
    segments = [0]
    for index in range(1, len(stamps)):
        dt_s = float(stamps[index] - stamps[index - 1])
        if dt_s <= 0.0 or dt_s > float(maximum_gap_s):
            integrals.append(np.zeros(3))
            segments.append(segments[-1] + 1)
            continue
        integrals.append(
            integrals[-1]
            + 0.5 * (values[index - 1] + values[index]) * dt_s
        )
        segments.append(segments[-1])
    return tuple(integrals), tuple(segments)


def _interpolate_prepared_gyro_integral(
    stamps, values, integrals, segments, stamp_s
):
    if (
        len(stamps) < 2
        or len(stamps) != len(values)
        or len(stamps) != len(integrals)
        or len(stamps) != len(segments)
    ):
        return None
    stamp_s = float(stamp_s)
    index = bisect_left(stamps, stamp_s)
    if index < len(stamps) and abs(stamps[index] - stamp_s) <= 1.0e-9:
        return integrals[index].copy(), segments[index]
    if index == 0 or index >= len(stamps):
        return None
    if segments[index - 1] != segments[index]:
        return None
    left_stamp, right_stamp = stamps[index - 1], stamps[index]
    span = right_stamp - left_stamp
    if span <= 0.0 or span > 0.12:
        return None
    ratio = (stamp_s - left_stamp) / span
    gyro = (1.0 - ratio) * values[index - 1] + ratio * values[index]
    dt_s = stamp_s - left_stamp
    integral = (
        integrals[index - 1]
        + 0.5 * (values[index - 1] + gyro) * dt_s
    )
    return integral, segments[index - 1]


def _integrate_prepared_gyro_vector(
    stamps, values, integrals, segments, start_s, end_s
):
    if float(end_s) <= float(start_s):
        return None
    start = _interpolate_prepared_gyro_integral(
        stamps, values, integrals, segments, start_s
    )
    end = _interpolate_prepared_gyro_integral(
        stamps, values, integrals, segments, end_s
    )
    if start is None or end is None or start[1] != end[1]:
        return None
    return end[0] - start[0]


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


def effective_time_offset(update, enabled=True, time_locked=None):
    """Expose time calibration independently from optional rotation locking."""
    locked = bool(update.locked) if time_locked is None else bool(time_locked)
    if not enabled or not locked:
        return 0.0
    offset_s = float(update.time_offset_s)
    return offset_s if math.isfinite(offset_s) else 0.0


def estimate_time_offset(
    lidar_rates,
    imu_samples,
    candidate_offsets_s,
    minimum_pairs=8,
    minimum_peak_separation_s=0.0,
):
    """Find the offset maximizing correlation of angular-speed magnitudes."""
    minimum_peak_separation_s = float(minimum_peak_separation_s)
    if minimum_peak_separation_s < 0.0:
        raise ValueError("minimum peak separation must be non-negative")
    gyro_stamps, gyro_values = _prepare_gyro_interpolation(imu_samples)
    return _estimate_time_offset_prepared(
        lidar_rates,
        gyro_stamps,
        gyro_values,
        candidate_offsets_s,
        minimum_pairs,
        minimum_peak_separation_s,
    )


def _estimate_time_offset_prepared(
    lidar_rates,
    gyro_stamps,
    gyro_values,
    candidate_offsets_s,
    minimum_pairs=8,
    minimum_peak_separation_s=0.0,
):
    """Estimate time offset using one already prepared IMU series."""
    minimum_peak_separation_s = float(minimum_peak_separation_s)
    if minimum_peak_separation_s < 0.0:
        raise ValueError("minimum peak separation must be non-negative")
    if len(lidar_rates) < minimum_pairs:
        return TimeOffsetCandidate(
            False, 0.0, -1.0, 0.0, 0, "insufficient_samples"
        )
    if len(gyro_stamps) < 2:
        return TimeOffsetCandidate(False, 0.0, -1.0, 0.0, 0, "insufficient_samples")
    scores = []
    maximum_overlap = 0
    sufficient_overlap = False
    for offset_s in candidate_offsets_s:
        lidar_values = []
        imu_values = []
        for item in lidar_rates:
            stamp_s, lidar_rate = item[:2]
            gyro = _interpolate_prepared_gyro(
                gyro_stamps, gyro_values, stamp_s + float(offset_s)
            )
            if gyro is None:
                continue
            lidar_values.append(float(lidar_rate))
            imu_values.append(float(np.linalg.norm(gyro)))
        maximum_overlap = max(maximum_overlap, len(lidar_values))
        if len(lidar_values) < minimum_pairs:
            continue
        sufficient_overlap = True
        lidar_centered = np.asarray(lidar_values) - np.mean(lidar_values)
        imu_centered = np.asarray(imu_values) - np.mean(imu_values)
        denominator = np.linalg.norm(lidar_centered) * np.linalg.norm(imu_centered)
        if denominator <= 1.0e-9:
            continue
        scores.append((
            float(lidar_centered @ imu_centered / denominator),
            float(offset_s),
            len(lidar_values),
        ))
    if not scores:
        reason = (
            "unexcited_angular_speed"
            if sufficient_overlap
            else "insufficient_overlapping_samples"
        )
        return TimeOffsetCandidate(
            False, 0.0, -1.0, 0.0, maximum_overlap, reason
        )
    scores.sort(reverse=True)
    best = scores[0]
    independent = [
        item for item in scores[1:]
        if abs(item[1] - best[1]) > minimum_peak_separation_s + 1.0e-12
    ]
    if not independent:
        if minimum_peak_separation_s <= 0.0 and len(scores) == 1:
            return TimeOffsetCandidate(
                True, best[1], best[0], best[0] + 1.0, best[2],
                "candidate_ready",
            )
        return TimeOffsetCandidate(
            False,
            best[1],
            best[0],
            0.0,
            best[2],
            "insufficient_independent_offset_hypotheses",
        )
    second = independent[0][0]
    return TimeOffsetCandidate(
        True, best[1], best[0], best[0] - second, best[2], "candidate_ready"
    )


def _estimate_interval_time_offset_prepared(
    intervals,
    gyro_stamps,
    gyro_values,
    candidate_offsets_s,
    lidar_to_body_rotation,
    minimum_pairs=8,
    minimum_peak_separation_s=0.0,
):
    """Align signed LiDAR and IMU rotations over identical time windows."""
    if len(intervals) < minimum_pairs or len(gyro_stamps) < 2:
        return TimeOffsetCandidate(
            False, 0.0, -1.0, 0.0, 0, "insufficient_samples"
        )
    rotation = np.asarray(lidar_to_body_rotation, dtype=float)
    if rotation.shape != (3, 3) or np.any(~np.isfinite(rotation)):
        raise ValueError("LiDAR-to-body rotation must be finite and 3x3")
    integrals, segments = _prepare_gyro_vector_integral(
        gyro_stamps, gyro_values
    )
    if not integrals:
        return TimeOffsetCandidate(
            False, 0.0, -1.0, 0.0, 0, "insufficient_samples"
        )
    offsets = tuple(float(value) for value in candidate_offsets_s)
    scores = []
    maximum_overlap = 0
    sufficient_overlap = False
    for offset_s in offsets:
        lidar_vectors = []
        imu_vectors = []
        weights = []
        for start_s, end_s, lidar_vector, interval_weight in intervals:
            imu_vector = _integrate_prepared_gyro_vector(
                gyro_stamps,
                gyro_values,
                integrals,
                segments,
                start_s + offset_s,
                end_s + offset_s,
            )
            if imu_vector is None:
                continue
            lidar_vectors.append(
                rotation @ np.asarray(lidar_vector, dtype=float)
            )
            imu_vectors.append(imu_vector)
            weights.append(max(1.0e-6, float(interval_weight)))
        maximum_overlap = max(maximum_overlap, len(lidar_vectors))
        if len(lidar_vectors) < minimum_pairs:
            continue
        sufficient_overlap = True
        lidar_array = np.asarray(lidar_vectors, dtype=float)
        imu_array = np.asarray(imu_vectors, dtype=float)
        weight_array = np.asarray(weights, dtype=float)
        weight_array /= np.sum(weight_array)
        lidar_mean = np.sum(weight_array[:, None] * lidar_array, axis=0)
        imu_mean = np.sum(weight_array[:, None] * imu_array, axis=0)
        sqrt_weights = np.sqrt(weight_array)[:, None]
        lidar_centered = (lidar_array - lidar_mean) * sqrt_weights
        imu_centered = (imu_array - imu_mean) * sqrt_weights
        denominator = (
            np.linalg.norm(lidar_centered) * np.linalg.norm(imu_centered)
        )
        if denominator <= 1.0e-9:
            continue
        correlation = float(
            np.sum(lidar_centered * imu_centered) / denominator
        )
        scores.append((correlation, offset_s, len(lidar_vectors)))
    if not scores:
        reason = (
            "unexcited_interval_rotation"
            if sufficient_overlap
            else "insufficient_overlapping_samples"
        )
        return TimeOffsetCandidate(
            False, 0.0, -1.0, 0.0, maximum_overlap, reason
        )
    scores.sort(reverse=True)
    best = scores[0]
    if len(offsets) > 1 and (
        math.isclose(best[1], min(offsets), abs_tol=1.0e-12)
        or math.isclose(best[1], max(offsets), abs_tol=1.0e-12)
    ):
        return TimeOffsetCandidate(
            False,
            best[1],
            best[0],
            0.0,
            best[2],
            "peak_at_search_boundary",
        )
    independent = [
        item
        for item in scores[1:]
        if abs(item[1] - best[1])
        > float(minimum_peak_separation_s) + 1.0e-12
    ]
    if not independent:
        return TimeOffsetCandidate(
            False,
            best[1],
            best[0],
            0.0,
            best[2],
            "insufficient_independent_offset_hypotheses",
        )
    return TimeOffsetCandidate(
        True,
        best[1],
        best[0],
        best[0] - independent[0][0],
        best[2],
        "candidate_ready",
    )


def estimate_interval_time_offset(
    intervals,
    imu_samples,
    candidate_offsets_s,
    observation_to_body_rotation=None,
    minimum_pairs=8,
    minimum_peak_separation_s=0.0,
):
    """Align signed observation rotations with gyro integration windows.

    Each interval is ``(start_s, end_s, rotation_vector, weight)``. The
    rotation vector is expressed in the observation frame and is transformed
    to the FCU body frame before correlation. Visual and LiDAR calibration can
    therefore share one timestamp convention and integration model.
    """
    rotation = (
        np.eye(3, dtype=float)
        if observation_to_body_rotation is None
        else np.asarray(observation_to_body_rotation, dtype=float)
    )
    if (
        rotation.shape != (3, 3)
        or np.any(~np.isfinite(rotation))
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6)
        or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6)
    ):
        raise ValueError("observation-to-body rotation must be a valid SO(3) matrix")
    gyro_stamps, gyro_values = _prepare_gyro_interpolation(imu_samples)
    return _estimate_interval_time_offset_prepared(
        tuple(intervals),
        gyro_stamps,
        gyro_values,
        candidate_offsets_s,
        rotation,
        minimum_pairs,
        minimum_peak_separation_s,
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
        minimum_correlation_margin=0.002,
        minimum_time_peak_separation_s=0.020,
        minimum_time_accumulated_rotation_rad=0.25,
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
        minimum_time_lock_candidate_separation_s=1.0,
        time_unlock_count=3,
        estimate_rotation=True,
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
        self.minimum_time_peak_separation_s = float(
            minimum_time_peak_separation_s
        )
        self.minimum_time_accumulated_rotation_rad = float(
            minimum_time_accumulated_rotation_rad
        )
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
        self.minimum_time_lock_candidate_separation_s = float(
            minimum_time_lock_candidate_separation_s
        )
        self.time_unlock_count = int(time_unlock_count)
        self.estimate_rotation = bool(estimate_rotation)
        self.last_solve_stamp_s = None
        if (
            not 0.0 < self.minimum_excitation_ratio <= 1.0
            or self.minimum_accumulated_rotation_rad <= 0.0
            or self.minimum_time_peak_separation_s < 0.0
            or self.minimum_time_accumulated_rotation_rad <= 0.0
            or not 0.0 < self.minimum_rotation_inlier_ratio <= 1.0
            or self.minimum_time_lock_candidate_separation_s < 0.0
            or self.time_unlock_count < 1
        ):
            raise ValueError("invalid calibration observability limits")
        self.motions = deque()
        self.time_offset_history = deque(maxlen=self.history_length)
        self.time_conflict_history = deque(maxlen=self.time_unlock_count)
        self.rotation_history = deque(maxlen=self.history_length)
        self.time_offset_s = 0.0
        self.lidar_to_body_rotation = np.eye(3)
        self.time_locked = False
        self.last_time_lock_candidate_stamp_s = None
        self.time_lock_candidate_count = 0
        self.time_lock_conflict_count = 0
        self.time_lock_revocations = 0
        # A fixed measured extrinsic is authoritative when rotation estimation
        # is disabled. It is considered available, but is never optimized.
        self.rotation_locked = not self.estimate_rotation
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

    def _update_time_lock(self, candidate_offset_s, candidate_stamp_s=None):
        """Update the time lock from one independent, high-confidence vote."""
        candidate_offset_s = float(candidate_offset_s)
        if not math.isfinite(candidate_offset_s):
            raise ValueError("time offset candidate must be finite")
        if candidate_stamp_s is not None:
            candidate_stamp_s = float(candidate_stamp_s)
            if not math.isfinite(candidate_stamp_s):
                raise ValueError("time offset candidate stamp must be finite")
            if (
                self.last_time_lock_candidate_stamp_s is not None
                and candidate_stamp_s - self.last_time_lock_candidate_stamp_s
                < self.minimum_time_lock_candidate_separation_s
            ):
                return False
            self.last_time_lock_candidate_stamp_s = candidate_stamp_s
        self.time_lock_candidate_count += 1

        if self.time_locked:
            if abs(candidate_offset_s - self.time_offset_s) > self.stability_tolerance_s:
                if self.time_conflict_history:
                    conflict_center = float(np.median(self.time_conflict_history))
                    if (
                        abs(candidate_offset_s - conflict_center)
                        > self.stability_tolerance_s
                    ):
                        self.time_conflict_history.clear()
                self.time_conflict_history.append(candidate_offset_s)
                self.time_lock_conflict_count += 1
                conflicts = np.asarray(self.time_conflict_history, dtype=float)
                if len(conflicts) >= self.time_unlock_count:
                    conflict_center = float(np.median(conflicts))
                    stable_conflict = bool(
                        np.max(np.abs(conflicts - conflict_center))
                        <= self.stability_tolerance_s
                    )
                    if stable_conflict:
                        self.time_locked = False
                        self.time_lock_revocations += 1
                        self.time_offset_history.clear()
                        self.time_conflict_history.clear()
                return False
            self.time_conflict_history.clear()
            self.time_offset_history.append(candidate_offset_s)
            target_offset = float(np.median(self.time_offset_history))
            self.time_offset_s = float(
                (1.0 - self.update_alpha) * self.time_offset_s
                + self.update_alpha * target_offset
            )
            return True

        if self.time_offset_history:
            cluster_center = float(np.median(self.time_offset_history))
            if abs(candidate_offset_s - cluster_center) > self.stability_tolerance_s:
                self.time_offset_history.clear()
        self.time_offset_history.append(candidate_offset_s)
        recent_offsets = np.asarray(self.time_offset_history, dtype=float)
        if len(recent_offsets) < self.lock_count:
            return False
        cluster_center = float(np.median(recent_offsets))
        stable = bool(
            np.max(np.abs(recent_offsets - cluster_center))
            <= self.stability_tolerance_s
        )
        if not stable:
            return False
        self.time_offset_s = cluster_center
        self.time_locked = True
        self.time_conflict_history.clear()
        return True

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
        gyro_stamps, gyro_values = _prepare_gyro_interpolation(imu_samples)
        _, intervals = self._lidar_rates()
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
        physical_time_rotation = float(sum(
            np.linalg.norm(rotation_vector)
            for _, _, rotation_vector, _ in intervals
        ))
        time_candidate = _estimate_interval_time_offset_prepared(
            intervals,
            gyro_stamps,
            gyro_values,
            self.candidate_offsets_s,
            self.lidar_to_body_rotation,
            self.minimum_pairs,
            self.minimum_time_peak_separation_s,
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
            and physical_time_rotation
            >= self.minimum_time_accumulated_rotation_rad
            and not sharp_turn
        )
        if time_accepted:
            self._update_time_lock(time_candidate.offset_s, stamp_s)
        if not self.estimate_rotation:
            self.last_excitation_ratio = 0.0
            self.last_accumulated_rotation_rad = physical_time_rotation
            self.last_weighted_accumulated_rotation_rad = float(sum(
                interval_weight * np.linalg.norm(rotation_vector)
                for _, _, rotation_vector, interval_weight in intervals
            ))
            self.last_unweighted_accumulated_rotation_rad = (
                physical_time_rotation
            )
            self.last_imu_accumulated_rotation_rad = 0.0
            self.last_motion_weight_mean = float(np.mean([
                interval_weight
                for _, _, _, interval_weight in intervals
            ])) if intervals else 0.0
            self.last_rotation_inlier_ratio = 0.0
            reason = "time_accepted_fixed_extrinsic" if time_accepted else (
                "sharp_turn_frozen" if sharp_turn
                else "time_unobservable_or_low_confidence"
            )
            self.last_update = CalibrationUpdate(
                time_accepted,
                self.time_locked,
                self.time_offset_s,
                self.lidar_to_body_rotation.copy(),
                time_candidate.correlation,
                time_candidate.margin,
                -1.0,
                (0.0, 0.0, 0.0),
                time_candidate.pair_count,
                reason,
            )
            return self.last_update
        rotation_offset = (
            time_candidate.offset_s if time_accepted else self.time_offset_s
        )
        imu_rotation_vectors = []
        lidar_rotation_vectors = []
        interval_weights = []
        for start_s, end_s, lidar_vector, interval_weight in intervals:
            imu_rotation = _integrate_prepared_gyro(
                gyro_stamps,
                gyro_values,
                start_s + rotation_offset,
                end_s + rotation_offset,
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
                    self.lidar_to_body_rotation = (
                        self.lidar_to_body_rotation
                        @ so3_exp(self.update_alpha * delta)
                    )
                self.rotation_locked = bool(
                    len(self.rotation_history) >= self.lock_count
                    and all(
                        np.linalg.norm(
                            so3_log(
                                self.rotation_history[index - 1].T
                                @ self.rotation_history[index]
                            )
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
