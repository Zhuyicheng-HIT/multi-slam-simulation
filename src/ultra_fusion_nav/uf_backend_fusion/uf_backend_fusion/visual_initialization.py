"""Conservative visual warm-up and camera/IMU time calibration."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .manifold import so3_log
from .spatiotemporal_calibration import estimate_interval_time_offset


@dataclass(frozen=True)
class VisualTimeCalibrationUpdate:
    accepted: bool
    locked: bool
    time_offset_s: float
    correlation: float
    margin: float
    pair_count: int
    reason: str
    candidate_offset_s: float = 0.0


class OnlineVisualTimeCalibrator:
    """Estimate ``t_imu = t_camera + td_C`` from PnP and FCU gyro rates."""

    def __init__(
        self,
        initial_offset_s=0.0,
        window_s=12.0,
        minimum_pairs=8,
        offset_range_s=0.060,
        offset_step_s=0.002,
        minimum_correlation=0.65,
        minimum_correlation_margin=0.02,
        minimum_peak_separation_s=0.010,
        reject_boundary_candidates=True,
        history_length=4,
        lock_count=3,
        stability_tolerance_s=0.006,
        minimum_accumulated_rotation_rad=0.10,
        minimum_interval_rotation_rad=0.001,
        minimum_lock_candidate_separation_s=1.0,
        minimum_interval_s=0.03,
        maximum_interval_s=0.50,
    ):
        if (
            window_s <= 0.0
            or minimum_pairs < 3
            or offset_step_s <= 0.0
            or offset_range_s < 0.0
            or minimum_peak_separation_s < 0.0
            or history_length < lock_count
            or lock_count < 1
            or stability_tolerance_s <= 0.0
            or minimum_accumulated_rotation_rad <= 0.0
            or minimum_interval_rotation_rad < 0.0
            or minimum_lock_candidate_separation_s < 0.0
            or minimum_interval_s <= 0.0
            or maximum_interval_s <= minimum_interval_s
        ):
            raise ValueError("invalid visual time calibration configuration")
        self.initial_offset_s = float(initial_offset_s)
        self.window_s = float(window_s)
        self.minimum_pairs = int(minimum_pairs)
        self.candidate_offsets_s = self.initial_offset_s + np.arange(
            -float(offset_range_s),
            float(offset_range_s) + 0.5 * float(offset_step_s),
            float(offset_step_s),
        )
        self.minimum_correlation = float(minimum_correlation)
        self.minimum_correlation_margin = float(minimum_correlation_margin)
        self.minimum_peak_separation_s = float(minimum_peak_separation_s)
        self.reject_boundary_candidates = bool(reject_boundary_candidates)
        self.offset_step_s = float(offset_step_s)
        self.lock_count = int(lock_count)
        self.stability_tolerance_s = float(stability_tolerance_s)
        self.minimum_accumulated_rotation_rad = float(
            minimum_accumulated_rotation_rad
        )
        self.minimum_interval_rotation_rad = float(
            minimum_interval_rotation_rad
        )
        self.minimum_lock_candidate_separation_s = float(
            minimum_lock_candidate_separation_s
        )
        self.minimum_interval_s = float(minimum_interval_s)
        self.maximum_interval_s = float(maximum_interval_s)
        self.motion_intervals = deque()
        self.offset_history = deque(maxlen=int(history_length))
        self.last_lock_candidate_stamp_s = None
        self.time_offset_s = self.initial_offset_s
        self.locked = False
        self.last_update = VisualTimeCalibrationUpdate(
            False, False, self.initial_offset_s, -1.0, 0.0, 0,
            "insufficient_samples",
        )

    @staticmethod
    def _validated_rotation(rotation):
        rotation = np.asarray(rotation, dtype=float)
        if (
            rotation.shape != (3, 3)
            or np.any(~np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=2.0e-3)
            or not math.isclose(np.linalg.det(rotation), 1.0, abs_tol=2.0e-3)
        ):
            raise ValueError("visual PnP rotation is not a valid SO(3) matrix")
        return rotation

    def update(
        self,
        previous_stamp_s,
        current_stamp_s,
        relative_rotation,
        imu_samples,
        rotation_body_camera=None,
    ):
        previous_stamp_s = float(previous_stamp_s)
        current_stamp_s = float(current_stamp_s)
        dt_s = current_stamp_s - previous_stamp_s
        if (
            not math.isfinite(previous_stamp_s)
            or not math.isfinite(current_stamp_s)
            or dt_s < self.minimum_interval_s
            or dt_s > self.maximum_interval_s
        ):
            self.last_update = VisualTimeCalibrationUpdate(
                False, self.locked, self.time_offset_s, -1.0, 0.0, 0,
                "invalid_visual_interval",
            )
            return self.last_update
        rotation = self._validated_rotation(relative_rotation)
        rotation_body_camera = self._validated_rotation(
            np.eye(3) if rotation_body_camera is None else rotation_body_camera
        )
        # solvePnP maps points from the previous camera coordinates into the
        # current camera coordinates. Its transpose is the physical camera
        # rotation. Conjugate it into FCU body axes before comparing it with
        # the signed gyro integral over the exact same interval.
        physical_body_rotation = (
            rotation_body_camera @ rotation.T @ rotation_body_camera.T
        )
        body_rotation_vector = so3_log(physical_body_rotation)
        if (
            np.linalg.norm(body_rotation_vector)
            < self.minimum_interval_rotation_rad
        ):
            self.last_update = VisualTimeCalibrationUpdate(
                False,
                self.locked,
                self.time_offset_s,
                -1.0,
                0.0,
                len(self.motion_intervals),
                "insufficient_visual_interval_rotation",
            )
            return self.last_update
        self.motion_intervals.append((
            previous_stamp_s,
            current_stamp_s,
            body_rotation_vector,
            1.0,
        ))
        self.motion_intervals = deque(sorted(
            self.motion_intervals, key=lambda item: item[1]
        ))
        newest_stamp_s = self.motion_intervals[-1][1]
        while (
            self.motion_intervals
            and newest_stamp_s - self.motion_intervals[0][1] > self.window_s
        ):
            self.motion_intervals.popleft()
        accumulated_rotation_rad = float(sum(
            np.linalg.norm(item[2]) for item in self.motion_intervals
        ))
        candidate = estimate_interval_time_offset(
            tuple(self.motion_intervals),
            tuple(imu_samples),
            self.candidate_offsets_s,
            observation_to_body_rotation=np.eye(3),
            minimum_pairs=self.minimum_pairs,
            minimum_peak_separation_s=self.minimum_peak_separation_s,
        )
        if not candidate.valid:
            self.last_update = VisualTimeCalibrationUpdate(
                False, self.locked, self.time_offset_s,
                candidate.correlation, candidate.margin,
                candidate.pair_count, candidate.reason,
                float(candidate.offset_s),
            )
            return self.last_update
        boundary_distance_s = min(
            abs(candidate.offset_s - float(self.candidate_offsets_s[0])),
            abs(candidate.offset_s - float(self.candidate_offsets_s[-1])),
        )
        if (
            self.reject_boundary_candidates
            and boundary_distance_s <= 0.5 * self.offset_step_s + 1.0e-12
        ):
            reason = "visual_time_offset_search_boundary"
        elif candidate.correlation < self.minimum_correlation:
            reason = "low_visual_imu_correlation"
        elif candidate.margin < self.minimum_correlation_margin:
            reason = "ambiguous_visual_time_offset"
        elif accumulated_rotation_rad < self.minimum_accumulated_rotation_rad:
            reason = "insufficient_visual_rotation"
        else:
            reason = "candidate_ready"
        accepted = reason == "candidate_ready"
        if accepted and not self.locked:
            if (
                self.last_lock_candidate_stamp_s is not None
                and current_stamp_s - self.last_lock_candidate_stamp_s + 1.0e-12
                < self.minimum_lock_candidate_separation_s
            ):
                accepted = False
                reason = "visual_time_offset_vote_not_independent"
            else:
                self.last_lock_candidate_stamp_s = current_stamp_s
        if not accepted and not self.locked:
            # Locking requires consecutive observable candidates; isolated
            # weak/ambiguous peaks must not accumulate. A vote from the same
            # overlapping window is deferred, not treated as a contradiction.
            if reason != "visual_time_offset_vote_not_independent":
                self.offset_history.clear()
        if accepted and not self.locked:
            self.offset_history.append(float(candidate.offset_s))
            recent = np.asarray(self.offset_history, dtype=float)
            if (
                recent.size >= self.lock_count
                and float(np.ptp(recent[-self.lock_count:]))
                <= self.stability_tolerance_s
            ):
                self.time_offset_s = float(
                    np.median(recent[-self.lock_count:])
                )
                self.locked = True
                reason = "visual_time_offset_locked"
            else:
                reason = "visual_time_offset_stabilizing"
        elif accepted:
            reason = "visual_time_offset_locked"
        self.last_update = VisualTimeCalibrationUpdate(
            accepted,
            self.locked,
            self.time_offset_s,
            candidate.correlation,
            candidate.margin,
            candidate.pair_count,
            reason,
            float(candidate.offset_s),
        )
        return self.last_update


@dataclass(frozen=True)
class VisualInitializationUpdate:
    ready: bool
    accepted: bool
    consecutive_batches: int
    reason: str


class VisualInitializationGate:
    """Admit vision only after consecutive cross-modal consistency checks."""

    def __init__(self, minimum_batches=3, require_time_lock=True):
        if minimum_batches < 1:
            raise ValueError("visual initialization needs at least one batch")
        self.minimum_batches = int(minimum_batches)
        self.require_time_lock = bool(require_time_lock)
        self.consecutive_batches = 0
        self.ready = False
        self.last_update = VisualInitializationUpdate(
            False, False, 0, "waiting_for_visual_batches"
        )

    def reset(self, reason="reset"):
        self.consecutive_batches = 0
        self.ready = False
        self.last_update = VisualInitializationUpdate(
            False, False, 0, str(reason)
        )
        return self.last_update

    def observe(self, *, geometrically_valid, time_locked):
        if self.ready:
            self.last_update = VisualInitializationUpdate(
                True, True, self.consecutive_batches, "visual_initialized"
            )
            return self.last_update
        if self.require_time_lock and not bool(time_locked):
            self.consecutive_batches = 0
            self.last_update = VisualInitializationUpdate(
                False, False, 0, "waiting_for_visual_time_lock"
            )
            return self.last_update
        if not bool(geometrically_valid):
            self.consecutive_batches = 0
            self.last_update = VisualInitializationUpdate(
                False, False, 0, "visual_geometry_inconsistent"
            )
            return self.last_update
        self.consecutive_batches += 1
        self.ready = self.consecutive_batches >= self.minimum_batches
        self.last_update = VisualInitializationUpdate(
            self.ready,
            True,
            self.consecutive_batches,
            "visual_initialized" if self.ready else "visual_initializing",
        )
        return self.last_update
