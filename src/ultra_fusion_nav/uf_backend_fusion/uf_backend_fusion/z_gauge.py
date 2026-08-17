"""Bounded global-height gauge for a locally consistent LiDAR map."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ZGaugeUpdate:
    initialized: bool
    active: bool
    offset_m: float
    variance_m2: float
    correction_m: float
    target_offset_m: float
    raw_target_offset_m: float
    reason: str


class LocalToGlobalZGauge:
    """Estimate ``z_global = z_local + offset`` without moving the local map.

    GNSS establishes the global datum. The offset is updated only while LiDAR
    Z is weak, so locally stable LiDAR/RGB-D geometry remains untouched. The
    bounded update prevents an accepted GNSS recovery from becoming a pose
    jump. This class never consumes pressure; barometer measurements stay in
    the local frame and cannot establish a global datum.
    """

    def __init__(
        self,
        *,
        initialization_samples=3,
        initialization_max_spread_m=0.30,
        target_history_size=1,
        update_time_constant_s=0.60,
        maximum_correction_rate_mps=1.0,
        maximum_correction_step_m=0.30,
        minimum_variance_m2=0.04,
        maximum_variance_m2=25.0,
    ):
        self.initialization_samples = int(initialization_samples)
        self.initialization_max_spread_m = float(
            initialization_max_spread_m
        )
        self.update_time_constant_s = float(update_time_constant_s)
        self.maximum_correction_rate_mps = float(
            maximum_correction_rate_mps
        )
        self.maximum_correction_step_m = float(maximum_correction_step_m)
        self.minimum_variance_m2 = float(minimum_variance_m2)
        self.maximum_variance_m2 = float(maximum_variance_m2)
        history_size = int(target_history_size)
        if (
            self.initialization_samples < 2
            or self.initialization_max_spread_m <= 0.0
            or history_size < 1
            or self.update_time_constant_s <= 0.0
            or self.maximum_correction_rate_mps <= 0.0
            or self.maximum_correction_step_m <= 0.0
            or self.minimum_variance_m2 <= 0.0
            or self.maximum_variance_m2 < self.minimum_variance_m2
        ):
            raise ValueError("Z gauge configuration is invalid")
        self.initialization_candidates = deque(
            maxlen=self.initialization_samples
        )
        self.target_history = deque(maxlen=history_size)
        self.initialized = False
        self.active = False
        self.offset_m = 0.0
        self.variance_m2 = self.maximum_variance_m2
        self.last_update_stamp_s = None
        self.last_reason = "uninitialized"

    def reset(self, reason="epoch_reset"):
        self.initialization_candidates.clear()
        self.target_history.clear()
        self.initialized = False
        self.active = False
        self.offset_m = 0.0
        self.variance_m2 = self.maximum_variance_m2
        self.last_update_stamp_s = None
        self.last_reason = str(reason)

    def _result(
        self, correction_m, target_offset_m, raw_target_offset_m=math.nan
    ):
        return ZGaugeUpdate(
            initialized=self.initialized,
            active=self.active,
            offset_m=self.offset_m,
            variance_m2=self.variance_m2,
            correction_m=float(correction_m),
            target_offset_m=float(target_offset_m),
            raw_target_offset_m=float(raw_target_offset_m),
            reason=self.last_reason,
        )

    def update(
        self,
        stamp_s,
        local_z_m,
        global_z_m,
        global_variance_m2,
        *,
        source_healthy,
        lidar_z_weak,
    ):
        stamp_s = float(stamp_s)
        local_z_m = float(local_z_m)
        global_z_m = float(global_z_m)
        global_variance_m2 = float(global_variance_m2)
        if (
            not bool(source_healthy)
            or not math.isfinite(stamp_s)
            or stamp_s <= 0.0
            or not math.isfinite(local_z_m)
            or not math.isfinite(global_z_m)
            or not math.isfinite(global_variance_m2)
            or global_variance_m2 <= 0.0
        ):
            self.active = False
            self.last_reason = "global_height_source_unavailable"
            return self._result(0.0, math.nan)
        target = global_z_m - local_z_m
        self.target_history.append(target)
        if not self.initialized:
            self.initialization_candidates.append(
                (target, global_variance_m2)
            )
            if len(self.initialization_candidates) < self.initialization_samples:
                self.last_reason = "collecting_global_height_datum"
                return self._result(0.0, target, target)
            offsets = np.asarray([
                value[0] for value in self.initialization_candidates
            ])
            if float(np.ptp(offsets)) > self.initialization_max_spread_m:
                self.initialization_candidates.popleft()
                self.last_reason = "global_height_datum_unstable"
                return self._result(0.0, target, target)
            self.offset_m = float(np.median(offsets))
            variances = np.asarray([
                value[1] for value in self.initialization_candidates
            ])
            robust_spread = float(np.median(
                np.abs(offsets - np.median(offsets))
            ))
            self.variance_m2 = min(
                self.maximum_variance_m2,
                max(
                    self.minimum_variance_m2,
                    float(np.median(variances)) + robust_spread ** 2,
                ),
            )
            self.initialized = True
            self.active = False
            self.last_update_stamp_s = stamp_s
            self.last_reason = "global_height_datum_initialized"
            return self._result(0.0, self.offset_m, target)
        if not bool(lidar_z_weak):
            self.active = False
            self.last_update_stamp_s = stamp_s
            self.last_reason = "lidar_z_observable"
            return self._result(0.0, target, target)
        dt_s = (
            max(0.0, stamp_s - self.last_update_stamp_s)
            if self.last_update_stamp_s is not None else 0.0
        )
        self.last_update_stamp_s = stamp_s
        if dt_s <= 0.0:
            self.active = False
            self.last_reason = "nonprogressing_global_height_stamp"
            return self._result(0.0, target, target)
        filtered_target = float(np.median(np.asarray(self.target_history)))
        alpha = 1.0 - math.exp(-dt_s / self.update_time_constant_s)
        requested = alpha * (filtered_target - self.offset_m)
        correction_limit = min(
            self.maximum_correction_step_m,
            self.maximum_correction_rate_mps * dt_s,
        )
        correction = float(np.clip(
            requested, -correction_limit, correction_limit
        ))
        self.offset_m += correction
        self.variance_m2 = min(
            self.maximum_variance_m2,
            max(
                self.minimum_variance_m2,
                (1.0 - alpha) * self.variance_m2
                + alpha * global_variance_m2,
            ),
        )
        self.active = True
        self.last_reason = "gnss_z_gauge_update"
        return self._result(correction, filtered_target, target)

    def global_z(self, local_z_m):
        return float(local_z_m) + (self.offset_m if self.initialized else 0.0)

    def local_z(self, global_z_m):
        return float(global_z_m) - (self.offset_m if self.initialized else 0.0)
