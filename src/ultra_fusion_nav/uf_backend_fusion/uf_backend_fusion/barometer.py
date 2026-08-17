"""Local-segment barometer fallback for the unified navigation backend."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BarometerHeightMeasurement:
    stamp_s: float
    height_m: float
    variance_m2: float
    segment_id: int


class LocalBarometerSegment:
    """Turn absolute pressure into a bounded local relative-height segment.

    The pressure-to-height datum is fitted only from samples immediately
    preceding activation and is anchored to the current backend Z. Ending the
    segment discards that datum, so pressure changes across rooms, floors, or
    weather intervals can never become one global altitude constraint.
    """

    def __init__(
        self,
        *,
        baseline_window_s=2.0,
        minimum_baseline_samples=8,
        minimum_baseline_span_s=0.8,
        maximum_sample_age_s=0.5,
        scale_height_m=8434.5,
        default_height_variance_m2=0.25,
        maximum_relative_height_m=30.0,
        buffer_capacity=512,
    ):
        self.baseline_window_s = float(baseline_window_s)
        self.minimum_baseline_samples = int(minimum_baseline_samples)
        self.minimum_baseline_span_s = float(minimum_baseline_span_s)
        self.maximum_sample_age_s = float(maximum_sample_age_s)
        self.scale_height_m = float(scale_height_m)
        self.default_height_variance_m2 = float(
            default_height_variance_m2
        )
        self.maximum_relative_height_m = float(maximum_relative_height_m)
        self.samples = deque(maxlen=int(buffer_capacity))
        if (
            self.baseline_window_s <= 0.0
            or self.minimum_baseline_samples < 2
            or not 0.0 < self.minimum_baseline_span_s
            <= self.baseline_window_s
            or self.maximum_sample_age_s <= 0.0
            or self.scale_height_m <= 0.0
            or self.default_height_variance_m2 <= 0.0
            or self.maximum_relative_height_m <= 0.0
            or int(buffer_capacity) < self.minimum_baseline_samples
        ):
            raise ValueError("barometer segment configuration is invalid")
        self.active = False
        self.segment_id = 0
        self.baseline_pressure_pa = math.nan
        self.anchor_height_m = math.nan
        self.last_emitted_stamp_s = -math.inf
        self.last_reason = "inactive"

    def add_sample(self, stamp_s, pressure_pa, pressure_variance_pa2=0.0):
        stamp_s = float(stamp_s)
        pressure_pa = float(pressure_pa)
        pressure_variance_pa2 = float(pressure_variance_pa2)
        # MAVLink SCALED_PRESSURE has no variance field.  MAVROS therefore
        # reports an unknown variance as 0, -1, or NaN depending on the
        # plugin/version.  Keep the pressure sample and let the configured
        # local-height variance represent that uncertainty; timestamp and
        # pressure validity remain hard requirements.
        if (
            not math.isfinite(pressure_variance_pa2)
            or pressure_variance_pa2 < 0.0
        ):
            pressure_variance_pa2 = 0.0
        if (
            not math.isfinite(stamp_s)
            or stamp_s <= 0.0
            or not math.isfinite(pressure_pa)
            or not 30000.0 <= pressure_pa <= 120000.0
        ):
            self.last_reason = "invalid_pressure_sample"
            return False
        if self.samples and stamp_s <= self.samples[-1][0]:
            self.last_reason = "nonmonotonic_pressure_stamp"
            return False
        self.samples.append((stamp_s, pressure_pa, pressure_variance_pa2))
        self.last_reason = "sample_buffered"
        return True

    def deactivate(self, reason="fallback_not_required", clear_samples=False):
        self.active = False
        self.baseline_pressure_pa = math.nan
        self.anchor_height_m = math.nan
        self.last_emitted_stamp_s = -math.inf
        if clear_samples:
            self.samples.clear()
        self.last_reason = str(reason)

    def reset(self, reason="segment_reset"):
        """Discard both the active datum and all pre-reset pressure history."""
        self.deactivate(reason, clear_samples=True)

    def _activate(self, stamp_s, anchor_height_m):
        start_s = stamp_s - self.baseline_window_s
        baseline = [
            sample for sample in self.samples
            if start_s <= sample[0] <= stamp_s
        ]
        if len(baseline) < self.minimum_baseline_samples:
            self.last_reason = "insufficient_baseline_samples"
            return False
        span_s = baseline[-1][0] - baseline[0][0]
        if span_s < self.minimum_baseline_span_s:
            self.last_reason = "insufficient_baseline_span"
            return False
        times = np.asarray([sample[0] - stamp_s for sample in baseline])
        pressures = np.asarray([sample[1] for sample in baseline])
        design = np.column_stack((times, np.ones(times.size)))
        _, intercept = np.linalg.lstsq(design, pressures, rcond=None)[0]
        if not math.isfinite(intercept) or not 30000.0 <= intercept <= 120000.0:
            self.last_reason = "invalid_baseline_fit"
            return False
        self.active = True
        self.segment_id += 1
        self.baseline_pressure_pa = float(intercept)
        self.anchor_height_m = float(anchor_height_m)
        self.last_emitted_stamp_s = -math.inf
        self.last_reason = "segment_activated"
        return True

    def measurement(self, stamp_s, anchor_height_m, fallback_required):
        stamp_s = float(stamp_s)
        anchor_height_m = float(anchor_height_m)
        if not bool(fallback_required):
            if self.active:
                self.deactivate(clear_samples=True)
            else:
                self.last_reason = "fallback_not_required"
            return None
        if (
            not math.isfinite(stamp_s)
            or not math.isfinite(anchor_height_m)
            or not self.samples
        ):
            self.last_reason = "pressure_or_state_unavailable"
            return None
        latest = self.samples[-1]
        age_s = stamp_s - latest[0]
        if age_s < 0.0 or age_s > self.maximum_sample_age_s:
            self.last_reason = "pressure_sample_stale"
            return None
        if not self.active and not self._activate(stamp_s, anchor_height_m):
            return None
        if latest[0] <= self.last_emitted_stamp_s:
            self.last_reason = "pressure_sample_already_consumed"
            return None
        relative_height_m = -self.scale_height_m * math.log(
            latest[1] / self.baseline_pressure_pa
        )
        if (
            not math.isfinite(relative_height_m)
            or abs(relative_height_m) > self.maximum_relative_height_m
        ):
            self.deactivate("relative_height_out_of_range")
            return None
        pressure_variance = latest[2]
        propagated_variance = (
            (self.scale_height_m / latest[1]) ** 2 * pressure_variance
            if pressure_variance > 0.0 else 0.0
        )
        variance_m2 = max(
            self.default_height_variance_m2, propagated_variance
        )
        self.last_emitted_stamp_s = latest[0]
        self.last_reason = "measurement_ready"
        return BarometerHeightMeasurement(
            stamp_s=latest[0],
            height_m=self.anchor_height_m + relative_height_m,
            variance_m2=variance_m2,
            segment_id=self.segment_id,
        )
