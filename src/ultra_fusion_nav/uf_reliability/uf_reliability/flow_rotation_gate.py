"""Stateful FCU-yaw-rate gate for planar optical-flow factors."""

from dataclasses import dataclass
import bisect
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class FlowRotationGateConfig:
    lower_yaw_rate_radps: float = 0.08
    upper_yaw_rate_radps: float = 0.30
    recovery_dwell_s: float = 0.8
    recovery_ramp_s: float = 1.5
    minimum_translation_m: float = 0.01
    minimum_translation_speed_mps: float = 0.0
    allow_compensated_rotation: bool = False


@dataclass(frozen=True)
class FlowRotationGateResult:
    weight: float
    hard_disabled: bool
    phase: str
    yaw_rate_abs_radps: float
    translation_ready: bool
    reason: str


def _sample_scalar(samples, timestamp_s, maximum_gap_s):
    times = [sample[0] for sample in samples]
    index = bisect.bisect_left(times, timestamp_s)
    if index < len(samples) and abs(times[index] - timestamp_s) <= 1.0e-9:
        return float(samples[index][1])
    if index == 0:
        return (
            float(samples[0][1])
            if times[0] - timestamp_s <= maximum_gap_s else None
        )
    if index >= len(samples):
        return (
            float(samples[-1][1])
            if timestamp_s - times[-1] <= maximum_gap_s else None
        )
    before = samples[index - 1]
    after = samples[index]
    gap_s = after[0] - before[0]
    if gap_s <= 0.0 or gap_s > maximum_gap_s:
        return None
    ratio = (timestamp_s - before[0]) / gap_s
    return float(before[1]) + ratio * (float(after[1]) - float(before[1]))


def _sample_vector(samples, timestamp_s, maximum_gap_s):
    times = [sample[0] for sample in samples]
    index = bisect.bisect_left(times, timestamp_s)
    if index < len(samples) and abs(times[index] - timestamp_s) <= 1.0e-9:
        return tuple(float(value) for value in samples[index][1])
    if index == 0:
        return (
            tuple(float(value) for value in samples[0][1])
            if times[0] - timestamp_s <= maximum_gap_s else None
        )
    if index >= len(samples):
        return (
            tuple(float(value) for value in samples[-1][1])
            if timestamp_s - times[-1] <= maximum_gap_s else None
        )
    before = samples[index - 1]
    after = samples[index]
    gap_s = after[0] - before[0]
    if gap_s <= 0.0 or gap_s > maximum_gap_s:
        return None
    ratio = (timestamp_s - before[0]) / gap_s
    return tuple(
        float(before[1][component])
        + ratio * (float(after[1][component]) - float(before[1][component]))
        for component in range(len(before[1]))
    )


def interval_mean_vector(
    samples: Iterable[Sequence[float]],
    start_s: float,
    end_s: float,
    maximum_gap_s: float = 0.12,
):
    """Average a timestamped finite vector over one integration interval."""
    start_s = float(start_s)
    end_s = float(end_s)
    maximum_gap_s = float(maximum_gap_s)
    if (
        not math.isfinite(start_s)
        or not math.isfinite(end_s)
        or end_s <= start_s
        or not math.isfinite(maximum_gap_s)
        or maximum_gap_s <= 0.0
    ):
        return None
    ordered = []
    dimension = None
    for sample in samples:
        try:
            timestamp = float(sample[0])
            vector = tuple(float(value) for value in sample[1])
        except (IndexError, TypeError, ValueError):
            continue
        if (
            not math.isfinite(timestamp)
            or not vector
            or not all(math.isfinite(value) for value in vector)
        ):
            continue
        if dimension is None:
            dimension = len(vector)
        if len(vector) == dimension:
            ordered.append((timestamp, vector))
    ordered.sort(key=lambda sample: sample[0])
    if not ordered or dimension is None:
        return None
    start_value = _sample_vector(ordered, start_s, maximum_gap_s)
    end_value = _sample_vector(ordered, end_s, maximum_gap_s)
    if start_value is None or end_value is None:
        return None
    points = [(start_s, start_value)]
    points.extend(sample for sample in ordered if start_s < sample[0] < end_s)
    points.append((end_s, end_value))
    integral = [0.0] * dimension
    for before, after in zip(points[:-1], points[1:]):
        dt_s = after[0] - before[0]
        if dt_s <= 0.0 or dt_s > maximum_gap_s:
            return None
        for component in range(dimension):
            integral[component] += 0.5 * (
                float(before[1][component]) + float(after[1][component])
            ) * dt_s
    duration = end_s - start_s
    return tuple(value / duration for value in integral)


def interval_mean_absolute_yaw_rate(
    samples: Iterable[Sequence[float]],
    start_s: float,
    end_s: float,
    maximum_gap_s: float = 0.12,
):
    """Average absolute FCU yaw rate over one optical-flow integration span."""
    start_s = float(start_s)
    end_s = float(end_s)
    maximum_gap_s = float(maximum_gap_s)
    if (
        not math.isfinite(start_s)
        or not math.isfinite(end_s)
        or end_s <= start_s
        or not math.isfinite(maximum_gap_s)
        or maximum_gap_s <= 0.0
    ):
        return None
    ordered = sorted(
        (float(sample[0]), float(sample[1]))
        for sample in samples
        if len(sample) >= 2
        and math.isfinite(float(sample[0]))
        and math.isfinite(float(sample[1]))
    )
    if not ordered:
        return None
    start_value = _sample_scalar(ordered, start_s, maximum_gap_s)
    end_value = _sample_scalar(ordered, end_s, maximum_gap_s)
    if start_value is None or end_value is None:
        return None
    points = [(start_s, start_value)]
    points.extend(sample for sample in ordered if start_s < sample[0] < end_s)
    points.append((end_s, end_value))
    integral = 0.0
    for before, after in zip(points[:-1], points[1:]):
        dt_s = after[0] - before[0]
        if dt_s <= 0.0 or dt_s > maximum_gap_s:
            return None
        integral += 0.5 * (abs(before[1]) + abs(after[1])) * dt_s
    return integral / (end_s - start_s)


class OpticalFlowRotationGate:
    """Down-weight turns and require stable translation before flow recovery."""

    PHASE_CODES = {
        "ACTIVE": 0.0,
        "TURNING": 1.0,
        "RECOVERY_DWELL": 2.0,
        "RECOVERING": 3.0,
        "YAW_RATE_UNAVAILABLE": 4.0,
    }

    def __init__(self, config=None):
        self.config = config or FlowRotationGateConfig()
        if not (
            0.0 <= self.config.lower_yaw_rate_radps
            < self.config.upper_yaw_rate_radps
        ):
            raise ValueError("yaw-rate thresholds must satisfy 0 <= lower < upper")
        if self.config.recovery_dwell_s < 0.0:
            raise ValueError("recovery dwell must be non-negative")
        if self.config.recovery_ramp_s <= 0.0:
            raise ValueError("recovery ramp must be positive")
        if self.config.minimum_translation_m < 0.0:
            raise ValueError("minimum recovery translation must be non-negative")
        if self.config.minimum_translation_speed_mps < 0.0:
            raise ValueError("minimum recovery translation speed must be non-negative")
        self.phase = "ACTIVE"
        self.stable_since_s = None
        self.ramp_since_s = None
        self.last_stamp_s = None

    def reset(self):
        self.phase = "ACTIVE"
        self.stable_since_s = None
        self.ramp_since_s = None
        self.last_stamp_s = None

    def _turning_weight(self, yaw_rate_abs_radps):
        span = (
            self.config.upper_yaw_rate_radps
            - self.config.lower_yaw_rate_radps
        )
        return max(
            0.0,
            min(
                1.0,
                (self.config.upper_yaw_rate_radps - yaw_rate_abs_radps) / span,
            ),
        )

    def _result(
        self, weight, phase, yaw_rate_abs_radps, translation_ready, reason,
    ):
        weight = max(0.0, min(1.0, float(weight)))
        return FlowRotationGateResult(
            weight=weight,
            hard_disabled=weight <= 0.0,
            phase=phase,
            yaw_rate_abs_radps=(
                -1.0 if yaw_rate_abs_radps is None else yaw_rate_abs_radps
            ),
            translation_ready=bool(translation_ready),
            reason=reason,
        )

    def update(
        self,
        stamp_s,
        yaw_rate_radps,
        translation_norm_m,
        observation_healthy,
        translation_interval_s=None,
        rotation_compensated=False,
    ):
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s):
            raise ValueError("flow rotation gate timestamp must be finite")
        if self.last_stamp_s is not None and stamp_s < self.last_stamp_s:
            self.reset()
        self.last_stamp_s = stamp_s
        translation_available = (
            bool(observation_healthy)
            and translation_norm_m is not None
            and math.isfinite(float(translation_norm_m))
        )
        if self.config.minimum_translation_speed_mps > 0.0:
            interval_valid = (
                translation_interval_s is not None
                and math.isfinite(float(translation_interval_s))
                and float(translation_interval_s) > 0.0
            )
            translation_ready = bool(
                translation_available
                and interval_valid
                and float(translation_norm_m) / float(translation_interval_s)
                >= self.config.minimum_translation_speed_mps
            )
        else:
            translation_ready = bool(
                translation_available
                and float(translation_norm_m) >= self.config.minimum_translation_m
            )
        if yaw_rate_radps is None or not math.isfinite(float(yaw_rate_radps)):
            self.phase = "YAW_RATE_UNAVAILABLE"
            self.stable_since_s = None
            self.ramp_since_s = None
            return self._result(
                0.0, self.phase, None, translation_ready,
                "fcu_yaw_rate_unavailable",
            )

        # ArduPilot does not reject a turn merely because yaw rate is high. It
        # compensates the optical-flow LOS rate with the FCU gyro first and
        # then lets the innovation gate decide.  Preserve the old hysteretic
        # behavior for callers that have not proven gyro compensation, while
        # allowing the APM-compatible path to remain usable during a turn.
        if (
            self.config.allow_compensated_rotation
            and rotation_compensated
            and observation_healthy
        ):
            self.phase = "ACTIVE"
            self.stable_since_s = None
            self.ramp_since_s = None
            return self._result(
                1.0,
                self.phase,
                abs(float(yaw_rate_radps)),
                translation_ready,
                "apm_rotation_compensated",
            )

        yaw_rate_abs = abs(float(yaw_rate_radps))
        if yaw_rate_abs > self.config.lower_yaw_rate_radps:
            self.phase = "TURNING"
            self.stable_since_s = None
            self.ramp_since_s = None
            weight = self._turning_weight(yaw_rate_abs)
            return self._result(
                weight,
                self.phase,
                yaw_rate_abs,
                translation_ready,
                (
                    "high_fcu_yaw_rate"
                    if weight <= 0.0 else "fcu_yaw_rate_downweight"
                ),
            )

        if self.phase in {"TURNING", "YAW_RATE_UNAVAILABLE"}:
            self.phase = "RECOVERY_DWELL"
            self.stable_since_s = stamp_s if translation_ready else None
            self.ramp_since_s = None

        if self.phase == "RECOVERY_DWELL":
            if not translation_ready:
                self.stable_since_s = None
                return self._result(
                    0.0,
                    self.phase,
                    yaw_rate_abs,
                    translation_ready,
                    "waiting_for_consistent_translation",
                )
            if self.stable_since_s is None:
                self.stable_since_s = stamp_s
            elapsed_s = stamp_s - self.stable_since_s
            if elapsed_s < self.config.recovery_dwell_s:
                return self._result(
                    0.0,
                    self.phase,
                    yaw_rate_abs,
                    translation_ready,
                    "flow_rotation_recovery_dwell",
                )
            self.phase = "RECOVERING"
            self.ramp_since_s = self.stable_since_s + self.config.recovery_dwell_s

        if self.phase == "RECOVERING":
            if not translation_ready:
                self.phase = "RECOVERY_DWELL"
                self.stable_since_s = None
                self.ramp_since_s = None
                return self._result(
                    0.0,
                    self.phase,
                    yaw_rate_abs,
                    translation_ready,
                    "waiting_for_consistent_translation",
                )
            progress = max(
                0.0,
                min(
                    1.0,
                    (stamp_s - self.ramp_since_s) / self.config.recovery_ramp_s,
                ),
            )
            if progress >= 1.0:
                self.phase = "ACTIVE"
                self.stable_since_s = None
                self.ramp_since_s = None
                return self._result(
                    1.0,
                    self.phase,
                    yaw_rate_abs,
                    translation_ready,
                    "flow_rotation_gate_active",
                )
            return self._result(
                progress,
                self.phase,
                yaw_rate_abs,
                translation_ready,
                "flow_rotation_recovery_ramp",
            )

        return self._result(
            1.0,
            "ACTIVE",
            yaw_rate_abs,
            translation_ready,
            "flow_rotation_gate_active",
        )
