from dataclasses import dataclass
from typing import Dict, Optional


MODALITIES = ("lidar", "gnss", "imu", "optical_flow", "vision")
CAPABILITY_SOURCES = {
    "propagation": ("imu",),
    "horizontal_position": ("lidar", "gnss", "vision"),
    "horizontal_velocity": ("lidar", "gnss", "optical_flow", "vision"),
    "horizontal_motion": ("lidar", "gnss", "optical_flow", "vision"),
    "vertical_position": ("lidar", "gnss", "vision"),
    "yaw_tracking": ("lidar", "imu", "vision"),
}
CAPABILITIES = tuple(CAPABILITY_SOURCES)


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


@dataclass(frozen=True)
class SchedulerConfig:
    active_modalities: tuple = MODALITIES
    required_modalities: tuple = ()
    minimum_usable_modalities: int = 1
    stale_after_s: float = 1.0
    degraded_threshold: float = 0.35
    risk_threshold: float = 0.60
    failsafe_threshold: float = 0.85
    factor_disable_threshold: float = 0.80
    factor_enable_threshold: float = 0.55
    minimum_weight: float = 0.05
    maximum_covariance_inflation: float = 20.0
    transition_dwell_s: float = 0.5
    recovery_dwell_s: float = 1.5
    recovered_hold_s: float = 1.0
    capability_observable_threshold: float = 0.15


@dataclass(frozen=True)
class ScheduleResult:
    health_state: str
    degradation_scores: Dict[str, float]
    reliability_weights: Dict[str, float]
    covariance_inflation: Dict[str, float]
    factor_enabled: Dict[str, bool]
    reasons: Dict[str, tuple]
    capability_support: Dict[str, float]
    capability_observable: Dict[str, bool]
    estimator_support: float
    relocalization_requested: bool


class ReliabilitySchedulerCore:
    """Stateful factor scheduler; inputs are normalized degradation scores."""

    def __init__(self, config: Optional[SchedulerConfig] = None):
        self.config = config or SchedulerConfig()
        self.active_modalities = tuple(
            name for name in self.config.active_modalities if name in MODALITIES
        )
        if not self.active_modalities:
            raise ValueError("at least one active modality is required")
        unknown_required = set(self.config.required_modalities).difference(
            self.active_modalities
        )
        if unknown_required:
            raise ValueError(
                "required modalities must also be active: "
                + ",".join(sorted(unknown_required))
            )
        if not 1 <= self.config.minimum_usable_modalities <= len(
            self.active_modalities
        ):
            raise ValueError(
                "minimum usable modalities must be within active modalities"
            )
        self.required_modalities = tuple(
            name
            for name in self.config.required_modalities
            if name in self.active_modalities
        )
        self.health_state = "FAILSAFE"
        self.state_since = None
        self.candidate_state = None
        self.candidate_since = None
        self.healthy_since = None
        self.recovered_since = None
        self.has_valid_state = False
        self.factor_enabled = {name: False for name in MODALITIES}

    def _target_state(
        self,
        operational_severity,
        valid_count,
        usable_count,
        required_usable,
        degraded_or_missing,
        relocalization_requested,
    ):
        if relocalization_requested:
            return "RELOCALIZING"
        if (
            valid_count == 0
            or not required_usable
            or usable_count < self.config.minimum_usable_modalities
        ):
            return "FAILSAFE"
        if operational_severity >= self.config.risk_threshold:
            return "RISK"
        if (
            self.config.minimum_usable_modalities > 1
            and
            usable_count == self.config.minimum_usable_modalities
            and usable_count < len(self.active_modalities)
        ):
            return "RISK"
        if (
            operational_severity >= self.config.degraded_threshold
            or degraded_or_missing
        ):
            return "DEGRADED"
        if self.health_state == "RECOVERED":
            return "RECOVERED"
        if self.health_state in {"DEGRADED", "RISK", "FAILSAFE", "RELOCALIZING"}:
            return "RECOVERED"
        return "NORMAL"

    def _transition(self, target, now):
        if target == self.health_state:
            if target == "RECOVERED" and self.recovered_since is not None:
                if now - self.recovered_since >= self.config.recovered_hold_s:
                    self.health_state = "NORMAL"
                    self.state_since = now
            return
        if target == "RECOVERED":
            if self.healthy_since is None:
                self.healthy_since = now
            if now - self.healthy_since < self.config.recovery_dwell_s:
                return
        else:
            self.healthy_since = None
        if self.candidate_state != target:
            self.candidate_state = target
            self.candidate_since = now
            if self.config.transition_dwell_s <= 0.0:
                self.health_state = target
                self.state_since = now
                self.candidate_state = None
                self.candidate_since = None
                self.recovered_since = now if target == "RECOVERED" else None
            return
        if self.candidate_since is None or now - self.candidate_since < self.config.transition_dwell_s:
            return
        self.health_state = target
        self.state_since = now
        self.candidate_state = None
        self.candidate_since = None
        self.recovered_since = now if target == "RECOVERED" else None

    def update(self, scores, now_s, relocalization_requested=False):
        now = float(now_s)
        degradation = {}
        weights = {}
        inflation = {}
        reasons = {}
        valid_count = 0
        valid_active_scores = {}
        for name in MODALITIES:
            if name not in self.active_modalities:
                self.factor_enabled[name] = False
                degradation[name] = 0.0
                weights[name] = 0.0
                inflation[name] = self.config.maximum_covariance_inflation
                reasons[name] = ("inactive_modality",)
                continue
            sample = scores.get(name)
            sample_age = float("inf")
            valid = False
            value = 1.0
            sample_reasons = []
            hard_gate_allowed = True
            if sample is not None:
                value = clamp(sample.get("degradation_score", 1.0))
                valid = bool(sample.get("valid", False))
                hard_gate_allowed = bool(sample.get("hard_gate_allowed", True))
                sample_age = max(0.0, now - float(sample.get("arrival_s", now)))
                sample_reasons = list(sample.get("reasons", ()))
                observation_count = max(0, int(sample.get("observation_count", 1)))
                minimum_observation_count = max(
                    1, int(sample.get("minimum_observation_count", 1))
                )
            else:
                observation_count = 0
                minimum_observation_count = 1
            stale = sample is None or sample_age > self.config.stale_after_s
            if stale or not valid:
                value = 1.0
                valid = False
                sample_reasons.append("score_stale_or_invalid")
            elif observation_count < minimum_observation_count:
                value = 1.0
                valid = False
                sample_reasons.append("insufficient_observations_eq15")
            degradation[name] = value
            weight = 1.0 - value if valid else 0.0
            weights[name] = clamp(weight)
            if name in self.active_modalities:
                if valid:
                    valid_count += 1
                    valid_active_scores[name] = value
            if self.factor_enabled[name]:
                if stale or not valid:
                    self.factor_enabled[name] = False
                elif value >= self.config.factor_disable_threshold:
                    if hard_gate_allowed:
                        self.factor_enabled[name] = False
                    else:
                        sample_reasons.append(
                            "hard_gate_blocked_by_evidence_policy"
                        )
            elif valid and value <= self.config.factor_enable_threshold:
                self.factor_enabled[name] = True
            inflation[name] = (
                min(self.config.maximum_covariance_inflation,
                    1.0 / max(self.config.minimum_weight, weights[name]))
                if self.factor_enabled[name]
                else self.config.maximum_covariance_inflation
            )
            reasons[name] = tuple(sample_reasons)
        usable_active_scores = {
            name: value
            for name, value in valid_active_scores.items()
            if self.factor_enabled[name]
            and value < self.config.failsafe_threshold
        }
        usable_count = len(usable_active_scores)
        required_usable = all(
            name in usable_active_scores for name in self.required_modalities
        )
        required_scores = [
            usable_active_scores[name]
            for name in self.required_modalities
            if name in usable_active_scores
        ]
        aiding_scores = [
            value
            for name, value in usable_active_scores.items()
            if name not in self.required_modalities
        ]
        operational_scores = required_scores[:]
        if aiding_scores:
            operational_scores.append(min(aiding_scores))
        operational_severity = max(operational_scores, default=0.0)
        degraded_or_missing = (
            usable_count < len(self.active_modalities)
            or any(
                value >= self.config.degraded_threshold
                for value in valid_active_scores.values()
            )
        )
        capability_support = {
            capability: max(
                (
                    weights[name]
                    for name in sources
                    if name in self.active_modalities and self.factor_enabled[name]
                ),
                default=0.0,
            )
            for capability, sources in CAPABILITY_SOURCES.items()
        }
        capability_observable = {
            name: support >= self.config.capability_observable_threshold
            for name, support in capability_support.items()
        }
        # A navigation solution remains useful when IMU propagation, horizontal
        # motion and relative yaw are each supported. A failed optional sensor
        # therefore cannot zero the complete estimator by itself.
        estimator_support = min(
            capability_support["propagation"],
            capability_support["horizontal_motion"],
            capability_support["yaw_tracking"],
        )
        first_healthy_observation = (
            not self.has_valid_state
            and valid_count > 0
            and valid_count == len(self.active_modalities)
            and usable_count == len(self.active_modalities)
            and operational_severity < self.config.degraded_threshold
            and not degraded_or_missing
            and not relocalization_requested
        )
        target = (
            "NORMAL" if first_healthy_observation else self._target_state(
                operational_severity,
                valid_count,
                usable_count,
                required_usable,
                degraded_or_missing,
                bool(relocalization_requested),
            )
        )
        if valid_count > 0:
            self.has_valid_state = True
        if target == "RECOVERED" and self.healthy_since is None:
            self.healthy_since = now
        self._transition(target, now)
        return ScheduleResult(
            health_state=self.health_state,
            degradation_scores=degradation,
            reliability_weights=weights,
            covariance_inflation=inflation,
            factor_enabled=dict(self.factor_enabled),
            reasons=reasons,
            capability_support=capability_support,
            capability_observable=capability_observable,
            estimator_support=estimator_support,
            relocalization_requested=bool(relocalization_requested),
        )
