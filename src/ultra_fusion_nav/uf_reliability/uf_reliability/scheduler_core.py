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
    modality_stale_after_s: tuple = ()
    degraded_threshold: float = 0.35
    risk_threshold: float = 0.60
    failsafe_threshold: float = 0.85
    factor_disable_threshold: float = 0.80
    factor_enable_threshold: float = 0.55
    minimum_weight: float = 0.05
    maximum_covariance_inflation: float = 20.0
    imu_soft_max_degradation: float = 0.80
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
        if self.config.stale_after_s <= 0.0:
            raise ValueError("score stale timeout must be positive")
        self.stale_after_s_by_modality = {
            name: float(self.config.stale_after_s) for name in MODALITIES
        }
        for entry in self.config.modality_stale_after_s:
            if len(entry) != 2 or entry[0] not in MODALITIES:
                raise ValueError("modality stale timeout entries must name a modality")
            timeout_s = float(entry[1])
            if timeout_s <= 0.0:
                raise ValueError("modality stale timeout must be positive")
            self.stale_after_s_by_modality[entry[0]] = timeout_s
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
        if not 0.0 <= self.config.imu_soft_max_degradation < self.config.failsafe_threshold:
            raise ValueError(
                "IMU soft maximum degradation must be below the failsafe threshold"
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
        self.last_update_s = None

    def _handle_clock_rewind(self, now):
        if self.last_update_s is not None and now < self.last_update_s:
            self.state_since = now
            self.candidate_state = None
            self.candidate_since = None
            self.healthy_since = None
            self.recovered_since = None
        self.last_update_s = now

    def _target_state(
        self,
        operational_severity,
        valid_count,
        usable_count,
        required_usable,
        degraded_or_missing,
        relocalization_requested,
        relocalization_failed,
    ):
        # A relocalization search is a recovery service, not an estimator
        # measurement. Its failure must not invalidate capabilities that are
        # still supported by live IMU/GNSS/flow/LiDAR factors. True pose loss
        # remains covered below by valid_count and usable_count. Required
        # modalities describe estimator capability, not an output kill switch:
        # one live independent source must keep the state stream available for
        # the safety controller to hold or relocalize.
        if relocalization_requested:
            return "RELOCALIZING"
        if (
            valid_count == 0
            or usable_count < self.config.minimum_usable_modalities
        ):
            return "FAILSAFE"
        if not required_usable:
            return "RISK"
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

    def update(
        self, scores, now_s, relocalization_requested=False,
        relocalization_failed=False,
    ):
        now = float(now_s)
        self._handle_clock_rewind(now)
        degradation = {}
        weights = {}
        inflation = {}
        reasons = {}
        valid_count = 0
        valid_active_scores = {}
        operational_scores_by_modality = {}
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
                arrival_s = float(sample.get("arrival_s", now))
                sample_age = (
                    float("inf") if arrival_s > now else now - arrival_s
                )
                sample_reasons = list(sample.get("reasons", ()))
                observation_count = max(0, int(sample.get("observation_count", 1)))
                minimum_observation_count = max(
                    1, int(sample.get("minimum_observation_count", 1))
                )
            else:
                observation_count = 0
                minimum_observation_count = 1
            stale = (
                sample is None
                or sample_age > self.stale_after_s_by_modality[name]
            )
            if stale or not valid:
                value = 1.0
                valid = False
                sample_reasons.append("score_stale_or_invalid")
            elif observation_count < minimum_observation_count:
                value = 1.0
                valid = False
                sample_reasons.append("insufficient_observations_eq15")
            degradation[name] = value
            # IMU is the propagation backbone. During a turn its score can
            # rise because excitation/residual terms are temporarily poor,
            # but that is a soft quality loss, not a reason to remove the
            # only state-propagation factor. Keep a bounded floor for this
            # case and reserve binary disabling for stale/invalid evidence or
            # an explicit hard gate.
            operational_value = value
            imu_hard_failure = (
                name == "imu" and "saturation_eq21" in sample_reasons
            )
            imu_soft_degradation = (
                name == "imu" and valid and hard_gate_allowed
                and not imu_hard_failure
            )
            gnss_provisional_bootstrap = bool(
                name == "gnss"
                and valid
                and not hard_gate_allowed
                and value < self.config.failsafe_threshold
                and "provisional_gnss_direct_evidence_only" in sample_reasons
            )
            if imu_soft_degradation:
                operational_value = min(
                    value, self.config.imu_soft_max_degradation)
                if value >= self.config.factor_disable_threshold:
                    sample_reasons.append("imu_propagation_soft_degradation")
            elif imu_hard_failure:
                operational_value = 1.0
            operational_scores_by_modality[name] = operational_value
            weight = 1.0 - operational_value if valid else 0.0
            weights[name] = clamp(weight)
            if name in self.active_modalities:
                if valid:
                    valid_count += 1
                    valid_active_scores[name] = value
            if imu_hard_failure:
                self.factor_enabled[name] = False
            elif name == "imu" and valid and hard_gate_allowed:
                # Keep valid IMU propagation enabled through high dynamics;
                # covariance inflation above carries the reliability penalty.
                self.factor_enabled[name] = True
            elif gnss_provisional_bootstrap:
                # Eq. 23 needs a backend innovation, but that innovation cannot
                # exist until at least one factor is considered. Direct
                # fix/covariance evidence may therefore start GNSS at its
                # conservative partial weight; the backend prefit NIS remains
                # the authoritative per-observation gate.
                self.factor_enabled[name] = True
                sample_reasons.append("gnss_provisional_bootstrap")
            elif self.factor_enabled[name]:
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
            # The backend already multiplies factor information by
            # reliability_weight. Applying reciprocal covariance inflation
            # here as well would square the attenuation, (1 - D)^2, which is
            # not part of Eq. (15)-(16). Keep the two output fields for the
            # factor contract, but apply continuous reliability exactly once.
            inflation[name] = (
                1.0 if self.factor_enabled[name]
                else self.config.maximum_covariance_inflation
            )
            reasons[name] = tuple(sample_reasons)
        usable_active_scores = {
            name: operational_scores_by_modality[name]
            for name, value in valid_active_scores.items()
            if self.factor_enabled[name]
            and operational_scores_by_modality[name] < self.config.failsafe_threshold
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
                bool(relocalization_failed),
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
