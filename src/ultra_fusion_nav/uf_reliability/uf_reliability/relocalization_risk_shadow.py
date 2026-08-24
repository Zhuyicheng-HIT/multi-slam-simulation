"""Explainable, non-authoritative relocalization risk evaluation.

This module never publishes or mutates the production relocalization request.
All scalar inputs are causal normalized risk indicators in ``[0, 1]``. Truth
labels are deliberately absent from the online sample contract.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, Optional, Tuple


class RiskLevel(IntEnum):
    NORMAL = 0
    WATCH = 1
    DEGRADED = 2
    RELOCALIZE = 3
    FAILSAFE = 4


@dataclass(frozen=True)
class RelocalizationRiskConfig:
    watch_threshold: float = 0.25
    degraded_threshold: float = 0.48
    relocalize_threshold: float = 0.72
    watch_dwell_s: float = 0.30
    degraded_dwell_s: float = 1.00
    relocalize_dwell_s: float = 1.50
    recovery_dwell_s: float = 2.00
    request_cooldown_s: float = 15.0
    estimator_support_minimum: float = 0.15
    drift_filter_tau_s: float = 3.0
    directional_watch_threshold: float = 0.65
    directional_max_contribution: float = 0.15
    repeated_failure_limit: int = 2

    def __post_init__(self):
        thresholds = (
            self.watch_threshold,
            self.degraded_threshold,
            self.relocalize_threshold,
        )
        if not 0.0 < thresholds[0] < thresholds[1] < thresholds[2] <= 1.0:
            raise ValueError("risk thresholds must be strictly increasing")
        if min(
            self.watch_dwell_s,
            self.degraded_dwell_s,
            self.relocalize_dwell_s,
            self.recovery_dwell_s,
            self.request_cooldown_s,
            self.drift_filter_tau_s,
        ) < 0.0:
            raise ValueError("risk timing parameters must be non-negative")
        if self.drift_filter_tau_s <= 0.0:
            raise ValueError("drift filter time constant must be positive")
        if self.repeated_failure_limit < 1:
            raise ValueError("repeated failure limit must be positive")


@dataclass(frozen=True)
class RelocalizationRiskSample:
    stamp_s: float
    scheduler_health: str = "NORMAL"
    source_degradation: Dict[str, float] = field(default_factory=dict)
    factor_enabled: Dict[str, bool] = field(default_factory=dict)
    capability_support: Dict[str, float] = field(default_factory=dict)
    estimator_support: float = 1.0
    position_innovation_risk: float = 0.0
    yaw_innovation_risk: float = 0.0
    residual_nis_risk: float = 0.0
    covariance_growth_risk: float = 0.0
    pose_jump_risk: float = 0.0
    velocity_consistency_risk: float = 0.0
    directional_weakness_shadow: float = 0.0
    directional_shadow_valid: bool = False
    request_sources: Tuple[str, ...] = ()
    relocalization_ready: bool = True
    relocalization_result: str = "NONE"
    epoch_applied: bool = False
    obstacle_state: str = "CLEAR"


@dataclass(frozen=True)
class RelocalizationRiskDecision:
    level: RiskLevel
    target_level: RiskLevel
    score: float
    drift_risk: float
    would_request: bool
    production_eligible: bool
    request_suppressed: bool
    duplicate_source_count: int
    reasons: Tuple[str, ...]


def _clamp(value: float) -> float:
    value = float(value)
    return max(0.0, min(1.0, value)) if math.isfinite(value) else 1.0


def _second_largest(values) -> float:
    ordered = sorted((_clamp(value) for value in values), reverse=True)
    if not ordered:
        return 0.0
    return ordered[1] if len(ordered) > 1 else 0.0


class RelocalizationRiskCore:
    """Stateful LEVEL 0-4 shadow evaluator with dwell and hysteresis."""

    def __init__(self, config: Optional[RelocalizationRiskConfig] = None):
        self.config = config or RelocalizationRiskConfig()
        self.level = RiskLevel.NORMAL
        self.level_since_s = 0.0
        self.candidate_level = RiskLevel.NORMAL
        self.candidate_since_s: Optional[float] = None
        self.last_stamp_s: Optional[float] = None
        self.drift_risk = 0.0
        self.request_latched = False
        self.last_success_s: Optional[float] = None
        self.failure_count = 0
        self.failsafe_latched = False
        self.last_result = "NONE"
        self.success_epoch_consumed = False
        self.duplicate_episode_active = False
        self.duplicate_request_episodes = 0
        self.shadow_requests = 0

    def _critical_support_missing(self, sample) -> Tuple[str, ...]:
        missing = []
        for name in ("propagation", "horizontal_motion", "yaw_tracking"):
            if _clamp(sample.capability_support.get(name, 0.0)) < (
                self.config.estimator_support_minimum
            ):
                missing.append(name)
        return tuple(missing)

    def _health_risk(self, sample) -> float:
        return {
            "NORMAL": 0.0,
            "RECOVERED": 0.18,
            "DEGRADED": 0.38,
            "RISK": 0.64,
            "RELOCALIZING": 0.76,
            "FAILSAFE": 0.80,
        }.get(str(sample.scheduler_health).upper(), 0.85)

    def _update_drift(self, sample, dt_s) -> Tuple[float, int]:
        indicators = (
            _clamp(sample.position_innovation_risk),
            _clamp(sample.yaw_innovation_risk),
            _clamp(sample.residual_nis_risk),
            _clamp(sample.covariance_growth_risk),
            _clamp(sample.velocity_consistency_risk),
        )
        active_count = sum(value >= 0.45 for value in indicators)
        ordered = sorted(indicators, reverse=True)
        persistent_input = 0.65 * ordered[0] + 0.35 * ordered[1]
        alpha = 1.0 - math.exp(
            -max(0.0, dt_s) / self.config.drift_filter_tau_s
        )
        self.drift_risk += alpha * (persistent_input - self.drift_risk)
        return self.drift_risk, active_count

    def _candidate_dwell(self, target) -> float:
        return {
            RiskLevel.WATCH: self.config.watch_dwell_s,
            RiskLevel.DEGRADED: self.config.degraded_dwell_s,
            RiskLevel.RELOCALIZE: self.config.relocalize_dwell_s,
            RiskLevel.FAILSAFE: 0.0,
        }.get(target, self.config.recovery_dwell_s)

    def _transition(self, target, now_s):
        if target == self.level:
            self.candidate_level = target
            self.candidate_since_s = None
            return
        if target != self.candidate_level:
            self.candidate_level = target
            self.candidate_since_s = now_s
        dwell = self._candidate_dwell(target)
        if target < self.level:
            dwell = self.config.recovery_dwell_s
        if self.candidate_since_s is None:
            self.candidate_since_s = now_s
        if now_s - self.candidate_since_s >= dwell:
            self.level = target
            self.level_since_s = now_s
            self.candidate_since_s = None

    def update(self, sample: RelocalizationRiskSample):
        now_s = float(sample.stamp_s)
        reasons = []
        if not math.isfinite(now_s):
            self.level = RiskLevel.FAILSAFE
            return RelocalizationRiskDecision(
                self.level, self.level, 1.0, self.drift_risk, False,
                False, True, 0, ("clock_nonfinite",),
            )
        if self.last_stamp_s is not None and now_s < self.last_stamp_s:
            self.level = RiskLevel.FAILSAFE
            self.level_since_s = now_s
            self.last_stamp_s = now_s
            return RelocalizationRiskDecision(
                self.level, self.level, 1.0, self.drift_risk, False,
                False, True, 0, ("clock_regressed",),
            )
        dt_s = 0.0 if self.last_stamp_s is None else now_s - self.last_stamp_s
        self.last_stamp_s = now_s

        result = str(sample.relocalization_result).upper()
        result_edge = result != self.last_result
        if result_edge:
            self.success_epoch_consumed = False
        if result_edge and result == "FAILED":
            self.failure_count += 1
            self.failsafe_latched = True
            reasons.append("relocalization_failed")
        if (
            result == "SUCCESS"
            and sample.epoch_applied
            and not self.success_epoch_consumed
        ):
            self.failure_count = 0
            self.failsafe_latched = False
            self.request_latched = False
            self.last_success_s = now_s
            self.success_epoch_consumed = True
            reasons.append("matching_epoch_recovery")
        self.last_result = result

        unique_sources = tuple(sorted(set(sample.request_sources)))
        duplicate_count = max(0, len(unique_sources) - 1)
        if duplicate_count and not self.duplicate_episode_active:
            self.duplicate_request_episodes += 1
            self.duplicate_episode_active = True
        if not unique_sources:
            self.duplicate_episode_active = False

        missing = self._critical_support_missing(sample)
        hard_unavailable = (
            bool(missing)
            or not math.isfinite(float(sample.estimator_support))
            or float(sample.estimator_support)
            < self.config.estimator_support_minimum
        )
        if missing:
            reasons.append("critical_support_missing:" + "+".join(missing))

        source_consensus = _second_largest(sample.source_degradation.values())
        drift_risk, drift_indicator_count = self._update_drift(sample, dt_s)
        integrity_risk = max(
            _clamp(sample.pose_jump_risk),
            _clamp(sample.residual_nis_risk),
            _clamp(sample.covariance_growth_risk),
        )
        health_risk = self._health_risk(sample)
        score = max(
            health_risk,
            0.45 * source_consensus
            + 0.35 * drift_risk
            + 0.20 * integrity_risk,
            0.60 * integrity_risk + 0.40 * drift_risk,
        )

        directional_only = False
        if sample.directional_shadow_valid:
            directional = _clamp(sample.directional_weakness_shadow)
            score = min(1.0, score + min(
                self.config.directional_max_contribution,
                self.config.directional_max_contribution * directional,
            ))
            directional_only = (
                directional >= self.config.directional_watch_threshold
                and source_consensus < self.config.watch_threshold
                and drift_indicator_count < 2
                and integrity_risk < self.config.watch_threshold
            )
            if directional >= self.config.directional_watch_threshold:
                reasons.append("directional_weakness_shadow")

        multi_signal = (
            source_consensus >= self.config.watch_threshold
            or drift_indicator_count >= 2
            or integrity_risk >= self.config.degraded_threshold
            or str(sample.scheduler_health).upper() in {"RISK", "FAILSAFE"}
        )
        production_eligible = multi_signal and not directional_only

        obstacle_unsafe = str(sample.obstacle_state).upper() in {
            "BRAKE", "HOVER", "HOVER_REQUIRED", "STALE", "UNHEALTHY",
        }
        external_request = bool(unique_sources)
        if (
            hard_unavailable
            or self.failsafe_latched
            or self.failure_count >= self.config.repeated_failure_limit
        ):
            target = RiskLevel.FAILSAFE
        elif result_edge and result == "FAILED":
            target = RiskLevel.FAILSAFE
        elif external_request:
            target = (
                RiskLevel.FAILSAFE
                if obstacle_unsafe else RiskLevel.RELOCALIZE
            )
        elif score >= self.config.relocalize_threshold and production_eligible:
            target = (
                RiskLevel.FAILSAFE
                if obstacle_unsafe else RiskLevel.RELOCALIZE
            )
        elif score >= self.config.degraded_threshold:
            target = RiskLevel.DEGRADED
        elif score >= self.config.watch_threshold or directional_only:
            target = RiskLevel.WATCH
        else:
            target = RiskLevel.NORMAL

        cooldown = (
            self.last_success_s is not None
            and now_s >= self.last_success_s
            and now_s - self.last_success_s < self.config.request_cooldown_s
        )
        request_suppressed = False
        if target == RiskLevel.RELOCALIZE:
            if not sample.relocalization_ready:
                target = RiskLevel.DEGRADED
                request_suppressed = True
                reasons.append("relocalization_not_ready")
            elif cooldown and not external_request:
                target = RiskLevel.DEGRADED
                request_suppressed = True
                reasons.append("post_recovery_cooldown")
            elif directional_only:
                target = RiskLevel.WATCH
                request_suppressed = True
                reasons.append("directional_shadow_cannot_trigger")

        previous_level = self.level
        self._transition(target, now_s)
        entered_relocalize = (
            previous_level != RiskLevel.RELOCALIZE
            and self.level == RiskLevel.RELOCALIZE
        )
        would_request = (
            entered_relocalize
            and not self.request_latched
            and not external_request
        )
        if external_request:
            self.request_latched = True
        if would_request:
            self.request_latched = True
            self.shadow_requests += 1
            reasons.append("shadow_request_edge")
        if (
            self.level <= RiskLevel.WATCH
            and not external_request
            and not cooldown
        ):
            self.request_latched = False

        if drift_risk >= self.config.watch_threshold:
            reasons.append("persistent_drift_evidence")
        if duplicate_count:
            reasons.append("multiple_request_sources")
        if obstacle_unsafe and target >= RiskLevel.RELOCALIZE:
            reasons.append("obstacle_veto_requires_hover")
        if not reasons:
            reasons.append("healthy")

        return RelocalizationRiskDecision(
            level=self.level,
            target_level=target,
            score=float(score),
            drift_risk=float(drift_risk),
            would_request=would_request,
            production_eligible=production_eligible,
            request_suppressed=request_suppressed,
            duplicate_source_count=duplicate_count,
            reasons=tuple(reasons),
        )
