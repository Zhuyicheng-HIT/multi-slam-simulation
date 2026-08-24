"""Deterministic REL-TRIGGER-004 production-vs-shadow scenario matrix."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from dataclasses import dataclass, replace
from pathlib import Path

from .relocalization_risk_shadow import (
    RelocalizationRiskCore,
    RelocalizationRiskSample,
    RiskLevel,
)


SCENARIOS = (
    "normal_flight",
    "short_lidar_recovery",
    "sustained_lidar_geometry",
    "single_direction_weakness",
    "gnss_degraded_lidar_healthy",
    "visual_degradation",
    "slow_position_drift",
    "yaw_drift",
    "multi_source_degradation",
    "relocalization_success_recovery",
    "relocalization_failure",
    "obstacle_relocalization_conflict",
)


EXPECTED_RELOCALIZE = {
    "sustained_lidar_geometry",
    "slow_position_drift",
    "yaw_drift",
    "relocalization_success_recovery",
    "relocalization_failure",
}


EXPECTED_FAILSAFE = {
    "multi_source_degradation",
    "obstacle_relocalization_conflict",
}


TRUTH_DEGRADATION_START = {
    "short_lidar_recovery": 10.0,
    "sustained_lidar_geometry": 10.0,
    "single_direction_weakness": 10.0,
    "gnss_degraded_lidar_healthy": 10.0,
    "visual_degradation": 10.0,
    "slow_position_drift": 18.0,
    "yaw_drift": 17.0,
    "multi_source_degradation": 10.0,
    "relocalization_success_recovery": 10.0,
    "relocalization_failure": 10.0,
    "obstacle_relocalization_conflict": 10.0,
}


@dataclass
class ProductionTriggerEmulator:
    """Exact timing semantics of the two current request producers."""

    scheduler_candidate_since: float | None = None
    scheduler_observations: int = 0
    scheduler_active: bool = False
    scheduler_last_request: float | None = None
    safety_state: str = "TRACKING"
    safety_since: float = 0.0
    safety_active: bool = False
    safety_last_release: float | None = None
    duplicate_episodes: int = 0
    duplicate_active: bool = False

    def update(self, sample):
        now_s = sample.stamp_s
        lidar = float(sample.source_degradation.get("lidar", 1.0))
        lidar_enabled = bool(sample.factor_enabled.get("lidar", False))
        horizontal_support = float(
            sample.capability_support.get("horizontal_position", 0.0)
        )
        lidar_candidate = (lidar >= 0.85 or not lidar_enabled) and (
            horizontal_support < 0.15
        )
        if lidar_candidate:
            self.scheduler_observations += 1
            if self.scheduler_candidate_since is None:
                self.scheduler_candidate_since = now_s
        else:
            self.scheduler_candidate_since = None
            self.scheduler_observations = 0
        cooldown_clear = (
            self.scheduler_last_request is None
            or now_s - self.scheduler_last_request >= 15.0
        )
        if (
            now_s >= 10.0
            and lidar_candidate
            and self.scheduler_observations >= 3
            and now_s - self.scheduler_candidate_since >= 1.0
            and cooldown_clear
            and not self.scheduler_active
        ):
            self.scheduler_active = True
            self.scheduler_last_request = now_s

        support = sample.capability_support
        obvious_loss = (
            float(sample.estimator_support) < 0.15
            or any(
                float(support.get(name, 0.0)) < 0.15
                for name in (
                    "propagation", "horizontal_motion", "yaw_tracking"
                )
            )
        )
        if self.safety_state == "TRACKING" and obvious_loss:
            self.safety_state = "LOSS_PENDING"
            self.safety_since = now_s
        elif self.safety_state == "LOSS_PENDING":
            if not obvious_loss:
                self.safety_state = "TRACKING"
            elif now_s - self.safety_since >= 0.30:
                self.safety_state = "HOLDING"
                self.safety_since = now_s
        elif self.safety_state == "HOLDING":
            if now_s - self.safety_since >= 1.0:
                if obvious_loss:
                    cooldown = (
                        self.safety_last_release is not None
                        and now_s - self.safety_last_release < 5.0
                    )
                    if sample.relocalization_ready and not cooldown:
                        self.safety_active = True
                    self.safety_state = "RELOCALIZING_HOLD"
                else:
                    self.safety_state = "RECOVERY_PENDING"
                self.safety_since = now_s
        elif self.safety_state == "RELOCALIZING_HOLD" and not obvious_loss:
            self.safety_state = "RECOVERY_PENDING"
            self.safety_since = now_s
        elif self.safety_state == "RECOVERY_PENDING":
            if obvious_loss:
                self.safety_state = "RELOCALIZING_HOLD"
                self.safety_since = now_s
            elif now_s - self.safety_since >= 0.75:
                self.safety_state = "TRACKING"
                if self.safety_active:
                    self.safety_last_release = now_s
                self.safety_active = False

        result = str(sample.relocalization_result).upper()
        if result == "FAILED" or (
            result == "SUCCESS" and sample.epoch_applied
        ):
            if self.scheduler_active or self.safety_active:
                self.safety_last_release = now_s
            self.scheduler_active = False
            self.safety_active = False

        sources = []
        if self.scheduler_active:
            sources.append("reliability_scheduler")
        if self.safety_active:
            sources.append("localization_safety")
        duplicate = len(sources) > 1
        if duplicate and not self.duplicate_active:
            self.duplicate_episodes += 1
        self.duplicate_active = duplicate
        return tuple(sources)


def _base_sample(stamp_s):
    return RelocalizationRiskSample(
        stamp_s=stamp_s,
        scheduler_health="NORMAL",
        source_degradation={
            "lidar": 0.08,
            "gnss": 0.10,
            "imu": 0.08,
            "optical_flow": 0.12,
            "vision": 0.10,
        },
        factor_enabled={
            "lidar": True,
            "gnss": True,
            "imu": True,
            "optical_flow": True,
            "vision": True,
        },
        capability_support={
            "propagation": 0.92,
            "horizontal_position": 0.90,
            "horizontal_motion": 0.90,
            "vertical_position": 0.86,
            "yaw_tracking": 0.90,
        },
        estimator_support=0.90,
    )


def scenario_sample(name, stamp_s, seed=0):
    sample = _base_sample(stamp_s)
    degradation = dict(sample.source_degradation)
    enabled = dict(sample.factor_enabled)
    support = dict(sample.capability_support)
    kwargs = {}
    event = 10.0 <= stamp_s
    if name == "short_lidar_recovery" and 10.0 <= stamp_s < 10.7:
        degradation["lidar"] = 0.93
        support["horizontal_position"] = 0.08
        kwargs.update(scheduler_health="RISK", residual_nis_risk=0.65)
    elif name == "sustained_lidar_geometry" and 10.0 <= stamp_s < 23.0:
        degradation["lidar"] = 0.94
        enabled["lidar"] = False
        support["horizontal_position"] = 0.08
        kwargs.update(
            scheduler_health="RISK",
            residual_nis_risk=1.00,
            covariance_growth_risk=0.90,
            position_innovation_risk=0.90,
        )
    elif name == "single_direction_weakness" and event:
        kwargs.update(
            directional_shadow_valid=True,
            directional_weakness_shadow=0.94,
        )
    elif name == "gnss_degraded_lidar_healthy" and 10.0 <= stamp_s < 22.0:
        degradation["gnss"] = 0.96
        enabled["gnss"] = False
        kwargs.update(scheduler_health="DEGRADED")
    elif name == "visual_degradation" and 10.0 <= stamp_s < 22.0:
        degradation["vision"] = 0.96
        enabled["vision"] = False
        kwargs.update(scheduler_health="DEGRADED")
    elif name == "slow_position_drift" and event:
        progress = min(1.0, max(0.0, (stamp_s - 10.0) / 12.0))
        degradation["lidar"] = 0.20 + 0.38 * progress
        degradation["gnss"] = 0.18 + 0.42 * progress
        kwargs.update(
            scheduler_health="DEGRADED" if progress > 0.55 else "NORMAL",
            position_innovation_risk=0.25 + 0.70 * progress,
            residual_nis_risk=0.30 + 0.55 * progress,
            covariance_growth_risk=0.20 + 0.72 * progress,
            velocity_consistency_risk=0.15 + 0.60 * progress,
        )
    elif name == "yaw_drift" and event:
        progress = min(1.0, max(0.0, (stamp_s - 10.0) / 10.0))
        degradation["lidar"] = 0.18 + 0.42 * progress
        degradation["imu"] = 0.16 + 0.44 * progress
        kwargs.update(
            scheduler_health="DEGRADED" if progress > 0.60 else "NORMAL",
            yaw_innovation_risk=0.25 + 0.72 * progress,
            residual_nis_risk=0.25 + 0.66 * progress,
            covariance_growth_risk=0.20 + 0.65 * progress,
        )
    elif name == "multi_source_degradation" and 10.0 <= stamp_s < 22.0:
        for source in degradation:
            degradation[source] = 0.92
        for source in enabled:
            enabled[source] = False
        for capability in support:
            support[capability] = 0.05
        kwargs.update(
            scheduler_health="FAILSAFE",
            estimator_support=0.05,
            residual_nis_risk=0.95,
            covariance_growth_risk=0.90,
        )
    elif name in {
        "relocalization_success_recovery",
        "relocalization_failure",
        "obstacle_relocalization_conflict",
    } and 10.0 <= stamp_s < 20.0:
        degradation["lidar"] = 0.95
        enabled["lidar"] = False
        support["horizontal_position"] = 0.06
        kwargs.update(
            scheduler_health="RISK",
            residual_nis_risk=1.00,
            covariance_growth_risk=0.90,
            position_innovation_risk=0.90,
        )
        if name == "obstacle_relocalization_conflict" and stamp_s >= 11.0:
            kwargs["obstacle_state"] = "BRAKE"
        if name == "relocalization_success_recovery" and stamp_s >= 14.0:
            kwargs["relocalization_result"] = "SUCCESS"
            kwargs["epoch_applied"] = stamp_s >= 14.2
            if stamp_s >= 14.2:
                degradation = dict(_base_sample(stamp_s).source_degradation)
                enabled = dict(_base_sample(stamp_s).factor_enabled)
                support = dict(_base_sample(stamp_s).capability_support)
                kwargs.update(
                    scheduler_health="RECOVERED",
                    residual_nis_risk=0.05,
                    covariance_growth_risk=0.05,
                    position_innovation_risk=0.05,
                )
        elif name == "relocalization_failure" and stamp_s >= 14.0:
            kwargs["relocalization_result"] = "FAILED"

    # Deterministic sub-threshold seed variation prevents a single exact trace
    # from hiding boundary sensitivity without changing scenario semantics.
    jitter = 0.002 * math.sin(0.37 * seed + 0.13 * stamp_s)
    degradation = {
        key: max(0.0, min(1.0, value + jitter))
        for key, value in degradation.items()
    }
    return replace(
        sample,
        source_degradation=degradation,
        factor_enabled=enabled,
        capability_support=support,
        **kwargs,
    )


def _percentile(values, percentile):
    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * float(percentile) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def run_scenario(name, seed=0, duration_s=30.0, rate_hz=10.0):
    shadow = RelocalizationRiskCore()
    production_observer = RelocalizationRiskCore()
    production = ProductionTriggerEmulator()
    first_level = {level.name: None for level in RiskLevel}
    first_production = None
    first_shadow_request = None
    latencies_us = []
    production_sources = ()
    previous_sources = ()
    step_count = int(round(duration_s * rate_hz)) + 1
    for step in range(step_count):
        stamp_s = step / rate_hz
        sample = scenario_sample(name, stamp_s, seed)
        production_sources = production.update(sample)
        started = time.perf_counter_ns()
        decision = shadow.update(sample)
        latencies_us.append((time.perf_counter_ns() - started) / 1000.0)
        production_observer.update(replace(
            sample, request_sources=production_sources
        ))
        if first_level[decision.level.name] is None:
            first_level[decision.level.name] = stamp_s
        if (
            production_sources
            and not previous_sources
            and first_production is None
        ):
            first_production = stamp_s
        if decision.would_request and first_shadow_request is None:
            first_shadow_request = stamp_s
        previous_sources = production_sources

    expected = name in EXPECTED_RELOCALIZE
    expected_failsafe = name in EXPECTED_FAILSAFE
    return {
        "scenario": name,
        "seed": seed,
        "truth_degradation_start_s": TRUTH_DEGRADATION_START.get(name),
        "expected_relocalize": expected,
        "expected_failsafe": expected_failsafe,
        "watch_s": first_level["WATCH"],
        "degraded_s": first_level["DEGRADED"],
        "relocalize_s": first_level["RELOCALIZE"],
        "failsafe_s": first_level["FAILSAFE"],
        "production_request_s": first_production,
        "shadow_request_s": first_shadow_request,
        "production_false_trigger": bool(
            first_production is not None
            and not expected
            and not expected_failsafe
        ),
        "shadow_false_trigger": bool(
            first_shadow_request is not None and not expected
        ),
        "production_missed_trigger": bool(
            first_production is None and expected
        ),
        "shadow_missed_trigger": bool(
            first_shadow_request is None and expected
        ),
        "production_duplicate_episodes": production.duplicate_episodes,
        "shadow_duplicate_episodes": (
            production_observer.duplicate_request_episodes
        ),
        "shadow_request_count": shadow.shadow_requests,
        "latency_p50_us": _percentile(latencies_us, 50),
        "latency_p95_us": _percentile(latencies_us, 95),
        "latency_p99_us": _percentile(latencies_us, 99),
    }


def run_matrix(seeds=range(5)):
    rows = [
        run_scenario(scenario, seed)
        for scenario in SCENARIOS
        for seed in seeds
    ]
    summary = {
        "runs": len(rows),
        "scenario_count": len(SCENARIOS),
        "seed_count": len(tuple(seeds)),
        "production_false_triggers": sum(
            row["production_false_trigger"] for row in rows
        ),
        "shadow_false_triggers": sum(
            row["shadow_false_trigger"] for row in rows
        ),
        "production_missed_triggers": sum(
            row["production_missed_trigger"] for row in rows
        ),
        "shadow_missed_triggers": sum(
            row["shadow_missed_trigger"] for row in rows
        ),
        "production_duplicate_episodes": sum(
            row["production_duplicate_episodes"] for row in rows
        ),
        "shadow_duplicate_episodes": sum(
            row["shadow_duplicate_episodes"] for row in rows
        ),
        "latency_p50_us": statistics.median(
            row["latency_p50_us"] for row in rows
        ),
        "latency_p95_us": _percentile(
            [row["latency_p95_us"] for row in rows], 95
        ),
        "latency_p99_us": _percentile(
            [row["latency_p99_us"] for row in rows], 99
        ),
    }
    return {"summary": summary, "runs": rows}


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-json",
        default="/tmp/rel_trigger_004_shadow_results.json",
    )
    parser.add_argument(
        "--output-csv",
        default="/tmp/rel_trigger_004_shadow_results.csv",
    )
    options = parser.parse_args(args=args)
    result = run_matrix()
    Path(options.output_json).write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with Path(options.output_csv).open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=result["runs"][0].keys())
        writer.writeheader()
        writer.writerows(result["runs"])
    print(json.dumps(result["summary"], sort_keys=True))


if __name__ == "__main__":
    main()
