from dataclasses import fields

import pytest

from uf_reliability.relocalization_risk_shadow import (
    RelocalizationRiskConfig,
    RelocalizationRiskCore,
    RelocalizationRiskSample,
    RiskLevel,
)
from uf_reliability.relocalization_trigger_shadow_matrix import (
    run_scenario,
    scenario_sample,
)


def test_truth_is_not_an_online_input():
    names = {item.name for item in fields(RelocalizationRiskSample)}
    assert "truth" not in names
    assert "ground_truth" not in names


def test_directional_shadow_alone_cannot_request():
    result = run_scenario("single_direction_weakness", seed=2)
    assert result["watch_s"] is not None
    assert result["shadow_request_s"] is None
    assert result["shadow_false_trigger"] is False


def test_short_lidar_transient_is_debounced():
    result = run_scenario("short_lidar_recovery", seed=1)
    assert result["production_request_s"] is None
    assert result["shadow_request_s"] is None


def test_normal_flight_has_no_false_trigger():
    result = run_scenario("normal_flight", seed=4)
    assert result["production_false_trigger"] is False
    assert result["shadow_false_trigger"] is False


def test_slow_drift_reaches_watch_before_offline_truth_boundary():
    result = run_scenario("slow_position_drift", seed=0)
    assert result["watch_s"] is not None
    assert result["watch_s"] < result["truth_degradation_start_s"]
    assert result["degraded_s"] is not None


def test_obstacle_conflict_escalates_fail_closed():
    result = run_scenario("obstacle_relocalization_conflict", seed=0)
    assert result["failsafe_s"] is not None


def test_relocalization_failure_escalates_failsafe():
    result = run_scenario("relocalization_failure", seed=0)
    assert result["failsafe_s"] is not None


def test_shared_request_duplicates_are_observed_not_republished():
    result = run_scenario("multi_source_degradation", seed=0)
    assert result["production_duplicate_episodes"] >= 1
    assert result["shadow_request_count"] <= 1


def test_timestamp_regression_is_failsafe():
    core = RelocalizationRiskCore()
    core.update(scenario_sample("normal_flight", 1.0))
    decision = core.update(scenario_sample("normal_flight", 0.5))
    assert decision.level == RiskLevel.FAILSAFE
    assert "clock_regressed" in decision.reasons


def test_invalid_threshold_order_is_rejected():
    with pytest.raises(ValueError):
        RelocalizationRiskConfig(
            watch_threshold=0.7,
            degraded_threshold=0.5,
            relocalize_threshold=0.8,
        )


def test_matching_epoch_starts_cooldown():
    core = RelocalizationRiskCore()
    for step in range(150):
        sample = scenario_sample(
            "relocalization_success_recovery", step / 10.0
        )
        core.update(sample)
    assert core.last_success_s is not None
    assert core.failure_count == 0


def test_directional_only_decision_is_not_production_eligible():
    core = RelocalizationRiskCore(RelocalizationRiskConfig(watch_dwell_s=0.0))
    sample = scenario_sample("single_direction_weakness", 12.0)
    decision = core.update(sample)
    assert decision.production_eligible is False
    assert decision.would_request is False
    assert decision.level <= RiskLevel.WATCH
