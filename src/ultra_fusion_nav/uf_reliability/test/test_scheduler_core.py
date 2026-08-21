import unittest

from uf_reliability.scheduler_core import ReliabilitySchedulerCore, SchedulerConfig


def score(value, valid=True, arrival_s=0.0, reasons=(), count=1, minimum=1,
          hard_gate_allowed=True):
    return {
        "degradation_score": value,
        "valid": valid,
        "hard_gate_allowed": hard_gate_allowed,
        "arrival_s": arrival_s,
        "reasons": reasons,
        "observation_count": count,
        "minimum_observation_count": minimum,
    }


class SchedulerCoreTest(unittest.TestCase):
    def test_relocalization_failure_does_not_override_healthy_capabilities(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("imu",),
            required_modalities=("imu",),
            minimum_usable_modalities=1,
            transition_dwell_s=0.0,
            recovery_dwell_s=0.0,
        ))
        scores = {"imu": {
            "degradation_score": 0.1,
            "valid": True,
            "arrival_s": 0.0,
        }}
        result = core.update(scores, 0.0)
        self.assertIn(result.health_state, ("NORMAL", "RECOVERED"))
        failed = core.update(scores, 1.0, relocalization_failed=True)
        self.assertIn(failed.health_state, ("NORMAL", "RECOVERED"))
        cleared = core.update(scores, 2.0, relocalization_requested=True)
        self.assertEqual(cleared.health_state, "RELOCALIZING")

    def test_relocalization_failure_cannot_hide_real_required_sensor_loss(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("imu", "gnss"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            transition_dwell_s=0.0,
            recovery_dwell_s=0.0,
        ))
        result = core.update({
            "imu": score(1.0, valid=False),
            "gnss": score(0.1),
        }, 0.0, relocalization_failed=True)
        self.assertEqual(result.health_state, "FAILSAFE")
    def setUp(self):
        self.core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("gnss", "optical_flow"),
            stale_after_s=1.0,
            transition_dwell_s=0.0,
            recovery_dwell_s=1.0,
            recovered_hold_s=0.5,
        ))

    def test_normal_continuous_weight_and_covariance(self):
        result = self.core.update({
            "gnss": score(0.10, arrival_s=0.0),
            "optical_flow": score(0.20, arrival_s=0.0),
        }, 0.1)
        self.assertEqual(result.health_state, "NORMAL")
        self.assertTrue(result.factor_enabled["gnss"])
        self.assertAlmostEqual(result.reliability_weights["gnss"], 0.90)
        self.assertEqual(result.covariance_inflation["gnss"], 1.0)

    def test_inactive_modalities_are_neutral_and_cannot_enable_factors(self):
        result = self.core.update({
            "lidar": score(0.0, arrival_s=0.0),
            "imu": score(0.0, arrival_s=0.0),
            "vision": score(0.0, arrival_s=0.0),
            "gnss": score(0.10, arrival_s=0.0),
            "optical_flow": score(0.20, arrival_s=0.0),
        }, 0.1)

        for name in ("lidar", "imu", "vision"):
            self.assertEqual(result.degradation_scores[name], 0.0)
            self.assertEqual(result.reliability_weights[name], 0.0)
            self.assertFalse(result.factor_enabled[name])
            self.assertEqual(result.reasons[name], ("inactive_modality",))

    def test_one_degraded_aiding_factor_does_not_fail_complete_system(self):
        result = self.core.update({
            "gnss": score(0.75, arrival_s=0.0),
            "optical_flow": score(0.10, arrival_s=0.0),
        }, 0.1)
        self.assertEqual(result.health_state, "DEGRADED")
        self.assertFalse(result.factor_enabled["gnss"])
        self.assertEqual(result.covariance_inflation["gnss"], 20.0)

    def test_required_imu_failure_is_risk_when_other_sources_survive(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("lidar", "gnss", "imu", "optical_flow"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            stale_after_s=1.0,
            transition_dwell_s=0.0,
        ))
        result = core.update({
            "lidar": score(0.10),
            "gnss": score(0.10),
            "imu": score(0.90, valid=False),
            "optical_flow": score(0.10),
        }, 0.1)
        self.assertEqual(result.health_state, "RISK")
        self.assertTrue(result.factor_enabled["lidar"])
        self.assertTrue(result.factor_enabled["gnss"])
        self.assertTrue(result.factor_enabled["optical_flow"])
        self.assertEqual(result.capability_support["propagation"], 0.0)

    def test_imu_only_keeps_bounded_estimator_available(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("lidar", "gnss", "imu", "optical_flow"),
            required_modalities=("imu",),
            minimum_usable_modalities=1,
            stale_after_s=1.0,
            transition_dwell_s=0.0,
        ))
        result = core.update({
            "lidar": score(1.0, valid=False),
            "gnss": score(1.0, valid=False),
            "imu": score(0.10),
            "optical_flow": score(1.0, valid=False),
        }, 0.1)
        self.assertNotEqual(result.health_state, "FAILSAFE")
        self.assertTrue(result.factor_enabled["imu"])
        self.assertGreater(result.capability_support["propagation"], 0.0)
        self.assertEqual(result.capability_support["horizontal_motion"], 0.0)
        self.assertEqual(result.estimator_support, 0.0)

    def test_each_single_healthy_source_keeps_output_available(self):
        modalities = ("lidar", "gnss", "imu", "optical_flow", "vision")
        for survivor in modalities:
            with self.subTest(survivor=survivor):
                core = ReliabilitySchedulerCore(SchedulerConfig(
                    active_modalities=modalities,
                    required_modalities=("imu",),
                    minimum_usable_modalities=1,
                    stale_after_s=1.0,
                    transition_dwell_s=0.0,
                    recovery_dwell_s=0.0,
                ))
                scores = {
                    name: score(
                        0.10 if name == survivor else 1.0,
                        valid=name == survivor,
                    )
                    for name in modalities
                }
                result = core.update(scores, 0.1)

                self.assertNotEqual(result.health_state, "FAILSAFE")
                self.assertTrue(result.factor_enabled[survivor])
                self.assertEqual(
                    sum(result.factor_enabled.values()),
                    1,
                )
                if survivor == "imu":
                    self.assertIn(
                        result.health_state,
                        ("DEGRADED", "RECOVERED", "NORMAL"),
                    )
                else:
                    self.assertEqual(result.health_state, "RISK")

    def test_lidar_can_use_a_longer_score_timeout_without_extending_imu(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("lidar", "imu"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            stale_after_s=0.5,
            modality_stale_after_s=(("lidar", 1.6),),
            transition_dwell_s=0.0,
            recovery_dwell_s=0.0,
        ))
        healthy = core.update({
            "lidar": score(0.10, arrival_s=0.0),
            "imu": score(0.10, arrival_s=0.0),
        }, 0.1)
        self.assertEqual(healthy.health_state, "NORMAL")

        lidar_still_fresh = core.update({
            "lidar": score(0.10, arrival_s=0.0),
            "imu": score(0.10, arrival_s=0.8),
        }, 0.8)
        self.assertNotEqual(lidar_still_fresh.health_state, "FAILSAFE")

        lidar_stale = core.update({
            "lidar": score(0.10, arrival_s=0.0),
            "imu": score(0.10, arrival_s=1.7),
        }, 1.7)
        self.assertEqual(lidar_stale.health_state, "FAILSAFE")

    def test_high_dynamic_imu_remains_propagation_factor(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("lidar", "gnss", "imu", "optical_flow"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            stale_after_s=1.0,
            transition_dwell_s=0.0,
        ))
        result = core.update({
            "lidar": score(0.10),
            "gnss": score(0.10),
            "imu": score(0.95, reasons=("high_dynamic",)),
            "optical_flow": score(1.0, valid=False),
        }, 0.1)
        self.assertTrue(result.factor_enabled["imu"])
        self.assertAlmostEqual(result.reliability_weights["imu"], 0.20)
        self.assertEqual(result.covariance_inflation["imu"], 1.0)
        self.assertNotEqual(result.health_state, "FAILSAFE")

    def test_saturated_imu_is_hard_disabled(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("lidar", "gnss", "imu"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            stale_after_s=1.0,
            transition_dwell_s=0.0,
        ))
        result = core.update({
            "lidar": score(0.10),
            "gnss": score(0.10),
            "imu": score(0.90, reasons=("saturation_eq21",)),
        }, 0.1)
        self.assertFalse(result.factor_enabled["imu"])
        self.assertEqual(result.covariance_inflation["imu"], 20.0)
        self.assertEqual(result.health_state, "RISK")

    def test_rotation_gated_flow_only_degrades_four_source_system(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("lidar", "gnss", "imu", "optical_flow"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            stale_after_s=1.0,
            transition_dwell_s=0.0,
        ))
        result = core.update({
            "lidar": score(0.10),
            "gnss": score(0.10),
            "imu": score(0.10),
            "optical_flow": score(1.0, valid=False),
        }, 0.1)
        self.assertEqual(result.health_state, "DEGRADED")
        self.assertTrue(result.factor_enabled["lidar"])
        self.assertTrue(result.factor_enabled["gnss"])
        self.assertTrue(result.factor_enabled["imu"])
        self.assertFalse(result.factor_enabled["optical_flow"])
        self.assertTrue(result.capability_observable["horizontal_motion"])
        self.assertGreater(result.estimator_support, 0.0)

    def test_flow_can_support_horizontal_motion_without_claiming_position(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("imu", "optical_flow"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            stale_after_s=1.0,
            transition_dwell_s=0.0,
        ))
        result = core.update({
            "imu": score(0.10),
            "optical_flow": score(0.20),
        }, 0.1)
        self.assertAlmostEqual(result.capability_support["propagation"], 0.90)
        self.assertAlmostEqual(result.capability_support["horizontal_motion"], 0.80)
        self.assertEqual(result.capability_support["horizontal_position"], 0.0)
        self.assertAlmostEqual(result.estimator_support, 0.80)

    def test_one_required_plus_one_aiding_modality_is_risk(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("lidar", "gnss", "imu", "optical_flow"),
            required_modalities=("imu",),
            minimum_usable_modalities=2,
            stale_after_s=1.0,
            transition_dwell_s=0.0,
        ))
        result = core.update({
            "lidar": score(1.0, valid=False),
            "gnss": score(0.10),
            "imu": score(0.10),
            "optical_flow": score(1.0, valid=False),
        }, 0.1)
        self.assertEqual(result.health_state, "RISK")

    def test_configuration_rejects_unobservable_requirements(self):
        with self.assertRaises(ValueError):
            ReliabilitySchedulerCore(SchedulerConfig(
                active_modalities=("gnss",),
                required_modalities=("imu",),
            ))
        with self.assertRaises(ValueError):
            ReliabilitySchedulerCore(SchedulerConfig(
                active_modalities=("gnss",),
                minimum_usable_modalities=2,
            ))

    def test_hysteresis_prevents_reenable_until_lower_threshold(self):
        self.core.update({
            "gnss": score(0.90, arrival_s=0.0),
            "optical_flow": score(0.10, arrival_s=0.0),
        }, 0.1)
        still_off = self.core.update({
            "gnss": score(0.60, arrival_s=0.1),
            "optical_flow": score(0.10, arrival_s=0.1),
        }, 0.2)
        self.assertFalse(still_off.factor_enabled["gnss"])
        reenabled = self.core.update({
            "gnss": score(0.50, arrival_s=0.2),
            "optical_flow": score(0.10, arrival_s=0.2),
        }, 0.3)
        self.assertTrue(reenabled.factor_enabled["gnss"])

    def test_provisional_gnss_breaks_innovation_bootstrap_deadlock(self):
        result = self.core.update({
            "gnss": score(
                0.55254698,
                arrival_s=0.0,
                reasons=(
                    "incomplete_paper_evidence",
                    "provisional_gnss_direct_evidence_only",
                ),
                hard_gate_allowed=False,
            ),
            "optical_flow": score(0.10, arrival_s=0.0),
        }, 0.1)

        self.assertTrue(result.factor_enabled["gnss"])
        self.assertAlmostEqual(
            result.reliability_weights["gnss"], 0.44745302
        )
        self.assertEqual(result.covariance_inflation["gnss"], 1.0)
        self.assertIn(
            "gnss_provisional_bootstrap", result.reasons["gnss"]
        )

    def test_nonprovisional_gnss_still_obeys_enable_hysteresis(self):
        result = self.core.update({
            "gnss": score(
                0.60,
                arrival_s=0.0,
                reasons=("large_covariance_eq23",),
                hard_gate_allowed=False,
            ),
            "optical_flow": score(0.10, arrival_s=0.0),
        }, 0.1)

        self.assertFalse(result.factor_enabled["gnss"])

    def test_soft_only_evidence_cannot_binary_disable_an_enabled_factor(self):
        self.core.update({
            "gnss": score(0.10, arrival_s=0.0),
            "optical_flow": score(0.10, arrival_s=0.0),
        }, 0.1)
        degraded = self.core.update({
            "gnss": score(
                0.95, arrival_s=0.1, hard_gate_allowed=False,
            ),
            "optical_flow": score(0.10, arrival_s=0.1),
        }, 0.2)

        self.assertTrue(degraded.factor_enabled["gnss"])
        self.assertEqual(degraded.covariance_inflation["gnss"], 1.0)
        self.assertIn(
            "hard_gate_blocked_by_evidence_policy",
            degraded.reasons["gnss"],
        )

    def test_stale_scores_fail_safe_and_relocalization_request_is_explicit(self):
        result = self.core.update({}, 2.0)
        self.assertEqual(result.health_state, "FAILSAFE")
        result = self.core.update({
            "gnss": score(0.10, arrival_s=2.0),
            "optical_flow": score(0.10, arrival_s=2.0),
        }, 2.1, relocalization_requested=True)
        self.assertEqual(result.health_state, "RELOCALIZING")

    def test_missing_optional_modality_degrades_without_disabling_valid_factor(self):
        result = self.core.update({
            "gnss": score(0.10, arrival_s=0.0),
        }, 0.1)
        self.assertEqual(result.health_state, "DEGRADED")
        self.assertTrue(result.factor_enabled["gnss"])
        self.assertFalse(result.factor_enabled["optical_flow"])
        self.assertEqual(result.covariance_inflation["optical_flow"], 20.0)

    def test_all_active_modalities_missing_forces_failsafe(self):
        result = self.core.update({}, 0.1)
        self.assertEqual(result.health_state, "FAILSAFE")
        self.assertFalse(result.factor_enabled["gnss"])
        self.assertFalse(result.factor_enabled["optical_flow"])

    def test_eq15_minimum_observation_count_disables_only_that_factor(self):
        result = self.core.update({
            "gnss": score(0.10, arrival_s=0.0, count=0, minimum=1),
            "optical_flow": score(0.10, arrival_s=0.0),
        }, 0.1)
        self.assertEqual(result.health_state, "DEGRADED")
        self.assertFalse(result.factor_enabled["gnss"])
        self.assertTrue(result.factor_enabled["optical_flow"])
        self.assertIn("insufficient_observations_eq15", result.reasons["gnss"])

    def test_recovery_requires_dwell_and_hold(self):
        self.core.update({
            "gnss": score(0.90, arrival_s=0.0),
            "optical_flow": score(0.90, arrival_s=0.0),
        }, 0.1)
        self.core.update({
            "gnss": score(0.10, arrival_s=0.2),
            "optical_flow": score(0.10, arrival_s=0.2),
        }, 0.2)
        self.assertEqual(self.core.health_state, "FAILSAFE")
        self.core.update({
            "gnss": score(0.10, arrival_s=1.0),
            "optical_flow": score(0.10, arrival_s=1.0),
        }, 1.0)
        self.assertEqual(self.core.health_state, "FAILSAFE")
        self.core.update({
            "gnss": score(0.10, arrival_s=1.3),
            "optical_flow": score(0.10, arrival_s=1.3),
        }, 1.3)
        self.assertEqual(self.core.health_state, "RECOVERED")
        self.core.update({
            "gnss": score(0.10, arrival_s=1.4),
            "optical_flow": score(0.10, arrival_s=1.4),
        }, 1.4)
        self.assertEqual(self.core.health_state, "RECOVERED")
        self.core.update({
            "gnss": score(0.10, arrival_s=1.9),
            "optical_flow": score(0.10, arrival_s=1.9),
        }, 1.9)
        self.assertEqual(self.core.health_state, "NORMAL")

    def test_future_source_timestamps_are_stale(self):
        result = self.core.update({
            "gnss": score(0.10, arrival_s=2.0),
            "optical_flow": score(0.10, arrival_s=2.0),
        }, 1.0)
        self.assertEqual(result.health_state, "FAILSAFE")
        self.assertFalse(result.factor_enabled["gnss"])
        self.assertIn("score_stale_or_invalid", result.reasons["gnss"])

    def test_clock_rewind_restarts_transition_dwell(self):
        core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=("gnss",),
            minimum_usable_modalities=1,
            stale_after_s=2.0,
            transition_dwell_s=1.0,
            recovery_dwell_s=0.0,
        ))
        healthy = {"gnss": score(0.10, arrival_s=10.0)}
        core.update(healthy, 10.0)
        core.update(healthy, 10.5)
        self.assertEqual(core.health_state, "FAILSAFE")

        rewound = {"gnss": score(0.10, arrival_s=1.0)}
        core.update(rewound, 1.0)
        core.update(rewound, 1.6)
        self.assertEqual(core.health_state, "FAILSAFE")
        core.update({"gnss": score(0.10, arrival_s=2.1)}, 2.1)
        self.assertEqual(core.health_state, "RECOVERED")


if __name__ == "__main__":
    unittest.main()
