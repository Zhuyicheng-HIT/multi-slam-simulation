import unittest

from uf_reliability.scheduler_core import ReliabilitySchedulerCore, SchedulerConfig


def score(value, valid=True, arrival_s=0.0, reasons=(), count=1, minimum=1):
    return {
        "degradation_score": value,
        "valid": valid,
        "arrival_s": arrival_s,
        "reasons": reasons,
        "observation_count": count,
        "minimum_observation_count": minimum,
    }


class SchedulerCoreTest(unittest.TestCase):
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
        self.assertAlmostEqual(result.covariance_inflation["gnss"], 1.0 / 0.90)

    def test_high_score_disables_factor_and_enters_risk(self):
        result = self.core.update({
            "gnss": score(0.75, arrival_s=0.0),
            "optical_flow": score(0.10, arrival_s=0.0),
        }, 0.1)
        self.assertEqual(result.health_state, "RISK")
        self.assertFalse(result.factor_enabled["gnss"])
        self.assertEqual(result.covariance_inflation["gnss"], 20.0)

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

    def test_stale_scores_fail_safe_and_relocalization_request_is_explicit(self):
        result = self.core.update({}, 2.0)
        self.assertEqual(result.health_state, "FAILSAFE")
        result = self.core.update({
            "gnss": score(0.10, arrival_s=2.0),
            "optical_flow": score(0.10, arrival_s=2.0),
        }, 2.1, relocalization_requested=True)
        self.assertEqual(result.health_state, "RELOCALIZING")

    def test_one_missing_optional_modality_degrades_without_disabling_valid_factor(self):
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


if __name__ == "__main__":
    unittest.main()
