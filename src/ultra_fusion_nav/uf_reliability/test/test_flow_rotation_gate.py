import unittest

from uf_reliability.flow_rotation_gate import (
    FlowRotationGateConfig,
    OpticalFlowRotationGate,
    interval_mean_absolute_yaw_rate,
)


class FlowRotationGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = OpticalFlowRotationGate(FlowRotationGateConfig(
            lower_yaw_rate_radps=0.08,
            upper_yaw_rate_radps=0.30,
            recovery_dwell_s=0.8,
            recovery_ramp_s=1.5,
            minimum_translation_m=0.01,
        ))

    def test_interval_yaw_rate_uses_the_flow_integration_span(self):
        samples = [(0.0, 0.10), (0.05, -0.20), (0.10, -0.30)]
        value = interval_mean_absolute_yaw_rate(samples, 0.0, 0.10)
        self.assertAlmostEqual(value, 0.20)
        self.assertIsNone(
            interval_mean_absolute_yaw_rate(samples, 0.5, 0.6)
        )

    def test_turn_downweights_then_waits_for_translation_and_ramps(self):
        active = self.gate.update(0.0, 0.02, 0.10, True)
        self.assertEqual(active.phase, "ACTIVE")
        self.assertEqual(active.weight, 1.0)

        turning = self.gate.update(1.0, 0.19, 0.10, True)
        self.assertEqual(turning.phase, "TURNING")
        self.assertAlmostEqual(turning.weight, 0.5)

        suppressed = self.gate.update(1.1, 0.35, 0.10, True)
        self.assertTrue(suppressed.hard_disabled)

        stationary = self.gate.update(1.2, 0.02, 0.0, True)
        self.assertEqual(stationary.phase, "RECOVERY_DWELL")
        self.assertEqual(stationary.reason, "waiting_for_consistent_translation")

        dwell_start = self.gate.update(2.0, 0.02, 0.10, True)
        self.assertEqual(dwell_start.weight, 0.0)
        still_dwelling = self.gate.update(2.7, 0.02, 0.10, True)
        self.assertEqual(still_dwelling.phase, "RECOVERY_DWELL")

        ramp_start = self.gate.update(2.81, 0.02, 0.10, True)
        self.assertEqual(ramp_start.phase, "RECOVERING")
        self.assertAlmostEqual(ramp_start.weight, 1.0 / 150.0)
        ramp_middle = self.gate.update(3.55, 0.02, 0.10, True)
        self.assertAlmostEqual(ramp_middle.weight, 0.5)
        recovered = self.gate.update(4.31, 0.02, 0.10, True)
        self.assertEqual(recovered.phase, "ACTIVE")
        self.assertEqual(recovered.weight, 1.0)

    def test_recovery_resets_when_translation_becomes_inconsistent(self):
        self.gate.update(0.0, 0.4, 0.10, True)
        self.gate.update(0.1, 0.01, 0.10, True)
        interrupted = self.gate.update(1.0, 0.01, 0.10, False)
        self.assertEqual(interrupted.phase, "RECOVERY_DWELL")
        self.assertTrue(interrupted.hard_disabled)
        restarted = self.gate.update(1.1, 0.01, 0.10, True)
        self.assertEqual(restarted.weight, 0.0)

    def test_missing_fcu_yaw_rate_hard_disables_flow(self):
        result = self.gate.update(1.0, None, 0.10, True)
        self.assertEqual(result.phase, "YAW_RATE_UNAVAILABLE")
        self.assertTrue(result.hard_disabled)

    def test_speed_threshold_is_independent_of_flow_sample_period(self):
        gate = OpticalFlowRotationGate(FlowRotationGateConfig(
            lower_yaw_rate_radps=0.08,
            upper_yaw_rate_radps=0.30,
            recovery_dwell_s=0.8,
            recovery_ramp_s=1.5,
            minimum_translation_m=0.01,
            minimum_translation_speed_mps=0.08,
        ))
        gate.update(0.0, 0.4, 0.006, True, translation_interval_s=0.034)
        recovering = gate.update(
            0.1, 0.01, 0.006, True, translation_interval_s=0.034
        )
        self.assertTrue(recovering.translation_ready)
        self.assertEqual(recovering.phase, "RECOVERY_DWELL")

        too_slow = gate.update(
            0.2, 0.01, 0.002, True, translation_interval_s=0.034
        )
        self.assertFalse(too_slow.translation_ready)
        self.assertEqual(too_slow.reason, "waiting_for_consistent_translation")

    def test_compensated_rotation_does_not_disable_flow(self):
        gate = OpticalFlowRotationGate(FlowRotationGateConfig(
            lower_yaw_rate_radps=0.08,
            upper_yaw_rate_radps=0.30,
            allow_compensated_rotation=True,
        ))
        result = gate.update(
            1.0, 0.80, 0.10, True,
            translation_interval_s=0.05,
            rotation_compensated=True,
        )
        self.assertEqual(result.phase, "ACTIVE")
        self.assertEqual(result.weight, 1.0)
        self.assertFalse(result.hard_disabled)
        self.assertEqual(result.reason, "apm_rotation_compensated")


if __name__ == "__main__":
    unittest.main()
