import unittest

from multi_slam_uav_sim.localization_safety import (
    HOLDING,
    LOSS_PENDING,
    RECOVERY_PENDING,
    RELOCALIZING_HOLD,
    TRACKING,
    LocalizationSafetyStateMachine,
    diagnostic_level_value,
    mission_hold_required,
    scheduler_localization_loss,
)


class LocalizationSafetyStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.machine = LocalizationSafetyStateMachine(
            loss_dwell_s=0.3,
            minimum_hold_s=1.0,
            recovery_dwell_s=0.5,
        )

    def test_transient_loss_does_not_stop_the_mission(self):
        decision = self.machine.update(True, 0.0)
        self.assertEqual(decision.state, LOSS_PENDING)
        self.assertFalse(decision.hold)
        decision = self.machine.update(False, 0.2)
        self.assertEqual(decision.state, TRACKING)
        self.assertFalse(decision.hold)

    def test_confirmed_loss_holds_for_at_least_one_second(self):
        self.machine.update(True, 0.0)
        decision = self.machine.update(True, 0.3)
        self.assertEqual(decision.state, HOLDING)
        self.assertTrue(decision.hold)
        self.assertFalse(decision.request_relocalization)

        decision = self.machine.update(False, 1.2)
        self.assertEqual(decision.state, HOLDING)
        self.assertTrue(decision.hold)
        self.assertFalse(decision.request_relocalization)
        decision = self.machine.update(False, 1.3)
        self.assertEqual(decision.state, RECOVERY_PENDING)
        self.assertTrue(decision.hold)
        self.assertFalse(decision.request_relocalization)

    def test_persistent_loss_stays_in_relocalizing_hold(self):
        self.machine.update(True, 0.0)
        decision = self.machine.update(True, 0.3)
        self.assertEqual(decision.state, HOLDING)
        self.assertFalse(decision.request_relocalization)
        decision = self.machine.update(True, 1.3)
        self.assertEqual(decision.state, RELOCALIZING_HOLD)
        self.assertTrue(decision.hold)
        self.assertTrue(decision.request_relocalization)
        decision = self.machine.update(True, 1.4)
        self.assertEqual(decision.state, RELOCALIZING_HOLD)
        self.assertFalse(decision.request_relocalization)

    def test_recovery_requires_stable_dwell_before_resume(self):
        self.machine.update(True, 0.0)
        self.machine.update(True, 0.3)
        self.machine.update(False, 1.3)
        decision = self.machine.update(False, 1.7)
        self.assertEqual(decision.state, RECOVERY_PENDING)
        self.assertTrue(decision.hold)
        decision = self.machine.update(False, 1.8)
        self.assertEqual(decision.state, TRACKING)
        self.assertFalse(decision.hold)
        self.assertTrue(decision.clear_relocalization_request)

    def test_relocalizing_label_does_not_force_a_deadlock_when_observable(self):
        lost, reason = scheduler_localization_loss(
            "RELOCALIZING",
            0.8,
            ("propagation", "horizontal_motion", "yaw_tracking"),
            (True, True, True),
            minimum_support=0.15,
        )
        self.assertFalse(lost)
        self.assertEqual(reason, "observable")

    def test_failsafe_with_observable_capabilities_is_not_pose_loss(self):
        lost, reason = scheduler_localization_loss(
            "FAILSAFE",
            0.8,
            ("propagation", "horizontal_motion", "yaw_tracking"),
            (True, True, True),
            0.15,
        )
        self.assertFalse(lost)
        self.assertEqual(reason, "observable")

    def test_missing_or_unobservable_critical_capability_is_an_obvious_loss(self):
        lost, reason = scheduler_localization_loss(
            "FAILSAFE", 0.8, (), (), 0.15)
        self.assertTrue(lost)
        self.assertTrue(reason.startswith("missing_capability_status_"))
        lost, reason = scheduler_localization_loss(
            "RISK",
            0.5,
            ("propagation", "horizontal_motion", "yaw_tracking"),
            (True, False, True),
            minimum_support=0.15,
        )
        self.assertTrue(lost)
        self.assertEqual(reason, "unobservable_horizontal_motion")

    def test_stale_or_nonfinite_unified_output_is_an_obvious_loss(self):
        arguments = (
            "NORMAL",
            0.8,
            ("propagation", "horizontal_motion", "yaw_tracking"),
            (True, True, True),
            0.15,
        )
        self.assertEqual(
            scheduler_localization_loss(
                *arguments, estimator_fresh=False)[1],
            "unified_odom_stale",
        )
        self.assertEqual(
            scheduler_localization_loss(
                *arguments, estimator_finite=False)[1],
            "unified_odom_nonfinite",
        )

    def test_external_nav_gate_rejection_is_an_obvious_loss(self):
        arguments = (
            "NORMAL",
            0.8,
            ("propagation", "horizontal_motion", "yaw_tracking"),
            (True, True, True),
            0.15,
        )
        self.assertEqual(
            scheduler_localization_loss(
                *arguments, external_nav_gate_fresh=False)[1],
            "external_nav_gate_stale",
        )
        self.assertEqual(
            scheduler_localization_loss(
                *arguments,
                external_nav_gate_healthy=False,
                external_nav_gate_reason="position_covariance_exceeds_limit",
            )[1],
            "external_nav_gate_position_covariance_exceeds_limit",
        )

    def test_diagnostic_level_accepts_integer_and_humble_byte_binding(self):
        self.assertEqual(diagnostic_level_value(0), 0)
        self.assertEqual(diagnostic_level_value(b"\x00"), 0)
        self.assertEqual(diagnostic_level_value(bytearray(b"\x02")), 2)
        with self.assertRaises(ValueError):
            diagnostic_level_value(b"")
        with self.assertRaises(ValueError):
            diagnostic_level_value(b"\x00\x01")

    def test_explicit_relocalization_request_holds_an_otherwise_healthy_mission(self):
        self.assertFalse(mission_hold_required(False, False, False))
        self.assertTrue(mission_hold_required(False, False, True))
        self.assertTrue(mission_hold_required(True, False, False))
        self.assertTrue(mission_hold_required(False, True, False))


if __name__ == "__main__":
    unittest.main()
