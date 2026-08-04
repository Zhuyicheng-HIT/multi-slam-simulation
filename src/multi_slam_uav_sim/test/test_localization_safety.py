import unittest

from multi_slam_uav_sim.localization_safety import (
    HOLDING,
    LOSS_PENDING,
    RECOVERY_PENDING,
    RELOCALIZING_HOLD,
    TRACKING,
    LocalizationSafetyStateMachine,
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
        self.assertTrue(decision.request_relocalization)

        decision = self.machine.update(False, 1.2)
        self.assertEqual(decision.state, HOLDING)
        self.assertTrue(decision.hold)
        decision = self.machine.update(False, 1.3)
        self.assertEqual(decision.state, RECOVERY_PENDING)
        self.assertTrue(decision.hold)

    def test_persistent_loss_stays_in_relocalizing_hold(self):
        self.machine.update(True, 0.0)
        self.machine.update(True, 0.3)
        decision = self.machine.update(True, 1.3)
        self.assertEqual(decision.state, RELOCALIZING_HOLD)
        self.assertTrue(decision.hold)

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

    def test_failsafe_or_missing_critical_capability_is_an_obvious_loss(self):
        self.assertTrue(scheduler_localization_loss(
            "FAILSAFE", 0.8, (), (), 0.15)[0])
        lost, reason = scheduler_localization_loss(
            "RISK",
            0.5,
            ("propagation", "horizontal_motion", "yaw_tracking"),
            (True, False, True),
            minimum_support=0.15,
        )
        self.assertTrue(lost)
        self.assertEqual(reason, "unobservable_horizontal_motion")


if __name__ == "__main__":
    unittest.main()
