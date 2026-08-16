import unittest
import math

from nav_msgs.msg import Odometry
from uf_interfaces.msg import FusionEpoch
from uf_sensor_pipeline.external_nav_gate import (
    ExternalNavGate,
    capability_covariance_scale,
    capability_support_allowed,
    fusion_epoch_advances,
    odometry_state_guard_reason,
    propagate_odometry,
    scheduler_state_allowed,
)


class SchedulerStateGateTest(unittest.TestCase):
    @staticmethod
    def odometry(stamp_s, position=(0.0, 0.0, 0.0), yaw=0.0):
        message = Odometry()
        message.header.stamp.sec = int(stamp_s)
        message.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1.0e9)
        message.pose.pose.position.x = position[0]
        message.pose.pose.position.y = position[1]
        message.pose.pose.position.z = position[2]
        message.pose.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * yaw)
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 0.01
        return message

    def test_only_explicit_control_safe_states_are_allowed(self):
        allowed = ("NORMAL", "RECOVERED")
        self.assertTrue(scheduler_state_allowed("NORMAL", allowed))
        self.assertTrue(scheduler_state_allowed("recovered", allowed))
        self.assertFalse(scheduler_state_allowed("DEGRADED", allowed))
        self.assertFalse(scheduler_state_allowed("FAILSAFE", allowed))
        self.assertFalse(scheduler_state_allowed("", allowed))

    def test_degraded_state_can_be_explicitly_allowed(self):
        allowed = ("NORMAL", "RECOVERED", "DEGRADED", "RISK")
        self.assertTrue(scheduler_state_allowed("DEGRADED", allowed))
        self.assertTrue(scheduler_state_allowed("RISK", allowed))
        self.assertFalse(scheduler_state_allowed("FAILSAFE", allowed))

    def test_required_capabilities_are_independent_of_optional_sensor(self):
        support = {"propagation": 0.9, "horizontal_motion": 0.7}
        self.assertTrue(capability_support_allowed(
            support, ("propagation", "horizontal_motion"), 0.15
        ))
        support["horizontal_motion"] = 0.0
        self.assertFalse(capability_support_allowed(
            support, ("propagation", "horizontal_motion"), 0.15
        ))

    def test_capability_loss_inflates_covariance_without_forcing_outage(self):
        self.assertAlmostEqual(
            capability_covariance_scale(0.8, 0.15, 5.0), 1.25
        )
        self.assertAlmostEqual(
            capability_covariance_scale(0.0, 0.15, 5.0), 5.0
        )
        self.assertEqual(
            capability_covariance_scale(0.0, 0.15, 5.0, enabled=False),
            1.0,
        )

    def test_only_new_applied_fusion_epoch_resets_jump_reference(self):
        self.assertTrue(fusion_epoch_advances(True, 4, 3))
        self.assertFalse(fusion_epoch_advances(False, 4, 3))
        self.assertFalse(fusion_epoch_advances(True, 3, 3))
        self.assertFalse(fusion_epoch_advances(True, 2, 3))

    def test_session_and_transaction_epoch_reset_is_ordered(self):
        gate = object.__new__(ExternalNavGate)
        gate.current_fusion_epoch = 4
        gate.current_fusion_session = 10
        gate.current_fusion_transaction = 90
        gate.minimum_epoch_stamp_s = 8.0
        gate.latest_source = object()
        gate.fusion_epoch_events = 0
        gate.fusion_session_events = 0
        gate.last_reason = "ok"

        new_session = FusionEpoch()
        new_session.session_id = 11
        new_session.reset_counter = 0
        new_session.applied = False
        gate._fusion_epoch(new_session)
        self.assertEqual(gate.current_fusion_session, 11)
        self.assertEqual(gate.current_fusion_epoch, 0)
        self.assertIsNone(gate.latest_source)
        self.assertIsNone(gate.minimum_epoch_stamp_s)

        committed = FusionEpoch()
        committed.header.stamp.sec = 12
        committed.header.stamp.nanosec = 250000000
        committed.applied = True
        committed.session_id = 11
        committed.transaction_id = 101
        committed.reset_counter = 1
        gate._fusion_epoch(committed)
        self.assertEqual(gate.current_fusion_transaction, 101)
        self.assertAlmostEqual(gate.minimum_epoch_stamp_s, 12.25)
        self.assertEqual(gate.fusion_epoch_events, 1)

        stale_session = FusionEpoch()
        stale_session.applied = True
        stale_session.session_id = 10
        stale_session.transaction_id = 102
        stale_session.reset_counter = 9
        gate._fusion_epoch(stale_session)
        self.assertEqual(gate.current_fusion_session, 11)
        self.assertEqual(gate.current_fusion_transaction, 101)

    def test_short_horizon_propagation_uses_body_twist_and_inflates_covariance(self):
        message = Odometry()
        message.header.stamp.sec = 10
        message.pose.pose.orientation.w = 1.0
        message.pose.pose.position.x = 1.0
        message.twist.twist.linear.x = 2.0
        message.twist.twist.angular.z = math.pi
        message.pose.covariance[0] = 0.01
        output = propagate_odometry(message, 10.5, covariance_scale=2.0)
        self.assertAlmostEqual(output.pose.pose.position.x, 2.0)
        self.assertAlmostEqual(output.pose.pose.orientation.z, math.sin(math.pi / 4.0))
        self.assertAlmostEqual(output.pose.pose.orientation.w, math.cos(math.pi / 4.0))
        self.assertGreater(output.pose.covariance[0], 0.02)
        self.assertEqual(output.header.stamp.sec, 10)
        self.assertEqual(output.header.stamp.nanosec, 500000000)

    def test_state_guard_accepts_bounded_motion(self):
        previous = self.odometry(10.0)
        current = self.odometry(10.1, position=(0.2, 0.0, 0.0), yaw=0.1)
        self.assertEqual(
            odometry_state_guard_reason(current, previous),
            "ok",
        )

    def test_state_guard_rejects_divergent_covariance(self):
        current = self.odometry(10.0)
        current.pose.covariance[7] = 100000.0
        self.assertEqual(
            odometry_state_guard_reason(current),
            "position_covariance_exceeds_limit",
        )

    def test_state_guard_can_keep_finite_degraded_covariance_streaming(self):
        current = self.odometry(10.1)
        current.pose.covariance[7] = 100000.0
        current.pose.covariance[35] = 1000.0
        self.assertEqual(
            odometry_state_guard_reason(
                current,
                stop_on_excessive_covariance=False,
            ),
            "ok",
        )

    def test_degraded_covariance_mode_still_rejects_physical_jump(self):
        previous = self.odometry(10.0)
        current = self.odometry(10.1, position=(8.0, 0.0, 0.0))
        current.pose.covariance[0] = 100000.0
        self.assertEqual(
            odometry_state_guard_reason(
                current,
                previous,
                stop_on_excessive_covariance=False,
            ),
            "position_jump_exceeds_limit",
        )

    def test_state_guard_rejects_position_and_orientation_jumps(self):
        previous = self.odometry(10.0)
        position_jump = self.odometry(10.1, position=(8.0, -2.0, 0.0))
        self.assertEqual(
            odometry_state_guard_reason(position_jump, previous),
            "position_jump_exceeds_limit",
        )
        orientation_jump = self.odometry(10.1, yaw=2.0)
        self.assertEqual(
            odometry_state_guard_reason(orientation_jump, previous),
            "orientation_jump_exceeds_limit",
        )

    def test_state_guard_rejects_nonmonotonic_timestamp(self):
        previous = self.odometry(10.0)
        current = self.odometry(10.0)
        self.assertEqual(
            odometry_state_guard_reason(current, previous),
            "nonmonotonic_state_timestamp",
        )


if __name__ == "__main__":
    unittest.main()
