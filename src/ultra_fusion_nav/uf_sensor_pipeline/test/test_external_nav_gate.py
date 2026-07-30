import unittest
import math

from nav_msgs.msg import Odometry
from uf_sensor_pipeline.external_nav_gate import (
    capability_support_allowed,
    propagate_odometry,
    scheduler_state_allowed,
)


class SchedulerStateGateTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
