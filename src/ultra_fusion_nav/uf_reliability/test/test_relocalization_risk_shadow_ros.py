import time
import unittest

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from uf_interfaces.msg import SchedulerState

from uf_reliability.relocalization_risk_shadow_node import (
    RelocalizationRiskShadowNode,
)


class RelocalizationRiskShadowRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_diagnostics_only_contract(self):
        shadow = RelocalizationRiskShadowNode()
        harness = Node("relocalization_risk_shadow_test_harness")
        received = []
        publisher = harness.create_publisher(
            SchedulerState, "/reliability/scheduler_state", 20
        )
        harness.create_subscription(
            DiagnosticArray,
            "/relocalization/shadow_risk",
            received.append,
            10,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(shadow)
        executor.add_node(harness)
        try:
            message = SchedulerState()
            message.header.stamp = harness.get_clock().now().to_msg()
            message.health_state = "NORMAL"
            message.modality_names = [
                "lidar", "gnss", "imu", "optical_flow", "vision"
            ]
            message.degradation_scores = [0.1] * 5
            message.reliability_weights = [0.9] * 5
            message.covariance_inflation = [1.0] * 5
            message.factor_enabled = [True] * 5
            message.reasons = [""] * 5
            message.capability_names = [
                "propagation", "horizontal_position", "horizontal_motion",
                "vertical_position", "yaw_tracking",
            ]
            message.capability_support = [0.9] * 5
            message.capability_observable = [True] * 5
            message.estimator_support = 0.9
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not received:
                publisher.publish(message)
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(received)
            values = {
                item.key: item.value
                for item in received[-1].status[0].values
            }
            self.assertEqual(values["shadow_only"], "True")
            self.assertEqual(values["level_name"], "NORMAL")
            request_publishers = harness.get_publishers_info_by_topic(
                "/relocalization/request"
            )
            self.assertFalse(any(
                endpoint.node_name == "relocalization_risk_shadow"
                for endpoint in request_publishers
            ))
        finally:
            executor.remove_node(harness)
            executor.remove_node(shadow)
            harness.destroy_node()
            shadow.destroy_node()


if __name__ == "__main__":
    unittest.main()
