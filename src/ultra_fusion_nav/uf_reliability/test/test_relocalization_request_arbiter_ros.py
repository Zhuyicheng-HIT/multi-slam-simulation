import os
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "UF_TEST_ROS_DOMAIN_ID", str(170 + os.getpid() % 20)
)

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from uf_interfaces.msg import RelocalizationRequestIntent  # noqa: E402

from uf_reliability.relocalization_request_arbiter_node import (  # noqa: E402
    RelocalizationRequestArbiter,
)


class IntentHarness(Node):
    def __init__(self):
        super().__init__("request_arbiter_harness")
        self.pub = self.create_publisher(
            RelocalizationRequestIntent, "/relocalization/request_intent", 10
        )
        self.history = []
        self.create_subscription(
            Bool,
            "/relocalization/request",
            lambda msg: self.history.append(bool(msg.data)),
            10,
        )
        self.sequences = {}

    def publish(self, source, active, lease=0.4):
        sequence = self.sequences.get(source, 0) + 1
        self.sequences[source] = sequence
        msg = RelocalizationRequestIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source_id = source
        msg.source_instance_id = source + "-instance"
        msg.sequence = sequence
        msg.episode_id = 1
        msg.active = bool(active)
        msg.lease_duration_s = float(lease)
        msg.reason = "ros_test"
        self.pub.publish(msg)


class RequestArbiterRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.arbiter = RelocalizationRequestArbiter()
        self.harness = IntentHarness()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.arbiter)
        self.executor.add_node(self.harness)
        self.spin(0.1)

    def tearDown(self):
        self.executor.remove_node(self.harness)
        self.executor.remove_node(self.arbiter)
        self.harness.destroy_node()
        self.arbiter.destroy_node()

    def spin(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.005)

    def test_single_publisher_and_or_owned_lifecycle(self):
        self.assertEqual(
            self.harness.count_publishers("/relocalization/request"), 1
        )
        self.harness.publish("reliability_scheduler", True)
        self.spin(0.08)
        self.assertTrue(self.harness.history[-1])
        self.harness.publish("localization_safety", True)
        self.spin(0.05)
        self.harness.publish("reliability_scheduler", False)
        self.spin(0.05)
        self.assertTrue(self.harness.history[-1])
        self.harness.publish("localization_safety", False)
        self.spin(0.05)
        self.assertFalse(self.harness.history[-1])
        self.assertEqual(self.harness.history.count(True), 1)

    def test_crashed_source_lease_expires(self):
        self.harness.publish("reliability_scheduler", True, lease=0.20)
        self.spin(0.08)
        self.assertTrue(self.harness.history[-1])
        self.spin(0.25)
        self.assertFalse(self.harness.history[-1])
        self.assertEqual(self.arbiter.core.expired_leases, 1)


if __name__ == "__main__":
    unittest.main()
