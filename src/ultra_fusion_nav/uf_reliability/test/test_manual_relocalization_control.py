import os
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "UF_TEST_ROS_DOMAIN_ID", str(150 + os.getpid() % 20)
)

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from uf_interfaces.srv import ManualRelocalization  # noqa: E402

from uf_reliability.manual_relocalization_control import (  # noqa: E402
    ManualRelocalizationControl,
)
from uf_reliability.relocalization_request_arbiter_node import (  # noqa: E402
    RelocalizationRequestArbiter,
)


class ManualControlHarness(Node):
    def __init__(self):
        super().__init__("manual_relocalization_control_harness")
        self.request_edges = []
        self.create_subscription(
            Bool,
            "/relocalization/request",
            lambda message: self.request_edges.append(bool(message.data)),
            10,
        )
        self.client = self.create_client(
            ManualRelocalization, "/relocalization/manual_control"
        )

    def call(self, command, *, episode=1, lease=0.4, stamp=None):
        request = ManualRelocalization.Request()
        request.command = command
        request.source = "manual_control"
        request.episode_id = episode
        request.lease_duration_s = lease
        if stamp is not None:
            request.timestamp = stamp
        return self.client.call_async(request)


class ManualRelocalizationControlRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.control = ManualRelocalizationControl()
        self.arbiter = RelocalizationRequestArbiter()
        self.harness = ManualControlHarness()
        self.executor = SingleThreadedExecutor()
        for node in (self.control, self.arbiter, self.harness):
            self.executor.add_node(node)
        self.assertTrue(self.harness.client.wait_for_service(timeout_sec=1.0))
        self.spin(0.08)

    def tearDown(self):
        for node in (self.harness, self.arbiter, self.control):
            self.executor.remove_node(node)
            node.destroy_node()

    def spin(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.005)

    def result(self, future):
        while not future.done():
            self.executor.spin_once(timeout_sec=0.005)
        return future.result()

    def test_start_duplicate_cancel_has_one_final_publisher(self):
        self.assertEqual(self.harness.count_publishers("/relocalization/request"), 1)
        started = self.result(
            self.harness.call(ManualRelocalization.Request.START, episode=77)
        )
        self.assertTrue(started.accepted)
        self.spin(0.06)
        self.assertEqual(self.harness.request_edges[-1], True)

        duplicate = self.result(
            self.harness.call(ManualRelocalization.Request.START, episode=77)
        )
        self.assertFalse(duplicate.accepted)
        self.assertEqual(duplicate.reason, "already_active")
        self.spin(0.06)
        self.assertEqual(self.harness.request_edges.count(True), 1)

        cancelled = self.result(self.harness.call(ManualRelocalization.Request.CANCEL))
        self.assertTrue(cancelled.accepted)
        self.spin(0.06)
        self.assertEqual(self.harness.request_edges[-1], False)
        self.assertEqual(self.harness.request_edges[:2], [True, False])

    def test_stale_timestamp_is_rejected_without_final_request(self):
        stale = Time(sec=1, nanosec=0)
        response = self.result(
            self.harness.call(ManualRelocalization.Request.START, stamp=stale)
        )
        self.assertFalse(response.accepted)
        self.assertEqual(response.reason, "stale_or_future_timestamp")
        self.spin(0.05)
        self.assertNotIn(True, self.harness.request_edges)

    def test_timestamp_regression_is_rejected_fail_closed(self):
        started = self.result(
            self.harness.call(ManualRelocalization.Request.START)
        )
        self.assertTrue(started.accepted)
        last = self.control._last_stamp_s
        regressed = Time()
        regressed.sec = int(last)
        regressed.nanosec = int((last - regressed.sec) * 1.0e9) - 1_000_000
        if regressed.nanosec < 0:
            regressed.sec -= 1
            regressed.nanosec += 1_000_000_000
        cancelled = self.result(
            self.harness.call(ManualRelocalization.Request.CANCEL, stamp=regressed)
        )
        self.assertFalse(cancelled.accepted)
        self.assertEqual(cancelled.reason, "timestamp_regression")
        self.spin(0.05)
        self.assertEqual(self.harness.request_edges[-1], True)

    def test_source_loss_expires_lease_fail_closed(self):
        response = self.result(
            self.harness.call(ManualRelocalization.Request.START, lease=0.2)
        )
        self.assertTrue(response.accepted)
        self.spin(0.06)
        self.assertEqual(self.harness.request_edges[-1], True)
        self.control._timer.cancel()  # Simulates loss of the sole manual source.
        self.spin(0.30)
        self.assertEqual(self.harness.request_edges[-1], False)
        self.assertGreaterEqual(self.arbiter.core.expired_leases, 1)


if __name__ == "__main__":
    unittest.main()
