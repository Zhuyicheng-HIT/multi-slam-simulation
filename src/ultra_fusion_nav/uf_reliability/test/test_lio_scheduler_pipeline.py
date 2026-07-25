import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from uf_interfaces.msg import LioDiagnostics, ReliabilityScore, SchedulerState

from uf_reliability.reliability_monitor import ReliabilityMonitor
from uf_reliability.reliability_scheduler import ReliabilityScheduler


class LioSchedulerHarness(Node):
    def __init__(self):
        super().__init__("lio_scheduler_harness")
        self.lio_pub = self.create_publisher(LioDiagnostics, "/lio/diagnostics", 20)
        self.latest_score = None
        self.latest_state = None
        self.create_subscription(
            ReliabilityScore,
            "/reliability/lidar_score",
            self._score,
            20,
        )
        self.create_subscription(
            SchedulerState,
            "/reliability/scheduler_state",
            self._state,
            20,
        )

    def _score(self, msg):
        self.latest_score = msg

    def _state(self, msg):
        self.latest_state = msg


def diagnostic(degraded=False):
    msg = LioDiagnostics()
    msg.input_points = 1200
    if degraded:
        msg.matched_points = 80
        msg.hessian_eigenvalues = [1.0e-8, 1.0e-8, 1.0e-6, 10.0, 20.0, 30.0]
        msg.normal_covariance_eigenvalues = [0.0, 0.0, 1.0]
        msg.axial_penalty = 0.8
    else:
        msg.matched_points = 1000
        msg.hessian_eigenvalues = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        msg.normal_covariance_eigenvalues = [0.1, 0.2, 0.7]
        msg.axial_penalty = 0.0
    msg.approximate = True
    msg.source = "test_external_geometry"
    return msg


class LioSchedulerPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.monitor = ReliabilityMonitor()
        self.scheduler = ReliabilityScheduler(parameter_overrides=[
            Parameter("transition_dwell_s", value=0.0),
            Parameter("recovery_dwell_s", value=0.0),
            Parameter("publish_rate_hz", value=30.0),
        ])
        self.harness = LioSchedulerHarness()
        self.executor = SingleThreadedExecutor()
        for node in (self.monitor, self.scheduler, self.harness):
            self.executor.add_node(node)

    def tearDown(self):
        for node in (self.harness, self.scheduler, self.monitor):
            self.executor.remove_node(node)
            node.destroy_node()

    def drive(self, msg, duration_s=0.35):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.harness.lio_pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.02)
        for _ in range(8):
            self.executor.spin_once(timeout_sec=0.02)

    def test_lio_diagnostics_control_lidar_factor_without_requiring_other_modalities(self):
        self.drive(diagnostic(degraded=False))
        self.assertIsNotNone(self.harness.latest_score)
        self.assertTrue(self.harness.latest_score.valid)
        self.assertLess(self.harness.latest_score.degradation_score, 0.3)
        self.assertEqual(self.harness.latest_score.observation_count, 1000)
        self.assertEqual(self.harness.latest_score.minimum_observation_count, 50)
        self.assertIn("score_complete", self.harness.latest_score.evidence_names)

        healthy = self.harness.latest_state
        self.assertIsNotNone(healthy)
        lidar_index = list(healthy.modality_names).index("lidar")
        self.assertEqual(healthy.health_state, "DEGRADED")
        self.assertTrue(healthy.factor_enabled[lidar_index])

        self.drive(diagnostic(degraded=True))
        failed = self.harness.latest_state
        self.assertGreater(self.harness.latest_score.degradation_score, 0.8)
        self.assertEqual(failed.health_state, "FAILSAFE")
        self.assertFalse(failed.factor_enabled[lidar_index])
        self.assertEqual(failed.covariance_inflation[lidar_index], 20.0)


if __name__ == "__main__":
    unittest.main()
