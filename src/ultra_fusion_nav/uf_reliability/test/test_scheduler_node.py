import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from uf_interfaces.msg import ReliabilityScore, SchedulerState

from uf_reliability.reliability_scheduler import ReliabilityScheduler
from uf_reliability.scheduler_core import MODALITIES


class SchedulerHarness(Node):
    def __init__(self):
        super().__init__("scheduler_harness")
        self.score_pubs = {
            name: self.create_publisher(
                ReliabilityScore,
                f"/reliability/{name}_score",
                qos_profile_sensor_data,
            )
            for name in MODALITIES
        }
        self.relocalization_pub = self.create_publisher(
            Bool, "/relocalization/request", 10)
        self.latest = None
        self.history = []
        self.create_subscription(
            SchedulerState,
            "/reliability/scheduler_state",
            self._state,
            20,
        )

    def _state(self, msg):
        self.latest = msg
        self.history.append(msg.health_state)

    def publish_scores(self, values):
        for name in MODALITIES:
            msg = ReliabilityScore()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.modality = name
            msg.degradation_score = float(values.get(name, 0.1))
            msg.reliability_weight = 1.0 - msg.degradation_score
            msg.valid = True
            msg.observation_count = 1
            msg.minimum_observation_count = 1
            self.score_pubs[name].publish(msg)


class SchedulerNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        overrides = [
            Parameter(
                "active_modalities",
                value=["lidar", "gnss", "imu", "optical_flow"],
            ),
            Parameter("required_modalities", value=["imu"]),
            Parameter("minimum_usable_modalities", value=2),
            Parameter("score_timeout_s", value=1.0),
            Parameter("transition_dwell_s", value=0.0),
            Parameter("recovery_dwell_s", value=0.15),
            Parameter("recovered_hold_s", value=0.15),
            Parameter("publish_rate_hz", value=20.0),
        ]
        self.scheduler = ReliabilityScheduler(parameter_overrides=overrides)
        self.harness = SchedulerHarness()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.scheduler)
        self.executor.add_node(self.harness)

    def tearDown(self):
        self.executor.remove_node(self.harness)
        self.executor.remove_node(self.scheduler)
        self.harness.destroy_node()
        self.scheduler.destroy_node()

    def drive(self, values, duration_s, relocalization=False):
        request = Bool()
        request.data = bool(relocalization)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.harness.publish_scores(values)
            self.harness.relocalization_pub.publish(request)
            self.executor.spin_once(timeout_sec=0.02)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.02)
        return self.harness.latest

    def drive_until_state(
        self, values, expected_state, timeout_s=1.0, relocalization=False
    ):
        request = Bool()
        request.data = bool(relocalization)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.harness.publish_scores(values)
            self.harness.relocalization_pub.publish(request)
            self.executor.spin_once(timeout_sec=0.02)
            if (
                self.harness.latest is not None
                and self.harness.latest.health_state == expected_state
            ):
                return self.harness.latest
        self.fail(
            f"scheduler did not publish {expected_state} within {timeout_s:.1f}s; "
            f"last={getattr(self.harness.latest, 'health_state', None)}"
        )

    def test_runtime_state_and_factor_sequence(self):
        healthy = self.drive({}, 0.30)
        self.assertEqual(healthy.health_state, "NORMAL")
        active_indices = [
            list(healthy.modality_names).index(name)
            for name in ("lidar", "gnss", "imu", "optical_flow")
        ]
        self.assertTrue(all(healthy.factor_enabled[index] for index in active_indices))
        vision_index = list(healthy.modality_names).index("vision")
        self.assertFalse(healthy.factor_enabled[vision_index])
        self.assertEqual(healthy.degradation_scores[vision_index], 0.0)
        self.assertEqual(healthy.reasons[vision_index], "inactive_modality")

        risk = self.drive({"gnss": 0.70}, 0.25)
        self.assertEqual(risk.health_state, "DEGRADED")
        gnss_index = list(risk.modality_names).index("gnss")
        self.assertTrue(risk.factor_enabled[gnss_index])
        self.assertGreater(risk.covariance_inflation[gnss_index], 3.0)

        high_dynamic = self.drive({"imu": 0.90}, 0.25)
        self.assertEqual(high_dynamic.health_state, "RISK")
        imu_index = list(high_dynamic.modality_names).index("imu")
        self.assertTrue(high_dynamic.factor_enabled[imu_index])
        self.assertAlmostEqual(
            high_dynamic.covariance_inflation[imu_index], 5.0, places=4)

        recovered = self.drive_until_state({}, "RECOVERED")
        self.assertEqual(recovered.health_state, "RECOVERED")
        normal = self.drive_until_state({}, "NORMAL")
        self.assertEqual(normal.health_state, "NORMAL")
        self.assertTrue(normal.factor_enabled[gnss_index])

        relocalizing = self.drive({}, 0.20, relocalization=True)
        self.assertEqual(relocalizing.health_state, "RELOCALIZING")


if __name__ == "__main__":
    unittest.main()
