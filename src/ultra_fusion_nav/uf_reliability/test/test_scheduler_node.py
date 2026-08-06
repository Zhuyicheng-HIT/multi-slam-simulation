import os
import time
import unittest
from types import SimpleNamespace

# ROS node tests publish production topic names. Keep them off the default
# simulation domain even when pytest is run concurrently with a flight.
os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "UF_TEST_ROS_DOMAIN_ID", str(200 + os.getpid() % 30)
)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from uf_interfaces.msg import (
    FusionEpoch,
    ReliabilityScore,
    RelocalizationResult,
    SchedulerState,
)

from uf_reliability.reliability_scheduler import (
    ReliabilityScheduler,
    automatic_relocalization_trigger,
)
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
        self.request_history = []
        self.create_subscription(
            SchedulerState,
            "/reliability/scheduler_state",
            self._state,
            20,
        )
        self.create_subscription(
            Bool,
            "/relocalization/request",
            lambda msg: self.request_history.append(bool(msg.data)),
            10,
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
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.harness.publish_scores(values)
                self.harness.relocalization_pub.publish(request)
                next_publish = now + 0.05
            self.executor.spin_once(timeout_sec=0.005)
        drain_deadline = time.monotonic() + 0.10
        while time.monotonic() < drain_deadline:
            self.executor.spin_once(timeout_sec=0.005)
        return self.harness.latest

    def drive_until_state(
        self, values, expected_state, timeout_s=1.0, relocalization=False
    ):
        request = Bool()
        request.data = bool(relocalization)
        deadline = time.monotonic() + timeout_s
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.harness.publish_scores(values)
                self.harness.relocalization_pub.publish(request)
                next_publish = now + 0.05
            self.executor.spin_once(timeout_sec=0.005)
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

    def test_automatic_relocalization_requires_persistent_lidar_loss(self):
        trigger, since = automatic_relocalization_trigger(
            0.90, False, 10.0, None, None, 1.0, 15.0, 0.85
        )
        self.assertFalse(trigger)
        self.assertEqual(since, 10.0)
        trigger, since = automatic_relocalization_trigger(
            0.90, False, 11.1, since, None, 1.0, 15.0, 0.85
        )
        self.assertTrue(trigger)
        trigger, _ = automatic_relocalization_trigger(
            0.90, False, 12.0, since, 11.1, 1.0, 15.0, 0.85
        )
        self.assertFalse(trigger)
        trigger, since = automatic_relocalization_trigger(
            0.20, True, 13.0, since, 11.1, 1.0, 15.0, 0.85
        )
        self.assertFalse(trigger)
        self.assertIsNone(since)

    def test_automatic_relocalization_requires_global_position_support_loss(self):
        trigger, since = automatic_relocalization_trigger(
            1.0, False, 10.0, None, None, 1.0, 15.0, 0.85,
            horizontal_position_supported=True,
            evidence_count=10,
            minimum_evidence_count=3,
        )
        self.assertFalse(trigger)
        self.assertIsNone(since)

    def test_automatic_relocalization_counts_fresh_lidar_evidence(self):
        trigger, since = automatic_relocalization_trigger(
            1.0, False, 10.0, None, None, 1.0, 15.0, 0.85,
            evidence_count=1,
            minimum_evidence_count=3,
        )
        self.assertFalse(trigger)
        self.assertEqual(since, 10.0)
        trigger, since = automatic_relocalization_trigger(
            1.0, False, 11.1, since, None, 1.0, 15.0, 0.85,
            evidence_count=3,
            minimum_evidence_count=3,
        )
        self.assertTrue(trigger)

    def test_scheduler_suppresses_lidar_relocalization_when_gnss_supports_pose(self):
        self.scheduler.relocalization_ready = True
        self.scheduler.first_lidar_score_s = 1.0
        result = SimpleNamespace(
            degradation_scores={"lidar": 1.0},
            factor_enabled={"lidar": False},
            capability_support={"horizontal_position": 0.99},
        )
        for index in range(5):
            self.scheduler.scores["lidar"] = {"arrival_s": 20.0 + index}
            self.scheduler._maybe_request_relocalization(result, 20.0 + index)

        self.assertFalse(self.scheduler.relocalization_requested)
        self.assertEqual(self.scheduler.automatic_relocalization_requests, 0)
        self.assertEqual(self.scheduler.relocalization_lidar_observations, 0)

        result.capability_support["horizontal_position"] = 0.0
        for index in range(3):
            now = 30.0 + index * 0.6
            self.scheduler.scores["lidar"] = {"arrival_s": now}
            self.scheduler._maybe_request_relocalization(result, now)

        self.assertTrue(self.scheduler.relocalization_requested)
        self.assertEqual(self.scheduler.automatic_relocalization_requests, 1)

    def test_registration_candidate_waits_for_backend_epoch_commit(self):
        request = Bool()
        request.data = True
        self.scheduler._relocalization(request)
        self.assertTrue(self.scheduler.relocalization_requested)

        candidate = RelocalizationResult()
        candidate.state = RelocalizationResult.SUCCESS
        candidate.accepted = True
        candidate.transaction_id = 100
        candidate.candidate_id = 7
        self.scheduler._relocalization_result(candidate)
        self.assertTrue(self.scheduler.relocalization_requested)
        self.assertTrue(self.scheduler.relocalization_candidate_accepted)

        stale_epoch = FusionEpoch()
        stale_epoch.applied = True
        stale_epoch.session_id = 10
        stale_epoch.transaction_id = 99
        stale_epoch.reset_counter = 1
        stale_epoch.candidate_id = 7
        self.scheduler._fusion_epoch(stale_epoch)
        self.assertTrue(self.scheduler.relocalization_requested)

        committed_epoch = FusionEpoch()
        committed_epoch.applied = True
        committed_epoch.session_id = 10
        committed_epoch.transaction_id = 100
        committed_epoch.reset_counter = 2
        committed_epoch.candidate_id = 7
        self.harness.request_history.clear()
        self.scheduler._fusion_epoch(committed_epoch)
        for _ in range(4):
            self.executor.spin_once(timeout_sec=0.01)
        self.assertFalse(self.scheduler.relocalization_requested)
        self.assertFalse(self.scheduler.relocalization_failed)
        self.assertEqual(self.scheduler.relocalization_commits, 1)
        self.assertIn(False, self.harness.request_history)

    def test_database_not_ready_is_recovery_failure_not_estimator_failure(self):
        healthy = self.drive({}, 0.20)
        self.assertEqual(healthy.health_state, "NORMAL")

        failed = RelocalizationResult()
        failed.state = RelocalizationResult.FAILED
        failed.reason = "database_not_ready"
        self.harness.request_history.clear()
        self.scheduler._relocalization_result(failed)
        for _ in range(4):
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(self.scheduler.relocalization_failed)
        self.assertEqual(self.scheduler.relocalization_failures, 1)
        self.assertIn(False, self.harness.request_history)

        after = self.drive({}, 0.20)
        self.assertNotEqual(after.health_state, "FAILSAFE")
        self.assertTrue(all(
            after.capability_observable[
                list(after.capability_names).index(name)
            ]
            for name in ("propagation", "horizontal_motion", "yaw_tracking")
        ))

    def test_relocalization_readiness_resets_automatic_trigger_dwell(self):
        self.scheduler.relocalization_candidate_since_s = 10.0
        self.scheduler.relocalization_lidar_observations = 4
        self.scheduler.last_relocalization_lidar_score_s = 9.0
        ready = Bool()
        ready.data = False
        self.scheduler._relocalization_ready(ready)
        self.assertFalse(self.scheduler.relocalization_ready)
        self.assertIsNone(self.scheduler.relocalization_candidate_since_s)
        self.assertEqual(self.scheduler.relocalization_lidar_observations, 0)
        self.assertIsNone(self.scheduler.last_relocalization_lidar_score_s)
        ready.data = True
        self.scheduler._relocalization_ready(ready)
        self.assertTrue(self.scheduler.relocalization_ready)

    def test_epoch_before_candidate_is_correlated_by_transaction(self):
        request = Bool()
        request.data = True
        self.scheduler._relocalization(request)

        epoch = FusionEpoch()
        epoch.applied = True
        epoch.session_id = 11
        epoch.transaction_id = 200
        epoch.reset_counter = 1
        epoch.candidate_id = 8
        self.scheduler._fusion_epoch(epoch)
        self.assertTrue(self.scheduler.relocalization_requested)

        candidate = RelocalizationResult()
        candidate.state = RelocalizationResult.SUCCESS
        candidate.accepted = True
        candidate.transaction_id = 200
        candidate.candidate_id = 8
        self.scheduler._relocalization_result(candidate)

        self.assertFalse(self.scheduler.relocalization_requested)
        self.assertFalse(self.scheduler.relocalization_failed)
        self.assertEqual(self.scheduler.relocalization_commits, 1)

    def test_committed_fact_overrides_local_timeout_state(self):
        request = Bool()
        request.data = True
        self.scheduler._relocalization(request)
        candidate = RelocalizationResult()
        candidate.state = RelocalizationResult.SUCCESS
        candidate.accepted = True
        candidate.transaction_id = 300
        candidate.candidate_id = 9
        self.scheduler._relocalization_result(candidate)
        self.scheduler._expire_relocalization_commit(
            self.scheduler.relocalization_commit_deadline_s + 0.01
        )
        self.assertTrue(self.scheduler.relocalization_failed)

        epoch = FusionEpoch()
        epoch.applied = True
        epoch.session_id = 12
        epoch.transaction_id = 300
        epoch.reset_counter = 1
        epoch.candidate_id = 9
        self.scheduler._fusion_epoch(epoch)

        self.assertFalse(self.scheduler.relocalization_requested)
        self.assertFalse(self.scheduler.relocalization_failed)
        self.assertEqual(self.scheduler.relocalization_commits, 1)


if __name__ == "__main__":
    unittest.main()
