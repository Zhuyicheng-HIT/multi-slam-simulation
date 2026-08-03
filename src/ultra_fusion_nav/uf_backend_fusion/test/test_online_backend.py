import math
import queue
import threading
import unittest
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from uf_backend_fusion.imu_preintegration import ImuSample
from uf_backend_fusion.native_lidar import rpy_to_rotation_matrix
from uf_backend_fusion.online_backend import (
    apply_flow_rotation_gate,
    apply_lidar_anchor_floor,
    covariance_update_due,
    flow_observation_delta,
    frd_to_enu_delta,
    fused_motion_reference,
    gnss_jump_rejected,
    gnss_covariance_diagonal,
    gnss_temporal_jump_rejected,
    imu_interval_covered,
    imu_interval_status,
    estimate_stationary_imu_bias,
    enqueue_latest,
    inflate_manifold_imu_covariance,
    lidar_bypass_allowed,
    lidar_prediction_innovation,
    manifold_motion_reference,
    native_frame_odometry,
    native_trigger_order_status,
    path_sample_due,
    scheduler_decision,
    select_gnss_observation,
    unwrap_yaw,
    UnifiedBackendNode,
    validate_optimized_state,
    yaw_to_quaternion,
)
from uf_reliability.flow_rotation_gate import FlowRotationGateResult


class OnlineBackendHelpersTest(unittest.TestCase):
    @staticmethod
    def _integrity_limits():
        return {
            "maximum_translation_correction_m": 1.0,
            "maximum_rotation_correction_rad": 0.5,
            "maximum_velocity_correction_mps": 5.0,
            "maximum_accel_bias_correction_mps2": 1.5,
            "maximum_gyro_bias_correction_radps": 0.3,
            "maximum_information_condition": 1.0e12,
            "information_rank_tolerance": 1.0e-9,
        }

    def test_optimization_integrity_accepts_finite_cost_reducing_state(self):
        initial = np.zeros(15)
        estimate = initial.copy()
        estimate[0] = 0.1
        result = validate_optimized_state(
            initial, estimate, np.eye(15), 10.0, 5.0,
            **self._integrity_limits(),
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.reason, "ok")
        self.assertEqual(result.latest_information_rank, 15)
        self.assertAlmostEqual(result.translation_correction_m, 0.1)

    def test_optimization_integrity_rejects_excessive_translation(self):
        initial = np.zeros(15)
        estimate = initial.copy()
        estimate[0] = 1.01
        result = validate_optimized_state(
            initial, estimate, np.eye(15), 10.0, 5.0,
            **self._integrity_limits(),
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "excessive_translation_correction")

    def test_optimization_integrity_rejects_indefinite_information(self):
        information = np.eye(15)
        information[0, 0] = -1.0
        result = validate_optimized_state(
            np.zeros(15), np.zeros(15), information, 10.0, 5.0,
            **self._integrity_limits(),
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "indefinite_latest_information")

    def test_optimization_integrity_rejects_cost_increase(self):
        result = validate_optimized_state(
            np.zeros(15), np.zeros(15), np.eye(15), 5.0, 5.1,
            **self._integrity_limits(),
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "optimization_cost_increased")

    def test_path_sampling_requires_motion_or_rotation(self):
        self.assertTrue(path_sample_due(
            None, None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.05, 0.02
        ))
        self.assertFalse(path_sample_due(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
            [0.01, 0.0, 0.0], [0.0, 0.0, 0.01], 0.05, 0.02
        ))
        self.assertTrue(path_sample_due(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
            [0.05, 0.0, 0.0], [0.0, 0.0, 0.0], 0.05, 0.02
        ))
        self.assertTrue(path_sample_due(
            [0.0, 0.0, 0.0], [0.0, 0.0, math.pi - 0.01],
            [0.0, 0.0, 0.0], [0.0, 0.0, -math.pi + 0.02], 0.05, 0.02
        ))

    def test_marginal_covariance_update_is_rate_limited_and_handles_reset(self):
        self.assertTrue(covariance_update_due(None, 10.0, 1.0))
        self.assertFalse(covariance_update_due(10.0, 10.9, 1.0))
        self.assertTrue(covariance_update_due(10.0, 11.0, 1.0))
        self.assertTrue(covariance_update_due(10.0, 9.0, 1.0))

    def test_unknown_gnss_covariance_uses_conservative_default(self):
        covariance = gnss_covariance_diagonal(
            np.zeros(9),
            covariance_type=0,
            default_variance=4.0,
        )

        np.testing.assert_allclose(covariance, [4.0, 4.0, 4.0])

    def test_known_gnss_covariance_keeps_valid_diagonal_and_floor(self):
        raw = np.zeros(9)
        raw[[0, 4, 8]] = [0.01, 0.25, 1.0]

        covariance = gnss_covariance_diagonal(
            raw,
            covariance_type=2,
            default_variance=4.0,
        )

        np.testing.assert_allclose(covariance, [0.04, 0.25, 1.0])

    def test_gnss_observation_is_consumed_once(self):
        observations = [
            {"stamp_s": 10.0, "id": "old"},
            {"stamp_s": 10.1, "id": "selected"},
            {"stamp_s": 10.4, "id": "future"},
        ]

        selected, stale, superseded = select_gnss_observation(
            observations, 10.12, maximum_age_s=2.0, future_tolerance_s=0.05
        )

        self.assertEqual(selected["id"], "selected")
        self.assertEqual(stale, 0)
        self.assertEqual(superseded, 1)
        self.assertEqual([item["id"] for item in observations], ["future"])

        selected, stale, superseded = select_gnss_observation(
            observations, 10.12, maximum_age_s=2.0, future_tolerance_s=0.05
        )
        self.assertIsNone(selected)
        self.assertEqual(stale, 0)
        self.assertEqual(superseded, 0)
        self.assertEqual([item["id"] for item in observations], ["future"])

    def test_gnss_observation_discards_stale_fix(self):
        observations = [{"stamp_s": 5.0}, {"stamp_s": 9.9}]

        selected, stale, superseded = select_gnss_observation(
            observations, 10.0, maximum_age_s=1.0, future_tolerance_s=0.0
        )

        self.assertEqual(selected["stamp_s"], 9.9)
        self.assertEqual(stale, 1)
        self.assertEqual(superseded, 0)
        self.assertEqual(observations, [])

    def test_full_imu_covariance_inflation_preserves_correlation_and_spd(self):
        covariance = np.eye(15)
        covariance[0, 3] = 0.4
        covariance[3, 0] = 0.4
        covariance[0, 9] = -0.2
        covariance[9, 0] = -0.2

        inflated = inflate_manifold_imu_covariance(
            covariance, motion_scale=25.0, minimum_bias_variance=2.0
        )

        self.assertAlmostEqual(inflated[0, 0], 25.0)
        self.assertAlmostEqual(inflated[0, 3], 10.0)
        self.assertAlmostEqual(inflated[0, 9], -1.0)
        self.assertAlmostEqual(inflated[9, 9], 2.0)
        self.assertGreater(float(np.min(np.linalg.eigvalsh(inflated))), 0.0)

    @staticmethod
    def _stationary_samples(
        orientation=(0.0, 0.0, 0.0),
        accel_bias=(0.12, -0.08, 0.20),
        gyro_bias=(0.01, -0.02, 0.005),
    ):
        expected_force = (
            rpy_to_rotation_matrix(orientation).T
            @ np.asarray([0.0, 0.0, 9.81])
        )
        acceleration = expected_force + np.asarray(accel_bias)
        return [
            ImuSample(
                stamp_s=index * 0.01,
                acceleration=tuple(acceleration),
                angular_velocity=tuple(gyro_bias),
            )
            for index in range(151)
        ]

    def test_stationary_fcu_imu_seeds_bias_in_lidar_orientation(self):
        orientation = (0.25, -0.18, 0.7)
        result = estimate_stationary_imu_bias(
            self._stationary_samples(orientation=orientation),
            orientation,
            end_stamp_s=1.5,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.reason, "ok")
        np.testing.assert_allclose(result.accel_bias, [0.12, -0.08, 0.20])
        np.testing.assert_allclose(result.gyro_bias, [0.01, -0.02, 0.005])
        self.assertEqual(result.sample_count, 151)
        self.assertAlmostEqual(result.span_s, 1.5)

    def test_startup_bias_rejects_rotation_and_acceleration_variation(self):
        rotating = [
            ImuSample(sample.stamp_s, sample.acceleration, (0.0, 0.0, 0.2))
            for sample in self._stationary_samples()
        ]
        rotation_result = estimate_stationary_imu_bias(
            rotating, (0.0, 0.0, 0.0), end_stamp_s=1.5
        )
        self.assertFalse(rotation_result.valid)
        self.assertEqual(
            rotation_result.reason, "mean_angular_rate_exceeds_limit"
        )

        vibrating = []
        for index, sample in enumerate(self._stationary_samples()):
            acceleration = np.asarray(sample.acceleration)
            acceleration[0] += 1.0 if index % 2 else -1.0
            vibrating.append(ImuSample(
                sample.stamp_s, tuple(acceleration), sample.angular_velocity
            ))
        vibration_result = estimate_stationary_imu_bias(
            vibrating, (0.0, 0.0, 0.0), end_stamp_s=1.5
        )
        self.assertFalse(vibration_result.valid)
        self.assertEqual(
            vibration_result.reason, "specific_force_variation_exceeds_limit"
        )

    def test_startup_bias_waits_for_observable_time_span(self):
        result = estimate_stationary_imu_bias(
            self._stationary_samples()[:30],
            (0.0, 0.0, 0.0),
            end_stamp_s=0.29,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "insufficient_observation_span")

    def test_native_factor_trigger_builds_same_frame_and_pose_header(self):
        header = Header()
        header.stamp.sec = 10
        header.stamp.nanosec = 125_000_000
        factor = SimpleNamespace(
            map_frame="camera_init",
            state_frame="body",
            linearization_pose=np.asarray([1.0, 2.0, 3.0, 0.1, -0.2, 0.3]),
        )

        odometry = native_frame_odometry(header, factor)

        self.assertEqual(odometry.header.stamp, header.stamp)
        self.assertEqual(odometry.header.frame_id, "camera_init")
        self.assertEqual(odometry.child_frame_id, "body")
        np.testing.assert_allclose([
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.position.z,
        ], [1.0, 2.0, 3.0])

    def test_native_factor_mode_ignores_perturbed_lio_odometry(self):
        dispatched = []
        owner = SimpleNamespace(
            input_trigger_mode="native_factor",
            counts={"lio_pose_inputs_ignored": 0},
            _dispatch_lio=lambda *args: dispatched.append(args),
        )
        nominal = Odometry()
        nominal.pose.pose.position.x = 1.0
        perturbed = Odometry()
        perturbed.pose.pose.position.x = 1001.0
        perturbed.pose.pose.position.y = -500.0

        UnifiedBackendNode._lio(owner, nominal)
        UnifiedBackendNode._lio(owner, perturbed)

        self.assertEqual(owner.counts["lio_pose_inputs_ignored"], 2)
        self.assertEqual(dispatched, [])

    def test_native_trigger_order_contract_is_explicit(self):
        self.assertEqual(
            native_trigger_order_status(None, None, 100, 7), ("accept", 0)
        )
        self.assertEqual(
            native_trigger_order_status(100, 7, 100, 7), ("duplicate", 0)
        )
        self.assertEqual(
            native_trigger_order_status(100, 7, 100, 8),
            ("sequence_conflict", 0),
        )
        self.assertEqual(
            native_trigger_order_status(100, 7, 99, 8), ("nonmonotonic", 0)
        )
        self.assertEqual(
            native_trigger_order_status(100, 7, 101, 6), ("sequence_reset", 0)
        )
        self.assertEqual(
            native_trigger_order_status(100, 7, 110, 10), ("accept", 2)
        )

    def test_native_worker_queue_coalesces_to_newest_frame(self):
        work_queue = queue.Queue(maxsize=2)
        work_queue.put_nowait("older")
        work_queue.put_nowait("stale")

        discarded = enqueue_latest(work_queue, "newest")

        self.assertEqual(discarded, 2)
        self.assertEqual(work_queue.get_nowait(), "newest")
        work_queue.task_done()

    def test_imu_interval_coverage_requires_sample_at_or_after_target(self):
        self.assertFalse(imu_interval_covered(None, 10.0))
        self.assertFalse(imu_interval_covered(9.999, 10.0))
        self.assertTrue(imu_interval_covered(10.0, 10.0))
        self.assertTrue(imu_interval_covered(10.01, 10.0))

    def test_imu_interval_status_sorts_arrivals_and_checks_internal_gaps(self):
        samples = [
            SimpleNamespace(stamp_s=10.10),
            SimpleNamespace(stamp_s=9.98),
            SimpleNamespace(stamp_s=10.04),
            SimpleNamespace(stamp_s=10.00),
        ]
        ready, reason, maximum_gap = imu_interval_status(
            samples, 10.0, 10.10, maximum_gap_s=0.07
        )
        self.assertTrue(ready)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(maximum_gap, 0.06)

        ready, reason, maximum_gap = imu_interval_status(
            samples, 10.0, 10.10, maximum_gap_s=0.05
        )
        self.assertFalse(ready)
        self.assertEqual(reason, "sample_gap_exceeds_limit")
        self.assertAlmostEqual(maximum_gap, 0.06)

    def test_lidar_anchor_floor_prevents_unobservable_yaw_gap(self):
        decision = scheduler_decision(0.0, enabled=False, inflation=20.0)
        protected = apply_lidar_anchor_floor(
            decision, minimum_effective_weight=0.10, maximum_inflation=5.0
        )
        self.assertTrue(protected["factor_enabled"])
        self.assertEqual(protected["reliability_weight"], 0.5)
        self.assertEqual(protected["covariance_inflation"], 5.0)
        self.assertAlmostEqual(
            protected["reliability_weight"]
            / protected["covariance_inflation"],
            0.10,
        )
        self.assertTrue(protected["anchor_override"])

    def test_rotation_gate_caps_weight_and_inflates_covariance(self):
        decision = scheduler_decision(0.8, enabled=True, inflation=1.25)
        gate = FlowRotationGateResult(
            weight=0.5,
            hard_disabled=False,
            phase="TURNING",
            yaw_rate_abs_radps=0.19,
            translation_ready=True,
            reason="fcu_yaw_rate_downweight",
        )
        gated = apply_flow_rotation_gate(decision, gate)
        self.assertTrue(gated["factor_enabled"])
        self.assertEqual(gated["reliability_weight"], 0.5)
        self.assertEqual(gated["covariance_inflation"], 2.0)
        self.assertIn("fcu_yaw_rate_downweight", gated["reasons"])

    def test_rotation_gate_hard_disable_overrides_scheduler(self):
        decision = scheduler_decision(1.0, enabled=True, inflation=1.0)
        gate = FlowRotationGateResult(
            weight=0.0,
            hard_disabled=True,
            phase="TURNING",
            yaw_rate_abs_radps=0.4,
            translation_ready=True,
            reason="high_fcu_yaw_rate",
        )
        gated = apply_flow_rotation_gate(decision, gate)
        self.assertFalse(gated["factor_enabled"])
        self.assertEqual(gated["reliability_weight"], 0.0)
        self.assertEqual(gated["covariance_inflation"], 20.0)

    def test_frd_axis_conversion_is_explicit(self):
        self.assertEqual(frd_to_enu_delta(1.0, 0.0, 0.0), (1.0, 0.0))
        east, north = frd_to_enu_delta(0.0, 1.0, 0.0)
        self.assertAlmostEqual(east, 0.0)
        self.assertAlmostEqual(north, -1.0)

    def test_flow_aggregation_rejects_nonpositive_distance(self):
        observation = flow_observation_delta([
            {
                "integrated_x": 0.0,
                "integrated_y": 1.0,
                "integrated_xgyro": 0.0,
                "integrated_ygyro": 0.0,
                "quality": 200,
                "distance_m": 1.0,
            },
            {
                "integrated_x": 1.0,
                "integrated_y": 1.0,
                "integrated_xgyro": 0.0,
                "integrated_ygyro": 0.0,
                "quality": 0,
                "distance_m": 0.0,
            },
        ], 0.0)
        self.assertIsNotNone(observation)
        np.testing.assert_allclose(observation["delta_position"], [1.0, 0.0, 0.0])
        np.testing.assert_allclose(observation["delta_body"], [1.0, 0.0, 0.0])
        self.assertEqual(observation["sample_count"], 1)

    def test_scheduler_decision_can_disable_factor(self):
        decision = scheduler_decision(0.0, enabled=False, inflation=20.0)
        self.assertFalse(decision["factor_enabled"])
        self.assertEqual(decision["reliability_weight"], 0.0)
        self.assertEqual(decision["covariance_inflation"], 20.0)

    def test_lidar_bypass_requires_explicit_mode_and_live_imu_backup(self):
        self.assertTrue(lidar_bypass_allowed(False, True, True, True))
        self.assertFalse(lidar_bypass_allowed(True, True, True, True))
        self.assertFalse(lidar_bypass_allowed(False, False, True, True))
        self.assertFalse(lidar_bypass_allowed(False, True, False, True))
        self.assertFalse(lidar_bypass_allowed(False, True, True, False))

    def test_gnss_jump_gate_rejects_large_innovation(self):
        self.assertFalse(gnss_jump_rejected([1.0, 2.0, 0.0], [3.0, 4.0, 0.0]))
        self.assertTrue(gnss_jump_rejected([1.0, 2.0, 0.0], [30.0, 2.0, 0.0]))
        self.assertTrue(gnss_jump_rejected([1.0, 2.0, 0.0], [float("nan"), 2.0, 0.0]))

    def test_gnss_temporal_gate_is_independent_of_fused_lidar_state(self):
        self.assertFalse(
            gnss_temporal_jump_rejected(
                [0.0, 0.0, 0.0], 10.0,
                [1.0, 0.0, 0.0], 10.1,
                gate_m=20.0, maximum_speed_mps=15.0,
            )
        )
        self.assertTrue(
            gnss_temporal_jump_rejected(
                [0.0, 0.0, 0.0], 10.0,
                [50.0, 0.0, 0.0], 10.1,
                gate_m=20.0, maximum_speed_mps=15.0,
            )
        )

    def test_fused_motion_reference_does_not_use_current_lio(self):
        state = np.zeros(15)
        state[:3] = [1.0, 2.0, 3.0]
        state[5] = 0.4
        state[6:9] = [2.0, -1.0, 0.5]

        reference = fused_motion_reference(state, 0.2)

        np.testing.assert_allclose(reference["position"], [1.4, 1.8, 3.1])
        np.testing.assert_allclose(reference["delta_position"], [0.4, -0.2, 0.1])
        self.assertEqual(reference["yaw"], 0.4)

    def test_lidar_prediction_innovation_uses_lidar_free_reference(self):
        reference = {
            "position": np.asarray([1.0, 2.0, 3.0]),
            "yaw": 3.10,
        }
        innovation = lidar_prediction_innovation(
            [1.3, 2.4, 3.0], -3.12, reference,
        )

        self.assertAlmostEqual(innovation["position_m"], 0.5)
        self.assertLess(innovation["yaw_rad"], 0.1)

    def test_manifold_motion_reference_uses_backend_prediction(self):
        previous = np.zeros(15)
        previous[:3] = [1.0, 2.0, 3.0]
        predicted = previous.copy()
        predicted[:3] = [1.4, 1.8, 3.1]
        predicted[3:6] = [0.1, -0.2, 0.4]

        reference = manifold_motion_reference(previous, predicted)

        np.testing.assert_allclose(reference["position"], predicted[:3])
        np.testing.assert_allclose(reference["delta_position"], [0.4, -0.2, 0.1])
        np.testing.assert_allclose(reference["orientation"], predicted[3:6])
        self.assertEqual(reference["yaw"], 0.4)

    def test_yaw_quaternion_is_normalized(self):
        quaternion = yaw_to_quaternion(1.2)
        self.assertAlmostEqual(sum(value * value for value in quaternion), 1.0)

    def test_yaw_unwrap_crosses_branch_cut_without_a_large_jump(self):
        previous = 3.10
        current = -3.12
        unwrapped = unwrap_yaw(previous, current)
        self.assertAlmostEqual(unwrapped, 3.163185307179586, places=6)

    def test_rejected_native_frame_is_consumed_without_state_commit(self):
        node = object.__new__(UnifiedBackendNode)
        node.counts = {"native_consumed_without_state_commit": 0}
        node.last_native_consumed_sequence = 4
        released = []
        node._release_pending_scan_requests = lambda: released.append(True)

        node._consume_native_sequence(5, state_committed=False)

        self.assertEqual(node.last_native_consumed_sequence, 5)
        self.assertEqual(node.counts["native_consumed_without_state_commit"], 1)
        self.assertEqual(released, [True])

    def test_retried_deferred_scan_request_is_idempotent(self):
        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.last_native_consumed_sequence = -1
        node.pending_scan_request_lock = threading.Lock()
        node.pending_scan_requests = {2: SimpleNamespace(scan_sequence=2)}
        node.scan_prediction_by_sequence = {}
        node.counts = {
            "scan_prediction_requests": 0,
            "scan_prediction_duplicate_requests": 0,
            "scan_prediction_stale_requests": 0,
            "scan_prediction_deferred": 0,
        }

        node._scan_request(SimpleNamespace(scan_sequence=2))

        self.assertEqual(node.counts["scan_prediction_requests"], 1)
        self.assertEqual(node.counts["scan_prediction_duplicate_requests"], 1)
        self.assertEqual(node.counts["scan_prediction_deferred"], 1)
        self.assertEqual(list(node.pending_scan_requests), [2])


if __name__ == "__main__":
    unittest.main()
