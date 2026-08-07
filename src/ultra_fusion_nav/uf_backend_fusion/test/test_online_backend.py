import math
import queue
import threading
import unittest
from collections import deque
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from uf_backend_fusion.imu_preintegration import ImuSample, preintegrate_manifold
from uf_backend_fusion.live_propagation import make_optimization_anchor
from uf_backend_fusion.native_lidar import (
    NativeFactorBuffer,
    rpy_to_rotation_matrix,
)
from uf_backend_fusion.online_backend import (
    apply_flow_rotation_gate,
    apply_lidar_anchor_floor,
    associate_visual_states,
    covariance_update_due,
    flow_los_observation,
    flow_observation_delta,
    select_flow_records,
    frd_to_enu_delta,
    fused_motion_reference,
    gnss_jump_rejected,
    gnss_covariance_diagonal,
    gnss_temporal_jump_rejected,
    imu_interval_covered,
    imu_interval_status,
    estimate_stationary_imu_bias,
    enqueue_latest,
    reanchor_imu_samples,
    frontend_map_commit_decision,
    inflate_manifold_imu_covariance,
    lidar_bypass_allowed,
    lidar_calibration_motion_from_message,
    lidar_prediction_gate,
    lidar_prediction_innovation,
    manifold_motion_reference,
    native_factor_epoch_alignment,
    native_factor_epoch_barrier_required,
    native_factor_epoch_status,
    native_frame_odometry,
    native_trigger_order_status,
    path_sample_due,
    pose_vector_to_matrix,
    matrix_to_pose_vector,
    scheduler_decision,
    select_gnss_observation,
    retain_stamped_records_after,
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

    @staticmethod
    def _calibration_motion(**overrides):
        values = {
            "accepted": True,
            "converged": True,
            "provenance": 1,
            "imu_aided": False,
            "backend_aided": False,
            "rotation_convention": "R_L_previous_from_L_current",
            "start_stamp": SimpleNamespace(sec=10, nanosec=0),
            "header": SimpleNamespace(
                stamp=SimpleNamespace(sec=10, nanosec=200_000_000),
                frame_id="mid360_link",
            ),
            "inlier_ratio": 0.8,
            "fitness_score": 0.01,
            "residual_rms_m": 0.04,
            "rotation_information_condition": 5.0,
            "rotation_information_eigenvalues": [10.0, 20.0, 50.0],
            "relative_rotation": SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        }
        values.update(overrides)
        return SimpleNamespace(**values)

    def test_visual_association_waits_for_a_real_right_state(self):
        waiting = associate_visual_states(
            1.10, 1.28, [1.0, 1.1, 1.2], tolerance_s=0.065
        )
        self.assertEqual(waiting.status, "wait")
        self.assertEqual(waiting.missing_side, "right")
        associated = associate_visual_states(
            1.10, 1.28, [1.0, 1.1, 1.2, 1.3], tolerance_s=0.065
        )
        self.assertEqual(associated.status, "associated")
        self.assertEqual((associated.previous_index, associated.current_index), (1, 3))

    def test_visual_association_applies_td_c_without_retimestamping(self):
        association = associate_visual_states(
            0.98,
            1.08,
            [1.0, 1.1, 1.2],
            camera_to_imu_time_offset_s=0.02,
            tolerance_s=0.01,
        )
        self.assertEqual(association.status, "associated")
        self.assertAlmostEqual(association.corrected_previous_stamp_s, 1.0)
        self.assertAlmostEqual(association.corrected_current_stamp_s, 1.1)

    def test_visual_association_rejects_a_missing_left_state(self):
        association = associate_visual_states(
            0.70, 0.82, [1.0, 1.1, 1.2], tolerance_s=0.065
        )
        self.assertEqual(association.status, "reject")
        self.assertEqual(association.missing_side, "left")

    def test_backend_owned_native_factor_never_reapplies_map_alignment(self):
        alignment = np.eye(4)
        alignment[:3, 3] = [4.0, -2.0, 0.5]

        np.testing.assert_allclose(
            native_factor_epoch_alignment(alignment, True), np.eye(4)
        )
        np.testing.assert_allclose(
            native_factor_epoch_alignment(alignment, False), alignment
        )

    def test_native_factor_epoch_alignment_rejects_invalid_matrix(self):
        with self.assertRaisesRegex(ValueError, "finite 4x4"):
            native_factor_epoch_alignment(np.eye(3), True)

    def test_backend_owned_relocalization_requires_factor_epoch_barrier(self):
        self.assertTrue(native_factor_epoch_barrier_required(True, True))
        self.assertFalse(native_factor_epoch_barrier_required(False, True))
        self.assertFalse(native_factor_epoch_barrier_required(True, False))

    def test_native_factor_epoch_status_keeps_reset_barrier_first(self):
        self.assertEqual(native_factor_epoch_status(0, 1, True, True), "barrier")
        self.assertEqual(native_factor_epoch_status(0, 1, False, True), "stale")
        self.assertEqual(native_factor_epoch_status(2, 1, False, True), "future")
        self.assertEqual(native_factor_epoch_status(1, 1, False, True), "current")

    def test_process_lio_counts_barrier_before_stale_or_future_epoch(self):
        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.state_reset_counter = 1
        node.last_native_factor_reset_counter = 0
        node.counts = {
            "relocalization_epoch_factor_drops": 0,
            "native_lidar_epoch_stale_rejected": 0,
            "native_lidar_epoch_future_rejected": 0,
        }
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=10, nanosec=0)
            )
        )
        factor = SimpleNamespace(reset_counter=0)
        node._apply_pending_relocalization = lambda _stamp: True

        node._process_lio(message, factor)

        self.assertEqual(node.counts["relocalization_epoch_factor_drops"], 1)
        self.assertEqual(node.counts["native_lidar_epoch_stale_rejected"], 0)
        self.assertEqual(node.last_reason, "relocalization_epoch_barrier")

        node._apply_pending_relocalization = lambda _stamp: False
        node._process_lio(message, factor)
        self.assertEqual(node.counts["native_lidar_epoch_stale_rejected"], 1)
        self.assertEqual(node.last_reason, "native_lidar_stale_epoch")

        factor.reset_counter = 2
        node._process_lio(message, factor)
        self.assertEqual(node.counts["native_lidar_epoch_future_rejected"], 1)
        self.assertEqual(node.last_reason, "native_lidar_future_epoch")

    def test_calibration_motion_requires_raw_lidar_provenance(self):
        sample = lidar_calibration_motion_from_message(
            self._calibration_motion()
        )
        self.assertAlmostEqual(sample.start_s, 10.0)
        self.assertAlmostEqual(sample.end_s, 10.2)
        self.assertGreater(sample.weight, 0.0)
        np.testing.assert_allclose(sample.relative_rotation, np.eye(3))

        with self.assertRaisesRegex(ValueError, "independent"):
            lidar_calibration_motion_from_message(
                self._calibration_motion(backend_aided=True)
            )
        with self.assertRaisesRegex(ValueError, "provenance"):
            lidar_calibration_motion_from_message(
                self._calibration_motion(provenance=0)
            )

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

    def test_optimization_integrity_identifies_nonfinite_state_source(self):
        estimate = np.zeros(15)
        estimate[7] = math.nan
        result = validate_optimized_state(
            np.zeros(15), estimate, np.eye(15), 10.0, 5.0,
            **self._integrity_limits(),
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "nonfinite_optimized_state")

    def test_optimization_integrity_identifies_nonfinite_cost_source(self):
        result = validate_optimized_state(
            np.zeros(15), np.zeros(15), np.eye(15), 10.0, math.nan,
            **self._integrity_limits(),
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "nonfinite_final_cost")

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

    def test_optimization_integrity_accepts_reduced_negative_local_cost(self):
        result = validate_optimized_state(
            np.zeros(15), np.zeros(15), np.eye(15), -5.0, -5.1,
            **self._integrity_limits(),
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.reason, "ok")

    def test_optimization_integrity_rejects_negative_local_cost_increase(self):
        result = validate_optimized_state(
            np.zeros(15), np.zeros(15), np.eye(15), -5.1, -5.0,
            **self._integrity_limits(),
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "optimization_cost_increased")

    def test_live_propagation_publishes_only_odom_without_graph_mutation(self):
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        class BackendProbe:
            state_count = 3
            factor_count = 9

        node = object.__new__(UnifiedBackendNode)
        node.live_propagation_enabled = True
        node.live_propagation_lidar_silence_timeout_s = 0.25
        node.live_propagation_minimum_interval_s = 0.08
        node.live_propagation_maximum_imu_age_s = 0.20
        node.backend_solver_mode = "manifold"
        node.imu_factor_enabled = True
        node.native_worker_stop = threading.Event()
        node.optimization_anchor_lock = threading.Lock()
        node.output_lock = threading.Lock()
        node.state_publication_lock = threading.RLock()
        node.state_reset_counter = 0
        node.optimization_anchor_generation = 4
        node.optimization_anchor = make_optimization_anchor(
            9.70, np.zeros(15), np.eye(15) * 0.01, 4
        )
        node.last_unified_output_stamp_s = 9.70
        node.last_native_input_arrival_s = 9.50
        node.last_scan_request_arrival_s = None
        node.last_lio_stamp = 9.70
        node.last_calibration_update = SimpleNamespace(
            accepted=False, time_offset_s=0.0
        )
        node.online_calibration_enabled = False
        node.calibration_apply_locked_values = False
        node.map_frame = "map"
        node.body_frame = "base_link"
        node.odom_pub = Recorder()
        node.backend = BackendProbe()
        node.last_output = None
        node.last_output_source = "none"
        node.last_live_propagation_reason = "not_attempted"
        node.last_exception = "none"
        node.counts = {
            "published": 0,
            "optimized_odom_nonmonotonic_suppressed": 0,
            "optimized_odom_published": 0,
            "optimized_states_committed": 0,
            "live_propagation_attempts": 0,
            "live_propagation_published": 0,
            "live_propagation_rejected": 0,
        }
        samples = [
            ImuSample(stamp, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
            for stamp in (9.70, 9.84, 9.98)
        ]
        measurement = preintegrate_manifold(
            samples, 9.70, 9.98, max_gap_s=0.30
        )
        self.assertTrue(measurement.valid)
        node._imu_snapshot = lambda: samples
        node._now_s = lambda: 10.0
        node._live_imu_measurement = (
            lambda _anchor, _target, _samples: (measurement, "ok")
        )
        node._process_lio = lambda *_args: self.fail(
            "live propagation must not enter the optimizer"
        )

        node._publish_live_propagation()

        self.assertEqual(len(node.odom_pub.messages), 1)
        self.assertEqual(node.backend.state_count, 3)
        self.assertEqual(node.backend.factor_count, 9)
        self.assertEqual(node.last_lio_stamp, 9.70)
        self.assertEqual(node.optimization_anchor.generation, 4)
        self.assertEqual(node.counts["live_propagation_published"], 1)
        self.assertEqual(node.counts["optimized_odom_published"], 0)
        self.assertEqual(node.last_output_source, "imu_propagated")
        self.assertEqual(node.last_live_propagation_reason, "ok")

        # Force an optimizer commit between stale-anchor preintegration and
        # the final publication transaction. The derived old state must not
        # acquire a newer output timestamp after that commit.
        node.odom_pub.messages.clear()
        node.last_unified_output_stamp_s = 9.70
        node.last_output = None
        node.last_output_source = "none"
        measurement_ready = threading.Event()
        release_measurement = threading.Event()

        def delayed_measurement(_anchor, _target, _samples):
            measurement_ready.set()
            self.assertTrue(release_measurement.wait(timeout=1.0))
            return measurement, "ok"

        node._live_imu_measurement = delayed_measurement
        with node.state_publication_lock:
            live_thread = threading.Thread(target=node._publish_live_propagation)
            live_thread.start()
            self.assertTrue(measurement_ready.wait(timeout=1.0))
            node._commit_optimization_anchor(
                9.75, np.zeros(15), np.eye(15) * 0.01
            )
            release_measurement.set()
        live_thread.join(timeout=1.0)
        self.assertFalse(live_thread.is_alive())
        self.assertFalse(node.odom_pub.messages)
        self.assertEqual(node.last_live_propagation_reason, "anchor_changed")
        self.assertEqual(node.optimization_anchor.generation, 5)

    def test_unified_odom_suppresses_timestamp_regression(self):
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = object.__new__(UnifiedBackendNode)
        node.output_lock = threading.Lock()
        node.odom_pub = Recorder()
        node.last_unified_output_stamp_s = 10.0
        node.last_output = None
        node.last_output_source = "imu_propagated"
        node.counts = {
            "published": 0,
            "optimized_odom_nonmonotonic_suppressed": 0,
            "optimized_odom_published": 0,
            "live_propagation_published": 0,
        }
        output = Odometry()
        output.header.stamp.sec = 10

        self.assertFalse(node._publish_unified_odom(output, "optimized"))
        self.assertEqual(len(node.odom_pub.messages), 0)
        self.assertEqual(
            node.counts["optimized_odom_nonmonotonic_suppressed"], 1
        )

    def test_scan_request_updates_lidar_frontend_activity(self):
        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.last_native_input_arrival_s = 9.0
        node.last_scan_request_arrival_s = None
        node.last_native_consumed_sequence = -1
        node.pending_scan_request_lock = threading.Lock()
        node.pending_scan_requests = {}
        node.scan_prediction_by_sequence = {}
        node.counts = {
            "scan_prediction_requests": 0,
            "scan_prediction_duplicate_requests": 0,
            "scan_prediction_stale_requests": 0,
            "scan_prediction_deferred": 0,
        }
        node._now_s = lambda: 10.0
        node._produce_scan_prediction = lambda _message: None

        node._scan_request(SimpleNamespace(scan_sequence=0))

        self.assertEqual(node.last_scan_request_arrival_s, 10.0)
        self.assertEqual(node._latest_lidar_frontend_activity_s(), 10.0)

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

    def test_frontend_map_commit_requires_health_and_bounded_covariance(self):
        covariance = np.diag([0.1, 0.2, 0.3, 0.01, 0.02, 0.03])
        allowed = frontend_map_commit_decision(
            "DEGRADED", 0.1, 1.0, True, covariance,
            ("NORMAL", "DEGRADED", "RECOVERED"), 4.0, 0.25,
        )
        self.assertTrue(allowed[0])
        self.assertEqual(allowed[1], "ok")
        self.assertAlmostEqual(allowed[2], 0.3)
        self.assertAlmostEqual(allowed[3], 0.03)

        relocalizing = frontend_map_commit_decision(
            "RELOCALIZING", 0.1, 1.0, True, covariance,
            ("NORMAL", "DEGRADED", "RECOVERED"), 4.0, 0.25,
        )
        self.assertFalse(relocalizing[0])
        self.assertEqual(relocalizing[1], "scheduler_relocalizing")

        uncertain = covariance.copy()
        uncertain[1, 1] = 4.01
        rejected = frontend_map_commit_decision(
            "NORMAL", 0.1, 1.0, True, uncertain,
            ("NORMAL", "DEGRADED", "RECOVERED"), 4.0, 0.25,
        )
        self.assertFalse(rejected[0])
        self.assertEqual(rejected[1], "position_variance")

    def test_frontend_map_commit_rejects_stale_or_invalid_covariance(self):
        covariance = np.eye(6)
        stale = frontend_map_commit_decision(
            "NORMAL", 1.01, 1.0, True, covariance,
            ("NORMAL",), 4.0, 2.0,
        )
        self.assertFalse(stale[0])
        self.assertEqual(stale[1], "scheduler_stale")

        covariance[4, 4] = math.nan
        invalid = frontend_map_commit_decision(
            "NORMAL", 0.1, 1.0, True, covariance,
            ("NORMAL",), 4.0, 2.0,
        )
        self.assertFalse(invalid[0])
        self.assertEqual(invalid[1], "pose_covariance_invalid")

    def test_frontend_map_commit_rejects_disabled_lidar_factor(self):
        rejected = frontend_map_commit_decision(
            "NORMAL", 0.1, 1.0, False, np.eye(6),
            ("NORMAL",), 4.0, 2.0,
        )
        self.assertFalse(rejected[0])
        self.assertEqual(rejected[1], "lidar_factor_rejected")

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

    def test_flow_los_observation_is_exposure_weighted(self):
        observation = flow_los_observation([
            {
                "integrated_x": 0.01,
                "integrated_y": 0.00,
                "integrated_xgyro": 0.00,
                "integrated_ygyro": 0.00,
                "integration_time_s": 0.01,
                "distance_m": 1.0,
            },
            {
                "integrated_x": 0.04,
                "integrated_y": 0.00,
                "integrated_xgyro": 0.00,
                "integrated_ygyro": 0.00,
                "integration_time_s": 0.04,
                "distance_m": 2.0,
            },
        ])
        self.assertIsNotNone(observation)
        self.assertAlmostEqual(observation["measurement_radps"][0], -1.0)
        self.assertAlmostEqual(observation["measurement_radps"][1], 0.0)
        self.assertAlmostEqual(observation["distance_m"], 1.8)
        self.assertEqual(observation["sample_count"], 2)

    def test_flow_selection_prefers_strict_interval_and_keeps_future(self):
        records = [
            {"stamp_s": 0.80},
            {"stamp_s": 1.05},
            {"stamp_s": 1.15},
            {"stamp_s": 1.25},
        ]
        selected, remaining, delayed = select_flow_records(
            records, 1.0, 1.2, 0.5,
        )
        self.assertEqual([item["stamp_s"] for item in selected], [1.05, 1.15])
        self.assertEqual([item["stamp_s"] for item in remaining], [1.25])
        self.assertFalse(delayed)

    def test_flow_selection_uses_bounded_late_sample(self):
        records = [
            {"stamp_s": 0.65},
            {"stamp_s": 0.82},
            {"stamp_s": 1.25},
        ]
        selected, remaining, delayed = select_flow_records(
            records, 0.9, 1.0, 0.25,
        )
        self.assertEqual([item["stamp_s"] for item in selected], [0.82])
        self.assertEqual([item["stamp_s"] for item in remaining], [1.25])
        self.assertTrue(delayed)

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

    def test_lidar_prediction_gate_rejects_local_map_jump(self):
        allowed, reason = lidar_prediction_gate(
            {"position_m": 1.01, "yaw_rad": 0.1}, 1.0, 0.5
        )
        self.assertFalse(allowed)
        self.assertEqual(reason, "lidar_prediction_position_gate")

    def test_lidar_prediction_gate_accepts_normal_frame(self):
        allowed, reason = lidar_prediction_gate(
            {"position_m": 0.2, "yaw_rad": 0.1}, 1.0, 0.5
        )
        self.assertTrue(allowed)
        self.assertEqual(reason, "ok")

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
        node.last_scan_request_arrival_s = None
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
        node._now_s = lambda: 10.0

        node._scan_request(SimpleNamespace(scan_sequence=2))

        self.assertEqual(node.counts["scan_prediction_requests"], 1)
        self.assertEqual(node.counts["scan_prediction_duplicate_requests"], 1)
        self.assertEqual(node.counts["scan_prediction_deferred"], 1)
        self.assertEqual(list(node.pending_scan_requests), [2])

    def test_relocalization_reset_starts_a_clean_estimator_epoch(self):
        class FakeBackend:
            def __init__(self):
                self.current = np.asarray([
                    1.0, 2.0, 3.0, 0.1, -0.2, 0.3,
                    1.0, 0.5, -0.2,
                    0.01, 0.02, 0.03, 0.001, 0.002, 0.003,
                ], dtype=float)
                self.reset_state = None
                self.reset_covariance = None

            @property
            def state_count(self):
                return 1

            def state(self, _index):
                return self.current.copy()

            def snapshot(self):
                return self.current.copy()

            def restore(self, snapshot):
                self.current = snapshot.copy()

            def reset(self, state, covariance):
                self.reset_state = np.asarray(state, dtype=float).copy()
                self.reset_covariance = np.asarray(
                    covariance, dtype=float
                ).copy()
                self.current = self.reset_state.copy()

        node = object.__new__(UnifiedBackendNode)
        node.relocalization_lock = threading.Lock()
        alignment = np.eye(4)
        alignment[:3, :3] = rpy_to_rotation_matrix([0.0, 0.0, 0.2])
        alignment[:3, 3] = [5.0, -1.0, 0.5]
        recovered_pose = np.eye(4)
        recovered_pose[:3, 3] = [2.0, 3.0, 4.0]
        covariance = np.diag(
            [0.1, 0.2, 0.3, 0.01, 0.02, 0.03]
        ).ravel()
        node.pending_relocalization = (
            9.4, alignment, recovered_pose, np.eye(4), covariance,
        )
        node.pending_relocalization_candidate_id = 42
        node.pending_relocalization_transaction_id = 99
        node.pending_relocalization_deadline_s = 12.0
        node._now_s = lambda: 10.1
        node.last_applied_relocalization_transaction_id = 0
        node.fusion_session_id = 1234
        published_epochs = []
        anchors_visible_during_epoch_publish = []

        def publish_epoch(message):
            anchors_visible_during_epoch_publish.append(
                node.optimization_anchor
            )
            published_epochs.append(message)

        node.fusion_epoch_pub = SimpleNamespace(
            publish=publish_epoch
        )
        node.map_frame = "map"
        node.backend = FakeBackend()
        original_state = node.backend.current.copy()
        node.map_from_lio = np.eye(4)
        node.last_lio_stamp = 10.0
        node.lio_origin = np.zeros(3, dtype=float)
        node.optimization_anchor_lock = threading.Lock()
        node.state_publication_lock = threading.RLock()
        node.optimization_anchor_generation = 2
        node.optimization_anchor = make_optimization_anchor(
            10.0, original_state, np.eye(15), 2
        )
        node.output_lock = threading.Lock()
        node.last_unified_output_stamp_s = 10.0
        node.imu_buffer_lock = threading.Lock()
        node.imu_buffer = deque([
            ImuSample(8.0, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
            ImuSample(9.0, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
            ImuSample(11.0, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
        ], maxlen=10000)
        node.gnss_lock = threading.Lock()
        node.gnss_buffer = deque([
            {"stamp_s": 9.0}, {"stamp_s": 10.0}, {"stamp_s": 11.0},
        ], maxlen=512)
        node.latest_gnss = node.gnss_buffer[-1]
        node.flow_buffer_lock = threading.Lock()
        node.flow_buffer = deque([
            {"stamp_s": 9.0}, {"stamp_s": 10.0}, {"stamp_s": 11.0},
        ], maxlen=3000)
        node.native_lidar_buffer = NativeFactorBuffer(max_size=8)
        node.native_lidar_buffer.push(SimpleNamespace(stamp_s=9.0))
        node.pending_lio = deque([object()], maxlen=32)
        node.pending_imu_lio = deque([object()], maxlen=64)
        node.native_work_queue = queue.Queue(maxsize=2)
        node.native_work_queue.put_nowait(
            (None, SimpleNamespace(scan_sequence=14))
        )
        node.last_native_consumed_sequence = 12
        node.scan_prediction_cache = deque([object()], maxlen=8)
        node.scan_prediction_by_sequence = {13: object()}
        node.pending_scan_request_lock = threading.Lock()
        node.pending_scan_requests = {15: object()}
        node.path = SimpleNamespace(poses=[object()])
        node.last_path_sample_position = np.ones(3)
        node.last_path_sample_orientation = np.ones(3)
        node.last_path_publish_stamp_s = 9.5
        node.last_output = object()
        node.last_state_covariance = np.eye(15)
        node.last_covariance_stamp_s = 9.5
        node.last_covariance_source = "marginal"
        node.last_scan_prediction_reason = "ok"
        node.last_live_propagation_reason = "ok"
        node.last_output_source = "optimized"
        node.active_transaction_snapshot = object()
        node.last_frontend_map_pose_reason = "ok"
        node.last_frontend_map_position_variance_m2 = 0.1
        node.last_frontend_map_orientation_variance_rad2 = 0.1
        node.last_lidar_map_eligible = True
        node.last_lidar_map_reason = "ok"
        node.native_lidar_prediction_gate_latched = True
        node.counts = {
            "native_worker_queue_discarded": 0,
            "relocalization_resets": 0,
            "relocalization_expired": 0,
            "optimized_states_committed": 0,
        }
        node.state_reset_counter = 7

        self.assertTrue(node._apply_pending_relocalization(10.1))

        np.testing.assert_allclose(node.map_from_lio, alignment)
        expected_pose = matrix_to_pose_vector(
            alignment @ pose_vector_to_matrix(original_state[:6])
        )
        np.testing.assert_allclose(
            node.backend.reset_state[:6], expected_pose
        )
        np.testing.assert_allclose(
            node.backend.reset_state[6:9],
            alignment[:3, :3] @ original_state[6:9],
        )
        np.testing.assert_allclose(
            node.backend.reset_state[9:15], original_state[9:15]
        )
        np.testing.assert_allclose(node.lio_origin, alignment[:3, 3])
        np.testing.assert_allclose(
            node.backend.reset_covariance[:6],
            [0.1, 0.2, 0.3, 0.01, 0.02, 0.03],
        )
        self.assertEqual(
            [sample.stamp_s for sample in node.imu_buffer], [9.0, 11.0]
        )
        self.assertEqual(
            [item["stamp_s"] for item in node.gnss_buffer], [11.0]
        )
        self.assertEqual(
            [item["stamp_s"] for item in node.flow_buffer], [11.0]
        )
        self.assertEqual(len(node.native_lidar_buffer), 0)
        self.assertFalse(node.pending_lio)
        self.assertFalse(node.pending_imu_lio)
        self.assertTrue(node.native_work_queue.empty())
        self.assertEqual(node.native_work_queue.unfinished_tasks, 0)
        self.assertEqual(node.last_native_consumed_sequence, 14)
        self.assertFalse(node.scan_prediction_cache)
        self.assertFalse(node.scan_prediction_by_sequence)
        self.assertFalse(node.pending_scan_requests)
        self.assertFalse(node.path.poses)
        np.testing.assert_allclose(
            node.last_state_covariance, np.diag(node.backend.reset_covariance)
        )
        self.assertEqual(node.last_covariance_stamp_s, 10.0)
        self.assertIsNone(node.last_output)
        self.assertIsNone(node.last_path_sample_position)
        self.assertIsNone(node.last_path_sample_orientation)
        self.assertEqual(node.last_lio_stamp, 10.0)
        self.assertIsNotNone(node.optimization_anchor)
        self.assertEqual(node.optimization_anchor.generation, 4)
        self.assertEqual(node.optimization_anchor.reset_counter, 8)
        np.testing.assert_allclose(
            node.optimization_anchor.state, node.backend.reset_state
        )
        self.assertEqual(node.counts["optimized_states_committed"], 1)
        self.assertAlmostEqual(
            node.last_relocalization_reset_stats["result_age_s"], 0.6
        )
        self.assertEqual(node.state_reset_counter, 8)
        self.assertEqual(node.counts["relocalization_resets"], 1)
        self.assertEqual(len(published_epochs), 1)
        self.assertEqual(anchors_visible_during_epoch_publish, [None])
        self.assertTrue(published_epochs[0].applied)
        self.assertEqual(published_epochs[0].reset_counter, 8)
        self.assertEqual(published_epochs[0].candidate_id, 42)
        self.assertEqual(published_epochs[0].transaction_id, 99)
        self.assertEqual(published_epochs[0].session_id, 1234)
        self.assertEqual(published_epochs[0].header.frame_id, "map")
        self.assertEqual(published_epochs[0].header.stamp.sec, 10)
        self.assertEqual(published_epochs[0].header.stamp.nanosec, 100000000)
        self.assertFalse(node.native_lidar_prediction_gate_latched)
        self.assertEqual(
            node.last_relocalization_reset_stats["gnss_discarded"], 2
        )
        self.assertFalse(node._apply_pending_relocalization(11.0))
        self.assertEqual(node.state_reset_counter, 8)

    def test_delayed_relocalization_result_is_bounded_and_queued(self):
        def pose(x):
            return SimpleNamespace(
                position=SimpleNamespace(x=float(x), y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )

        class FakeBackend:
            @property
            def state_count(self):
                return 1

        node = object.__new__(UnifiedBackendNode)
        node.backend_solver_mode = "manifold"
        node.backend = FakeBackend()
        node.last_lio_stamp = 10.0
        node.relocalization_state_tolerance_s = 0.25
        node.relocalization_result_max_age_s = 2.0
        node.relocalization_pending_timeout_s = 2.0
        node.last_applied_relocalization_transaction_id = 0
        node.relocalization_lock = threading.Lock()
        node.pending_relocalization = None
        node.pending_relocalization_candidate_id = 0
        node.pending_relocalization_transaction_id = 0
        node.pending_relocalization_deadline_s = None
        node.counts = {"relocalization_rejections": 0}
        node._now_s = lambda: 10.0
        message = SimpleNamespace(
            state=2,
            accepted=True,
            transaction_id=17,
            candidate_id=3,
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=9, nanosec=200_000_000)
            ),
            map_from_lio=pose(1.0),
            source_lio_pose=pose(2.0),
            pose=SimpleNamespace(
                pose=pose(3.0), covariance=[0.0] * 36
            ),
        )

        node._relocalization_result(message)

        self.assertIsNotNone(node.pending_relocalization)
        self.assertEqual(node.pending_relocalization_transaction_id, 17)
        self.assertEqual(node.counts["relocalization_rejections"], 0)
        node.pending_relocalization = None
        node.pending_relocalization_transaction_id = 0
        message.transaction_id = 18
        message.header.stamp = SimpleNamespace(sec=7, nanosec=0)

        node._relocalization_result(message)

        self.assertIsNone(node.pending_relocalization)
        self.assertEqual(node.counts["relocalization_rejections"], 1)
        self.assertIn("stale", node.last_exception)

    def test_relocalization_backend_failure_restores_pending_transaction(self):
        class FailingBackend:
            def __init__(self):
                self.current = np.arange(15, dtype=float)

            @property
            def state_count(self):
                return 1

            def state(self, _index):
                return self.current.copy()

            def snapshot(self):
                return self.current.copy()

            def restore(self, snapshot):
                self.current = snapshot.copy()

            def reset(self, state, covariance):
                self.current = np.asarray(state, dtype=float).copy()
                raise RuntimeError("reset failed")

        node = object.__new__(UnifiedBackendNode)
        node.relocalization_lock = threading.Lock()
        pending = (
            10.0, np.eye(4), np.eye(4), np.eye(4), np.eye(6).ravel(),
        )
        node.pending_relocalization = pending
        node.pending_relocalization_candidate_id = 4
        node.pending_relocalization_transaction_id = 77
        node.pending_relocalization_deadline_s = 12.0
        node._now_s = lambda: 10.1
        node.backend = FailingBackend()
        original_state = node.backend.current.copy()
        node.map_from_lio = np.eye(4)
        node.last_lio_stamp = 10.0
        node.optimization_anchor_lock = threading.Lock()
        node.state_publication_lock = threading.RLock()
        node.state_reset_counter = 0
        node.optimization_anchor_generation = 1
        node.optimization_anchor = make_optimization_anchor(
            10.0, original_state, np.eye(15), 1
        )

        with self.assertRaisesRegex(RuntimeError, "reset failed"):
            node._apply_pending_relocalization(10.1)

        np.testing.assert_allclose(node.backend.current, original_state)
        np.testing.assert_allclose(
            node.optimization_anchor.state, original_state
        )
        self.assertEqual(node.optimization_anchor.generation, 3)
        self.assertIs(node.pending_relocalization, pending)
        self.assertEqual(node.pending_relocalization_transaction_id, 77)

    def test_expired_relocalization_is_never_committed_late(self):
        node = object.__new__(UnifiedBackendNode)
        node.relocalization_lock = threading.Lock()
        node.pending_relocalization = (
            10.0, np.eye(4), np.eye(4), np.eye(4), np.eye(6).ravel(),
        )
        node.pending_relocalization_candidate_id = 4
        node.pending_relocalization_transaction_id = 77
        node.pending_relocalization_deadline_s = 10.05
        node._now_s = lambda: 10.1
        node.counts = {"relocalization_expired": 0}
        node.last_reason = "pending"

        self.assertFalse(node._apply_pending_relocalization(10.1))
        self.assertIsNone(node.pending_relocalization)
        self.assertEqual(node.pending_relocalization_transaction_id, 0)
        self.assertEqual(node.counts["relocalization_expired"], 1)
        self.assertEqual(node.last_reason, "relocalization_pending_expired")

    def test_epoch_helpers_keep_future_records_and_imu_boundary_sample(self):
        records = [
            {"stamp_s": 9.0}, {"stamp_s": 10.0}, {"stamp_s": 10.1},
        ]
        self.assertEqual(
            [item["stamp_s"] for item in retain_stamped_records_after(
                records, 10.0
            )],
            [10.1],
        )
        samples = [
            ImuSample(8.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ImuSample(9.5, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ImuSample(10.5, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ]
        self.assertEqual(
            [sample.stamp_s for sample in reanchor_imu_samples(samples, 10.0)],
            [9.5, 10.5],
        )


if __name__ == "__main__":
    unittest.main()
