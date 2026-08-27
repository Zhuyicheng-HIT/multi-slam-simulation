import ast
import gc
import inspect
import math
import queue
import threading
import textwrap
import unittest
from collections import deque
from types import SimpleNamespace

import numpy as np
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from uf_backend_fusion.imu_preintegration import ImuSample, preintegrate_manifold
from uf_backend_fusion.live_propagation import make_optimization_anchor
from uf_backend_fusion.axis_reliability import barometer_activation_required
from uf_backend_fusion.manifold_window import ManifoldSlidingWindowBackend
from uf_backend_fusion.native_lidar import (
    NativeFactorBuffer,
    rpy_to_rotation_matrix,
)
from uf_backend_fusion.online_backend import (
    GarbageCollectionProfiler,
    add_visual_observation_once,
    attach_frontend_map_commit_eligibility,
    axis_map_protection,
    axis_observability_latch,
    axis_information_handoff,
    backend_diagnostic_level_message,
    apply_flow_rotation_gate,
    apply_gnss_prefit_gate,
    apply_lidar_anchor_floor,
    associate_visual_states,
    combine_visual_reliability_decisions,
    cap_weak_subspace_against_absolute_information,
    committed_state_missing_imu_factor,
    consume_timestamped_reliability_score,
    covariance_update_due,
    flow_los_observation,
    flow_observation_delta,
    mtf01p_flow_speed_gate,
    mtf01p_range_sigma_m,
    optical_flow_displacement_covariance_m2,
    select_flow_records,
    frd_to_enu_delta,
    fused_motion_reference,
    frontend_activation_odometry,
    gnss_jump_rejected,
    gnss_covariance_diagonal,
    gnss_axis_information_scale,
    gnss_prefit_axis_nis,
    gnss_prefit_statistics,
    bounded_axis_reanchor_target,
    time_compensate_gnss_observation,
    gnss_temporal_jump_rejected,
    imu_interval_covered,
    imu_interval_status,
    imu_samples_for_interval,
    imu_samples_covering_interval,
    estimate_stationary_imu_bias,
    enqueue_latest,
    reanchor_imu_samples,
    delayed_frontend_map_commit_candidate,
    diagnostic_imu_factor_disabled,
    directional_information,
    frontend_map_commit_decision,
    inflate_manifold_imu_covariance,
    lidar_bypass_allowed,
    lidar_calibration_motion_from_message,
    lidar_prediction_factor_admission,
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
    pose_translation_profile_information,
    prune_imu_buffer_before,
    scale_conditional_translation_normal,
    matrix_to_pose_vector,
    scheduler_decision,
    seed_calibrator_rotation_nonblocking,
    select_nonlinear_iteration_budget,
    select_gnss_observation,
    retain_stamped_records_after,
    visual_factor_score_wait_status,
    visual_factor_score_for_mode,
    visual_factor_score_source_stamp,
    visual_time_calibration_imu_coverage,
    visual_batch_information_scale,
    unwrap_yaw,
    UnifiedBackendNode,
    validate_optimized_state,
    yaw_to_quaternion,
)
from uf_reliability.flow_rotation_gate import FlowRotationGateResult


class OnlineBackendHelpersTest(unittest.TestCase):
    def test_diagnostic_imu_factor_switch_uses_measurement_time(self):
        self.assertTrue(diagnostic_imu_factor_disabled(True, -1.0, 10.0))
        self.assertFalse(diagnostic_imu_factor_disabled(False, -1.0, 100.0))
        self.assertFalse(diagnostic_imu_factor_disabled(False, 60.0, 59.999))
        self.assertTrue(diagnostic_imu_factor_disabled(False, 60.0, 60.0))

    def test_subspace_information_cap_preserves_strong_rotated_mode(self):
        direction = np.asarray([1.0, 1.0, 0.0]) / math.sqrt(2.0)
        weak = np.outer(direction, direction)
        base = np.eye(3) - 0.999 * weak
        previous = base.copy()
        lidar = 50.0 * weak
        absolute = 10.0 * weak

        capped, ratios = cap_weak_subspace_against_absolute_information(
            base, previous, lidar, absolute
        )

        self.assertAlmostEqual(float(direction @ capped @ direction), 0.0002)
        strong = np.asarray([1.0, -1.0, 0.0]) / math.sqrt(2.0)
        self.assertAlmostEqual(float(strong @ capped @ strong), 1.0)
        self.assertAlmostEqual(float(ratios.min()), 0.2)

    def test_directional_information_projects_onto_unit_direction(self):
        information = np.diag([4.0, 9.0, 16.0])
        self.assertAlmostEqual(
            directional_information(information, [3.0, 4.0, 0.0]), 7.2
        )
        self.assertEqual(
            directional_information(information, [0.0, 0.0, 0.0]), 0.0
        )

    def test_nonlinear_iteration_budget_preserves_recovery_headroom(self):
        self.assertEqual(
            select_nonlinear_iteration_budget(2, 4, 5, state_count=20), 2
        )
        self.assertEqual(
            select_nonlinear_iteration_budget(2, 4, 5, state_count=2), 4
        )
        self.assertEqual(
            select_nonlinear_iteration_budget(
                2, 4, 5, state_count=20, recovery_active=True
            ),
            5,
        )
        with self.assertRaises(ValueError):
            select_nonlinear_iteration_budget(0, 4, 5, state_count=20)

    def test_frontend_activation_pose_stays_in_local_fastlio_frame(self):
        message = Odometry()
        message.header.frame_id = "camera_init"
        message.child_frame_id = "body"
        message.pose.pose.position.z = 2.0

        activation = frontend_activation_odometry(
            message, "camera_init", "body"
        )

        self.assertEqual(activation.header.frame_id, "camera_init")
        self.assertEqual(activation.child_frame_id, "body")
        self.assertEqual(activation.pose.pose.position.z, 2.0)
        message.header.frame_id = "fusion_map"
        with self.assertRaisesRegex(ValueError, "local map frame"):
            frontend_activation_odometry(message, "camera_init", "body")

    def test_pose_translation_profile_eliminates_rotation_coupling(self):
        information = np.diag([10.0, 20.0, 30.0, 4.0, 5.0, 6.0])
        information[2, 3] = information[3, 2] = 6.0
        np.testing.assert_allclose(
            pose_translation_profile_information(information),
            [10.0, 20.0, 21.0],
        )

    def test_axis_reanchor_target_is_bounded_without_changing_sign(self):
        target, clipped = bounded_axis_reanchor_target(3.0, 5.0, 0.15)
        self.assertAlmostEqual(target, 3.15)
        self.assertTrue(clipped)
        target, clipped = bounded_axis_reanchor_target(3.0, 2.0, 0.15)
        self.assertAlmostEqual(target, 2.85)
        self.assertTrue(clipped)
        target, clipped = bounded_axis_reanchor_target(3.0, 3.04, 0.15)
        self.assertAlmostEqual(target, 3.04)
        self.assertFalse(clipped)

    def test_conditional_translation_scaling_preserves_rotation_and_coupling(self):
        rotation = np.diag([7.0, 8.0, 9.0])
        coupling = np.asarray([
            [0.3, -0.1, 0.2],
            [0.0, 0.4, -0.2],
            [0.5, 0.1, 0.3],
        ])
        schur = np.diag([30.0, 20.0, 10.0])
        information = np.zeros((6, 6), dtype=float)
        information[3:, 3:] = rotation
        information[:3, 3:] = coupling
        information[3:, :3] = coupling.T
        information[:3, :3] = (
            schur + coupling @ np.linalg.inv(rotation) @ coupling.T
        )
        gradient = np.asarray([1.0, 2.0, 3.0, -0.5, 0.2, 0.1])

        scaled_h, scaled_g = scale_conditional_translation_normal(
            information, gradient, [1.0, 1.0, 0.01]
        )
        np.testing.assert_allclose(scaled_h[3:, 3:], rotation)
        np.testing.assert_allclose(scaled_h[:3, 3:], coupling)
        np.testing.assert_allclose(scaled_g[3:], gradient[3:])
        original_conditional_gradient = (
            gradient[:3]
            - coupling @ np.linalg.inv(rotation) @ gradient[3:]
        )
        scaled_conditional_gradient = (
            scaled_g[:3]
            - coupling @ np.linalg.inv(rotation) @ scaled_g[3:]
        )
        original_optimum = np.linalg.solve(schur, original_conditional_gradient)
        scaled_schur = np.diag([30.0, 20.0, 0.1])
        scaled_optimum = np.linalg.solve(
            scaled_schur, scaled_conditional_gradient
        )
        np.testing.assert_allclose(scaled_optimum, original_optimum)
        np.testing.assert_allclose(
            pose_translation_profile_information(scaled_h),
            [30.0, 20.0, 0.1],
            rtol=1.0e-10,
            atol=1.0e-10,
        )
        self.assertGreaterEqual(
            float(np.min(np.linalg.eigvalsh(scaled_h))), -1.0e-10
        )

    def test_axis_handoff_uses_alternative_only_for_weak_axis(self):
        scales, latched = axis_information_handoff(
            [100.0, 80.0, 2000.0],
            [0.8, 0.7, 0.05],
            [2.0, 0.0, 3.0],
            [False, False, False],
        )
        np.testing.assert_allclose(scales, [1.0, 1.0, 0.0015])
        np.testing.assert_array_equal(latched, [False, False, True])

    def test_axis_handoff_has_hysteresis_and_restores_without_alternative(self):
        scales, latched = axis_information_handoff(
            [100.0, 100.0, 50.0],
            [0.8, 0.8, 0.25],
            [0.0, 0.0, 5.0],
            [False, False, True],
        )
        self.assertAlmostEqual(scales[2], 0.1)
        self.assertTrue(latched[2])
        scales, latched = axis_information_handoff(
            [100.0, 100.0, 50.0],
            [0.8, 0.8, 0.25],
            [0.0, 0.0, 0.0],
            latched,
        )
        np.testing.assert_allclose(scales, np.ones(3))
        np.testing.assert_array_equal(latched, [False, False, False])

    def test_axis_handoff_respects_enabled_axis_mask(self):
        scales, latched = axis_information_handoff(
            [100.0, 80.0, 2000.0],
            [0.05, 0.05, 0.05],
            [2.0, 2.0, 3.0],
            [True, True, False],
            enabled_axes=[False, False, True],
        )
        np.testing.assert_allclose(scales, [1.0, 1.0, 0.0015])
        np.testing.assert_array_equal(latched, [False, False, True])

    def test_axis_handoff_weak_z_uses_healthy_gnss_support(self):
        # The GNSS factor remains admitted; only the weak LiDAR Z block is
        # reduced to the independently available Z information.
        gnss = apply_gnss_prefit_gate(
            scheduler_decision(1.0, enabled=True, inflation=1.0),
            prefit_xy_nis=0.1,
            prefit_z_nis=0.1,
        )
        self.assertTrue(gnss["factor_enabled"])
        self.assertTrue(gnss["gnss_z_admitted"])
        scales, latched = axis_information_handoff(
            [100.0, 80.0, 20000.0],
            [0.80, 0.70, 0.05],
            [0.0, 0.0, 5.0],
            [False, False, False],
            enabled_axes=[False, False, True],
            enter_support=0.30,
            exit_support=0.35,
            minimum_lidar_information_scale=1.0e-5,
            maximum_lidar_to_alternative_ratio=1.0,
        )
        np.testing.assert_allclose(scales[:2], [1.0, 1.0])
        self.assertAlmostEqual(scales[2], 5.0 / 20000.0)
        np.testing.assert_array_equal(latched, [False, False, True])
        native_normal = np.diag([100.0, 80.0, 20000.0, 10.0, 10.0, 10.0])
        scaled_normal, _ = scale_conditional_translation_normal(
            native_normal, np.zeros(6), scales
        )
        np.testing.assert_allclose(
            pose_translation_profile_information(scaled_normal),
            [100.0, 80.0, 5.0],
        )

    def test_axis_handoff_weak_z_accepts_barometer_alternative_only(self):
        # A local barometer can own Z without changing either strong
        # horizontal LiDAR axis or the GNSS admission policy.
        scales, latched = axis_information_handoff(
            [100.0, 80.0, 20000.0],
            [0.80, 0.70, 0.05],
            [0.0, 0.0, 20.0],
            [False, False, False],
            enabled_axes=[False, False, True],
            enter_support=0.30,
            exit_support=0.35,
            minimum_lidar_information_scale=1.0e-5,
            maximum_lidar_to_alternative_ratio=1.0,
        )
        np.testing.assert_allclose(scales, [1.0, 1.0, 0.001])
        np.testing.assert_array_equal(latched, [False, False, True])

    def test_axis_observability_latch_is_independent_per_axis(self):
        latched = axis_observability_latch(
            [0.60, 0.20, 0.34], [False, False, False]
        )
        np.testing.assert_array_equal(latched, [False, True, True])
        latched = axis_observability_latch(
            [0.44, 0.46, 0.40], latched
        )
        np.testing.assert_array_equal(latched, [False, False, True])

    def test_axis_map_protection_requires_weak_axis_and_independent_evidence(self):
        protected, sources = axis_map_protection(
            [False, True, True],
            [2.0, -0.21, 0.05],
            gnss_fresh=True,
            barometer_active=True,
            gnss_disagreement_m=0.20,
        )
        np.testing.assert_array_equal(protected, [False, True, True])
        self.assertEqual(sources[0], "none")
        self.assertEqual(sources[1], "gnss_disagreement")
        self.assertEqual(sources[2], "barometer_fallback")

    def test_axis_map_protection_ignores_stale_gnss(self):
        protected, _ = axis_map_protection(
            [True, True, True],
            [5.0, 5.0, 5.0],
            gnss_fresh=False,
            barometer_active=False,
        )
        np.testing.assert_array_equal(protected, [False, False, False])

    def test_barometer_starts_when_fresh_gnss_z_is_rejected(self):
        self.assertTrue(
            barometer_activation_required(
                lidar_z_weak=False,
                alternative_z_information=0.0,
                stamp_s=10.0,
                gnss_prefit_stamp_s=9.95,
                gnss_max_age_s=0.5,
                gnss_z_admitted=False,
                gnss_z_nis=12.0,
                gnss_z_nis_gate=9.0,
            )
        )

    def test_barometer_stays_out_when_another_z_source_is_healthy(self):
        self.assertFalse(
            barometer_activation_required(
                lidar_z_weak=True,
                alternative_z_information=0.4,
                stamp_s=10.0,
                gnss_prefit_stamp_s=9.95,
                gnss_max_age_s=0.5,
                gnss_z_admitted=False,
                gnss_z_nis=12.0,
                gnss_z_nis_gate=9.0,
            )
        )

    def test_barometer_does_not_start_from_stale_gnss_conflict(self):
        self.assertFalse(
            barometer_activation_required(
                lidar_z_weak=False,
                alternative_z_information=0.0,
                stamp_s=10.0,
                gnss_prefit_stamp_s=8.0,
                gnss_max_age_s=0.5,
                gnss_z_admitted=False,
                gnss_z_nis=12.0,
                gnss_z_nis_gate=9.0,
            )
        )

    def test_imu_interval_slice_keeps_interpolation_boundaries(self):
        samples = [
            ImuSample(stamp, np.zeros(3), np.zeros(3))
            for stamp in (0.0, 0.1, 0.2, 0.3, 0.4)
        ]
        selected = imu_samples_covering_interval(samples, 0.15, 0.25)
        self.assertEqual(
            [sample.stamp_s for sample in selected],
            [0.1, 0.2, 0.3],
        )

    def test_imu_interval_selection_scans_unsorted_history_without_copying_all(self):
        samples = deque([
            ImuSample(stamp, np.zeros(3), np.zeros(3))
            for stamp in (0.4, 0.0, 0.3, 0.1, 0.2, 0.5)
        ])

        selected = imu_samples_for_interval(samples, 0.15, 0.35)

        self.assertEqual(
            [sample.stamp_s for sample in selected],
            [0.1, 0.2, 0.3, 0.4],
        )

    def test_imu_pruning_keeps_one_interpolation_sample(self):
        samples = deque([
            ImuSample(stamp, np.zeros(3), np.zeros(3))
            for stamp in (0.0, 0.1, 0.2, 0.3, 0.4)
        ], maxlen=100)

        removed = prune_imu_buffer_before(samples, 0.25)

        self.assertEqual(removed, 2)
        self.assertEqual(
            [sample.stamp_s for sample in samples],
            [0.2, 0.3, 0.4],
        )

    def test_visual_time_calibration_waits_for_complete_offset_search(self):
        offsets = np.asarray([-0.12, 0.0, 0.12])
        incomplete = [
            ImuSample(stamp, np.zeros(3), np.zeros(3))
            for stamp in np.arange(0.80, 1.16, 0.01)
        ]
        complete = incomplete + [
            ImuSample(stamp, np.zeros(3), np.zeros(3))
            for stamp in np.arange(1.16, 1.23, 0.01)
        ]
        missing_history = [
            ImuSample(stamp, np.zeros(3), np.zeros(3))
            for stamp in np.arange(0.95, 1.23, 0.01)
        ]

        self.assertEqual(
            visual_time_calibration_imu_coverage(
                incomplete, 1.0, 1.1, offsets
            ),
            "wait_future",
        )
        self.assertEqual(
            visual_time_calibration_imu_coverage(
                complete, 1.0, 1.1, offsets
            ),
            "ready",
        )
        self.assertEqual(
            visual_time_calibration_imu_coverage(
                missing_history, 1.0, 1.1, offsets
            ),
            "missing_history",
        )

    def test_every_literal_backend_counter_is_initialized(self):
        tree = ast.parse(textwrap.dedent(inspect.getsource(UnifiedBackendNode)))
        initialized = set()
        used = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict):
                for target in node.targets:
                    if (
                        isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id == "self"
                        and target.attr == "counts"
                    ):
                        initialized.update(
                            key.value
                            for key in node.value.keys
                            if isinstance(key, ast.Constant)
                            and isinstance(key.value, str)
                        )
            if isinstance(node, ast.Subscript):
                value = node.value
                key = node.slice
                if (
                    isinstance(value, ast.Attribute)
                    and isinstance(value.value, ast.Name)
                    and value.value.id == "self"
                    and value.attr == "counts"
                    and isinstance(key, ast.Constant)
                    and isinstance(key.value, str)
                ):
                    used.add(key.value)

        self.assertTrue(initialized)
        self.assertEqual(used - initialized, set())

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

    def test_gc_profiler_reports_generation_without_disabling_gc(self):
        enabled_before = gc.isenabled()
        profiler = GarbageCollectionProfiler()
        before = profiler.snapshot()
        gc.collect(0)
        after = profiler.snapshot()
        profiler.close()
        self.assertGreaterEqual(after["collections"][0], before["collections"][0] + 1)
        self.assertGreaterEqual(after["duration_ms"][0], before["duration_ms"][0])
        self.assertEqual(gc.isenabled(), enabled_before)

    def test_calibration_seed_is_nonblocking_and_idempotent(self):
        class Calibrator:
            def __init__(self):
                self.initial_rotation_set = False
                self.last_update = None
                self.calls = 0

            def set_initial_rotation(self, rotation):
                self.calls += 1
                self.initial_rotation_set = True
                self.last_update = np.asarray(rotation, dtype=float).copy()

        calibrator = Calibrator()
        lock = threading.Lock()
        rotation = np.eye(3)

        lock.acquire()
        update, reason = seed_calibrator_rotation_nonblocking(
            calibrator, lock, rotation
        )
        self.assertIsNone(update)
        self.assertEqual(reason, "calibration_busy")
        self.assertEqual(calibrator.calls, 0)
        lock.release()

        update, reason = seed_calibrator_rotation_nonblocking(
            calibrator, lock, rotation
        )
        self.assertEqual(reason, "initialized")
        np.testing.assert_allclose(update, rotation)
        self.assertEqual(calibrator.calls, 1)

        lock.acquire()
        update, reason = seed_calibrator_rotation_nonblocking(
            calibrator, lock, rotation
        )
        self.assertIsNone(update)
        self.assertEqual(reason, "already_initialized")
        self.assertEqual(calibrator.calls, 1)
        lock.release()

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

    def test_prebootstrap_visual_keeps_time_shadow_without_queue(self):
        node = object.__new__(UnifiedBackendNode)
        node.visual_candidate_sequence = 0
        node.visual_pending_enabled = True
        node.visual_pending_max_queue = 64
        node.visual_pending_latest_horizon_s = 0.35
        node.visual_state_stamps = deque([1.0], maxlen=8)
        node.visual_lock = threading.Lock()
        node.pending_visual_candidates = deque()
        node.pending_visual_keys = set()
        node.visual_factor_score_wait_started = {}
        node.visual_tracks = deque()
        node.visual_timing_reason_counts = {
            "prebootstrap_window_unavailable": 0,
            "superseded_by_newer_candidate": 0,
        }
        node.last_visual_reason = "none"
        node.counts = {
            "visual_received": 0,
            "visual_prebootstrap_dropped": 0,
            "visual_pending_enqueued": 0,
            "visual_pending_superseded": 0,
            "visual_pending_overflow": 0,
            "visual_rejected_time": 0,
            "visual_duplicate_candidates": 0,
            "visual_time_calibration_geometry_rejected": 0,
        }
        node._now_s = lambda: 1.3
        node._publish_visual_timing = lambda *_args, **_kwargs: None
        node._visual_pnp_metrics = lambda _message: {"selected": [object()]}
        node._visual_pnp_admissible = lambda _message, _metrics: True
        calibration_updates = []
        node._update_visual_time_calibration = (
            lambda _message, previous, current:
            calibration_updates.append((previous, current))
        )

        def message(previous_s, current_s):
            return SimpleNamespace(
                previous_stamp=SimpleNamespace(
                    sec=int(previous_s),
                    nanosec=int(round((previous_s % 1.0) * 1.0e9)),
                ),
                header=SimpleNamespace(
                    stamp=SimpleNamespace(
                        sec=int(current_s),
                        nanosec=int(round((current_s % 1.0) * 1.0e9)),
                    )
                ),
            )

        node._visual_tracks(message(1.0, 1.2))

        self.assertEqual(calibration_updates, [(1.0, 1.2)])
        self.assertEqual(node.counts["visual_received"], 1)
        self.assertEqual(node.counts["visual_prebootstrap_dropped"], 1)
        self.assertEqual(node.counts["visual_pending_enqueued"], 0)
        self.assertFalse(node.pending_visual_candidates)
        self.assertEqual(
            node.last_visual_reason, "prebootstrap_window_unavailable"
        )

        node.visual_state_stamps.append(1.1)
        node._visual_tracks(message(1.1, 1.3))

        self.assertEqual(calibration_updates[-1], (1.1, 1.3))
        self.assertEqual(node.counts["visual_received"], 2)
        self.assertEqual(node.counts["visual_prebootstrap_dropped"], 1)
        self.assertEqual(node.counts["visual_pending_enqueued"], 1)
        self.assertEqual(len(node.pending_visual_candidates), 1)

        node._visual_tracks(message(1.7, 1.8))

        self.assertEqual(len(node.pending_visual_candidates), 1)
        self.assertEqual(
            node.pending_visual_candidates[0].key,
            (1_700_000_000, 1_800_000_000),
        )
        self.assertEqual(node.counts["visual_pending_superseded"], 1)
        self.assertEqual(
            node.visual_timing_reason_counts[
                "superseded_by_newer_candidate"
            ],
            1,
        )

    def test_rgbd_depth_geometry_is_timestamp_matched_and_consumed_once(self):
        node = object.__new__(UnifiedBackendNode)
        node.rgbd_depth_factor_enabled = True
        node.rgbd_depth_factor_tolerance_s = 0.01
        node.rgbd_depth_factor_minimum_tracks = 4
        node.rgbd_depth_factor_maximum_tracks = 8
        node.rgbd_depth_factor_maximum_rmse_m = 0.20
        node.rgbd_depth_factor_information_scale = 0.25
        node.rgbd_depth_healthy_lidar_profile_information = 8000.0
        node.rgbd_depth_healthy_lidar_stride = 4
        node.rgbd_depth_candidate_sequence = 0
        node.visual_information_reference_tracks = 4
        node.visual_minimum_projectable_track_ratio = 0.8
        node.visual_rotation_body_camera = np.eye(3)
        node.visual_translation_body_camera = np.zeros(3)
        node.rgbd_geometry_lock = threading.Lock()
        node.rgbd_geometry_tracks = deque(maxlen=128)
        node.last_rgbd_depth_reason = "none"
        node.last_rgbd_depth_track_count = 0
        node.last_rgbd_depth_prefit_rmse_m = -1.0
        node.counts = {
            "rgbd_geometry_received": 0,
            "rgbd_geometry_matched": 0,
            "rgbd_geometry_missing": 0,
            "rgbd_depth_factor_attempts": 0,
            "rgbd_depth_factors": 0,
            "rgbd_depth_rejected_tracks": 0,
            "rgbd_depth_rejected_prefit": 0,
            "rgbd_depth_skipped_healthy_lidar": 0,
        }
        node.backend = ManifoldSlidingWindowBackend(max_states=2)
        previous_state = np.zeros(15)
        current_state = np.zeros(15)
        current_state[2] = 0.1
        previous_index = node.backend.add_state(previous_state)
        current_index = node.backend.add_state(current_state)

        def stamp(value):
            return SimpleNamespace(
                sec=int(value),
                nanosec=int(round((value % 1.0) * 1.0e9)),
            )

        tracks = []
        for index, (x, y) in enumerate((
                (-0.2, -0.1), (0.2, -0.1),
                (-0.2, 0.1), (0.2, 0.1))):
            tracks.append(SimpleNamespace(
                previous_x=x,
                previous_y=y,
                previous_depth_m=2.0,
                previous_depth_variance_m2=0.0004,
                current_depth_m=1.9,
                current_depth_variance_m2=0.0004,
                track_age=3,
                grid_cell=index,
            ))
        geometry = SimpleNamespace(
            previous_stamp=stamp(1.0),
            header=SimpleNamespace(stamp=stamp(1.1)),
            tracks=tracks,
        )
        visual = SimpleNamespace(
            previous_stamp=stamp(1.0),
            header=SimpleNamespace(stamp=stamp(1.1)),
        )
        node._rgbd_geometry_tracks(geometry)
        decision = {
            "factor_enabled": True,
            "reliability_weight": 1.0,
            "covariance_inflation": 1.0,
        }
        self.assertTrue(node._add_rgbd_depth_factor(
            visual, previous_index, current_index, decision
        ))
        self.assertEqual(node.counts["rgbd_depth_factors"], 1)
        self.assertEqual(node.backend._factors[-1]["name"], "rgbd_depth")
        self.assertAlmostEqual(node.last_rgbd_depth_prefit_rmse_m, 0.0)
        node.last_lidar_map_eligible = True
        node.last_native_vertical_profile_information = 9000.0
        geometry_next = SimpleNamespace(
            previous_stamp=stamp(1.1),
            header=SimpleNamespace(stamp=stamp(1.2)),
            tracks=tracks,
        )
        visual_next = SimpleNamespace(
            previous_stamp=stamp(1.1),
            header=SimpleNamespace(stamp=stamp(1.2)),
        )
        node._rgbd_geometry_tracks(geometry_next)
        self.assertFalse(node._add_rgbd_depth_factor(
            visual_next, previous_index, current_index, decision
        ))
        self.assertEqual(node.last_rgbd_depth_reason, "skipped_healthy_lidar_z")
        self.assertEqual(node.counts["rgbd_depth_skipped_healthy_lidar"], 1)
        self.assertFalse(node._add_rgbd_depth_factor(
            visual, previous_index, current_index, decision
        ))
        self.assertEqual(
            node.last_rgbd_depth_reason, "matching_geometry_missing"
        )

    def test_rgbd_auxiliary_queues_keep_only_latest_batch(self):
        node = object.__new__(UnifiedBackendNode)
        node.rgbd_depth_factor_tolerance_s = 0.01
        node.rgbd_geometry_lock = threading.Lock()
        node.rgbd_direct_lock = threading.Lock()
        node.rgbd_geometry_tracks = deque(maxlen=2)
        node.rgbd_direct_tracks = deque(maxlen=2)
        node.counts = {
            "rgbd_geometry_received": 0,
            "rgbd_geometry_superseded": 0,
            "rgbd_geometry_matched": 0,
            "rgbd_direct_received": 0,
            "rgbd_direct_superseded": 0,
            "rgbd_direct_matched": 0,
        }

        def stamp(value):
            return SimpleNamespace(
                sec=int(value),
                nanosec=int(round((value % 1.0) * 1.0e9)),
            )

        def batch(previous_s, current_s, marker):
            return SimpleNamespace(
                previous_stamp=stamp(previous_s),
                header=SimpleNamespace(stamp=stamp(current_s)),
                marker=marker,
            )

        node._rgbd_geometry_tracks(batch(1.0, 1.1, "old-geometry"))
        node._rgbd_geometry_tracks(batch(1.1, 1.2, "middle-geometry"))
        node._rgbd_geometry_tracks(batch(1.2, 1.3, "new-geometry"))
        node._rgbd_direct_tracks(batch(1.0, 1.1, "old-direct"))
        node._rgbd_direct_tracks(batch(1.1, 1.2, "middle-direct"))
        node._rgbd_direct_tracks(batch(1.2, 1.3, "new-direct"))

        self.assertEqual(len(node.rgbd_geometry_tracks), 2)
        self.assertEqual(len(node.rgbd_direct_tracks), 2)
        self.assertEqual(node.counts["rgbd_geometry_superseded"], 1)
        self.assertEqual(node.counts["rgbd_direct_superseded"], 1)

        middle = batch(1.1, 1.2, "target")
        matched_geometry = node._matched_rgbd_geometry(middle)
        matched_direct = node._matched_rgbd_direct(middle)
        self.assertEqual(matched_geometry.marker, "middle-geometry")
        self.assertEqual(matched_direct.marker, "middle-direct")
        latest = batch(1.2, 1.3, "target-latest")
        self.assertEqual(
            node._matched_rgbd_geometry(latest).marker, "new-geometry"
        )
        self.assertEqual(
            node._matched_rgbd_direct(latest).marker, "new-direct"
        )
        self.assertIsNone(node._matched_rgbd_geometry(middle))
        self.assertIsNone(node._matched_rgbd_direct(middle))

    def test_rgbd_direct_batch_is_one_combined_factor_and_consumed_once(self):
        node = object.__new__(UnifiedBackendNode)
        node.rgbd_depth_factor_tolerance_s = 0.01
        node.rgbd_direct_factor_minimum_tracks = 4
        node.rgbd_direct_factor_maximum_tracks = 8
        node.rgbd_direct_factor_maximum_depth_rmse_m = 0.20
        node.rgbd_direct_factor_maximum_photometric_rmse = 0.60
        node.rgbd_direct_depth_information_scale = 0.25
        node.rgbd_direct_photometric_information_scale = 0.10
        node.visual_information_reference_tracks = 4
        node.visual_minimum_projectable_track_ratio = 0.8
        node.visual_rotation_body_camera = np.eye(3)
        node.visual_translation_body_camera = np.zeros(3)
        node.rgbd_direct_lock = threading.Lock()
        node.rgbd_direct_tracks = deque(maxlen=32)
        node.last_rgbd_direct_reason = "none"
        node.last_rgbd_direct_track_count = 0
        node.last_rgbd_direct_photometric_information_scale = 1.0
        node.counts = {
            "rgbd_direct_received": 0,
            "rgbd_direct_matched": 0,
            "rgbd_direct_missing": 0,
            "rgbd_direct_factor_attempts": 0,
            "rgbd_direct_factors": 0,
            "rgbd_direct_rejected_tracks": 0,
            "rgbd_direct_rejected_prefit": 0,
            "rgbd_direct_photometric_downweighted": 0,
            "visual_factor_attempts": 0,
            "visual_factor_score_invalid": 0,
            "visual_quality_rejected_dv": 0,
            "visual_factors": 0,
        }
        node.visual_factor_mode = "rgbd_direct"
        node.backend_solver_mode = "manifold"
        node._decision = lambda *_args, **_kwargs: {
            "factor_enabled": True,
            "reliability_weight": 1.0,
            "covariance_inflation": 1.0,
            "reasons": (),
        }
        node._visual_pnp_metrics = lambda _message: {
            "selected": [],
            "inlier_ratio": 0.0,
            "mean_reprojection_px": -1.0,
            "rank": 0,
            "condition": float("inf"),
        }
        node.backend = ManifoldSlidingWindowBackend(max_states=2)
        previous_index = node.backend.add_state(np.zeros(15))
        current_index = node.backend.add_state(np.zeros(15))

        def stamp(value):
            return SimpleNamespace(
                sec=int(value),
                nanosec=int(round((value % 1.0) * 1.0e9)),
            )

        tracks = []
        for index, (x, y) in enumerate((
                (-0.2, -0.1), (0.2, -0.1),
                (-0.2, 0.1), (0.2, 0.1))):
            tracks.append(SimpleNamespace(
                previous_x=x,
                previous_y=y,
                previous_depth_m=2.0,
                previous_depth_variance_m2=0.0004,
                current_x=x,
                current_y=y,
                current_depth_m=2.0,
                current_depth_variance_m2=0.0004,
                previous_intensity=0.1 * index,
                current_intensity=0.1 * index,
                current_gradient_x_normalized=5.0,
                current_gradient_y_normalized=3.0,
                photometric_variance=0.15 ** 2,
                track_age=3,
                grid_cell=index,
            ))
        direct = SimpleNamespace(
            previous_stamp=stamp(1.0),
            header=SimpleNamespace(stamp=stamp(1.1)),
            tracks=tracks,
        )
        visual = SimpleNamespace(
            previous_stamp=stamp(1.0),
            header=SimpleNamespace(stamp=stamp(1.1)),
            pnp_valid=False,
        )
        node._rgbd_direct_tracks(direct)
        decision = {
            "factor_enabled": True,
            "reliability_weight": 1.0,
            "covariance_inflation": 1.0,
        }

        self.assertTrue(node._add_visual_message_factor(
            visual,
            previous_index,
            current_index,
            factor_score={
                "valid": True,
                "weight": 1.0,
                "degradation_score": 0.0,
                "reasons": (),
            },
        ))
        self.assertEqual(node.backend._factors[-1]["name"], "rgbd_direct")
        self.assertEqual(node.counts["rgbd_direct_factors"], 1)
        self.assertEqual(node.counts["visual_factors"], 1)
        self.assertEqual(node.last_visual_reason, "accepted_rgbd_direct")

        shifted_tracks = [
            SimpleNamespace(**{
                **vars(track),
                "current_intensity": track.current_intensity + 2.0,
            })
            for track in tracks
        ]
        shifted = SimpleNamespace(
            previous_stamp=stamp(1.1),
            header=SimpleNamespace(stamp=stamp(1.2)),
            tracks=shifted_tracks,
        )
        shifted_visual = SimpleNamespace(
            previous_stamp=stamp(1.1),
            header=SimpleNamespace(stamp=stamp(1.2)),
        )
        node._rgbd_direct_tracks(shifted)
        self.assertTrue(node._add_rgbd_direct_factor(
            shifted_visual, previous_index, current_index, decision
        ))
        self.assertEqual(
            node.last_rgbd_direct_reason,
            "accepted_photometric_downweighted",
        )
        self.assertLess(
            node.last_rgbd_direct_photometric_information_scale, 1.0
        )
        self.assertEqual(
            node.counts["rgbd_direct_photometric_downweighted"], 1
        )
        self.assertFalse(node._add_rgbd_direct_factor(
            visual, previous_index, current_index, decision
        ))
        self.assertEqual(node.last_rgbd_direct_reason,
                         "matching_direct_tracks_missing")

    def test_pending_visual_left_of_window_is_discarded(self):
        node = object.__new__(UnifiedBackendNode)
        candidate = SimpleNamespace(
            key=(200_000_000, 400_000_000),
            message=SimpleNamespace(
                previous_stamp=SimpleNamespace(sec=0, nanosec=200_000_000),
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=400_000_000)
                ),
            ),
            arrival_ros_s=0.4,
            arrival_wall_s=0.0,
        )
        node.visual_lock = threading.Lock()
        node.pending_visual_candidates = deque([candidate])
        node.pending_visual_keys = {candidate.key}
        node.visual_factor_score_wait_started = {candidate.key: (0.4, 0.0)}
        node.visual_state_tolerance_s = 0.065
        node.visual_pending_max_wait_s = 0.60
        node.visual_pending_max_wall_wait_s = 3.0
        node.visual_timing_reason_counts = {"pre_window_stale": 0}
        node.last_visual_reason = "none"
        node.counts = {
            "visual_pending_pre_window_dropped": 0,
            "visual_rejected_time": 0,
            "visual_pending_expired": 0,
        }
        node._effective_visual_time_offset_s = lambda: 0.0
        node._now_s = lambda: 1.1
        node._record_phase_timing = lambda *_args: None
        timing = []
        node._publish_visual_timing = (
            lambda _candidate, outcome, reason, _stamps, _association=None:
            timing.append((outcome, reason))
        )

        staged = node._stage_pending_visual_factors([1.0, 1.1])

        self.assertEqual(staged, [])
        self.assertFalse(node.pending_visual_candidates)
        self.assertFalse(node.pending_visual_keys)
        self.assertFalse(node.visual_factor_score_wait_started)
        self.assertEqual(
            node.counts["visual_pending_pre_window_dropped"], 1
        )
        self.assertEqual(node.counts["visual_rejected_time"], 1)
        self.assertEqual(node.counts["visual_pending_expired"], 0)
        self.assertEqual(timing, [("rejected", "pre_window_stale")])

    def test_visual_factor_score_is_consumed_exactly_once(self):
        records = (
            {"sequence": 1, "source_stamp_s": 10.000, "weight": 0.8},
            {"sequence": 2, "source_stamp_s": 10.020, "weight": 0.7},
        )
        matched, retained, error_s = consume_timestamped_reliability_score(
            records, 10.019, tolerance_s=0.005
        )
        self.assertEqual(matched["sequence"], 2)
        self.assertAlmostEqual(error_s, 0.001)
        self.assertEqual(tuple(record["sequence"] for record in retained), (1,))

        matched_again, retained_again, _ = (
            consume_timestamped_reliability_score(
                retained, 10.019, tolerance_s=0.005
            )
        )
        self.assertIsNone(matched_again)
        self.assertEqual(retained_again, retained)

    def test_visual_factor_score_outside_tolerance_is_retained(self):
        records = (
            {"sequence": 1, "source_stamp_s": 5.0, "weight": 0.8},
        )
        matched, retained, error_s = consume_timestamped_reliability_score(
            records, 5.02, tolerance_s=0.01
        )
        self.assertIsNone(matched)
        self.assertEqual(retained, records)
        self.assertAlmostEqual(error_s, 0.02)

    def test_visual_factor_score_wait_expires_on_either_clock(self):
        waiting = visual_factor_score_wait_status(
            10.0, 20.0, 10.20, 20.20, 0.25, 0.25
        )
        self.assertEqual(waiting[0], "wait")
        ros_expired = visual_factor_score_wait_status(
            10.0, 20.0, 10.25, 20.10, 0.25, 0.25
        )
        self.assertEqual(ros_expired[0], "expired")
        wall_expired = visual_factor_score_wait_status(
            10.0, 20.0, 10.10, 20.25, 0.25, 0.25
        )
        self.assertEqual(wall_expired[0], "expired")

    def test_fixed_reliability_does_not_require_dynamic_visual_score(self):
        score = visual_factor_score_for_mode("fixed", None)
        self.assertTrue(score["valid"])
        self.assertEqual(score["weight"], 1.0)
        self.assertEqual(score["degradation_score"], 0.0)
        self.assertIn("fixed_reliability_mode", score["reasons"])

    def test_dynamic_reliability_preserves_missing_visual_score(self):
        self.assertIsNone(visual_factor_score_for_mode("dynamic", None))

    def test_fixed_visual_score_uses_candidate_source_stamp(self):
        score = visual_factor_score_for_mode("fixed", None)
        self.assertAlmostEqual(
            visual_factor_score_source_stamp(score, 12.34), 12.34
        )
        score["source_stamp_s"] = 12.30
        self.assertAlmostEqual(
            visual_factor_score_source_stamp(score, 12.34), 12.30
        )

    def test_visual_factor_score_mode_rejects_unknown_mode(self):
        with self.assertRaisesRegex(ValueError, "dynamic or fixed"):
            visual_factor_score_for_mode("hybrid", None)

    def test_visual_batch_information_is_capped_at_reference_equivalent(self):
        self.assertEqual(visual_batch_information_scale(8, 20), 1.0)
        self.assertEqual(visual_batch_information_scale(20, 20), 1.0)
        self.assertEqual(visual_batch_information_scale(40, 20), 2.0)
        with self.assertRaises(ValueError):
            visual_batch_information_scale(0, 20)

    def test_one_d435_batch_adds_exactly_one_factor_representation(self):
        class Backend:
            def __init__(self):
                self.reprojection_calls = 0

            def add_visual_reprojection(self, *_args, **_kwargs):
                self.reprojection_calls += 1

        backend = Backend()
        rgbd_calls = []
        representation = add_visual_observation_once(
            backend, 0, 1, (), {},
            lambda: rgbd_calls.append("rgbd") or True,
        )
        self.assertEqual(representation, "rgbd_depth")
        self.assertEqual(rgbd_calls, ["rgbd"])
        self.assertEqual(backend.reprojection_calls, 0)

        representation = add_visual_observation_once(
            backend, 0, 1, (), {}, lambda: False,
        )
        self.assertEqual(representation, "paper_reprojection")
        self.assertEqual(backend.reprojection_calls, 1)

    def test_visual_factor_decision_uses_conservative_intersection(self):
        sensor = {
            "factor_enabled": True,
            "reliability_weight": 0.7,
            "covariance_inflation": 1.5,
            "degradation_score": 0.3,
            "reasons": ("camera_healthy",),
        }
        factor = {
            "valid": True,
            "weight": 0.4,
            "degradation_score": 0.6,
            "reasons": ("weak_selected_track_retention",),
        }
        combined = combine_visual_reliability_decisions(sensor, factor)
        self.assertTrue(combined["factor_enabled"])
        self.assertAlmostEqual(combined["reliability_weight"], 0.4)
        self.assertAlmostEqual(combined["covariance_inflation"], 2.5)
        self.assertAlmostEqual(combined["degradation_score"], 0.6)
        self.assertIn("camera_healthy", combined["reasons"])
        self.assertIn(
            "factor:weak_selected_track_retention", combined["reasons"]
        )

    def test_invalid_visual_factor_score_cannot_override_sensor_health(self):
        sensor = {
            "factor_enabled": True,
            "reliability_weight": 0.9,
            "covariance_inflation": 1.2,
            "reasons": ("camera_healthy",),
        }
        factor = {
            "valid": False,
            "weight": 0.9,
            "degradation_score": 0.1,
            "reasons": ("pnp_geometric_verification_failed",),
        }
        combined = combine_visual_reliability_decisions(sensor, factor)
        self.assertFalse(combined["factor_enabled"])
        self.assertEqual(combined["reliability_weight"], 0.0)
        self.assertEqual(combined["covariance_inflation"], 20.0)
        self.assertIn("camera_healthy", combined["reasons"])
        self.assertIn(
            "factor:pnp_geometric_verification_failed", combined["reasons"]
        )

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

    def test_independent_frontend_factors_use_persistent_map_alignment(self):
        self.assertEqual(native_factor_epoch_status(0, 1, True, False), "current")
        self.assertEqual(native_factor_epoch_status(0, 1, False, False), "current")
        self.assertEqual(native_factor_epoch_status(2, 1, False, False), "current")

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
        node.live_propagation_maximum_output_age_s = 0.20
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

    def test_live_activity_uses_published_state_not_scan_request(self):
        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.last_native_input_arrival_s = 9.0
        node.last_scan_request_arrival_s = None
        node.last_unified_output_stamp_s = 9.3
        node.output_lock = threading.Lock()
        node.last_native_consumed_sequence = -1
        node.pending_scan_request_lock = threading.Lock()
        node.pending_scan_requests = {}
        node.pending_scan_request_first_seen_s = {}
        node.scan_prediction_missing_factor_grace_s = 0.5
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
        self.assertEqual(node._latest_lidar_frontend_activity_s(), 9.0)

    def test_live_activity_falls_back_to_native_input_before_first_output(self):
        node = object.__new__(UnifiedBackendNode)
        node.last_native_input_arrival_s = 9.0
        node.last_scan_request_arrival_s = 10.0
        node.last_unified_output_stamp_s = None
        node.output_lock = threading.Lock()

        self.assertEqual(node._latest_lidar_frontend_activity_s(), 9.0)

    def test_scan_prediction_contract_stays_healthy_with_cache_hits(self):
        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.scan_prediction_contract_failure_threshold = 3
        node.scan_prediction_contract_request_timeout_s = 1.0
        node.scan_prediction_contract_lock = threading.RLock()
        node.scan_prediction_contract_established = False
        node.scan_prediction_contract_violated = False
        node.scan_prediction_contract_consecutive_failures = 0
        node.scan_prediction_contract_reason = "waiting_for_handshake"
        node.scan_prediction_contract_first_failure_sequence = -1
        node.scan_prediction_contract_first_failure_stamp_s = -1.0
        node.last_scan_request_arrival_s = 10.0
        node.counts = {
            "scan_prediction_contract_trips": 0,
            "scan_prediction_contract_recoveries": 0,
        }

        node._record_scan_prediction_contract_success()
        node._record_scan_prediction_contract_success()

        self.assertTrue(node.scan_prediction_contract_established)
        self.assertFalse(node.scan_prediction_contract_violated)
        self.assertEqual(node.scan_prediction_contract_reason, "ok")
        self.assertEqual(node.scan_prediction_contract_consecutive_failures, 0)
        self.assertEqual(node.counts["scan_prediction_contract_trips"], 0)

    def test_scan_prediction_contract_violation_is_diagnostic_error(self):
        level, message = backend_diagnostic_level_message(
            "scan_prediction_not_reusable:cache_miss",
            0,
            True,
            "consecutive_cache_miss",
        )

        self.assertEqual(level, DiagnosticStatus.ERROR)
        self.assertEqual(
            message,
            "scan_prediction_contract_violation:consecutive_cache_miss",
        )

    def test_missing_scan_handshake_fails_fast_and_suppresses_unified_odom(self):
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.scan_prediction_contract_failure_threshold = 3
        node.scan_prediction_contract_request_timeout_s = 1.0
        node.scan_prediction_contract_lock = threading.RLock()
        node.scan_prediction_contract_established = False
        node.scan_prediction_contract_violated = False
        node.scan_prediction_contract_consecutive_failures = 0
        node.scan_prediction_contract_reason = "waiting_for_handshake"
        node.scan_prediction_contract_first_failure_sequence = -1
        node.scan_prediction_contract_first_failure_stamp_s = -1.0
        node.last_scan_request_arrival_s = None
        node.output_lock = threading.Lock()
        node.last_unified_output_stamp_s = None
        node.odom_pub = Recorder()
        node.counts = {
            "scan_prediction_contract_trips": 0,
            "scan_prediction_contract_recoveries": 0,
            "scan_prediction_contract_output_suppressed": 0,
            "optimized_odom_nonmonotonic_suppressed": 0,
            "published": 0,
            "optimized_odom_published": 0,
            "live_propagation_published": 0,
        }
        node._now_s = lambda: 10.0

        node._record_scan_prediction_contract_failure("cache_miss", 40, 9.8)
        node._record_scan_prediction_contract_failure("cache_miss", 41, 9.9)
        self.assertFalse(node.scan_prediction_contract_violated)
        node._record_scan_prediction_contract_failure("cache_miss", 42, 10.0)

        output = Odometry()
        output.header.stamp.sec = 10
        self.assertFalse(node._publish_unified_odom(output, "imu_propagated"))
        self.assertTrue(node.scan_prediction_contract_violated)
        self.assertEqual(
            node.scan_prediction_contract_reason,
            "consecutive_cache_miss",
        )
        self.assertEqual(node.scan_prediction_contract_first_failure_sequence, 40)
        self.assertEqual(node.counts["scan_prediction_contract_trips"], 1)
        self.assertEqual(
            node.counts["scan_prediction_contract_output_suppressed"], 1
        )
        self.assertFalse(node.odom_pub.messages)

        node.last_scan_request_arrival_s = 11.0
        node._record_scan_prediction_contract_success()
        self.assertTrue(node._scan_prediction_contract_allows_output(11.0))
        self.assertEqual(node.counts["scan_prediction_contract_recoveries"], 1)

    def test_established_scan_handshake_times_out_before_imu_only_output(self):
        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.scan_prediction_contract_failure_threshold = 3
        node.scan_prediction_contract_request_timeout_s = 1.0
        node.scan_prediction_contract_lock = threading.RLock()
        node.scan_prediction_contract_established = True
        node.scan_prediction_contract_violated = False
        node.scan_prediction_contract_consecutive_failures = 0
        node.scan_prediction_contract_reason = "ok"
        node.scan_prediction_contract_first_failure_sequence = -1
        node.scan_prediction_contract_first_failure_stamp_s = -1.0
        node.last_scan_request_arrival_s = 8.0
        node.last_native_sequence = 55
        node.counts = {
            "scan_prediction_contract_trips": 0,
            "scan_prediction_contract_recoveries": 0,
        }

        self.assertFalse(node._scan_prediction_contract_allows_output(10.0))
        self.assertTrue(node.scan_prediction_contract_violated)
        self.assertEqual(node.scan_prediction_contract_reason, "request_timeout")
        self.assertEqual(node.counts["scan_prediction_contract_trips"], 1)

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

    def test_delayed_frontend_map_commit_selects_stabilized_state(self):
        states = [np.full(15, float(index)) for index in range(8)]
        stamps = [10.0 + 0.1 * index for index in range(8)]

        delayed = delayed_frontend_map_commit_candidate(states, stamps, 7)
        self.assertAlmostEqual(delayed[0], 10.0)
        np.testing.assert_allclose(delayed[1], states[0])
        delayed[1][0] = 99.0
        self.assertEqual(states[0][0], 0.0)

        self.assertIsNone(
            delayed_frontend_map_commit_candidate(states[:7], stamps[:7], 7)
        )

    def test_delayed_frontend_map_commit_uses_its_original_eligibility(self):
        candidate = (10.0, np.zeros(15))
        history = {
            10_000_000_000: (False, "axis_protection:z:gnss_disagreement"),
            10_700_000_000: (True, "ok"),
        }
        attached = attach_frontend_map_commit_eligibility(candidate, history)
        self.assertFalse(attached[2])
        self.assertEqual(
            attached[3], "axis_protection:z:gnss_disagreement"
        )

        missing = attach_frontend_map_commit_eligibility(
            (11.0, np.zeros(15)), history
        )
        self.assertFalse(missing[2])
        self.assertEqual(missing[3], "eligibility_missing")

    def test_delayed_frontend_map_commit_rejects_misaligned_history(self):
        with self.assertRaisesRegex(ValueError, "states and stamps"):
            delayed_frontend_map_commit_candidate(
                [np.zeros(15), np.ones(15)], [10.0], 1
            )

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

    def test_gnss_prefit_nis_uses_prediction_covariance_plus_measurement(self):
        residual, innovation_covariance, nis = gnss_prefit_statistics(
            predicted_position=[2.0, 0.0, 0.0],
            predicted_position_covariance=np.eye(3) * 4.0,
            measured_position=[0.0, 0.0, 0.0],
            measurement_covariance=[1.0, 1.0, 1.0],
        )

        np.testing.assert_allclose(residual, [2.0, 0.0, 0.0])
        np.testing.assert_allclose(innovation_covariance, np.eye(3) * 5.0)
        self.assertAlmostEqual(nis, 0.8)
        self.assertNotAlmostEqual(nis, 4.0)

    def test_gnss_prefit_axis_nis_uses_marginal_xy_and_z_blocks(self):
        xy_nis, z_nis = gnss_prefit_axis_nis(
            residual=[2.0, 2.0, 3.0],
            innovation_covariance=[
                [4.0, 1.0, 0.5],
                [1.0, 4.0, 0.2],
                [0.5, 0.2, 9.0],
            ],
        )

        self.assertAlmostEqual(xy_nis, 1.6)
        self.assertAlmostEqual(z_nis, 1.0)

    def test_gnss_axis_information_scale_is_full_then_robust_with_floor(self):
        self.assertEqual(gnss_axis_information_scale(6.635, 6.635), 1.0)
        self.assertAlmostEqual(
            gnss_axis_information_scale(26.54, 6.635), 0.5
        )
        self.assertEqual(
            gnss_axis_information_scale(1.0e12, 6.635, 0.01), 0.01
        )

    def test_gnss_prefit_gate_is_authoritative_after_scheduler_weighting(self):
        admitted = apply_gnss_prefit_gate(
            scheduler_decision(0.8, enabled=True, inflation=1.25),
            prefit_xy_nis=0.8,
            prefit_z_nis=0.2,
        )
        self.assertTrue(admitted["factor_enabled"])
        self.assertAlmostEqual(admitted["reliability_weight"], 0.8)
        self.assertAlmostEqual(admitted["covariance_inflation"], 1.25)
        self.assertEqual(admitted["admission_reason"], "admitted_all_axes")
        self.assertTrue(admitted["gnss_xy_admitted"])
        self.assertTrue(admitted["gnss_z_admitted"])

        z_with_xy_robust = apply_gnss_prefit_gate(
            scheduler_decision(1.0, enabled=True, inflation=1.0),
            prefit_xy_nis=36.840,
            prefit_z_nis=6.635,
        )
        self.assertTrue(z_with_xy_robust["factor_enabled"])
        self.assertFalse(z_with_xy_robust["gnss_xy_admitted"])
        self.assertTrue(z_with_xy_robust["gnss_z_admitted"])
        self.assertAlmostEqual(
            z_with_xy_robust["gnss_xy_information_scale"], 0.5
        )
        self.assertEqual(z_with_xy_robust["gnss_z_information_scale"], 1.0)
        self.assertEqual(
            z_with_xy_robust["admission_reason"],
            "admitted_z_with_xy_robust",
        )

        xy_with_z_robust = apply_gnss_prefit_gate(
            scheduler_decision(1.0, enabled=True, inflation=1.0),
            prefit_xy_nis=9.210,
            prefit_z_nis=26.540,
        )
        self.assertTrue(xy_with_z_robust["factor_enabled"])
        self.assertTrue(xy_with_z_robust["gnss_xy_admitted"])
        self.assertFalse(xy_with_z_robust["gnss_z_admitted"])
        self.assertEqual(xy_with_z_robust["gnss_xy_information_scale"], 1.0)
        self.assertAlmostEqual(
            xy_with_z_robust["gnss_z_information_scale"], 0.5
        )
        self.assertEqual(
            xy_with_z_robust["admission_reason"],
            "admitted_xy_with_z_robust",
        )

        recovery = apply_gnss_prefit_gate(
            scheduler_decision(0.8, enabled=True, inflation=1.25),
            prefit_xy_nis=9.211,
            prefit_z_nis=6.636,
        )
        self.assertTrue(recovery["factor_enabled"])
        self.assertTrue(recovery["gnss_recovery_floor"])
        self.assertAlmostEqual(recovery["reliability_weight"], 0.8)
        self.assertAlmostEqual(recovery["covariance_inflation"], 1.25)
        self.assertEqual(
            recovery["admission_reason"], "admitted_robust_all_axes"
        )

        scheduler_disabled = apply_gnss_prefit_gate(
            scheduler_decision(0.8, enabled=False, inflation=20.0),
            prefit_xy_nis=0.1,
            prefit_z_nis=0.1,
        )
        self.assertFalse(scheduler_disabled["factor_enabled"])
        self.assertEqual(
            scheduler_disabled["admission_reason"], "scheduler_disabled"
        )

    def test_gnss_prefit_prediction_propagates_anchor_covariance(self):
        samples = [
            ImuSample(stamp, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
            for stamp in (1.0, 1.05, 1.10)
        ]
        measurement = preintegrate_manifold(
            samples, 1.0, 1.10, max_gap_s=0.06
        )
        node = object.__new__(UnifiedBackendNode)
        node.optimization_anchor_lock = threading.Lock()
        node.optimization_anchor = make_optimization_anchor(
            1.0, np.zeros(15), np.eye(15), generation=1
        )

        prediction, reason = node._gnss_prefit_prediction(1.10, measurement)

        self.assertEqual(reason, "ok")
        position, covariance = prediction
        np.testing.assert_allclose(position, np.zeros(3), atol=1.0e-9)
        self.assertEqual(covariance.shape, (3, 3))
        self.assertTrue(np.all(np.isfinite(covariance)))
        self.assertGreater(float(np.min(np.linalg.eigvalsh(covariance))), 0.0)

    def test_gnss_time_compensation_preserves_observation_innovation(self):
        measured = np.asarray([2.0, -1.0, 4.0])
        observation_prediction = np.asarray([1.5, -0.5, 3.0])
        factor_prediction = np.asarray([1.9, -0.7, 3.8])
        transported, covariance, delta = time_compensate_gnss_observation(
            measured,
            [0.4, 0.5, 0.6],
            observation_prediction,
            np.diag([0.2, 0.3, 0.4]),
            factor_prediction,
            np.diag([0.3, 0.35, 0.6]),
        )

        np.testing.assert_allclose(delta, [0.4, -0.2, 0.8])
        np.testing.assert_allclose(transported, [2.4, -1.2, 4.8])
        np.testing.assert_allclose(covariance, [0.5, 0.55, 0.8])
        np.testing.assert_allclose(
            factor_prediction - transported,
            observation_prediction - measured,
        )

    def test_gnss_factor_counter_counts_only_enabled_solver_records(self):
        class Backend:
            def __init__(self):
                self.calls = []

            def add_gnss(self, index, position, covariance, decision):
                self.calls.append((index, position, covariance, decision))

        def node_with_decision(decision):
            node = object.__new__(UnifiedBackendNode)
            node.projector = object()
            node.lio_origin = np.zeros(3)
            node.gnss_lock = threading.Lock()
            node.gnss_buffer = deque([{
                "stamp_s": 10.0,
                "position_enu": np.zeros(3),
                "covariance": [1.0, 1.0, 1.0],
                "status": 0,
                "temporal_jump": False,
            }])
            node.gnss_max_age_s = 2.0
            node.gnss_future_tolerance_s = 0.05
            node.gnss_xy_nis_gate = 9.210
            node.gnss_z_nis_gate = 6.635
            node.gnss_minimum_reliability_weight = 0.05
            node.gnss_minimum_axis_information_scale = 0.01
            node.backend = Backend()
            node._decision = lambda *_args, **_kwargs: dict(decision)
            node.last_gnss_time_compensation_age_s = 0.0
            node.last_gnss_time_compensation_delta_m = np.zeros(3)
            node.last_gnss_time_compensation_variance_m2 = np.zeros(3)
            node.last_gnss_time_compensation_reason = "not_attempted"
            node.counts = {
                "gnss_stale_discarded": 0,
                "gnss_superseded": 0,
                "gnss_consumed": 0,
                "gnss_factor_attempts": 0,
                "gnss_jump_rejected": 0,
                "gnss_prefit_covariance_unavailable": 0,
                "gnss_prefit_invalid": 0,
                "gnss_prefit_valid": 0,
                "gnss_factor_records": 0,
                "gnss_factors": 0,
                "gnss_provisional_bootstrap_admitted": 0,
                "gnss_disabled_scheduler": 0,
                "gnss_rejected_nis": 0,
                "gnss_rejected_low_weight": 0,
                "gnss_invalid_fix_rejected": 0,
                "gnss_xy_rejected_nis": 0,
                "gnss_z_rejected_nis": 0,
                "gnss_xy_admitted": 0,
                "gnss_z_admitted": 0,
                "gnss_xy_robust_downweighted": 0,
                "gnss_z_robust_downweighted": 0,
                "gnss_all_axes_inconsistent": 0,
                "gnss_prefit_recovery_floor": 0,
            }
            return node

        enabled = node_with_decision(
            scheduler_decision(0.8, enabled=True, inflation=1.25)
        )
        enabled._gnss_factor(10.0, [2.0, 0.0, 0.0], 3, np.eye(3) * 4.0)
        self.assertEqual(enabled.counts["gnss_factor_records"], 1)
        self.assertEqual(enabled.counts["gnss_factors"], 1)
        self.assertTrue(enabled.backend.calls[0][3]["factor_enabled"])

        compensated = node_with_decision(
            scheduler_decision(1.0, enabled=True, inflation=1.0)
        )
        compensated.gnss_buffer[0]["stamp_s"] = 9.6
        compensated.gnss_buffer[0]["position_enu"] = np.asarray(
            [1.0, 2.0, 3.0]
        )
        compensated._gnss_factor(
            10.0,
            [0.7, 1.6, 2.5],
            3,
            np.diag([0.3, 0.3, 0.4]),
            factor_velocity=[0.5, 0.25, 1.25],
        )
        np.testing.assert_allclose(
            compensated.backend.calls[0][1], [1.2, 2.1, 3.5]
        )
        np.testing.assert_allclose(
            compensated.backend.calls[0][2], [1.0, 1.0, 1.0]
        )
        self.assertAlmostEqual(
            compensated.last_gnss_time_compensation_age_s, 0.4
        )
        self.assertEqual(
            compensated.last_gnss_time_compensation_reason, "applied"
        )

        disabled = node_with_decision(
            scheduler_decision(0.8, enabled=False, inflation=20.0)
        )
        disabled_decision = scheduler_decision(
            0.8, enabled=False, inflation=20.0
        )

        def unexpected_decision_lookup(*_args, **_kwargs):
            raise AssertionError("caller-provided decision must be reused")

        disabled._decision = unexpected_decision_lookup
        disabled._gnss_factor(
            10.0, [2.0, 0.0, 0.0], 3, None,
            "scheduler_disabled_before_prediction", disabled_decision,
        )
        self.assertEqual(disabled.counts["gnss_factor_records"], 0)
        self.assertEqual(disabled.counts["gnss_factors"], 0)
        self.assertEqual(disabled.counts["gnss_disabled_scheduler"], 1)
        self.assertEqual(
            disabled.counts["gnss_prefit_covariance_unavailable"], 0
        )
        self.assertFalse(disabled.backend.calls)

        xy_rejected = node_with_decision(
            scheduler_decision(1.0, enabled=True, inflation=1.0)
        )
        xy_rejected._gnss_factor(
            10.0, [10.0, 0.0, 0.0], 3, np.eye(3) * 0.01
        )
        self.assertEqual(xy_rejected.counts["gnss_factor_records"], 1)
        self.assertEqual(xy_rejected.counts["gnss_xy_rejected_nis"], 1)
        self.assertEqual(xy_rejected.counts["gnss_z_admitted"], 1)
        xy_scale = xy_rejected.backend.calls[0][3][
            "gnss_xy_information_scale"
        ]
        self.assertGreaterEqual(xy_scale, 0.01)
        self.assertLess(xy_scale, 1.0)
        np.testing.assert_allclose(
            xy_rejected.backend.calls[0][2],
            [1.0 / xy_scale, 1.0 / xy_scale, 1.0],
        )
        self.assertEqual(
            xy_rejected.backend.calls[0][3]["admission_reason"],
            "admitted_z_with_xy_robust",
        )

        z_rejected = node_with_decision(
            scheduler_decision(1.0, enabled=True, inflation=1.0)
        )
        z_rejected._gnss_factor(
            10.0, [0.0, 0.0, 10.0], 3, np.eye(3) * 0.01
        )
        self.assertEqual(z_rejected.counts["gnss_factor_records"], 1)
        self.assertEqual(z_rejected.counts["gnss_xy_admitted"], 1)
        self.assertEqual(z_rejected.counts["gnss_z_rejected_nis"], 1)
        z_scale = z_rejected.backend.calls[0][3][
            "gnss_z_information_scale"
        ]
        self.assertGreaterEqual(z_scale, 0.01)
        self.assertLess(z_scale, 1.0)
        np.testing.assert_allclose(
            z_rejected.backend.calls[0][2], [1.0, 1.0, 1.0 / z_scale]
        )
        self.assertEqual(
            z_rejected.backend.calls[0][3]["admission_reason"],
            "admitted_xy_with_z_robust",
        )

        z_reanchor = node_with_decision(
            scheduler_decision(1.0, enabled=True, inflation=1.0)
        )
        z_reanchor.gnss_z_reanchor_enabled = True
        z_reanchor.gnss_z_reanchor_maximum_step_m = 0.15
        z_reanchor.gnss_z_reanchor_minimum_consecutive = 1
        z_reanchor.gnss_z_recovery_information_scale = 0.50
        z_reanchor.last_native_isotropic_information_support = np.asarray(
            [1.0, 1.0, 0.1]
        )
        z_reanchor.axis_handoff_enter_support = 0.35
        z_reanchor._gnss_factor(
            10.0, [0.0, 0.0, 10.0], 3, np.eye(3) * 0.01
        )
        self.assertAlmostEqual(z_reanchor.backend.calls[0][1][2], 9.85)
        self.assertTrue(z_reanchor.last_gnss_z_reanchor_applied)
        self.assertEqual(z_reanchor.counts["gnss_z_reanchor_factors"], 1)
        self.assertEqual(z_reanchor.counts["gnss_z_recovery_factors"], 1)
        self.assertGreaterEqual(
            z_reanchor.backend.calls[0][3]["gnss_z_information_scale"], 0.50
        )

        recovery = node_with_decision(
            scheduler_decision(1.0, enabled=True, inflation=1.0)
        )
        recovery._gnss_factor(
            10.0, [10.0, 0.0, 10.0], 3, np.eye(3) * 0.01
        )
        self.assertEqual(recovery.counts["gnss_factor_records"], 1)
        self.assertEqual(recovery.counts["gnss_rejected_nis"], 0)
        self.assertEqual(recovery.counts["gnss_all_axes_inconsistent"], 1)
        self.assertEqual(recovery.counts["gnss_prefit_recovery_floor"], 1)
        recovery_decision = recovery.backend.calls[0][3]
        self.assertLess(recovery_decision["gnss_xy_information_scale"], 1.0)
        self.assertLess(recovery_decision["gnss_z_information_scale"], 1.0)
        self.assertTrue(np.all(np.isfinite(recovery.backend.calls[0][2])))
        self.assertAlmostEqual(
            recovery_decision["reliability_weight"], 1.0
        )
        self.assertAlmostEqual(
            recovery_decision["covariance_inflation"], 1.0
        )

        low_weight = node_with_decision(
            scheduler_decision(0.04, enabled=True, inflation=20.0)
        )
        low_weight._gnss_factor(
            10.0, [0.1, 0.0, 0.0], 3, np.eye(3) * 4.0
        )
        self.assertEqual(low_weight.counts["gnss_factor_records"], 0)
        self.assertEqual(low_weight.counts["gnss_rejected_low_weight"], 1)
        self.assertFalse(low_weight.backend.calls)

        invalid_fix = node_with_decision(
            scheduler_decision(1.0, enabled=True, inflation=1.0)
        )
        invalid_fix.gnss_buffer[0]["status"] = -1
        invalid_fix._gnss_factor(
            10.0, [0.0, 0.0, 0.0], 3, np.eye(3) * 4.0
        )
        self.assertEqual(invalid_fix.counts["gnss_invalid_fix_rejected"], 1)
        self.assertFalse(invalid_fix.backend.calls)

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

    def test_native_worker_skips_dequeued_frame_when_newer_is_pending(self):
        owner = SimpleNamespace(
            native_work_queue=queue.Queue(maxsize=1),
            counts={"native_worker_latest_skipped": 0},
            last_reason="none",
        )
        consumed = []
        owner._consume_native_sequence = lambda sequence, state_committed, intentional_latest_skip=False: consumed.append(
            (sequence, state_committed, intentional_latest_skip)
        )
        owner.native_work_queue.put_nowait(
            (Header(), SimpleNamespace(scan_sequence=12))
        )

        skipped = UnifiedBackendNode._native_worker_frame_superseded(
            owner, SimpleNamespace(scan_sequence=11)
        )

        self.assertTrue(skipped)
        self.assertEqual(owner.counts["native_worker_latest_skipped"], 1)
        self.assertEqual(owner.last_reason, "native_worker_latest_only_skip")
        self.assertEqual(consumed, [(11, False, True)])

    def test_native_worker_processes_frame_when_no_newer_is_pending(self):
        owner = SimpleNamespace(
            native_work_queue=queue.Queue(maxsize=1),
            counts={"native_worker_latest_skipped": 0},
            last_reason="none",
        )
        consumed = []
        owner._consume_native_sequence = lambda *args, **kwargs: consumed.append(
            (args, kwargs)
        )

        skipped = UnifiedBackendNode._native_worker_frame_superseded(
            owner, SimpleNamespace(scan_sequence=11)
        )

        self.assertFalse(skipped)
        self.assertEqual(owner.counts["native_worker_latest_skipped"], 0)
        self.assertEqual(consumed, [])

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

    def test_imu_pair_timeout_requires_committed_state_without_factor(self):
        self.assertFalse(
            committed_state_missing_imu_factor(True, True, 10, 11)
        )
        self.assertTrue(
            committed_state_missing_imu_factor(True, True, 10, 10)
        )
        self.assertFalse(
            committed_state_missing_imu_factor(False, True, 10, 10)
        )
        self.assertFalse(
            committed_state_missing_imu_factor(True, False, 10, 10)
        )

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

    def test_mtf01p_range_accuracy_matches_vendor_contract(self):
        self.assertAlmostEqual(mtf01p_range_sigma_m(0.08), 0.04)
        self.assertAlmostEqual(mtf01p_range_sigma_m(2.0), 0.04)
        self.assertAlmostEqual(mtf01p_range_sigma_m(5.0), 0.10)
        self.assertTrue(math.isinf(mtf01p_range_sigma_m(float("nan"))))

    def test_range_facet_builder_uses_explicit_manifold_state_slices(self):
        node = UnifiedBackendNode.__new__(UnifiedBackendNode)
        node.range_facet_enabled = True
        node.range_facet_minimum_support_points = 3
        node.range_facet_maximum_plane_rmse_m = 0.05
        node.range_facet_denominator_epsilon = 0.05
        node.range_facet_facet_margin_m = 0.25
        node.range_facet_timestamp_tolerance_s = 0.08
        node.range_facet_mahalanobis_gate = 9.0
        node.minimum_flow_distance_m = 0.08
        node.maximum_flow_distance_m = 12.0
        node.flow_sensor_offset_body_m = np.asarray([0.0, 0.0, -0.35])
        node.last_flow_range_sigma_m = 0.04
        native = SimpleNamespace(
            plane_normals=np.tile([0.0, 0.0, 1.0], (4, 1)),
            plane_points=np.asarray([
                [-1.0, -1.0, 1.0],
                [1.0, -1.0, 1.0],
                [1.0, 1.0, 1.0],
                [-1.0, 1.0, 1.0],
            ]),
            stamp_s=10.0,
            linearization_pose=np.asarray([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]),
            lidar_to_body_rotation=np.eye(3),
            lidar_to_body_translation=np.zeros(3),
        )
        observation, result = node._build_range_facet_observation(
            [{"stamp_s": 10.0}],
            {"distance_m": 0.65},
            native,
            np.asarray([0.0, 0.0, 2.0] + [0.0] * 12),
            10.0,
        )
        self.assertIsNotNone(observation)
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.predicted_range_m, 0.65)

    def test_flow_covariance_includes_range_accuracy_without_z_row(self):
        covariance = optical_flow_displacement_covariance_m2(
            [1.0, 0.0], 1.0, base_sigma_m=0.10
        )
        self.assertEqual(len(covariance), 2)
        self.assertAlmostEqual(covariance[0], 0.10 ** 2 + 0.04 ** 2)
        self.assertAlmostEqual(covariance[1], covariance[0])

    def test_mtf01p_speed_envelope_scales_with_range(self):
        valid, speed, limit = mtf01p_flow_speed_gate(
            [0.70, 0.0], 0.10, 1.0, margin=1.10
        )
        self.assertTrue(valid)
        self.assertAlmostEqual(speed, 7.0)
        self.assertAlmostEqual(limit, 7.7)
        self.assertFalse(
            mtf01p_flow_speed_gate(
                [0.80, 0.0], 0.10, 1.0, margin=1.10
            )[0]
        )
        self.assertTrue(
            mtf01p_flow_speed_gate(
                [0.80, 0.0], 0.10, 2.0, margin=1.10
            )[0]
        )

    def test_flow_aggregation_rejects_nonpositive_distance(self):
        observation = flow_observation_delta([
            {
                "integrated_x": 0.0,
                "integrated_y": 1.0,
                "integrated_xgyro": 0.0,
                "integrated_ygyro": 0.0,
                "quality": 200,
                "distance_m": 1.0,
                "integration_time_s": 0.01,
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
        self.assertAlmostEqual(observation["integration_s"], 0.01)

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

    def test_disabled_flow_consumes_interval_without_geometry_work(self):
        node = object.__new__(UnifiedBackendNode)
        node.flow_buffer_lock = threading.Lock()
        node.flow_buffer = deque([
            {"stamp_s": 10.05},
            {"stamp_s": 10.20},
        ], maxlen=3000)
        node.flow_max_age_s = 0.5
        node.counts = {
            "flow_factor_attempts": 0,
            "flow_disabled_scheduler": 0,
        }
        node.last_flow_reason = "unavailable"
        node._decision = lambda *_args, **_kwargs: scheduler_decision(
            0.0, enabled=False, inflation=20.0
        )

        node._flow_factor(
            10.0, 10.1, 0.0, 0, 1, [0.0, 0.0],
            previous_state=np.zeros(15),
        )

        self.assertEqual(node.counts["flow_factor_attempts"], 1)
        self.assertEqual(node.counts["flow_disabled_scheduler"], 1)
        self.assertEqual(node.last_flow_reason, "scheduler_disabled")
        self.assertEqual(
            [item["stamp_s"] for item in node.flow_buffer], [10.20]
        )

    def test_flow_lever_arm_reuses_cycle_imu_samples(self):
        node = object.__new__(UnifiedBackendNode)
        node.flow_lever_arm_compensation_enabled = True
        node.flow_sensor_offset_body_m = (0.0, 0.0, -0.35)
        node.flow_rotation_imu_max_gap_s = 0.20
        node.counts = {
            "flow_lever_arm_unavailable": 0,
            "flow_lever_arm_per_exposure": 0,
            "flow_lever_arm_interval_fallback": 0,
            "flow_lever_arm_compensated": 0,
        }
        node.flow_lever_arm_displacement_norms = deque(maxlen=64)
        node.last_flow_lever_arm_displacement = None

        def unexpected_snapshot():
            raise AssertionError("cycle IMU samples must be reused")

        node._imu_snapshot = unexpected_snapshot
        correction, evidence = node._flow_lever_arm_correction(
            [{
                "stamp_s": 10.05,
                "integration_time_s": 0.10,
                "integrated_x": 0.0,
                "integrated_y": 0.0,
                "integrated_xgyro": 0.0,
                "integrated_ygyro": 0.0,
                "distance_m": 2.0,
            }],
            9.95,
            10.05,
            np.zeros(15),
            [
                (9.90, (0.0, 0.0, 0.2)),
                (10.00, (0.0, 0.0, 0.2)),
                (10.10, (0.0, 0.0, 0.2)),
            ],
        )

        self.assertEqual(evidence["source"], "per_exposure_imu")
        self.assertEqual(node.counts["flow_lever_arm_per_exposure"], 1)
        self.assertTrue(np.all(np.isfinite(correction)))

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

    def test_lidar_prediction_rejection_disables_only_current_factor(self):
        admission = lidar_prediction_factor_admission(
            {"position_m": 0.2, "yaw_rad": 0.51},
            1.0,
            0.5,
            consecutive_rejections=0,
        )

        self.assertFalse(admission["factor_enabled"])
        self.assertEqual(admission["reason"], "lidar_prediction_yaw_gate")
        self.assertEqual(admission["consecutive_rejections"], 1)
        self.assertFalse(admission["recovered"])
        self.assertFalse(admission["recovery_floor"])

    def test_lidar_prediction_gate_does_not_reinject_frame_inconsistent_factor(self):
        admission = lidar_prediction_factor_admission(
            {"position_m": 1.2, "yaw_rad": 0.1},
            1.0,
            0.5,
            consecutive_rejections=2,
            recovery_after_rejections=3,
            recovery_geometry_usable=True,
        )

        self.assertFalse(admission["factor_enabled"])
        self.assertEqual(admission["reason"], "lidar_prediction_position_gate")
        self.assertEqual(admission["consecutive_rejections"], 3)
        self.assertFalse(admission["recovered"])
        self.assertFalse(admission["recovery_floor"])

    def test_lidar_prediction_rejection_stays_disabled_without_usable_geometry(self):
        admission = lidar_prediction_factor_admission(
            {"position_m": 1.2, "yaw_rad": 0.1},
            1.0,
            0.5,
            consecutive_rejections=20,
            recovery_after_rejections=3,
            recovery_geometry_usable=False,
        )

        self.assertFalse(admission["factor_enabled"])
        self.assertEqual(admission["consecutive_rejections"], 21)
        self.assertFalse(admission["recovery_floor"])

    def test_lidar_prediction_gate_recovers_on_next_healthy_factor(self):
        admission = lidar_prediction_factor_admission(
            {"position_m": 0.2, "yaw_rad": 0.1},
            1.0,
            0.5,
            consecutive_rejections=3,
        )

        self.assertTrue(admission["factor_enabled"])
        self.assertEqual(admission["reason"], "ok")
        self.assertEqual(admission["consecutive_rejections"], 0)
        self.assertTrue(admission["recovered"])
        self.assertFalse(admission["recovery_floor"])

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
        node.pending_scan_request_first_seen_s = {2: 9.9}
        node.scan_prediction_missing_factor_grace_s = 0.5
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

    def test_retried_scan_request_skips_missing_predecessor_after_grace(self):
        node = object.__new__(UnifiedBackendNode)
        node.frontend_scan_prediction_enabled = True
        node.last_scan_request_arrival_s = None
        node.last_native_consumed_sequence = 34
        node.pending_scan_request_lock = threading.Lock()
        request = SimpleNamespace(scan_sequence=36)
        node.pending_scan_requests = {36: request}
        node.pending_scan_request_first_seen_s = {36: 10.0}
        node.scan_prediction_missing_factor_grace_s = 0.5
        node.scan_prediction_by_sequence = {}
        node.counts = {
            "scan_prediction_requests": 0,
            "scan_prediction_duplicate_requests": 0,
            "scan_prediction_stale_requests": 0,
            "scan_prediction_deferred": 0,
            "scan_prediction_missing_factor_skips": 0,
        }
        node._now_s = lambda: 10.6
        produced = []
        node._produce_scan_prediction = produced.append

        node._scan_request(request)

        self.assertEqual(node.last_native_consumed_sequence, 35)
        self.assertEqual(
            node.counts["scan_prediction_missing_factor_skips"], 1
        )
        self.assertFalse(node.pending_scan_requests)
        self.assertFalse(node.pending_scan_request_first_seen_s)
        self.assertEqual(produced, [request])

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
        node.pending_scan_request_first_seen_s = {15: 9.0}
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
        node.relocalization_future_wait_timeout_s = 8.0
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
                stamp=SimpleNamespace(sec=10, nanosec=800_000_000)
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
        self.assertEqual(
            node.last_reason,
            "relocalization_waiting_for_backend_state",
        )
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
