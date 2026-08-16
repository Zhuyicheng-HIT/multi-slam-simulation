import math
import unittest

import numpy as np

from uf_backend_fusion.imu_preintegration import ImuSample
from uf_backend_fusion.manifold import so3_exp, so3_log
from uf_backend_fusion.spatiotemporal_calibration import (
    CalibrationUpdate,
    LidarMotionSample,
    OnlineSpatiotemporalCalibrator,
    _estimate_interval_time_offset_prepared,
    _estimate_time_offset_prepared,
    _integrate_gyro,
    _integrate_prepared_gyro,
    _integrate_prepared_orientation,
    _prepare_gyro_interpolation,
    _prepare_gyro_orientation_trajectory,
    effective_time_offset,
    estimate_time_offset,
)


class SpatiotemporalCalibrationTest(unittest.TestCase):
    def test_prepared_gyro_integration_matches_public_helper(self):
        samples = [
            ImuSample(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (0.1 + 0.01 * index, -0.02, 0.15),
            )
            for index in range(31)
        ]
        stamps, values = _prepare_gyro_interpolation(samples)
        expected = _integrate_gyro(samples, 0.035, 0.265)
        actual = _integrate_prepared_gyro(
            stamps, values, 0.035, 0.265
        )
        np.testing.assert_allclose(actual, expected, atol=1.0e-12)

    def test_prepared_time_offset_matches_public_helper(self):
        samples = [
            ImuSample(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (0.4 + 0.2 * math.sin(index * 0.07), 0.0, 0.0),
            )
            for index in range(401)
        ]
        rates = [
            (stamp, 0.4 + 0.2 * math.sin((stamp + 0.02) * 7.0))
            for stamp in np.arange(0.5, 3.5, 0.1)
        ]
        offsets = np.arange(-0.05, 0.0501, 0.005)
        expected = estimate_time_offset(
            rates, samples, offsets, minimum_pairs=10
        )
        stamps, values = _prepare_gyro_interpolation(samples)
        actual = _estimate_time_offset_prepared(
            rates, stamps, values, offsets, minimum_pairs=10
        )
        self.assertEqual(actual, expected)

    def test_tentative_time_offset_is_diagnostic_only_until_locked(self):
        tentative = CalibrationUpdate(
            True, False, -0.08, np.eye(3), 0.85, 0.10, -1.0,
            (0.0, 0.0, 0.0), 20, "accepted",
        )
        locked = CalibrationUpdate(
            True, True, 0.035, np.eye(3), 0.90, 0.12, 0.01,
            (0.01, 0.02, 0.03), 30, "accepted",
        )
        self.assertEqual(effective_time_offset(tentative), 0.0)
        self.assertAlmostEqual(
            effective_time_offset(tentative, time_locked=True), -0.08
        )
        self.assertEqual(effective_time_offset(locked, enabled=False), 0.0)
        self.assertAlmostEqual(effective_time_offset(locked), 0.035)

    def test_time_offset_correlation_recovers_delayed_imu(self):
        true_offset = 0.035

        def gyro_at(stamp):
            return np.asarray([
                0.12 + 0.05 * math.sin(1.7 * stamp),
                0.08 * math.cos(1.1 * stamp),
                0.20 + 0.07 * math.sin(2.3 * stamp),
            ])

        samples = [
            ImuSample(index * 0.01, (0.0, 0.0, 0.0), tuple(gyro_at(index * 0.01)))
            for index in range(601)
        ]
        rates = [
            (index * 0.05, float(np.linalg.norm(gyro_at(index * 0.05 + true_offset))))
            for index in range(1, 95)
        ]
        result = estimate_time_offset(
            rates, samples, np.arange(-0.10, 0.1001, 0.005), minimum_pairs=20
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.offset_s, true_offset, delta=0.006)
        self.assertGreater(result.correlation, 0.95)

    def test_time_offset_margin_uses_an_independent_peak(self):
        true_offset = 0.020

        def rate(stamp):
            return 0.5 + 0.2 * math.sin(4.1 * stamp) + 0.1 * math.sin(9.3 * stamp)

        samples = [
            ImuSample(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (rate(index * 0.01 - true_offset), 0.0, 0.0),
            )
            for index in range(601)
        ]
        rates = [(stamp, rate(stamp)) for stamp in np.arange(0.5, 5.5, 0.1)]
        offsets = np.arange(-0.05, 0.0501, 0.002)
        adjacent = estimate_time_offset(
            rates, samples, offsets, minimum_pairs=20
        )
        independent = estimate_time_offset(
            rates,
            samples,
            offsets,
            minimum_pairs=20,
            minimum_peak_separation_s=0.010,
        )

        self.assertAlmostEqual(independent.offset_s, true_offset, delta=0.004)
        self.assertGreater(independent.margin, adjacent.margin)

    def test_interval_time_offset_uses_matching_integration_windows(self):
        true_offset = 0.035

        def gyro_at(stamp):
            return np.asarray([
                0.15 * math.sin(2.3 * stamp),
                0.10 * math.cos(1.7 * stamp + 0.2),
                0.35 * math.sin(3.1 * stamp)
                + 0.12 * math.sin(7.3 * stamp),
            ])

        samples = [
            ImuSample(
                index * 0.005,
                (0.0, 0.0, 0.0),
                tuple(gyro_at(index * 0.005)),
            )
            for index in range(1601)
        ]
        stamps, values = _prepare_gyro_interpolation(samples)
        orientations, segments = _prepare_gyro_orientation_trajectory(
            stamps, values
        )
        intervals = []
        for start_s in np.arange(0.5, 6.8, 0.18):
            end_s = start_s + 0.16
            rotation = _integrate_prepared_orientation(
                stamps,
                values,
                orientations,
                segments,
                start_s + true_offset,
                end_s + true_offset,
            )
            intervals.append((start_s, end_s, so3_log(rotation), 1.0))
        result = _estimate_interval_time_offset_prepared(
            intervals,
            stamps,
            values,
            np.arange(-0.10, 0.1001, 0.005),
            np.eye(3),
            minimum_pairs=20,
            minimum_peak_separation_s=0.020,
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.offset_s, true_offset, delta=0.006)
        self.assertGreater(result.correlation, 0.95)
        self.assertGreater(result.margin, 0.002)

    def test_interval_time_offset_locks_after_stable_updates(self):
        true_offset = -0.040

        def gyro_at(stamp):
            return np.asarray([
                0.14 * math.sin(2.5 * stamp),
                0.11 * math.cos(1.9 * stamp),
                0.30 * math.sin(3.4 * stamp)
                + 0.10 * math.cos(7.7 * stamp),
            ])

        samples = [
            ImuSample(
                index * 0.005,
                (0.0, 0.0, 0.0),
                tuple(gyro_at(index * 0.005)),
            )
            for index in range(1801)
        ]
        stamps, values = _prepare_gyro_interpolation(samples)
        orientations, segments = _prepare_gyro_orientation_trajectory(
            stamps, values
        )
        calibrator = OnlineSpatiotemporalCalibrator(
            window_s=7.0,
            minimum_pairs=20,
            minimum_correlation=0.70,
            minimum_correlation_margin=0.002,
            minimum_time_peak_separation_s=0.020,
            minimum_time_accumulated_rotation_rad=0.25,
            sharp_turn_rate_radps=1.5,
            lock_count=3,
            solve_period_s=0.0,
        )
        calibrator.set_initial_rotation(np.eye(3))
        for start_s in np.arange(0.6, 7.6, 0.18):
            end_s = start_s + 0.16
            rotation = _integrate_prepared_orientation(
                stamps,
                values,
                orientations,
                segments,
                start_s + true_offset,
                end_s + true_offset,
            )
            calibrator.update(
                LidarMotionSample(start_s, end_s, rotation, weight=1.0),
                samples,
            )

        self.assertTrue(calibrator.time_locked)
        self.assertAlmostEqual(
            calibrator.time_offset_s, true_offset, delta=0.006
        )

    def test_time_lock_requires_one_consecutive_candidate_cluster(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=3,
            lock_count=3,
            stability_tolerance_s=0.008,
        )

        self.assertFalse(calibrator._update_time_lock(-0.015, 0.0))
        self.assertFalse(calibrator._update_time_lock(-0.035, 1.0))
        self.assertFalse(calibrator._update_time_lock(-0.035, 2.0))
        self.assertEqual(list(calibrator.time_offset_history), [-0.035, -0.035])
        self.assertFalse(calibrator.time_locked)

        self.assertTrue(calibrator._update_time_lock(-0.030, 3.0))
        self.assertTrue(calibrator.time_locked)
        self.assertAlmostEqual(calibrator.time_offset_s, -0.035)

    def test_time_lock_ignores_an_isolated_conflicting_candidate(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=3,
            lock_count=3,
            stability_tolerance_s=0.008,
        )
        for stamp_s, offset_s in enumerate((-0.005, 0.0, 0.005)):
            calibrator._update_time_lock(offset_s, stamp_s)
        locked_offset_s = calibrator.time_offset_s

        self.assertTrue(calibrator.time_locked)
        self.assertFalse(calibrator._update_time_lock(0.050, 3.0))
        self.assertTrue(calibrator.time_locked)
        self.assertAlmostEqual(calibrator.time_offset_s, locked_offset_s)

    def test_time_lock_votes_require_independent_motion_intervals(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=3,
            lock_count=3,
            minimum_time_lock_candidate_separation_s=1.0,
        )

        self.assertFalse(calibrator._update_time_lock(0.020, 10.0))
        self.assertFalse(calibrator._update_time_lock(0.020, 10.2))
        self.assertFalse(calibrator._update_time_lock(0.020, 10.4))
        self.assertEqual(calibrator.time_lock_candidate_count, 1)
        self.assertEqual(list(calibrator.time_offset_history), [0.020])

        self.assertFalse(calibrator._update_time_lock(0.020, 11.1))
        self.assertTrue(calibrator._update_time_lock(0.020, 12.2))
        self.assertTrue(calibrator.time_locked)

    def test_stable_high_confidence_contradictions_revoke_time_lock(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=3,
            lock_count=3,
            stability_tolerance_s=0.008,
            minimum_time_lock_candidate_separation_s=1.0,
            time_unlock_count=3,
        )
        for stamp_s in (0.0, 1.0, 2.0):
            calibrator._update_time_lock(0.0, stamp_s)
        self.assertTrue(calibrator.time_locked)

        self.assertFalse(calibrator._update_time_lock(0.050, 3.0))
        self.assertFalse(calibrator._update_time_lock(0.052, 4.0))
        self.assertTrue(calibrator.time_locked)
        self.assertFalse(calibrator._update_time_lock(0.048, 5.0))

        self.assertFalse(calibrator.time_locked)
        self.assertEqual(calibrator.time_lock_revocations, 1)
        self.assertEqual(calibrator.time_lock_conflict_count, 3)

    def test_static_intervals_never_lock_time_offset(self):
        samples = [
            ImuSample(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
            for index in range(801)
        ]
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=8,
            minimum_correlation_margin=0.0,
            minimum_time_accumulated_rotation_rad=0.25,
            lock_count=3,
        )
        calibrator.set_initial_rotation(np.eye(3))
        for start_s in np.arange(0.5, 5.5, 0.2):
            calibrator.update(
                LidarMotionSample(start_s, start_s + 0.15, np.eye(3)),
                samples,
            )

        self.assertFalse(calibrator.time_locked)
        self.assertEqual(
            calibrator.last_time_candidate.reason,
            "unexcited_interval_rotation",
        )

    def test_inconsistent_interval_rotations_never_lock_time_offset(self):
        rng = np.random.default_rng(7)
        samples = [
            ImuSample(
                index * 0.005,
                (0.0, 0.0, 0.0),
                (
                    0.15 * math.sin(index * 0.013),
                    0.10 * math.cos(index * 0.017),
                    0.20 * math.sin(index * 0.023),
                ),
            )
            for index in range(1601)
        ]
        calibrator = OnlineSpatiotemporalCalibrator(
            window_s=7.0,
            minimum_pairs=12,
            minimum_correlation=0.70,
            minimum_correlation_margin=0.002,
            minimum_time_accumulated_rotation_rad=0.25,
            lock_count=3,
        )
        calibrator.set_initial_rotation(np.eye(3))
        for start_s in np.arange(0.6, 6.8, 0.18):
            random_vector = rng.normal(0.0, 0.08, size=3)
            calibrator.update(
                LidarMotionSample(
                    start_s,
                    start_s + 0.16,
                    so3_exp(random_vector),
                ),
                samples,
            )

        self.assertFalse(calibrator.time_locked)
        self.assertLess(calibrator.last_time_candidate.correlation, 0.70)

    def test_interval_time_offset_rejects_boundary_peak(self):
        samples = [
            ImuSample(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, math.sin(index * 0.07)),
            )
            for index in range(601)
        ]
        stamps, values = _prepare_gyro_interpolation(samples)
        orientations, segments = _prepare_gyro_orientation_trajectory(
            stamps, values
        )
        intervals = []
        for start_s in np.arange(0.5, 4.8, 0.2):
            end_s = start_s + 0.15
            rotation = _integrate_prepared_orientation(
                stamps,
                values,
                orientations,
                segments,
                start_s + 0.05,
                end_s + 0.05,
            )
            intervals.append((start_s, end_s, so3_log(rotation), 1.0))
        result = _estimate_interval_time_offset_prepared(
            intervals,
            stamps,
            values,
            [-0.05, 0.0, 0.05],
            np.eye(3),
            minimum_pairs=8,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "peak_at_search_boundary")

    def test_time_offset_reports_insufficient_overlap(self):
        samples = [
            ImuSample(index * 0.01, (0.0, 0.0, 0.0), (0.1, 0.0, 0.0))
            for index in range(20)
        ]
        rates = [(1.0 + index * 0.1, 0.1 + 0.01 * index) for index in range(8)]
        result = estimate_time_offset(
            rates,
            samples,
            [-0.01, 0.0, 0.01],
            minimum_pairs=5,
            minimum_peak_separation_s=0.005,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "insufficient_overlapping_samples")

    def test_rotation_hand_eye_requires_three_axis_excitation(self):
        extrinsic = so3_exp(np.asarray([0.25, -0.18, 0.31]))
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=5,
            time_offset_range_s=0.0,
            minimum_correlation=0.2,
            minimum_correlation_margin=0.0,
            minimum_excitation_eigenvalue=1.0e-6,
            maximum_rotation_residual_rad=0.08,
            update_alpha=1.0,
            lock_count=1,
        )
        increments = [
            np.asarray([0.08, 0.00, 0.02]),
            np.asarray([0.00, -0.07, 0.03]),
            np.asarray([0.04, 0.05, 0.00]),
            np.asarray([-0.05, 0.02, 0.06]),
            np.asarray([0.03, -0.04, -0.05]),
            np.asarray([0.06, 0.03, 0.04]),
            np.asarray([-0.02, 0.06, -0.03]),
        ]
        body_gyros = [extrinsic @ increment / 0.1 for increment in increments]
        imu_samples = []
        for index in range(701):
            stamp_s = index * 0.001
            segment = min(len(body_gyros) - 1, int(stamp_s / 0.1))
            imu_samples.append(ImuSample(
                stamp_s, (0.0, 0.0, 0.0), tuple(body_gyros[segment])
            ))
        body_rotation = np.eye(3)
        stamp = 0.0
        calibrator.set_initial_rotation(extrinsic)
        for increment in increments:
            next_body = body_rotation @ extrinsic @ so3_exp(increment) @ extrinsic.T
            next_stamp = stamp + 0.1
            body_rotation, stamp = next_body, next_stamp
            update = calibrator.update(
                LidarMotionSample(
                    stamp - 0.1, stamp, so3_exp(increment), weight=0.9
                ),
                imu_samples,
            )
        self.assertTrue(update.accepted)
        self.assertGreater(update.excitation_eigenvalues[0], 1.0e-6)
        self.assertLess(
            np.linalg.norm(so3_log(extrinsic.T @ update.lidar_to_body_rotation)),
            0.03,
        )

    def test_yaw_only_motion_does_not_claim_rotation_observability(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=4,
            minimum_correlation=0.2,
            minimum_correlation_margin=0.0,
            minimum_excitation_eigenvalue=1.0e-5,
        )
        samples = []
        for index in range(1, 8):
            stamp = index * 0.1
            increment = np.asarray([0.0, 0.0, 0.04 + 0.01 * (index % 2)])
            gyro = increment / 0.1
            samples.append(ImuSample(stamp - 0.1, (0.0, 0.0, 0.0), tuple(gyro)))
            samples.append(ImuSample(stamp, (0.0, 0.0, 0.0), tuple(gyro)))
            update = calibrator.update(
                LidarMotionSample(stamp - 0.1, stamp, so3_exp(increment)),
                samples,
            )
        self.assertLess(update.excitation_eigenvalues[0], 1.0e-5)
        self.assertFalse(calibrator.rotation_locked)
        np.testing.assert_allclose(
            update.lidar_to_body_rotation, np.eye(3), atol=1.0e-12
        )
        self.assertEqual(calibrator.last_time_candidate.reason,
                         "candidate_ready")
        self.assertGreater(calibrator.last_accumulated_rotation_rad, 0.0)
        self.assertGreaterEqual(
            calibrator.last_unweighted_accumulated_rotation_rad,
            calibrator.last_accumulated_rotation_rad,
        )
        self.assertGreater(calibrator.last_imu_accumulated_rotation_rad, 0.0)
        self.assertAlmostEqual(calibrator.last_motion_weight_mean, 1.0)
        self.assertEqual(calibrator.last_rotation_inlier_ratio, 0.0)

    def test_low_quality_weights_do_not_shrink_physical_rotation_gate(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=3,
            time_offset_range_s=0.0,
            minimum_correlation=0.2,
            minimum_correlation_margin=0.0,
            minimum_excitation_eigenvalue=1.0e-6,
            minimum_excitation_ratio=0.05,
            minimum_accumulated_rotation_rad=0.20,
            minimum_rotation_inlier_ratio=0.70,
            maximum_rotation_residual_rad=0.08,
            lock_count=1,
        )
        increments = [
            np.asarray([0.08, 0.0, 0.0]),
            np.asarray([0.0, 0.08, 0.0]),
            np.asarray([0.0, 0.0, 0.08]),
        ]
        imu_samples = []
        for index in range(301):
            stamp_s = index * 0.001
            segment = min(len(increments) - 1, int(stamp_s / 0.1))
            imu_samples.append(ImuSample(
                stamp_s,
                (0.0, 0.0, 0.0),
                tuple(increments[segment] / 0.1),
            ))
        for index, increment in enumerate(increments):
            update = calibrator.update(
                LidarMotionSample(
                    round(index * 0.1, 10),
                    round((index + 1) * 0.1, 10),
                    so3_exp(increment),
                    weight=0.1,
                ),
                imu_samples,
            )

        self.assertAlmostEqual(calibrator.last_accumulated_rotation_rad, 0.24)
        self.assertAlmostEqual(
            calibrator.last_weighted_accumulated_rotation_rad, 0.024
        )
        self.assertTrue(calibrator.rotation_locked)
        self.assertGreaterEqual(update.rotation_residual_rad, 0.0)

    def test_solve_throttle_keeps_collecting_lidar_poses(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=3,
            solve_period_s=1.0,
        )
        first = calibrator.update(
            LidarMotionSample(0.0, 0.1, np.eye(3)), []
        )
        throttled = calibrator.update(
            LidarMotionSample(0.1, 0.2, np.eye(3)), []
        )
        next_solve = calibrator.update(
            LidarMotionSample(0.9, 1.1, np.eye(3)), []
        )

        self.assertNotEqual(first.reason, "update_throttled")
        self.assertEqual(throttled.reason, "update_throttled")
        self.assertNotEqual(next_solve.reason, "update_throttled")
        self.assertEqual(len(calibrator.motions), 3)

    def test_backend_or_imu_pose_cannot_be_passed_as_calibration_motion(self):
        calibrator = OnlineSpatiotemporalCalibrator(minimum_pairs=3)
        with self.assertRaisesRegex(ValueError, "LidarMotionSample"):
            calibrator.update((0.0, 0.1, np.eye(3)), [])

    def test_nonmonotonic_independent_motion_is_rejected(self):
        calibrator = OnlineSpatiotemporalCalibrator(minimum_pairs=3)
        calibrator.update(LidarMotionSample(0.0, 0.1, np.eye(3)), [])
        with self.assertRaisesRegex(ValueError, "nonmonotonic"):
            calibrator.update(LidarMotionSample(0.05, 0.15, np.eye(3)), [])


if __name__ == "__main__":
    unittest.main()
