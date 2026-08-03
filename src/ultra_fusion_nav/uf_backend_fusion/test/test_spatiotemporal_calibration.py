import math
import unittest

import numpy as np

from uf_backend_fusion.imu_preintegration import ImuSample
from uf_backend_fusion.manifold import so3_exp, so3_log
from uf_backend_fusion.spatiotemporal_calibration import (
    CalibrationUpdate,
    OnlineSpatiotemporalCalibrator,
    effective_time_offset,
    estimate_time_offset,
)


class SpatiotemporalCalibrationTest(unittest.TestCase):
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
        imu_samples = []
        body_rotation = np.eye(3)
        stamp = 0.0
        calibrator.update(stamp, body_rotation, extrinsic, imu_samples)
        increments = [
            np.asarray([0.08, 0.00, 0.02]),
            np.asarray([0.00, -0.07, 0.03]),
            np.asarray([0.04, 0.05, 0.00]),
            np.asarray([-0.05, 0.02, 0.06]),
            np.asarray([0.03, -0.04, -0.05]),
            np.asarray([0.06, 0.03, 0.04]),
            np.asarray([-0.02, 0.06, -0.03]),
        ]
        for increment in increments:
            next_body = body_rotation @ extrinsic @ so3_exp(increment) @ extrinsic.T
            next_stamp = stamp + 0.1
            body_gyro = so3_log(body_rotation.T @ next_body) / 0.1
            imu_samples.extend([
                ImuSample(stamp, (0.0, 0.0, 0.0), tuple(body_gyro)),
                ImuSample(next_stamp, (0.0, 0.0, 0.0), tuple(body_gyro)),
            ])
            body_rotation, stamp = next_body, next_stamp
            update = calibrator.update(stamp, body_rotation, extrinsic, imu_samples)
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
        rotation = np.eye(3)
        calibrator.update(0.0, rotation, np.eye(3), samples)
        for index in range(1, 8):
            stamp = index * 0.1
            increment = np.asarray([0.0, 0.0, 0.04 + 0.01 * (index % 2)])
            next_rotation = rotation @ so3_exp(increment)
            gyro = increment / 0.1
            samples.append(ImuSample(stamp - 0.1, (0.0, 0.0, 0.0), tuple(gyro)))
            samples.append(ImuSample(stamp, (0.0, 0.0, 0.0), tuple(gyro)))
            update = calibrator.update(stamp, next_rotation, np.eye(3), samples)
            rotation = next_rotation
        self.assertLess(update.excitation_eigenvalues[0], 1.0e-5)
        self.assertFalse(calibrator.rotation_locked)
        np.testing.assert_allclose(
            update.lidar_to_body_rotation, np.eye(3), atol=1.0e-12
        )

    def test_solve_throttle_keeps_collecting_lidar_poses(self):
        calibrator = OnlineSpatiotemporalCalibrator(
            minimum_pairs=3,
            solve_period_s=1.0,
        )
        first = calibrator.update(0.0, np.eye(3), np.eye(3), [])
        throttled = calibrator.update(0.1, np.eye(3), np.eye(3), [])
        next_solve = calibrator.update(1.0, np.eye(3), np.eye(3), [])

        self.assertNotEqual(first.reason, "update_throttled")
        self.assertEqual(throttled.reason, "update_throttled")
        self.assertNotEqual(next_solve.reason, "update_throttled")
        self.assertEqual(len(calibrator.poses), 3)


if __name__ == "__main__":
    unittest.main()
