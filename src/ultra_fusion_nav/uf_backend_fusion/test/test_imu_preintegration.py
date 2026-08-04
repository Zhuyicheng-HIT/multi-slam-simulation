import math
import unittest

import numpy as np

from uf_backend_fusion.imu_preintegration import ImuSample, preintegrate


def samples_for(duration, acceleration, angular_velocity=(0.0, 0.0, 0.0), step=0.01):
    count = int(round(duration / step))
    return [
        ImuSample(
            index * step,
            tuple(acceleration),
            tuple(angular_velocity),
        )
        for index in range(count + 1)
    ]


class ImuPreintegrationTest(unittest.TestCase):
    def test_stationary_specific_force_cancels_gravity(self):
        result = preintegrate(
            samples_for(1.0, (0.0, 0.0, 9.81)), 0.0, 1.0
        )
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.delta_position, [0.0, 0.0, 0.0], atol=1.0e-9)
        np.testing.assert_allclose(result.delta_velocity, [0.0, 0.0, 0.0], atol=1.0e-9)

    def test_constant_horizontal_specific_force_matches_kinematics(self):
        result = preintegrate(
            samples_for(1.0, (1.0, 0.0, 9.81)), 0.0, 1.0
        )
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.delta_position, [0.5, 0.0, 0.0], atol=1.0e-3)
        np.testing.assert_allclose(result.delta_velocity, [1.0, 0.0, 0.0], atol=1.0e-3)

    def test_constant_yaw_rate_returns_quaternion_increment(self):
        result = preintegrate(
            samples_for(1.0, (0.0, 0.0, 9.81), (0.0, 0.0, math.pi / 2.0)),
            0.0, 1.0,
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.delta_quaternion[0], math.sqrt(0.5), places=3)
        self.assertAlmostEqual(result.delta_quaternion[3], math.sqrt(0.5), places=3)

    def test_bias_jacobians_follow_acceleration_bias_sign(self):
        result = preintegrate(
            samples_for(1.0, (0.0, 0.0, 9.81)), 0.0, 1.0
        )
        position_jacobian = np.asarray(
            result.jacobian_delta_position_accel_bias
        ).reshape(3, 3)
        velocity_jacobian = np.asarray(
            result.jacobian_delta_velocity_accel_bias
        ).reshape(3, 3)
        self.assertLess(position_jacobian[2, 2], -0.45)
        self.assertLess(velocity_jacobian[2, 2], -0.90)
        self.assertTrue(np.all(np.isfinite(position_jacobian)))
        self.assertTrue(np.all(np.isfinite(velocity_jacobian)))

    def test_gyro_bias_jacobian_follows_so3_increment(self):
        result = preintegrate(
            samples_for(1.0, (0.0, 0.0, 9.81), (0.0, 0.0, math.pi / 2.0)),
            0.0, 1.0,
        )
        rotation_jacobian = np.asarray(
            result.jacobian_delta_rotation_gyro_bias
        ).reshape(3, 3)
        self.assertAlmostEqual(rotation_jacobian[2, 2], -1.0, places=3)
        self.assertLess(rotation_jacobian[0, 0], -0.6)
        self.assertLess(rotation_jacobian[1, 1], -0.6)

    def test_uncovered_interval_and_long_gap_are_reported(self):
        uncovered = preintegrate(samples_for(1.0, (0.0, 0.0, 9.81)), 1.1, 1.5)
        self.assertFalse(uncovered.valid)
        self.assertEqual(uncovered.reason, "interval_not_covered")
        sparse = [
            ImuSample(0.0, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
            ImuSample(0.5, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
            ImuSample(1.0, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
        ]
        gapped = preintegrate(sparse, 0.0, 1.0, max_gap_s=0.1)
        self.assertFalse(gapped.valid)
        self.assertEqual(gapped.reason, "sample_gap_exceeds_limit")


if __name__ == "__main__":
    unittest.main()
