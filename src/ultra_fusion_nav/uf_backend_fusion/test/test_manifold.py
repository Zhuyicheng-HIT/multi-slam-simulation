import math
import unittest

import numpy as np

from uf_backend_fusion.imu_preintegration import (
    ImuSample,
    _quat_to_rotvec,
    preintegrate_manifold,
)
from uf_backend_fusion.manifold import (
    so3_exp,
    so3_log,
    state_local,
    state_plus,
)


def samples(duration, acceleration, gyro=(0.0, 0.0, 0.0), step=0.01):
    return [
        ImuSample(stamp, tuple(acceleration), tuple(gyro))
        for stamp in np.arange(0.0, duration + 0.5 * step, step)
    ]


class ManifoldTest(unittest.TestCase):
    def test_so3_exp_log_round_trip(self):
        vector = np.asarray([0.2, -0.1, 0.7])
        np.testing.assert_allclose(so3_log(so3_exp(vector)), vector, atol=1.0e-10)

    def test_state_plus_and_local_are_inverse_for_local_increment(self):
        state = np.asarray([1, 2, 3, 0.2, -0.1, 2.8, 4, 5, 6, 0, 0, 0, 0, 0, 0], dtype=float)
        increment = np.linspace(-0.02, 0.02, 15)
        recovered = state_local(state, state_plus(state, increment))
        np.testing.assert_allclose(recovered, increment, atol=1.0e-10)

    def test_manifold_preintegration_does_not_hide_gravity(self):
        result = preintegrate_manifold(
            samples(1.0, (0.0, 0.0, 9.81)), 0.0, 1.0
        )
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.delta_velocity, [0.0, 0.0, 9.81], atol=1.0e-8)
        np.testing.assert_allclose(result.delta_position, [0.0, 0.0, 4.905], atol=1.0e-8)
        covariance = np.asarray(result.covariance).reshape(15, 15)
        np.testing.assert_allclose(covariance, covariance.T, atol=1.0e-14)
        self.assertGreater(float(np.min(np.linalg.eigvalsh(covariance))), 0.0)
        self.assertGreater(abs(covariance[0, 3]), 0.0)

    def test_full_covariance_matches_white_noise_closed_form(self):
        accel_noise = 0.13
        gyro_noise = 0.025
        accel_bias_walk = 0.002
        gyro_bias_walk = 0.0003
        result = preintegrate_manifold(
            samples(1.0, (0.0, 0.0, 0.0)),
            0.0,
            1.0,
            accel_noise_density=accel_noise,
            gyro_noise_density=gyro_noise,
            accel_bias_random_walk=accel_bias_walk,
            gyro_bias_random_walk=gyro_bias_walk,
        )
        covariance = np.asarray(result.covariance).reshape(15, 15)

        np.testing.assert_allclose(
            np.diag(covariance[0:3, 0:3]),
            np.full(3, accel_noise ** 2 / 3.0),
            rtol=2.0e-3,
        )
        np.testing.assert_allclose(
            np.diag(covariance[0:3, 3:6]),
            np.full(3, accel_noise ** 2 / 2.0),
            rtol=2.0e-3,
        )
        np.testing.assert_allclose(
            np.diag(covariance[3:6, 3:6]),
            np.full(3, accel_noise ** 2),
            rtol=2.0e-3,
        )
        np.testing.assert_allclose(
            np.diag(covariance[6:9, 6:9]),
            np.full(3, gyro_noise ** 2),
            rtol=2.0e-3,
        )
        np.testing.assert_allclose(
            np.diag(covariance[9:12, 9:12]),
            np.full(3, accel_bias_walk ** 2),
            rtol=2.0e-3,
        )
        np.testing.assert_allclose(
            np.diag(covariance[12:15, 12:15]),
            np.full(3, gyro_bias_walk ** 2),
            rtol=2.0e-3,
        )

    def test_bias_linearization_is_applied_once(self):
        result = preintegrate_manifold(
            samples(1.0, (0.3, 0.0, 9.81), gyro=(0.0, 0.0, 0.2)),
            0.0,
            1.0,
            accel_bias=(0.3, 0.0, 0.0),
            gyro_bias=(0.0, 0.0, 0.2),
        )
        np.testing.assert_allclose(result.delta_velocity, [0.0, 0.0, 9.81], atol=1.0e-8)
        np.testing.assert_allclose(
            so3_log(so3_exp([0.0, 0.0, 0.0])),
            [0.0, 0.0, 0.0],
        )
        self.assertAlmostEqual(result.delta_quaternion[0], 1.0, places=8)

    def test_constant_yaw_rate_keeps_rotation_on_so3(self):
        result = preintegrate_manifold(
            samples(1.0, (0.0, 0.0, 9.81), gyro=(0.0, 0.0, math.pi / 2.0)),
            0.0,
            1.0,
        )
        self.assertAlmostEqual(result.delta_quaternion[0], math.sqrt(0.5), places=3)
        self.assertAlmostEqual(result.delta_quaternion[3], math.sqrt(0.5), places=3)

    def test_gyro_bias_jacobian_uses_right_rotation_perturbation(self):
        interval = samples(
            1.2,
            (0.4, -0.2, 9.81),
            gyro=(0.45, -0.30, 0.70),
        )
        nominal = preintegrate_manifold(interval, 0.0, 1.2)
        nominal_rotation = so3_exp(
            _quat_to_rotvec(np.asarray(nominal.delta_quaternion))
        )
        numerical = np.zeros((3, 3))
        epsilon = 1.0e-6
        for axis in range(3):
            perturbation = np.zeros(3)
            perturbation[axis] = epsilon
            plus = preintegrate_manifold(
                interval, 0.0, 1.2, gyro_bias=perturbation
            )
            minus = preintegrate_manifold(
                interval, 0.0, 1.2, gyro_bias=-perturbation
            )
            plus_rotation = so3_exp(
                _quat_to_rotvec(np.asarray(plus.delta_quaternion))
            )
            minus_rotation = so3_exp(
                _quat_to_rotvec(np.asarray(minus.delta_quaternion))
            )
            plus_local = so3_log(nominal_rotation.T @ plus_rotation)
            minus_local = so3_log(nominal_rotation.T @ minus_rotation)
            numerical[:, axis] = (plus_local - minus_local) / (2.0 * epsilon)

        analytic = np.asarray(
            nominal.jacobian_delta_rotation_gyro_bias
        ).reshape(3, 3)
        np.testing.assert_allclose(analytic, numerical, atol=2.0e-6)


if __name__ == "__main__":
    unittest.main()
