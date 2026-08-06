import unittest
from dataclasses import replace
from types import MethodType

import numpy as np

from uf_backend_fusion.imu_preintegration import (
    ImuSample,
    preintegrate_manifold,
)
from uf_backend_fusion.manifold_window import (
    ManifoldSlidingWindowBackend,
    huber_loss_and_weight,
    imu_residual,
    imu_residual_jacobians,
    propagate_state,
)
from uf_backend_fusion.manifold import STATE_SIZE, numerical_state_jacobian
from uf_backend_fusion.native_lidar import NativeLidarPoseNormal


def stationary_measurement(duration=1.0):
    samples = [
        ImuSample(index * 0.01, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
        for index in range(int(duration / 0.01) + 1)
    ]
    return preintegrate_manifold(samples, 0.0, duration)


def plane_factor(point, normal, plane_point):
    point = np.asarray(point, dtype=float).reshape(1, 3)
    normal = np.asarray(normal, dtype=float).reshape(1, 3)
    plane_point = np.asarray(plane_point, dtype=float).reshape(1, 3)
    return NativeLidarPoseNormal(
        stamp_ns=1_000_000_000,
        stamp_s=1.0,
        scan_sequence=1,
        matched_points=1,
        candidate_points=1,
        linearization_pose=np.zeros(6),
        pose_hessian=np.eye(6),
        pose_gradient=np.zeros(6),
        residual_squared=0.0,
        measurement_variance=1.0e-3,
        source="test",
        map_frame="map",
        state_frame="body",
        sensor_frame="lidar",
        correspondences_valid=True,
        lidar_points=point,
        plane_normals=normal,
        plane_points=plane_point,
        lidar_to_body_rotation=np.eye(3),
        lidar_to_body_translation=np.zeros(3),
    )


class ManifoldWindowTest(unittest.TestCase):
    def test_marginal_covariance_uses_information_not_solver_damping(self):
        backend = ManifoldSlidingWindowBackend(max_states=2, damping=100.0)
        state = np.zeros(15)
        index = backend.add_state(state)
        variance = np.linspace(0.01, 0.15, 15)
        backend.add_prior(index, state, covariance=variance)

        covariance = backend.marginal_covariance()

        np.testing.assert_allclose(np.diag(covariance), variance, atol=1.0e-10)
        np.testing.assert_allclose(
            covariance - np.diag(np.diag(covariance)),
            np.zeros((15, 15)),
            atol=1.0e-10,
        )

    def test_marginal_covariance_marks_unobservable_directions_uncertain(self):
        backend = ManifoldSlidingWindowBackend(max_states=2)
        backend.add_state(np.zeros(15))

        covariance = backend.marginal_covariance(unobservable_variance=1234.0)

        np.testing.assert_allclose(covariance, np.eye(15) * 1234.0)

    def test_relocalization_reset_discards_old_window_and_adds_prior(self):
        backend = ManifoldSlidingWindowBackend(max_states=4)
        backend.add_state(np.zeros(15))
        backend.add_state(np.ones(15) * 0.1)
        backend.add_prior(0, np.zeros(15), covariance=0.2)
        recovered = np.arange(15, dtype=float) * 0.01

        index = backend.reset(recovered, covariance=np.ones(15) * 0.05)

        self.assertEqual(index, 0)
        self.assertEqual(backend.state_count, 1)
        self.assertEqual(backend.factor_count, 1)
        np.testing.assert_allclose(backend.state(0), recovered)
        self.assertEqual(backend.factor_summary()[0].name, "prior")

    def test_visual_odometry_factor_recovers_relative_se3_motion(self):
        backend = ManifoldSlidingWindowBackend(max_states=2)
        backend.add_state(np.zeros(15))
        initial = np.zeros(15)
        initial[:3] = [0.6, 0.3, -0.1]
        initial[3:6] = [0.0, 0.0, -0.1]
        backend.add_state(initial)
        backend.add_prior(0, np.zeros(15), covariance=1.0e-5)
        backend.add_visual_odometry(
            0, 1, [1.0, 0.0, 0.0], [0.0, 0.0, 0.2],
            covariance=[0.01] * 3 + [0.0025] * 3,
        )

        estimate = backend.optimize()[1]

        np.testing.assert_allclose(estimate[:3], [1.0, 0.0, 0.0], atol=2.0e-3)
        np.testing.assert_allclose(estimate[3:6], [0.0, 0.0, 0.2], atol=2.0e-3)
        self.assertEqual(backend.factor_summary()[-1].name, "visual_odometry")
        diagnostics = backend.latest_factor_diagnostics("visual_odometry")
        self.assertTrue(diagnostics["enabled"])
        self.assertTrue(diagnostics["finite"])
        self.assertEqual(diagnostics["state_indices"], [0, 1])
        self.assertEqual(len(diagnostics["residual"]), 6)
        self.assertEqual(set(diagnostics["jacobian_frobenius_norms"]), {"0", "1"})
        self.assertLess(diagnostics["residual_norm"], 0.01)
    def test_huber_loss_is_symmetric_and_continuous_at_threshold(self):
        residual = np.asarray([0.0, 2.5, -2.5, 10.0, -10.0])
        loss, weight = huber_loss_and_weight(residual, 2.5)

        np.testing.assert_allclose(loss, [0.0, 3.125, 3.125, 21.875, 21.875])
        np.testing.assert_allclose(weight, [1.0, 1.0, 1.0, 0.25, 0.25])

    def test_point_plane_huber_limits_one_large_outlier(self):
        base = plane_factor([0, 0, 0], [1, 0, 0], [0, 0, 0])
        points = np.zeros((21, 3))
        normals = np.zeros((21, 3))
        normals[:, 0] = 1.0
        plane_points = np.zeros((21, 3))
        plane_points[-1, 0] = 100.0
        factor = replace(
            base,
            matched_points=21,
            candidate_points=21,
            measurement_variance=1.0,
            lidar_points=points,
            plane_normals=normals,
            plane_points=plane_points,
        )

        robust = ManifoldSlidingWindowBackend(max_states=2, lidar_huber_delta=2.5)
        robust.add_state(np.zeros(15))
        robust.add_native_lidar_correspondences(0, factor)
        robust.optimize()
        least_squares = ManifoldSlidingWindowBackend(
            max_states=2, lidar_huber_delta=0.0
        )
        least_squares.add_state(np.zeros(15))
        least_squares.add_native_lidar_correspondences(0, factor)
        least_squares.optimize()

        self.assertLess(abs(robust.state(0)[0]), 0.2)
        self.assertGreater(least_squares.state(0)[0], 4.0)

    def test_lm_rejection_does_not_commit_candidate_state(self):
        backend = ManifoldSlidingWindowBackend(max_states=2, lm_max_trials=6)
        backend.add_state(np.zeros(15))

        def inconsistent_normal(instance, factors=None, states=None):
            values = instance._states if states is None else states
            hessian = np.eye(STATE_SIZE)
            gradient = np.zeros(STATE_SIZE)
            gradient[0] = -1.0
            return hessian, gradient, float(values[0][0] ** 2)

        backend._normal = MethodType(inconsistent_normal, backend)
        backend.optimize()

        np.testing.assert_array_equal(backend.state(0), np.zeros(15))
        self.assertEqual(backend.last_rejected_steps, 6)

    def test_analytic_imu_jacobians_match_right_local_finite_difference(self):
        samples = [
            ImuSample(
                index * 0.01,
                (0.3 + 0.01 * index, -0.2, 9.7 + 0.005 * index),
                (0.25, -0.18 + 0.002 * index, 0.31),
            )
            for index in range(21)
        ]
        measurement = preintegrate_manifold(
            samples,
            0.0,
            0.2,
            accel_bias=(0.03, -0.02, 0.01),
            gyro_bias=(0.01, -0.015, 0.02),
        )
        state_i = np.asarray([
            1.0, -2.0, 0.5,
            0.25, -0.35, 0.45,
            0.8, -0.4, 0.2,
            0.05, -0.01, 0.03,
            0.018, -0.022, 0.027,
        ])
        state_j = np.asarray([
            1.18, -2.06, 0.54,
            0.31, -0.29, 0.52,
            0.9, -0.32, 0.16,
            0.052, -0.012, 0.031,
            0.019, -0.021, 0.029,
        ])
        states = [state_i, state_j]

        residual, analytic_i, analytic_j = imu_residual_jacobians(
            states, 0, 1, measurement, np.asarray([0.0, 0.0, -9.81])
        )
        residual_function = lambda values: imu_residual(
            values, 0, 1, measurement, np.asarray([0.0, 0.0, -9.81])
        )
        numerical_i = numerical_state_jacobian(residual_function, states, 0)
        numerical_j = numerical_state_jacobian(residual_function, states, 1)

        np.testing.assert_allclose(residual, residual_function(states), atol=1.0e-12)
        np.testing.assert_allclose(analytic_i, numerical_i, atol=3.0e-5, rtol=3.0e-5)
        np.testing.assert_allclose(analytic_j, numerical_j, atol=3.0e-5, rtol=3.0e-5)

    def test_stationary_imu_propagation_and_residual(self):
        measurement = stationary_measurement()
        first = np.zeros(15)
        second = propagate_state(first, measurement)

        np.testing.assert_allclose(second[:9], np.zeros(9), atol=1.0e-8)
        residual = imu_residual(
            [first, second], 0, 1, measurement, np.asarray([0.0, 0.0, -9.81])
        )
        np.testing.assert_allclose(residual, np.zeros(15), atol=1.0e-8)

    def test_point_plane_factor_is_relinearized_at_current_state(self):
        backend = ManifoldSlidingWindowBackend(max_states=2)
        initial = np.zeros(15)
        initial[0] = 0.5
        index = backend.add_state(initial)
        backend.add_native_lidar_correspondences(
            index,
            plane_factor([0, 0, 0], [1, 0, 0], [0, 0, 0]),
        )

        backend.optimize()

        self.assertAlmostEqual(backend.state(0)[0], 0.0, places=6)
        self.assertLess(backend.last_cost, 1.0e-10)

    def test_imu_factor_optimizes_to_propagated_state(self):
        measurement = stationary_measurement()
        backend = ManifoldSlidingWindowBackend(max_states=3)
        first = np.zeros(15)
        second = np.zeros(15)
        second[0] = 0.2
        first_index = backend.add_state(first)
        backend.add_prior(first_index, first, covariance=np.full(15, 1.0e-6))
        second_index = backend.add_state(second)
        backend.add_imu_preintegrated(first_index, second_index, measurement)

        backend.optimize()

        np.testing.assert_allclose(backend.state(1)[:9], np.zeros(9), atol=2.0e-5)
        residual = backend.latest_factor_residual("imu_preintegrated")
        self.assertIsNotNone(residual)
        self.assertLess(residual.mahalanobis_squared, 1.0e-4)

    def test_imu_mahalanobis_uses_full_covariance(self):
        measurement = stationary_measurement()
        covariance = np.eye(15)
        covariance[0, 1] = 0.8
        covariance[1, 0] = 0.8
        measurement = replace(
            measurement, covariance=tuple(covariance.ravel())
        )
        first = np.zeros(15)
        second = propagate_state(first, measurement)
        second[0:2] += 1.0
        backend = ManifoldSlidingWindowBackend(max_states=2)
        previous = backend.add_state(first)
        current = backend.add_state(second)
        backend.add_imu_preintegrated(previous, current, measurement)

        residual = backend.latest_factor_residual("imu_preintegrated")
        expected = np.asarray([1.0, 1.0]) @ np.linalg.solve(
            covariance[0:2, 0:2], np.asarray([1.0, 1.0])
        )
        self.assertAlmostEqual(residual.mahalanobis_squared, expected)

    def test_schur_marginalization_preserves_bounded_window(self):
        backend = ManifoldSlidingWindowBackend(max_states=2)
        first = backend.add_state(np.zeros(15))
        backend.add_prior(first, np.zeros(15), covariance=np.ones(15))
        second = backend.add_state(np.zeros(15))
        backend.add_optical_flow(first, second, [1.0, 0.0, 0.0])
        backend.optimize()
        third = backend.add_state(backend.state(-1))
        backend.add_optical_flow(third - 1, third, [1.0, 0.0, 0.0])
        backend.optimize()

        self.assertEqual(backend.state_count, 2)
        self.assertTrue(any(
            factor.name == "marginal_prior" for factor in backend.factor_summary()
        ))
        self.assertAlmostEqual(backend.state(-1)[0], 2.0, places=4)

    def test_schur_prior_is_independent_of_solver_damping(self):
        def build(damping):
            backend = ManifoldSlidingWindowBackend(max_states=2, damping=damping)
            first = backend.add_state(np.zeros(15))
            backend.add_prior(first, np.zeros(15), covariance=np.ones(15))
            second = backend.add_state(np.zeros(15))
            backend.add_optical_flow(first, second, [1.0, 0.0, 0.0])
            # add_state marginalizes before the new state's factors are added,
            # so this captures exactly the first-state Schur prior.
            backend.add_state(np.zeros(15))
            prior = next(
                factor for factor in backend._factors
                if factor["name"] == "marginal_prior"
            )
            return prior["normal_hessian"].copy(), prior["normal_gradient"].copy()

        small = build(1.0e-9)
        large = build(1.0e3)
        np.testing.assert_allclose(small[0], large[0], atol=1.0e-12)
        np.testing.assert_allclose(small[1], large[1], atol=1.0e-12)

    def test_imu_factor_can_be_reintegrated_in_place(self):
        measurement = stationary_measurement()
        backend = ManifoldSlidingWindowBackend(max_states=3)
        previous = backend.add_state(np.zeros(15))
        current = backend.add_state(np.zeros(15))
        backend.add_imu_preintegrated(previous, current, measurement)
        replacement = replace(
            measurement,
            accel_bias_linearization=(0.1, 0.0, 0.0),
        )
        self.assertTrue(
            backend.replace_imu_preintegrated(previous, current, replacement)
        )
        self.assertIs(
            backend._factors[-1]["measurement"], replacement
        )
        self.assertFalse(backend.replace_imu_preintegrated(0, 2, replacement))

    def test_transaction_snapshot_restores_states_factors_and_solver_metadata(self):
        backend = ManifoldSlidingWindowBackend(max_states=3)
        first = backend.add_state(np.zeros(15))
        backend.add_prior(first, np.zeros(15), covariance=np.ones(15))
        backend.optimize()
        snapshot = backend.snapshot()
        expected_state = backend.state(0)
        expected_factors = backend.factor_summary()
        expected_initial_cost = backend.last_initial_cost
        expected_cost = backend.last_cost

        second = backend.add_state(np.ones(15))
        backend.add_optical_flow(first, second, [1.0, 0.0, 0.0])
        backend.optimize()
        self.assertEqual(backend.state_count, 2)

        backend.restore(snapshot)
        self.assertEqual(backend.state_count, 1)
        np.testing.assert_allclose(backend.state(0), expected_state)
        self.assertEqual(backend.factor_summary(), expected_factors)
        self.assertEqual(backend.last_initial_cost, expected_initial_cost)
        self.assertEqual(backend.last_cost, expected_cost)

    def test_latest_state_information_is_finite_symmetric_and_undamped(self):
        backend = ManifoldSlidingWindowBackend(max_states=2, damping=100.0)
        index = backend.add_state(np.zeros(15))
        variances = np.linspace(0.5, 2.0, 15)
        backend.add_prior(index, np.zeros(15), covariance=variances)
        backend.optimize()

        information = backend.latest_state_information()
        self.assertEqual(information.shape, (15, 15))
        np.testing.assert_allclose(information, information.T, atol=1.0e-12)
        np.testing.assert_allclose(
            np.diag(information), 1.0 / variances, rtol=1.0e-10
        )


if __name__ == "__main__":
    unittest.main()
