import unittest
from dataclasses import replace
import threading
import time
from types import MethodType

import numpy as np

try:
    from uf_backend_core_cpp import (
        imu_preintegrated_graph_normal as cpp_imu_preintegrated_graph_normal,
        lidar_point_plane_graph_normal as cpp_lidar_point_plane_graph_normal,
        lidar_point_plane_graph_normal_axis_scaled as cpp_lidar_point_plane_graph_normal_axis_scaled,
        lidar_point_plane_normal as cpp_lidar_point_plane_normal,
        lidar_point_plane_normal_axis_scaled as cpp_lidar_point_plane_normal_axis_scaled,
        state_plus_batch as cpp_state_plus_batch,
    )
except ImportError:
    cpp_imu_preintegrated_graph_normal = None
    cpp_lidar_point_plane_graph_normal = None
    cpp_lidar_point_plane_graph_normal_axis_scaled = None
    cpp_lidar_point_plane_normal = None
    cpp_lidar_point_plane_normal_axis_scaled = None
    cpp_state_plus_batch = None

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
from uf_backend_fusion.manifold import (
    STATE_SIZE,
    numerical_state_jacobian,
    so3_right_jacobian_inverse,
    state_local,
    state_plus,
)
from uf_backend_fusion.native_lidar import (
    NativeLidarPoseNormal,
    point_plane_residual_jacobian,
)


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
        reset_counter=0,
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
    @unittest.skipIf(
        cpp_state_plus_batch is None, "C++ backend core is not installed"
    )
    def test_cpp_state_plus_batch_matches_python(self):
        generator = np.random.default_rng(20260812)
        states = generator.normal(size=(8, STATE_SIZE))
        states[:, 3:6] *= 0.4
        increments = generator.normal(scale=0.02, size=(8, STATE_SIZE))
        states[0, 4] = 0.5 * np.pi - 1.0e-8
        expected = np.asarray([
            state_plus(state, increment)
            for state, increment in zip(states, increments)
        ])

        actual = cpp_state_plus_batch(states, increments)

        np.testing.assert_allclose(actual, expected, atol=2.0e-12, rtol=2.0e-12)

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

    @unittest.skipIf(
        cpp_lidar_point_plane_graph_normal is None,
        "batched C++ LiDAR backend core is not installed",
    )
    def test_cpp_batched_lidar_graph_matches_scalar_cpp_factors(self):
        backend = ManifoldSlidingWindowBackend(max_states=3)
        first = backend.add_state(np.asarray([
            0.1, -0.2, 0.3, 0.02, -0.03, 0.04,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]))
        second = backend.add_state(np.asarray([
            0.2, -0.1, 0.4, -0.01, 0.05, -0.02,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]))
        backend.add_native_lidar_correspondences(
            first, plane_factor([1, 0, 0], [1, 0, 0], [0.8, 0, 0])
        )
        backend.add_native_lidar_correspondences(
            second, plane_factor([0, 1, 0], [0, 1, 0], [0, 1.2, 0])
        )

        batched_hessian, batched_gradient, batched_cost = backend._normal()
        scalar_hessian = np.zeros_like(batched_hessian)
        scalar_gradient = np.zeros_like(batched_gradient)
        scalar_cost = 0.0
        for factor in backend._factors:
            _, _, factor_cost = backend._factor_normal(
                factor,
                backend._states,
                scalar_hessian,
                scalar_gradient,
            )
            scalar_cost += factor_cost

        np.testing.assert_allclose(
            batched_hessian, scalar_hessian, atol=1.0e-12, rtol=1.0e-12
        )
        np.testing.assert_allclose(
            batched_gradient, scalar_gradient, atol=1.0e-12, rtol=1.0e-12
        )
        self.assertAlmostEqual(batched_cost, scalar_cost, places=12)

    @unittest.skipIf(
        cpp_lidar_point_plane_normal_axis_scaled is None,
        "axis-scaled C++ LiDAR backend core is not installed",
    )
    def test_axis_scaled_lidar_scales_translation_jacobian_only(self):
        factor = plane_factor([1.2, -0.4, 0.7], [0.6, -0.3, 0.74], [0.1, 0.2, -0.3])
        pose = np.asarray([0.2, -0.1, 0.4, 0.08, -0.04, 0.12])
        scale = np.asarray([0.25, 0.0, 0.64])
        base_hessian, base_gradient, base_cost = cpp_lidar_point_plane_normal(
            pose,
            factor.lidar_points,
            factor.plane_normals,
            factor.plane_points,
            factor.lidar_to_body_rotation,
            factor.lidar_to_body_translation,
            np.asarray([factor.measurement_variance]),
            0.7,
            2.5,
        )
        scaled_hessian, scaled_gradient, scaled_cost = (
            cpp_lidar_point_plane_normal_axis_scaled(
                pose,
                factor.lidar_points,
                factor.plane_normals,
                factor.plane_points,
                factor.lidar_to_body_rotation,
                factor.lidar_to_body_translation,
                scale,
                np.asarray([factor.measurement_variance]),
                0.7,
                2.5,
            )
        )
        jacobian_scale = np.diag(np.r_[np.sqrt(scale), np.ones(3)])
        np.testing.assert_allclose(
            scaled_hessian,
            jacobian_scale @ base_hessian @ jacobian_scale,
            atol=1.0e-12,
            rtol=1.0e-12,
        )
        np.testing.assert_allclose(
            scaled_gradient,
            jacobian_scale @ base_gradient,
            atol=1.0e-12,
            rtol=1.0e-12,
        )
        self.assertAlmostEqual(scaled_cost, base_cost, places=12)

    @unittest.skipIf(
        cpp_lidar_point_plane_graph_normal_axis_scaled is None,
        "axis-scaled batched C++ LiDAR backend core is not installed",
    )
    def test_axis_scaled_lidar_graph_matches_scalar_and_python_paths(self):
        scaled = np.asarray([0.16, 0.0, 0.49])
        backend = ManifoldSlidingWindowBackend(max_states=3)
        state = np.asarray([
            0.1, -0.2, 0.3, 0.02, -0.03, 0.04,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ])
        index = backend.add_state(state)
        backend.add_native_lidar_correspondences(
            index,
            plane_factor([1.0, 0.2, -0.4], [0.4, 0.2, 0.9], [0.2, -0.1, 0.3]),
            axis_information_scale=scaled,
        )
        hessian, gradient, cost = backend._normal()
        scalar_hessian = np.zeros_like(hessian)
        scalar_gradient = np.zeros_like(gradient)
        _, _, scalar_cost = backend._factor_normal(
            backend._factors[0], backend._states,
            scalar_hessian, scalar_gradient,
        )
        np.testing.assert_allclose(hessian, scalar_hessian, atol=1.0e-12)
        np.testing.assert_allclose(gradient, scalar_gradient, atol=1.0e-12)
        self.assertAlmostEqual(cost, scalar_cost, places=12)

        python_backend = ManifoldSlidingWindowBackend(
            max_states=3, cpp_math_core_enabled=False
        )
        python_index = python_backend.add_state(state)
        python_backend.add_native_lidar_correspondences(
            python_index,
            plane_factor([1.0, 0.2, -0.4], [0.4, 0.2, 0.9], [0.2, -0.1, 0.3]),
            axis_information_scale=scaled,
        )
        python_hessian, python_gradient, python_cost = python_backend._normal()
        np.testing.assert_allclose(hessian, python_hessian, atol=1.0e-12)
        np.testing.assert_allclose(gradient, python_gradient, atol=1.0e-12)
        self.assertAlmostEqual(cost, python_cost, places=12)

    def test_barometer_factor_constrains_only_vertical_position(self):
        backend = ManifoldSlidingWindowBackend(max_states=2)
        state = np.zeros(15)
        state[:3] = [3.0, -2.0, 5.0]
        backend.add_state(state)
        backend.add_barometer_local_z(0, 4.0, 0.25)

        hessian, gradient, _ = backend._normal()

        self.assertEqual(hessian[2, 2], 4.0)
        self.assertEqual(gradient[2], 4.0)
        self.assertEqual(hessian[0, 0], 0.0)
        self.assertEqual(hessian[1, 1], 0.0)

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

    def test_rejected_lm_trials_evaluate_cost_without_relinearizing(self):
        backend = ManifoldSlidingWindowBackend(
            max_states=2, max_iterations=1, lm_max_trials=4
        )
        index = backend.add_state(np.zeros(15))
        backend.add_gnss(index, [1.0, 0.0, 0.0], covariance=0.2)
        normal_calls = 0
        cost_calls = 0

        def tracked_normal(instance, factors=None, states=None):
            nonlocal normal_calls
            normal_calls += 1
            values = instance._states if states is None else states
            hessian = np.eye(STATE_SIZE)
            gradient = np.zeros(STATE_SIZE)
            gradient[0] = -1.0
            return hessian, gradient, float(values[0][0] ** 2)

        def rejected_cost(instance, factors=None, states=None):
            nonlocal cost_calls
            cost_calls += 1
            return float("inf")

        backend._normal = MethodType(tracked_normal, backend)
        backend._cost = MethodType(rejected_cost, backend)
        backend.optimize()

        self.assertEqual(normal_calls, 2)
        self.assertEqual(cost_calls, 3)
        self.assertEqual(backend.last_rejected_steps, 4)

    def test_optimize_accepts_a_per_cycle_iteration_budget(self):
        backend = ManifoldSlidingWindowBackend(
            max_states=2, max_iterations=4, lm_max_trials=2
        )
        index = backend.add_state(np.zeros(15))
        backend.add_gnss(index, [1.0, 0.0, 0.0], covariance=0.2)

        backend.optimize(max_iterations=1)

        self.assertEqual(backend.last_iteration_budget, 1)
        self.assertLessEqual(backend.last_iterations, 1)
        with self.assertRaises(ValueError):
            backend.optimize(max_iterations=0)

    def test_cost_only_and_shared_normal_accumulation_are_equivalent(self):
        measurement = stationary_measurement(duration=0.1)
        backend = ManifoldSlidingWindowBackend(max_states=3)
        first_state = np.zeros(15)
        second_state = np.zeros(15)
        second_state[:3] = [0.1, -0.05, 0.02]
        first = backend.add_state(first_state)
        second = backend.add_state(second_state)
        backend.add_prior(first, first_state, covariance=np.ones(15) * 0.1)
        backend.add_imu_preintegrated(first, second, measurement)
        backend.add_gnss(second, [0.0, 0.0, 0.0], covariance=0.5)
        backend.add_optical_flow_body(
            first, second, [0.0, 0.0, 0.0], covariance=0.2
        )
        backend.add_native_lidar_correspondences(
            second,
            plane_factor([0, 0, 0], [1, 0, 0], [0, 0, 0]),
        )

        expected_hessian, expected_gradient, expected_cost = backend._normal()
        dimension = backend.state_count * STATE_SIZE
        shared_hessian = np.zeros((dimension, dimension))
        shared_gradient = np.zeros(dimension)
        accumulated_cost = 0.0
        for factor in backend._factors:
            returned_hessian, returned_gradient, factor_cost = (
                backend._factor_normal(
                    factor,
                    backend._states,
                    shared_hessian,
                    shared_gradient,
                )
            )
            self.assertIs(returned_hessian, shared_hessian)
            self.assertIs(returned_gradient, shared_gradient)
            accumulated_cost += factor_cost

        np.testing.assert_allclose(shared_hessian, expected_hessian, atol=1.0e-12)
        np.testing.assert_allclose(shared_gradient, expected_gradient, atol=1.0e-12)
        self.assertAlmostEqual(accumulated_cost, expected_cost, places=12)
        self.assertAlmostEqual(backend._cost(), expected_cost, places=12)

    def test_sparse_factor_blocks_match_dense_jacobian_assembly(self):
        backend = ManifoldSlidingWindowBackend(max_states=3)
        first_state = np.zeros(15)
        first_state[3:6] = [0.08, -0.05, 0.12]
        second_state = np.zeros(15)
        second_state[:6] = [0.2, -0.1, 0.05, 0.1, -0.04, 0.14]
        first = backend.add_state(first_state)
        second = backend.add_state(second_state)
        backend.add_gnss(second, [0.1, -0.2, 0.0], covariance=0.5)
        backend.add_optical_flow(
            first, second, [0.05, -0.02, 0.01], covariance=0.2
        )
        backend.add_optical_flow_body(
            first, second, [0.04, -0.03, 0.0], covariance=0.3
        )
        backend.add_native_lidar_correspondences(
            second,
            plane_factor([0.4, -0.2, 0.1], [1, 0, 0], [0, 0, 0]),
        )
        dimension = backend.state_count * STATE_SIZE

        for factor in backend._factors:
            actual_hessian, actual_gradient, actual_cost = (
                backend._factor_normal(factor, backend._states)
            )
            reference_hessian = np.zeros((dimension, dimension))
            reference_gradient = np.zeros(dimension)
            if factor["name"] == "lidar_point_plane":
                index = factor["indices"][0]
                residual, pose_jacobian = point_plane_residual_jacobian(
                    factor["measurement"], backend._states[index][:6]
                )
                jacobian = np.zeros((residual.size, STATE_SIZE))
                jacobian[:, :6] = pose_jacobian
                _, robust_weight = huber_loss_and_weight(
                    residual / np.sqrt(factor["variance"]),
                    backend.lidar_huber_delta,
                )
                information = (
                    factor["effective_weight"]
                    * robust_weight
                    / factor["variance"]
                )
                jacobians = {index: jacobian}
            else:
                residual = backend._residual(factor, backend._states)
                information = factor["effective_weight"] / factor["variance"]
                jacobians = {
                    index: numerical_state_jacobian(
                        lambda values: backend._residual(factor, values),
                        backend._states,
                        index,
                    )
                    for index in factor["indices"]
                }
            weighted_residual = information * residual
            for first_index, first_jacobian in jacobians.items():
                first_block = slice(
                    first_index * STATE_SIZE,
                    (first_index + 1) * STATE_SIZE,
                )
                reference_gradient[first_block] += (
                    first_jacobian.T @ weighted_residual
                )
                for second_index, second_jacobian in jacobians.items():
                    second_block = slice(
                        second_index * STATE_SIZE,
                        (second_index + 1) * STATE_SIZE,
                    )
                    reference_hessian[first_block, second_block] += (
                        first_jacobian.T
                        @ (information[:, None] * second_jacobian)
                    )
            np.testing.assert_allclose(
                actual_hessian, reference_hessian, atol=2.0e-7, rtol=2.0e-7
            )
            np.testing.assert_allclose(
                actual_gradient, reference_gradient, atol=2.0e-7, rtol=2.0e-7
            )
            self.assertAlmostEqual(
                actual_cost, backend._factor_cost(factor, backend._states),
                places=12,
            )

    def test_optical_flow_factors_do_not_constrain_vertical_position(self):
        backend = ManifoldSlidingWindowBackend(max_states=2)
        first_state = np.zeros(15)
        first_state[3:6] = [0.15, -0.10, 0.30]
        second_state = first_state.copy()
        second_state[2] = 4.0
        first = backend.add_state(first_state)
        second = backend.add_state(second_state)
        backend.add_optical_flow_body(
            first, second, [0.0, 0.0, 0.0], covariance=[0.01, 0.01]
        )

        factor = backend._factors[-1]
        residual = backend._residual(factor, backend._states)
        hessian, gradient, _ = backend._factor_normal(
            factor, backend._states)

        self.assertEqual(factor["residual_dimension"], 2)
        np.testing.assert_allclose(residual, np.zeros(2), atol=1.0e-12)
        self.assertAlmostEqual(float(hessian[2, 2]), 0.0)
        self.assertAlmostEqual(float(hessian[STATE_SIZE + 2, STATE_SIZE + 2]), 0.0)
        self.assertAlmostEqual(float(gradient[2]), 0.0)
        self.assertAlmostEqual(float(gradient[STATE_SIZE + 2]), 0.0)

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

        def residual_function(values):
            return imu_residual(
                values, 0, 1, measurement, np.asarray([0.0, 0.0, -9.81])
            )
        numerical_i = numerical_state_jacobian(residual_function, states, 0)
        numerical_j = numerical_state_jacobian(residual_function, states, 1)

        np.testing.assert_allclose(residual, residual_function(states), atol=1.0e-12)
        np.testing.assert_allclose(analytic_i, numerical_i, atol=3.0e-5, rtol=3.0e-5)
        np.testing.assert_allclose(analytic_j, numerical_j, atol=3.0e-5, rtol=3.0e-5)

    def test_cpp_imu_normal_matches_python_path(self):
        measurement = stationary_measurement(duration=0.2)
        backend = ManifoldSlidingWindowBackend(max_states=2)
        first = np.asarray([
            0.4, -0.2, 0.7,
            0.12, -0.08, 0.21,
            0.3, -0.15, 0.05,
            0.02, -0.01, 0.03,
            0.004, -0.006, 0.008,
        ])
        second = np.asarray([
            0.47, -0.23, 0.72,
            0.15, -0.04, 0.25,
            0.34, -0.11, 0.02,
            0.021, -0.012, 0.031,
            0.005, -0.005, 0.009,
        ])
        previous = backend.add_state(first)
        current = backend.add_state(second)
        backend.add_imu_preintegrated(previous, current, measurement)
        factor = backend._factors[-1]

        backend.cpp_math_core_enabled = False
        python_hessian, python_gradient, python_cost = backend._factor_normal(
            factor, backend._states
        )
        python_candidate_cost = backend._factor_cost(factor, backend._states)

        backend.cpp_math_core_enabled = True
        cpp_hessian, cpp_gradient, cpp_cost = backend._factor_normal(
            factor, backend._states
        )
        cpp_candidate_cost = backend._factor_cost(factor, backend._states)

        np.testing.assert_allclose(
            cpp_hessian, python_hessian, atol=1.0e-9, rtol=1.0e-9
        )
        np.testing.assert_allclose(
            cpp_gradient, python_gradient, atol=1.0e-9, rtol=1.0e-9
        )
        self.assertAlmostEqual(cpp_cost, python_cost, places=10)
        self.assertAlmostEqual(
            cpp_candidate_cost, python_candidate_cost, places=10
        )

    @unittest.skipIf(
        cpp_imu_preintegrated_graph_normal is None,
        "batched C++ IMU backend core is not installed",
    )
    def test_cpp_batched_imu_graph_matches_scalar_cpp_factors(self):
        measurement = stationary_measurement(duration=0.1)
        backend = ManifoldSlidingWindowBackend(max_states=4)
        first = backend.add_state(np.zeros(STATE_SIZE))
        second_state = propagate_state(
            backend.state(first), measurement, backend.gravity
        )
        second_state[0] += 0.02
        second = backend.add_state(second_state)
        third_state = propagate_state(
            backend.state(second), measurement, backend.gravity
        )
        third_state[1] -= 0.015
        third = backend.add_state(third_state)
        backend.add_imu_preintegrated(first, second, measurement)
        backend.add_imu_preintegrated(second, third, measurement)

        batched_hessian, batched_gradient, batched_cost = backend._normal()
        scalar_hessian = np.zeros_like(batched_hessian)
        scalar_gradient = np.zeros_like(batched_gradient)
        scalar_cost = 0.0
        for factor in backend._factors:
            _, _, factor_cost = backend._factor_normal(
                factor,
                backend._states,
                scalar_hessian,
                scalar_gradient,
            )
            scalar_cost += factor_cost

        np.testing.assert_allclose(
            batched_hessian, scalar_hessian, atol=1.0e-10, rtol=1.0e-10
        )
        np.testing.assert_allclose(
            batched_gradient, scalar_gradient, atol=1.0e-10, rtol=1.0e-10
        )
        self.assertAlmostEqual(batched_cost, scalar_cost, places=11)

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

    def test_transaction_snapshot_isolated_from_marginalization_index_updates(self):
        backend = ManifoldSlidingWindowBackend(max_states=2)
        first = backend.add_state(np.zeros(15))
        backend.add_prior(first, np.zeros(15), covariance=np.ones(15))
        second = backend.add_state(np.zeros(15))
        backend.add_optical_flow(first, second, [0.0, 0.0, 0.0])
        snapshot = backend.snapshot()
        expected_indices = [factor["indices"] for factor in snapshot.factors]

        backend.add_state(np.zeros(15))
        self.assertEqual(
            [factor["indices"] for factor in snapshot.factors],
            expected_indices,
        )
        backend.restore(snapshot)
        self.assertEqual(
            [factor["indices"] for factor in backend._factors],
            expected_indices,
        )

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

    def test_opt_in_profiler_is_bounded_and_reports_solver_stages(self):
        disabled = ManifoldSlidingWindowBackend(max_states=2)
        index = disabled.add_state(np.zeros(15))
        disabled.add_prior(index, np.zeros(15), covariance=np.ones(15))
        disabled.optimize()
        self.assertEqual(disabled.profile_summary(), {})

        backend = ManifoldSlidingWindowBackend(
            max_states=2, profiling_enabled=True, profiling_capacity=64
        )
        index = backend.add_state(np.zeros(15))
        backend.add_prior(index, np.ones(15), covariance=np.ones(15))
        backend.optimize()
        profile = backend.profile_summary()
        self.assertIn("factor_graph_linearization", profile)
        self.assertIn("factor_prior", profile)
        self.assertIn("linear_solve", profile)
        self.assertIn("state_update", profile)
        self.assertIn("optimize_total", profile)
        self.assertIn("graph_assembly", profile)
        for values in profile.values():
            self.assertGreater(values["count"], 0)
            self.assertLessEqual(values["p50_ms"], values["p95_ms"])
            self.assertLessEqual(values["p95_ms"], values["max_ms"])
        cycle = backend.last_profile_cycle
        self.assertGreater(cycle["optimize_total"], 0.0)
        self.assertGreater(cycle["factor_graph_linearization"], 0.0)
        self.assertGreater(cycle["graph_assembly"], 0.0)
        self.assertFalse(cycle["marginalization_happened"])

    def test_profiler_summary_is_safe_during_worker_updates(self):
        backend = ManifoldSlidingWindowBackend(
            max_states=2, profiling_enabled=True, profiling_capacity=64
        )
        errors = []

        def writer():
            try:
                for index in range(300):
                    backend._profile_stop(
                        f"concurrent_stage_{index}", time.perf_counter_ns()
                    )
            except Exception as error:  # pragma: no cover - assertion below
                errors.append(error)

        thread = threading.Thread(target=writer)
        thread.start()
        while thread.is_alive():
            backend.profile_summary()
        thread.join()

        self.assertEqual(errors, [])
        self.assertEqual(len(backend.profile_summary()), 300)

    def test_transaction_profile_marks_marginalization(self):
        backend = ManifoldSlidingWindowBackend(
            max_states=2, profiling_enabled=True, profiling_capacity=64
        )
        first = backend.add_state(np.zeros(15))
        backend.add_prior(first, np.zeros(15), covariance=np.ones(15))
        second = backend.add_state(np.zeros(15))
        backend.add_prior(second, np.zeros(15), covariance=np.ones(15))
        backend.optimize()
        backend.begin_profile_cycle()
        third = backend.add_state(np.zeros(15))
        backend.add_prior(third, np.zeros(15), covariance=np.ones(15))
        backend.optimize()
        cycle = backend.finish_profile_cycle()
        self.assertTrue(cycle["marginalization_happened"])
        self.assertGreater(cycle["marginalization"], 0.0)
        self.assertGreater(cycle["optimize_total"], 0.0)

    def test_marginal_prior_block_transform_matches_dense_jacobian(self):
        rng = np.random.default_rng(17)
        backend = ManifoldSlidingWindowBackend(max_states=3)
        references = [rng.normal(scale=0.05, size=15) for _ in range(2)]
        states = [reference + rng.normal(scale=0.02, size=15)
                  for reference in references]
        matrix = rng.normal(size=(2 * STATE_SIZE, 2 * STATE_SIZE))
        local_hessian = matrix.T @ matrix + np.eye(2 * STATE_SIZE)
        local_gradient = rng.normal(size=2 * STATE_SIZE)
        factor = {
            "name": "marginal_prior",
            "indices": (0, 1),
            "enabled": True,
            "normal_hessian": local_hessian,
            "normal_gradient": local_gradient,
            "references": references,
        }
        hessian, gradient, _ = backend._factor_normal(factor, states)
        local = np.concatenate([
            state_local(reference, state)
            for reference, state in zip(references, states)
        ])
        dense_jacobian = np.eye(2 * STATE_SIZE)
        for block in range(2):
            rotation = slice(block * STATE_SIZE + 3, block * STATE_SIZE + 6)
            dense_jacobian[rotation, rotation] = so3_right_jacobian_inverse(
                local[rotation]
            )
        expected_gradient = dense_jacobian.T @ (
            local_hessian @ local + local_gradient
        )
        expected_hessian = dense_jacobian.T @ local_hessian @ dense_jacobian
        np.testing.assert_allclose(hessian, expected_hessian, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(gradient, expected_gradient, rtol=1e-12, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
