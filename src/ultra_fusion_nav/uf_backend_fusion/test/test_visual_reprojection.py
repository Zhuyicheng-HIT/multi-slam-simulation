import unittest

import numpy as np

from uf_backend_fusion.manifold import STATE_SIZE, state_plus
from uf_backend_fusion.manifold_window import ManifoldSlidingWindowBackend
from uf_backend_fusion.visual_reprojection import (
    RgbdDepthTrackBatch,
    RgbdDirectTrackBatch,
    VisualTrackBatch,
    rgbd_depth_residual_jacobians,
    rgbd_direct_residual_jacobians,
    validate_visual_linearization,
    visual_pose_observability,
    visual_reprojection_residual,
    visual_reprojection_residual_jacobians,
)


class VisualReprojectionTest(unittest.TestCase):
    def setUp(self):
        self.anchor = np.zeros(STATE_SIZE)
        self.current = np.zeros(STATE_SIZE)
        self.current[0] = 0.10
        self.current[5] = 0.02
        anchor = np.asarray(
            [[-0.2, -0.1], [0.1, -0.15], [0.2, 0.2], [-0.1, 0.25]])
        depth = np.asarray([2.0, 2.5, 3.0, 2.2])
        seed = VisualTrackBatch(anchor, anchor, 1.0 / depth, 2.5e-5)
        predicted, _, _, valid = visual_reprojection_residual_jacobians(
            self.anchor, self.current, seed
        )
        self.assertEqual(len(valid), len(anchor))
        self.tracks = VisualTrackBatch(
            anchor, anchor + predicted.reshape(-1, 2), 1.0 / depth, 2.5e-5
        )

    def test_zero_residual_for_consistent_tracks(self):
        residual = visual_reprojection_residual(
            self.anchor, self.current, self.tracks
        )
        np.testing.assert_allclose(residual, 0.0, atol=1.0e-12)

    def test_analytic_jacobian_matches_manifold_finite_difference(self):
        residual, anchor_jacobian, current_jacobian, _ = (
            visual_reprojection_residual_jacobians(
                self.anchor, self.current, self.tracks
            )
        )
        epsilon = 1.0e-7
        for state_index, analytic in enumerate(
                (anchor_jacobian, current_jacobian)):
            numeric = np.zeros_like(analytic)
            for column in range(6):
                delta = np.zeros(STATE_SIZE)
                delta[column] = epsilon
                states_plus = [self.anchor.copy(), self.current.copy()]
                states_minus = [self.anchor.copy(), self.current.copy()]
                states_plus[state_index] = state_plus(
                    states_plus[state_index], delta)
                states_minus[state_index] = state_plus(
                    states_minus[state_index], -delta)
                numeric[:, column] = (
                    visual_reprojection_residual(*states_plus, self.tracks)
                    - visual_reprojection_residual(*states_minus, self.tracks)
                ) / (2.0 * epsilon)
            np.testing.assert_allclose(
                analytic[:, :6], numeric[:, :6], atol=2.0e-5)
        self.assertEqual(residual.size, 8)

    def test_factor_corrects_pose_without_touching_other_modalities(self):
        backend = ManifoldSlidingWindowBackend(max_states=4, max_iterations=8)
        first = backend.add_state(self.anchor)
        wrong = self.current.copy()
        wrong[1] += 0.08
        second = backend.add_state(wrong)
        backend.add_prior(first, self.anchor, covariance=1.0e-8)
        backend.add_visual_reprojection(first, second, self.tracks)
        before = np.linalg.norm(
            visual_reprojection_residual(
                self.anchor, wrong, self.tracks))
        backend.optimize()
        after = np.linalg.norm(
            visual_reprojection_residual(
                backend.state(first),
                backend.state(second),
                self.tracks))
        self.assertLess(after, before * 0.1)
        rmse, dimension = backend.latest_factor_rmse("visual_reprojection")
        self.assertEqual(dimension, 8)
        self.assertLess(rmse, before / np.sqrt(8.0) * 0.1)

    def test_rgbd_depth_completes_metric_line_of_sight_constraint(self):
        anchor_pixels = np.asarray([
            [-0.2, -0.1], [0.1, -0.15], [0.2, 0.2], [-0.1, 0.25]
        ])
        anchor_depth = np.asarray([2.0, 2.5, 3.0, 2.2])
        seed = RgbdDepthTrackBatch(
            anchor_pixels, anchor_depth, anchor_depth, np.full(4, 0.01 ** 2)
        )
        predicted, _, _, valid = rgbd_depth_residual_jacobians(
            self.anchor, self.current, seed
        )
        self.assertEqual(valid.size, 4)
        tracks = RgbdDepthTrackBatch(
            anchor_pixels,
            anchor_depth,
            anchor_depth + predicted,
            np.full(4, 0.01 ** 2),
        )
        residual, anchor_jacobian, current_jacobian, _ = (
            rgbd_depth_residual_jacobians(self.anchor, self.current, tracks)
        )
        np.testing.assert_allclose(residual, 0.0, atol=1.0e-12)

        epsilon = 1.0e-7
        for state_index, analytic in enumerate(
                (anchor_jacobian, current_jacobian)):
            numeric = np.zeros_like(analytic)
            for column in range(6):
                delta = np.zeros(STATE_SIZE)
                delta[column] = epsilon
                plus = [self.anchor.copy(), self.current.copy()]
                minus = [self.anchor.copy(), self.current.copy()]
                plus[state_index] = state_plus(plus[state_index], delta)
                minus[state_index] = state_plus(minus[state_index], -delta)
                numeric[:, column] = (
                    rgbd_depth_residual_jacobians(*plus, tracks)[0]
                    - rgbd_depth_residual_jacobians(*minus, tracks)[0]
                ) / (2.0 * epsilon)
            np.testing.assert_allclose(
                analytic[:, :6], numeric[:, :6], atol=2.0e-5
            )

        backend = ManifoldSlidingWindowBackend(max_states=2, max_iterations=8)
        previous = backend.add_state(self.anchor)
        wrong = self.current.copy()
        wrong[2] += 0.08
        current = backend.add_state(wrong)
        backend.add_prior(previous, self.anchor, covariance=1.0e-8)
        backend.add_rgbd_depth(previous, current, tracks)
        before = np.linalg.norm(
            rgbd_depth_residual_jacobians(self.anchor, wrong, tracks)[0]
        )
        backend.optimize()
        after = np.linalg.norm(rgbd_depth_residual_jacobians(
            backend.state(previous), backend.state(current), tracks
        )[0])
        self.assertLess(after, before * 0.1)

    def test_rgbd_direct_depth_and_photometric_jacobians_match_numeric(self):
        anchor_pixels = np.asarray([
            [-0.2, -0.1], [0.1, -0.15], [0.2, 0.2], [-0.1, 0.25]
        ])
        depth = np.asarray([2.0, 2.5, 3.0, 2.2])
        seed_visual = VisualTrackBatch(
            anchor_pixels, anchor_pixels, 1.0 / depth, 2.5e-5
        )
        projected, _, _, valid = visual_reprojection_residual_jacobians(
            self.anchor, self.current, seed_visual
        )
        self.assertEqual(valid.size, 4)
        current_pixels = anchor_pixels + projected.reshape(-1, 2)
        seed_depth = RgbdDepthTrackBatch(
            anchor_pixels, depth, depth, np.full(4, 0.01 ** 2)
        )
        predicted_depth = rgbd_depth_residual_jacobians(
            self.anchor, self.current, seed_depth
        )[0]
        gradient = np.asarray([
            [12.0, 4.0], [-8.0, 6.0], [5.0, -9.0], [7.0, 3.0]
        ])
        previous_intensity = np.asarray([0.1, -0.2, 0.3, -0.1])
        tracks = RgbdDirectTrackBatch(
            anchor_pixels,
            current_pixels,
            depth,
            depth + predicted_depth,
            np.full(4, 0.01 ** 2),
            previous_intensity,
            previous_intensity,
            gradient,
            np.full(4, 0.15 ** 2),
        )
        values = rgbd_direct_residual_jacobians(
            self.anchor, self.current, tracks
        )
        np.testing.assert_allclose(values[0], 0.0, atol=1.0e-12)
        np.testing.assert_allclose(values[1], 0.0, atol=1.0e-12)

        def residual(states):
            result = rgbd_direct_residual_jacobians(
                states[0], states[1], tracks
            )
            return np.concatenate((result[0], result[1]))

        analytic_blocks = (
            np.vstack((values[2], values[4])),
            np.vstack((values[3], values[5])),
        )
        epsilon = 1.0e-7
        for state_index, analytic in enumerate(analytic_blocks):
            numeric = np.zeros_like(analytic)
            for column in range(6):
                delta = np.zeros(STATE_SIZE)
                delta[column] = epsilon
                plus = [self.anchor.copy(), self.current.copy()]
                minus = [self.anchor.copy(), self.current.copy()]
                plus[state_index] = state_plus(plus[state_index], delta)
                minus[state_index] = state_plus(minus[state_index], -delta)
                numeric[:, column] = (
                    residual(plus) - residual(minus)
                ) / (2.0 * epsilon)
            np.testing.assert_allclose(
                analytic[:, :6], numeric[:, :6], atol=2.0e-5
            )

        backend = ManifoldSlidingWindowBackend(max_states=2)
        previous = backend.add_state(self.anchor)
        wrong = self.current.copy()
        wrong[1] += 0.04
        current = backend.add_state(wrong)
        backend.add_rgbd_direct(previous, current, tracks)
        factor = backend._factors[-1]
        backend.cpp_math_core_enabled = False
        python_hessian, python_gradient, python_cost = backend._factor_normal(
            factor, backend._states
        )
        backend.cpp_math_core_enabled = True
        cpp_hessian, cpp_gradient, cpp_cost = backend._factor_normal(
            factor, backend._states
        )
        np.testing.assert_allclose(cpp_hessian, python_hessian, atol=1.0e-7)
        np.testing.assert_allclose(cpp_gradient, python_gradient, atol=1.0e-8)
        self.assertAlmostEqual(cpp_cost, python_cost, places=9)

    def test_cpp_rgbd_depth_normal_matches_python_with_extrinsic_and_huber(self):
        angle = 0.13
        rotation_body_camera = np.asarray([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        translation_body_camera = np.asarray([0.08, -0.03, 0.04])
        anchor_pixels = np.asarray([
            [-0.2, -0.1], [0.1, -0.15], [0.2, 0.2], [-0.1, 0.25]
        ])
        anchor_depth = np.asarray([2.0, 2.5, 3.0, 2.2])
        seed = RgbdDepthTrackBatch(
            anchor_pixels,
            anchor_depth,
            anchor_depth,
            np.full(4, 0.01 ** 2),
            rotation_body_camera,
            translation_body_camera,
        )
        prediction = rgbd_depth_residual_jacobians(
            self.anchor, self.current, seed
        )[0]
        tracks = RgbdDepthTrackBatch(
            anchor_pixels,
            anchor_depth,
            anchor_depth + prediction + np.asarray([0.0, 0.0, 0.06, 0.0]),
            np.full(4, 0.01 ** 2),
            rotation_body_camera,
            translation_body_camera,
        )
        wrong = self.current.copy()
        wrong[2] += 0.08
        backend = ManifoldSlidingWindowBackend(max_states=2)
        previous = backend.add_state(self.anchor)
        current = backend.add_state(wrong)
        backend.add_rgbd_depth(previous, current, tracks)
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
        np.testing.assert_allclose(cpp_hessian, python_hessian, atol=1.0e-8)
        np.testing.assert_allclose(cpp_gradient, python_gradient, atol=1.0e-9)
        self.assertAlmostEqual(cpp_cost, python_cost, places=10)
        self.assertAlmostEqual(
            cpp_candidate_cost, python_candidate_cost, places=10
        )

    def test_cost_only_path_matches_visual_robust_linearization(self):
        backend = ManifoldSlidingWindowBackend(max_states=4)
        first = backend.add_state(self.anchor)
        wrong = self.current.copy()
        wrong[1] += 0.08
        second = backend.add_state(wrong)
        backend.add_visual_reprojection(first, second, self.tracks)
        factor = backend._factors[-1]

        _, _, normal_cost = backend._factor_normal(factor, backend._states)

        self.assertAlmostEqual(
            backend._factor_cost(factor, backend._states), normal_cost, places=12
        )

    def test_cpp_visual_normal_matches_python_with_extrinsic_and_huber(self):
        angle = 0.18
        rotation_body_camera = np.asarray([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        translation_body_camera = np.asarray([0.08, -0.03, 0.04])
        seed = VisualTrackBatch(
            self.tracks.anchor_normalized,
            self.tracks.anchor_normalized,
            self.tracks.inverse_depth,
            self.tracks.variance,
            rotation_body_camera,
            translation_body_camera,
        )
        prediction, _, _, valid = visual_reprojection_residual_jacobians(
            self.anchor, self.current, seed
        )
        self.assertEqual(valid.size, seed.track_count)
        tracks = VisualTrackBatch(
            seed.anchor_normalized,
            seed.current_normalized + prediction.reshape(-1, 2),
            seed.inverse_depth,
            seed.variance,
            rotation_body_camera,
            translation_body_camera,
        )
        wrong = self.current.copy()
        wrong[1] += 0.08
        backend = ManifoldSlidingWindowBackend(max_states=2)
        previous = backend.add_state(self.anchor)
        current = backend.add_state(wrong)
        backend.add_visual_reprojection(previous, current, tracks)
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

    def test_invalid_depth_and_covariance_are_rejected(self):
        with self.assertRaises(ValueError):
            VisualTrackBatch([[0.0, 0.0]], [[0.0, 0.0]], [0.0], 1.0)
        with self.assertRaises(ValueError):
            VisualTrackBatch([[0.0, 0.0]], [[0.0, 0.0]], [1.0], -1.0)

    def test_linearization_check_accepts_consistent_tracks(self):
        check = validate_visual_linearization(
            self.anchor, self.current, self.tracks, 500.0, 500.0,
        )
        self.assertTrue(check.valid, check.reason)
        self.assertEqual(check.valid_track_count, len(self.tracks.inverse_depth))
        self.assertGreaterEqual(check.jacobian_rank, 3)
        self.assertTrue(np.isfinite(check.jacobian_condition_number))
        self.assertGreater(check.information_trace, 0.0)
        self.assertLess(check.reprojection_rmse_px, 1.0e-8)

    def test_linearization_check_rejects_cross_modal_innovation(self):
        inconsistent = VisualTrackBatch(
            self.tracks.anchor_normalized,
            self.tracks.current_normalized + np.asarray([0.08, 0.0]),
            self.tracks.inverse_depth,
            self.tracks.variance,
        )
        check = validate_visual_linearization(
            self.anchor, self.current, inconsistent, 500.0, 500.0,
            maximum_reprojection_rmse_px=6.0,
        )
        self.assertFalse(check.valid)
        self.assertEqual(check.reason, "state_innovation_reprojection_rmse")
        self.assertGreater(check.reprojection_rmse_px, 6.0)

    def test_pose_observability_rejects_rank_deficient_rgbd_geometry(self):
        varied = np.asarray([
            [x, y, 1.5 + 0.4 * ((column + row) % 4)]
            for column, x in enumerate(np.linspace(-1.0, 1.0, 8))
            for row, y in enumerate(np.linspace(-0.7, 0.7, 6))
        ])
        line = np.asarray([
            [x, 0.0, 2.0] for x in np.linspace(-1.0, 1.0, 48)
        ])
        rank, condition = visual_pose_observability(
            varied, np.eye(3), np.zeros(3)
        )
        self.assertEqual(rank, 6)
        self.assertLess(condition, 500.0)
        rank, condition = visual_pose_observability(
            line, np.eye(3), np.zeros(3)
        )
        self.assertLess(rank, 6)
        self.assertTrue(np.isinf(condition))


if __name__ == "__main__":
    unittest.main()
