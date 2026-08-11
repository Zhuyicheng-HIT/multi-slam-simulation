import unittest

import numpy as np

from uf_backend_fusion.manifold import STATE_SIZE, state_plus
from uf_backend_fusion.manifold_window import ManifoldSlidingWindowBackend
from uf_backend_fusion.visual_reprojection import (
    VisualTrackBatch,
    validate_visual_linearization,
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


if __name__ == "__main__":
    unittest.main()
