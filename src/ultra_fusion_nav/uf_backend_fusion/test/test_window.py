import unittest

import numpy as np

from uf_backend_fusion.ablation import run_ablation
from uf_backend_fusion.window import (
    SPARSE_SOLVER_AVAILABLE,
    STATE_SIZE,
    SlidingWindowBackend,
)


class SlidingWindowBackendTest(unittest.TestCase):
    def test_dynamic_weight_rejects_a_gnss_jump(self):
        rows = run_ablation("/tmp/uf_backend_ablation_test.csv")
        fixed = rows[0]["position_rmse_m"]
        dynamic = rows[1]["position_rmse_m"]
        self.assertLess(dynamic, fixed * 0.9)

    def test_factor_switch_and_covariance_inflation_are_recorded(self):
        backend = SlidingWindowBackend()
        backend.add_state()
        backend.add_state()
        backend.add_prior(0, np.zeros(STATE_SIZE), covariance=1.0e-4)
        backend.add_optical_flow(
            0, 1, [1.0, 0.0, 0.0], covariance=0.1,
            decision={"factor_enabled": False, "reliability_weight": 0.0, "covariance_inflation": 20.0},
        )
        record = backend.factor_summary()[-1]
        self.assertFalse(record.enabled)
        self.assertEqual(record.covariance_inflation, 20.0)
        self.assertEqual(record.effective_weight, 0.0)

    def test_imu_and_relative_factors_recover_a_state(self):
        backend = SlidingWindowBackend()
        backend.add_state()
        backend.add_state()
        backend.add_prior(0, np.zeros(STATE_SIZE), covariance=1.0e-4)
        backend.add_imu_delta(
            0, 1, [0.5, 0.0, 0.0], [1.0, 0.0, 0.0], covariance=0.01
        )
        estimate = backend.optimize()[1]
        np.testing.assert_allclose(estimate[:3], [0.5, 0.0, 0.0], atol=1.0e-3)
        np.testing.assert_allclose(estimate[6:9], [1.0, 0.0, 0.0], atol=1.0e-3)

    def test_window_keeps_only_recent_states(self):
        backend = SlidingWindowBackend(max_states=2)
        for _ in range(4):
            backend.add_state()
        self.assertEqual(backend.state_count, 2)

    def test_marginal_prior_keeps_disabled_lidar_interval_observable(self):
        backend = SlidingWindowBackend(max_states=2, solver="dense")
        backend.add_state()
        backend.add_state()
        backend.add_prior(0, np.zeros(STATE_SIZE), covariance=1.0e-4)
        backend.add_imu_delta(
            0, 1, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], covariance=0.01
        )
        backend.optimize()

        backend.add_state()
        backend.add_imu_delta(
            0, 1, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], covariance=0.01
        )
        backend.add_lidar_pose(
            1, [100.0, 0.0, 0.0], [0.0, 0.0, 0.0], covariance=0.01,
            decision={
                "factor_enabled": False,
                "reliability_weight": 0.0,
                "covariance_inflation": 20.0,
            },
        )
        estimate = backend.optimize()[1]

        self.assertAlmostEqual(estimate[0], 2.0, places=2)
        self.assertTrue(np.all(np.isfinite(estimate)))
        summaries = backend.factor_summary()
        self.assertTrue(any(record.name == "marginal_prior" for record in summaries))
        self.assertFalse(summaries[-1].enabled)

    @unittest.skipUnless(SPARSE_SOLVER_AVAILABLE, "scipy sparse solver unavailable")
    def test_sparse_and_dense_solvers_agree_when_available(self):
        dense = SlidingWindowBackend(max_states=3, solver="dense")
        sparse = SlidingWindowBackend(max_states=3, solver="sparse")
        for backend in (dense, sparse):
            backend.add_state()
            backend.add_state()
            backend.add_prior(0, np.zeros(STATE_SIZE), covariance=1.0e-4)
            backend.add_lidar_pose(
                1, [0.4, -0.2, 0.1], [0.0, 0.0, 0.2],
                covariance=[0.01] * 3 + [0.02] * 3,
            )
        np.testing.assert_allclose(dense.optimize()[1], sparse.optimize()[1], atol=1.0e-8)
        self.assertEqual(dense.solver, "dense")
        self.assertEqual(sparse.solver, "sparse")

    def test_bias_aware_imu_factor_contains_bias_random_walk_rows(self):
        backend = SlidingWindowBackend(max_states=3, solver="dense")
        backend.add_state()
        backend.add_state()
        zero_jacobian = np.zeros(9)
        backend.add_bias_aware_imu(
            0, 1, 0.1,
            delta_position=[0.0, 0.0, 0.0],
            delta_velocity=[0.0, 0.0, 0.0],
            delta_rotation=[0.0, 0.0, 0.0],
            position_accel_bias_jacobian=zero_jacobian,
            position_gyro_bias_jacobian=zero_jacobian,
            velocity_accel_bias_jacobian=zero_jacobian,
            velocity_gyro_bias_jacobian=zero_jacobian,
            rotation_gyro_bias_jacobian=zero_jacobian,
            gravity=[0.0, 0.0, 0.0],
            covariance=[0.1] * 9,
            bias_random_walk_covariance=[0.01] * 6,
        )
        summary = backend.factor_summary()
        self.assertEqual(summary[-1].name, "imu_preintegrated")
        self.assertEqual(summary[-1].residual_dimension, 15)
        self.assertTrue(summary[-1].enabled)

    def test_disabled_imu_factor_still_exports_nominal_residual_evidence(self):
        backend = SlidingWindowBackend(max_states=3, solver="dense")
        backend.add_state()
        backend.add_state()
        backend.add_prior(0, np.zeros(STATE_SIZE), covariance=1.0e-4)
        backend.add_prior(1, np.zeros(STATE_SIZE), covariance=1.0e-4)
        zero_jacobian = np.zeros(9)
        backend.add_bias_aware_imu(
            0, 1, 0.1,
            delta_position=[1.0, 0.0, 0.0],
            delta_velocity=[0.0, 0.0, 0.0],
            delta_rotation=[0.0, 0.0, 0.0],
            position_accel_bias_jacobian=zero_jacobian,
            position_gyro_bias_jacobian=zero_jacobian,
            velocity_accel_bias_jacobian=zero_jacobian,
            velocity_gyro_bias_jacobian=zero_jacobian,
            rotation_gyro_bias_jacobian=zero_jacobian,
            gravity=[0.0, 0.0, 0.0],
            covariance=[0.25] * 9,
            bias_random_walk_covariance=[0.25] * 6,
            decision={
                "factor_enabled": False,
                "reliability_weight": 0.0,
                "covariance_inflation": 20.0,
            },
        )
        backend.optimize()

        residual = backend.latest_factor_residual("imu_preintegrated")

        self.assertIsNotNone(residual)
        self.assertFalse(residual.enabled)
        self.assertEqual(residual.residual_dimension, 15)
        self.assertAlmostEqual(residual.mahalanobis_squared, 4.0, places=2)
        nominal = backend.latest_factor_residual(
            "imu_preintegrated", covariance=[0.05] * 15
        )
        self.assertAlmostEqual(nominal.mahalanobis_squared, 20.0, places=2)


if __name__ == "__main__":
    unittest.main()
