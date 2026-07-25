import unittest

import numpy as np

from uf_backend_fusion.ablation import run_ablation
from uf_backend_fusion.window import STATE_SIZE, SlidingWindowBackend


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


if __name__ == "__main__":
    unittest.main()
