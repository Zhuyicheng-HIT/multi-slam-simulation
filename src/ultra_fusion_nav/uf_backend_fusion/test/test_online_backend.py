import unittest

import numpy as np

from uf_backend_fusion.online_backend import (
    flow_observation_delta,
    frd_to_enu_delta,
    scheduler_decision,
    unwrap_yaw,
    yaw_to_quaternion,
)


class OnlineBackendHelpersTest(unittest.TestCase):
    def test_frd_axis_conversion_is_explicit(self):
        self.assertEqual(frd_to_enu_delta(1.0, 0.0, 0.0), (1.0, 0.0))
        east, north = frd_to_enu_delta(0.0, 1.0, 0.0)
        self.assertAlmostEqual(east, 0.0)
        self.assertAlmostEqual(north, -1.0)

    def test_flow_aggregation_rejects_nonpositive_distance(self):
        observation = flow_observation_delta([
            {
                "integrated_x": 0.0,
                "integrated_y": 1.0,
                "integrated_xgyro": 0.0,
                "integrated_ygyro": 0.0,
                "quality": 200,
                "distance_m": 1.0,
            },
            {
                "integrated_x": 1.0,
                "integrated_y": 1.0,
                "integrated_xgyro": 0.0,
                "integrated_ygyro": 0.0,
                "quality": 0,
                "distance_m": 0.0,
            },
        ], 0.0)
        self.assertIsNotNone(observation)
        np.testing.assert_allclose(observation["delta_position"], [1.0, 0.0, 0.0])
        self.assertEqual(observation["sample_count"], 1)

    def test_scheduler_decision_can_disable_factor(self):
        decision = scheduler_decision(0.0, enabled=False, inflation=20.0)
        self.assertFalse(decision["factor_enabled"])
        self.assertEqual(decision["reliability_weight"], 0.0)
        self.assertEqual(decision["covariance_inflation"], 20.0)

    def test_yaw_quaternion_is_normalized(self):
        quaternion = yaw_to_quaternion(1.2)
        self.assertAlmostEqual(sum(value * value for value in quaternion), 1.0)

    def test_yaw_unwrap_crosses_branch_cut_without_a_large_jump(self):
        previous = 3.10
        current = -3.12
        unwrapped = unwrap_yaw(previous, current)
        self.assertAlmostEqual(unwrapped, 3.163185307179586, places=6)


if __name__ == "__main__":
    unittest.main()
