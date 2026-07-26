import unittest

import numpy as np

from uf_backend_fusion.online_backend import (
    flow_observation_delta,
    frd_to_enu_delta,
    fused_motion_reference,
    gnss_jump_rejected,
    lidar_bypass_allowed,
    lidar_prediction_innovation,
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

    def test_lidar_bypass_requires_explicit_mode_and_live_imu_backup(self):
        self.assertTrue(lidar_bypass_allowed(False, True, True, True))
        self.assertFalse(lidar_bypass_allowed(True, True, True, True))
        self.assertFalse(lidar_bypass_allowed(False, False, True, True))
        self.assertFalse(lidar_bypass_allowed(False, True, False, True))
        self.assertFalse(lidar_bypass_allowed(False, True, True, False))

    def test_gnss_jump_gate_rejects_large_innovation(self):
        self.assertFalse(gnss_jump_rejected([1.0, 2.0, 0.0], [3.0, 4.0, 0.0]))
        self.assertTrue(gnss_jump_rejected([1.0, 2.0, 0.0], [30.0, 2.0, 0.0]))
        self.assertTrue(gnss_jump_rejected([1.0, 2.0, 0.0], [float("nan"), 2.0, 0.0]))

    def test_fused_motion_reference_does_not_use_current_lio(self):
        state = np.zeros(15)
        state[:3] = [1.0, 2.0, 3.0]
        state[5] = 0.4
        state[6:9] = [2.0, -1.0, 0.5]

        reference = fused_motion_reference(state, 0.2)

        np.testing.assert_allclose(reference["position"], [1.4, 1.8, 3.1])
        np.testing.assert_allclose(reference["delta_position"], [0.4, -0.2, 0.1])
        self.assertEqual(reference["yaw"], 0.4)

    def test_lidar_prediction_innovation_uses_lidar_free_reference(self):
        reference = {
            "position": np.asarray([1.0, 2.0, 3.0]),
            "yaw": 3.10,
        }
        innovation = lidar_prediction_innovation(
            [1.3, 2.4, 3.0], -3.12, reference,
        )

        self.assertAlmostEqual(innovation["position_m"], 0.5)
        self.assertLess(innovation["yaw_rad"], 0.1)

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
