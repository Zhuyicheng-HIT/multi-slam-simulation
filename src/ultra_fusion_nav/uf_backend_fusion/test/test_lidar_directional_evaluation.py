import unittest

import numpy as np

from uf_backend_fusion.lidar_directional_evaluation import _factor, run_matrix
from uf_backend_fusion.native_lidar import lidar_directional_reliability


class LidarDirectionalEvaluationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = run_matrix(repeats=2)
        cls.by_name = {}
        for item in cls.result["runs"]:
            cls.by_name.setdefault(item["name"], []).append(item)

    def test_complete_matrix_and_hard_failure_boundary(self):
        self.assertEqual(len(self.by_name), 13)
        for name in (
            "complete_dropout", "stale_lidar", "timestamp_invalid",
            "nonfinite_or_corrupt",
        ):
            self.assertTrue(all(
                item["admission"] == "HARD_REJECT"
                for item in self.by_name[name]
            ))
            self.assertTrue(all(
                item["factor_admitted"] == 0 for item in self.by_name[name]
            ))

    def test_geometric_degradation_preserves_factor(self):
        for name, items in self.by_name.items():
            if name in {
                "complete_dropout", "stale_lidar", "timestamp_invalid",
                "nonfinite_or_corrupt",
            }:
                continue
            self.assertTrue(all(
                item["admission"] == "ADMIT_DIRECTIONAL" for item in items
            ))
            self.assertTrue(all(item["factor_admitted"] == 1 for item in items))

    def test_expected_weak_subspace_is_detected_without_truth_input(self):
        for name, items in self.by_name.items():
            if items[0].get("weak_direction_angle_deg") is None:
                continue
            self.assertLess(
                max(item["weak_direction_angle_deg"] for item in items),
                1.0,
                name,
            )
        self.assertFalse(self.result["contract"]["truth_used_online"])

    def test_rotated_corridor_distinguishes_xyz_from_subspace(self):
        items = self.by_name["corridor_45deg"]
        self.assertTrue(all(
            np.allclose(item["axis_information_scale"], [1.0, 1.0, 1.0])
            for item in items
        ))
        self.assertTrue(all(
            min(item["subspace_information_scale"]) < 1.0 for item in items
        ))
        xyz = np.mean([item["weak_projected_error"]["xyz"] for item in items])
        subspace = np.mean([item["weak_projected_error"]["subspace"] for item in items])
        self.assertLess(subspace, xyz)

    def test_normal_scene_does_not_trigger_directional_handoff(self):
        for item in self.by_name["normal_3d_room"]:
            self.assertTrue(np.allclose(item["axis_information_scale"], [1.0] * 3))
            self.assertTrue(np.allclose(item["subspace_information_scale"], [1.0] * 3))

    def test_health_consistency_and_observability_are_separate_layers(self):
        factor = _factor(np.diag([20.0, 200.0, 180.0]))
        healthy = lidar_directional_reliability(factor, consistency=0.5)
        unhealthy = lidar_directional_reliability(
            factor, chain_healthy=False, consistency=0.5
        )
        np.testing.assert_allclose(
            healthy.reliability_eigenspace, [0.05, 0.45, 0.5]
        )
        np.testing.assert_allclose(unhealthy.reliability_eigenspace, [0.0] * 3)
        self.assertEqual(healthy.health, 1.0)
        self.assertEqual(unhealthy.health, 0.0)


if __name__ == "__main__":
    unittest.main()
