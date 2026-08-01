
import math
import unittest
from pathlib import Path

import yaml

from multi_slam_uav_sim.d435i_cross_session_analysis import (
    alignment_error,
    relative_ground_truth,
)
from multi_slam_uav_sim.d435i_cross_session_monitor import compose


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class CrossSessionGeometryTest(unittest.TestCase):
    def test_compose_rotates_odometry_into_map(self):
        half = math.sqrt(0.5)
        transform = ((2.0, 1.0, 0.0), (0.0, 0.0, half, half))
        pose = ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        position, quaternion = compose(transform, pose)
        self.assertAlmostEqual(position[0], 2.0, places=6)
        self.assertAlmostEqual(position[1], 2.0, places=6)
        self.assertAlmostEqual(quaternion[2], half, places=6)
        self.assertAlmostEqual(quaternion[3], half, places=6)

    def test_ground_truth_is_relative_to_session1_origin(self):
        origin = {"x": 1.0, "y": 2.0, "z": 0.2,
                  "yaw_rad": math.pi / 2.0}
        row = {"gt_x": "1.0", "gt_y": "3.0", "gt_z": "0.7",
               "gt_yaw_rad": str(math.pi)}
        x, y, z, yaw = relative_ground_truth(row, origin)
        self.assertAlmostEqual(x, 1.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 0.5, places=6)
        self.assertAlmostEqual(yaw, math.pi / 2.0, places=6)

    def test_alignment_error_uses_map_aligned_pose(self):
        origin = {"x": 0.0, "y": 0.0, "z": 0.2, "yaw_rad": 0.0}
        row = {
            "gt_x": "2.0", "gt_y": "0.0", "gt_z": "0.7",
            "gt_yaw_rad": "0.1", "map_x": "2.03", "map_y": "0.04",
            "map_z": "0.5", "map_yaw_rad": "0.12",
        }
        position, yaw_error, _ = alignment_error(row, origin)
        self.assertAlmostEqual(position, 0.05, places=6)
        self.assertAlmostEqual(yaw_error, math.degrees(0.02), places=6)


class CrossSessionConfigurationTest(unittest.TestCase):
    def test_localization_profile_preserves_feature_baseline(self):
        config = yaml.safe_load((PACKAGE_ROOT / "config" /
            "d435i_rtabmap_localization.yaml").read_text(encoding="utf-8"))[
                "d435i_rtabmap"]
        self.assertFalse(config["launch"]["delete_db_on_start"])
        self.assertFalse(
            config["rtabmap_parameters"]["Mem/IncrementalMemory"])
        self.assertTrue(
            config["rtabmap_parameters"]["Mem/InitWMWithAllNodes"])
        self.assertEqual(config["odometry_parameters"]["Vis/MinInliers"], 10)
        self.assertFalse(config["launch"]["approx_sync"])

    def test_frozen_conditions_stay_inside_verified_box(self):
        config = yaml.safe_load((PACKAGE_ROOT / "config" /
            "d435i_relocalization_conditions.yaml").read_text(
                encoding="utf-8"))
        self.assertEqual(len(config["conditions"]), 8)
        self.assertTrue({
            "start_same", "start_offset", "start_yaw_offset", "start_reverse"
        }.issubset(config["conditions"]))
        self.assertEqual(config["reference_route"]["distance_m"], 4.50)
        for condition in config["conditions"].values():
            self.assertGreaterEqual(float(condition["x_m"]), -1.0)
            self.assertLessEqual(float(condition["x_m"]), 5.0)
            self.assertGreaterEqual(float(condition["y_m"]), -3.0)
            self.assertLessEqual(float(condition["y_m"]), 3.0)
            self.assertGreaterEqual(float(condition["z_m"]), 0.40)
            self.assertLessEqual(float(condition["z_m"]), 0.90)


if __name__ == "__main__":
    unittest.main()
