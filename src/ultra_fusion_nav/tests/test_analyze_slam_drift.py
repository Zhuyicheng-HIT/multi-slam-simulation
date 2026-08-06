import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


def find_analyzer():
    start = Path(__file__).resolve()
    for parent in start.parents:
        candidate = parent / "tools" / "analyze_slam_drift.py"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("tools/analyze_slam_drift.py not found above test directory")


MODULE_PATH = find_analyzer()
SPEC = importlib.util.spec_from_file_location("analyze_slam_drift", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AnalyzerTimingTest(unittest.TestCase):
    def test_nearest_records_uses_header_stamp(self):
        source = [(100.0, 1_000_000_000, 0.0), (200.0, 2_000_000_000, 0.0)]
        target = [(500.0, 1_005_000_000, 0.0), (600.0, 2_005_000_000, 0.0)]

        matches = MODULE.nearest_records(source, target, max_delta_s=0.01)

        self.assertEqual(len(matches), 2)
        self.assertEqual(matches[0][1][1], 1_005_000_000)

    def test_nearest_records_rejects_large_stamp_delta(self):
        source = [(100.0, 1_000_000_000, 0.0)]
        target = [(100.0, 1_200_000_000, 0.0)]

        self.assertEqual(MODULE.nearest_records(source, target, max_delta_s=0.05), [])

    def test_timing_stats_exposes_scheduling_jitter(self):
        records = [(1.0, 1_000_000_000), (1.11, 1_100_000_000), (1.19, 1_200_000_000)]

        stats = MODULE.timing_stats(records)

        self.assertAlmostEqual(stats["stamp_period_median_ms"], 100.0, places=6)
        self.assertGreater(stats["arrival_minus_stamp_jitter_p95_ms"], 10.0)

    def test_invalid_reference_warns_without_failing_lio(self):
        valid, failures, warnings = MODULE.assess_coupling(0.3, 0.2)

        self.assertFalse(valid)
        self.assertEqual(failures, [])
        self.assertEqual(len(warnings), 1)

    def test_valid_reference_can_fail_lio_coupling(self):
        valid, failures, warnings = MODULE.assess_coupling(0.4, 0.8)

        self.assertTrue(valid)
        self.assertEqual(len(failures), 1)
        self.assertEqual(warnings, [])

    def test_align_xy_removes_rigid_frame_offset_without_scaling(self):
        truth = np.asarray([[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]])
        rotation = np.asarray([[0.0, -1.0], [1.0, 0.0]])
        estimate = (rotation.T @ (truth - np.asarray([4.0, -3.0])).T).T

        aligned, _, _ = MODULE.align_xy(estimate, truth)

        np.testing.assert_allclose(aligned, truth, atol=1.0e-9)

    def test_wrap_angle_removes_full_turn_branch_error(self):
        wrapped = MODULE.wrap_angle(np.deg2rad([270.0, -270.0, 361.0]))

        np.testing.assert_allclose(
            np.rad2deg(wrapped), [-90.0, 90.0, 1.0], atol=1.0e-9)

    def test_static_yaw_alignment_uses_initial_heading(self):
        truth_xy = np.asarray([[2.0, -1.0], [2.001, -1.002]])
        arbitrary_xy_rotation = np.asarray([[0.0, -1.0], [1.0, 0.0]])
        offset, basis, excitation = MODULE.yaw_alignment_offset(
            np.deg2rad([100.0, 100.0]),
            np.deg2rad([5.0, 5.0]),
            arbitrary_xy_rotation,
            truth_xy,
        )

        self.assertEqual(basis, "initial_heading_low_translation")
        self.assertLess(excitation, 0.5)
        self.assertAlmostEqual(np.rad2deg(offset), -95.0, places=6)

    def test_livox_custom_packet_extractor_rejects_nonfinite_points(self):
        message = SimpleNamespace(
            point_num=3,
            points=[
                SimpleNamespace(x=1.0, y=2.0, z=3.0),
                SimpleNamespace(x=float("nan"), y=0.0, z=0.0),
                SimpleNamespace(x=4.0, y=5.0, z=6.0),
            ],
        )

        points = MODULE.livox_xyz(message, max_points=10)

        self.assertEqual(points.shape, (2, 3))
        np.testing.assert_allclose(points[0], [1.0, 2.0, 3.0])


if __name__ == "__main__":
    unittest.main()
