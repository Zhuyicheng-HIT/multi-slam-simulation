import struct
import unittest

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from uf_lio_adapter.geometry import TemporalVoxelFilter, cloud_xyz, geometry_diagnostics


class GeometryDiagnosticsTest(unittest.TestCase):
    def test_structured_cloud_produces_matches_and_finite_hessian(self):
        grid = np.asarray([
            (0.2 * x, 0.2 * y, 0.0)
            for x in range(-5, 6)
            for y in range(-5, 6)
        ])
        current = grid + np.asarray([0.05, -0.03, 0.02])

        result = geometry_diagnostics(current, grid, voxel_size=0.5)

        self.assertEqual(result["matched_points"], len(current))
        self.assertLess(result["residual_p95_m"], 0.1)
        self.assertTrue(np.all(np.isfinite(result["hessian_eigenvalues"])))
        self.assertGreater(result["map_quality"], 0.5)

    def test_empty_previous_cloud_is_invalid(self):
        result = geometry_diagnostics(np.ones((3, 3)), np.empty((0, 3)))

        self.assertEqual(result["matched_points"], 0)
        self.assertEqual(result["map_quality"], 0.0)

    def test_missing_previous_cloud_is_initial_state(self):
        result = geometry_diagnostics(np.ones((3, 3)), None)

        self.assertEqual(result["matched_points"], 0)
        self.assertEqual(result["map_quality"], 0.0)

    def test_planar_geometry_exposes_degenerate_axes(self):
        plane = np.asarray([
            (0.2 * x, 0.2 * y, 0.0)
            for x in range(-5, 6)
            for y in range(-5, 6)
        ])
        current = plane + np.asarray([0.0, 0.0, 0.02])

        result = geometry_diagnostics(current, plane, voxel_size=0.4)

        self.assertGreater(result["matched_points"], 80)
        self.assertLess(result["residual_p95_m"], 0.03)
        self.assertGreater(result["hessian_condition"], 1.0e6)
        self.assertLess(result["normal_covariance_eigenvalues"][0], 1.0e-6)
        self.assertGreater(result["axial_penalty"], 0.3)

    def test_cloud_xyz_reads_float_fields(self):
        msg = PointCloud2()
        msg.height = 1
        msg.width = 2
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 24
        msg.data = struct.pack("<ffffff", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

        points = cloud_xyz(msg)

        np.testing.assert_allclose(points, np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))

    def test_temporal_filter_requires_persistence_and_rejects_transient_voxel(self):
        filter_ = TemporalVoxelFilter(window_frames=2, min_static_support=2, neighbor_radius=0)
        stable = np.asarray([[0.1, 0.1, 0.1], [1.1, 0.1, 0.1]])

        first = filter_.classify(stable, voxel_size=1.0)
        second = filter_.classify(stable, voxel_size=1.0)
        third = filter_.classify(
            np.vstack((stable, np.asarray([[9.1, 0.1, 0.1]]))), voxel_size=1.0
        )

        self.assertEqual(len(first["uncertain_points"]), 2)
        self.assertEqual(len(second["uncertain_points"]), 2)
        self.assertEqual(len(third["static_points"]), 2)
        self.assertEqual(len(third["dynamic_points"]), 1)
        self.assertEqual(len(third["uncertain_points"]), 0)
        self.assertAlmostEqual(third["feature_repeatability"], 2.0 / 3.0)
        self.assertTrue(third["window_warm"])


if __name__ == "__main__":
    unittest.main()
