import unittest

import numpy as np

from uf_backend_fusion.axis_reliability import (
    AxisReliabilityProfile,
    combine_axis_reliability,
)


class AxisReliabilityTest(unittest.TestCase):
    def test_one_healthy_source_keeps_its_axis_supported(self):
        lidar = AxisReliabilityProfile(
            "lidar", 1.0, [1.0, 1.0, 1.0], [0.9, 0.9, 0.0],
            [False, False, False],
        )
        gnss = AxisReliabilityProfile(
            "gnss", 0.95, [1.0, 1.0, 0.9], [1.0, 1.0, 1.0],
            [True, True, True],
        )
        summary = combine_axis_reliability((lidar, gnss))

        self.assertGreater(summary.reliability_xyz[2], 0.85)
        self.assertIn("gnss", summary.supporting_sources_xyz[2])
        self.assertGreater(summary.global_reliability_xyz[2], 0.85)

    def test_flow_and_barometer_have_disjoint_axis_support(self):
        flow = AxisReliabilityProfile(
            "flow", 1.0, [0.8, 0.8, 1.0], [1.0, 1.0, 0.0],
            [False, False, False],
        )
        barometer = AxisReliabilityProfile(
            "barometer", 1.0, [1.0, 1.0, 0.7], [0.0, 0.0, 1.0],
            [False, False, False],
        )
        summary = combine_axis_reliability((flow, barometer))

        np.testing.assert_allclose(summary.reliability_xyz, [0.8, 0.8, 0.7])
        np.testing.assert_allclose(summary.global_reliability_xyz, [0.0] * 3)
        self.assertEqual(summary.supporting_sources_xyz[2], ("barometer",))


if __name__ == "__main__":
    unittest.main()
