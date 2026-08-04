import math
import unittest

from multi_slam_uav_sim.s_curve_path import (
    generate_s_curve,
    polyline_length,
    resample_polyline,
)


class SCurvePathTest(unittest.TestCase):
    def test_s_curve_has_continuous_safe_endpoints_and_expected_bounds(self):
        points = generate_s_curve(12.0, 4.5, 5.0, 1.0, 241, 1)
        self.assertEqual(len(points), 241)
        self.assertAlmostEqual(points[0][0], 0.0, places=9)
        self.assertAlmostEqual(points[-1][0], 0.0, places=9)
        self.assertAlmostEqual(points[0][1], -6.0, places=9)
        self.assertAlmostEqual(points[-1][1], 6.0, places=9)
        self.assertAlmostEqual(points[0][2], 5.0, places=9)
        self.assertAlmostEqual(points[-1][2], 5.0, places=9)
        self.assertAlmostEqual(max(point[0] for point in points), 4.5, places=6)
        self.assertAlmostEqual(min(point[0] for point in points), -4.5, places=6)
        self.assertGreaterEqual(min(point[2] for point in points), 4.0)
        self.assertLessEqual(max(point[2] for point in points), 6.0)
        self.assertGreater(polyline_length(points), 20.0)

    def test_reverse_pass_connects_without_position_or_altitude_jump(self):
        points = generate_s_curve(12.0, 4.5, 5.0, 1.0)
        reverse = list(reversed(points))
        self.assertEqual(points[-1], reverse[0])
        self.assertEqual(reverse[-1], points[0])

    def test_arc_length_resampling_preserves_endpoints_and_spacing(self):
        source = generate_s_curve(12.0, 4.5, 5.0, 1.0, 481)
        output = resample_polyline(source, 0.08)
        self.assertEqual(output[0], source[0])
        self.assertEqual(output[-1], source[-1])
        distances = [
            math.dist(first, second)
            for first, second in zip(output[:-1], output[1:])
        ]
        self.assertTrue(distances)
        self.assertLessEqual(max(distances), 0.0801)
        self.assertGreater(min(distances[:-1]), 0.079)

    def test_invalid_clearance_inputs_are_rejected_by_path_generator(self):
        with self.assertRaises(ValueError):
            generate_s_curve(0.0, 1.0, 5.0, 1.0)
        with self.assertRaises(ValueError):
            resample_polyline([(0.0, 0.0, 0.0)] * 2, 0.1)


if __name__ == "__main__":
    unittest.main()
