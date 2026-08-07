import math
import unittest

from multi_slam_uav_sim.s_curve_path import (
    backend_error_to_fcu_setpoint,
    generate_calibration_figure_eight,
    generate_s_curve,
    normalize_angle,
    polyline_length,
    resample_polyline,
)


class SCurvePathTest(unittest.TestCase):
    def test_calibration_figure_eight_is_closed_level_and_bounded(self):
        center = (2.0, -3.0, 5.0)
        points = generate_calibration_figure_eight(center, 1.0, 161)

        self.assertEqual(points[0], center)
        self.assertEqual(points[-1], center)
        self.assertEqual(points[len(points) // 2], center)
        self.assertTrue(all(point[2] == center[2] for point in points))
        self.assertAlmostEqual(max(point[0] for point in points), 3.0, places=6)
        self.assertAlmostEqual(min(point[0] for point in points), 1.0, places=6)
        self.assertLessEqual(max(abs(point[1] - center[1]) for point in points), 0.5)
        self.assertGreater(polyline_length(points), 5.5)
        self.assertLess(polyline_length(points), 6.5)

    def test_calibration_figure_eight_rejects_invalid_geometry(self):
        with self.assertRaises(ValueError):
            generate_calibration_figure_eight((0.0, 0.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            generate_calibration_figure_eight((0.0, math.nan, 0.0), 1.0)

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

    def test_two_vertical_cycles_produce_repeated_climbs_and_descents(self):
        points = generate_s_curve(12.0, 4.5, 5.0, 1.0, samples=9,
                                  vertical_cycles=2)
        self.assertAlmostEqual(points[1][2], 6.0, places=6)
        self.assertAlmostEqual(points[3][2], 4.0, places=6)
        self.assertAlmostEqual(points[5][2], 6.0, places=6)
        self.assertAlmostEqual(points[7][2], 4.0, places=6)
        self.assertAlmostEqual(points[0][2], points[-1][2], places=9)

    def test_backend_error_drives_fcu_adapter_without_using_fcu_feedback(self):
        command = backend_error_to_fcu_setpoint(
            backend_position=(1.0, 2.0, 3.0),
            backend_target=(3.0, 2.0, 4.0),
            fcu_position=(10.0, 20.0, 30.0),
            backend_to_fcu_yaw=math.pi / 2.0,
            max_horizontal_offset=1.0,
            max_vertical_offset=0.5,
        )
        self.assertAlmostEqual(command[0], 10.0, places=6)
        self.assertAlmostEqual(command[1], 21.0, places=6)
        self.assertAlmostEqual(command[2], 30.5, places=6)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)

    def test_backend_setpoint_rejects_invalid_limits(self):
        with self.assertRaises(ValueError):
            backend_error_to_fcu_setpoint(
                (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                0.0, 0.0, 1.0)

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
