import math
import unittest

from uf_sensor_pipeline.fcu_observation import (
    integrate_flu_gyro_as_sensor_frd,
    integrate_flu_gyro_with_arrival_fallback,
    legacy_pixel_flow_to_sensor_frd,
    legacy_flow_rate_to_sensor_frd,
    valid_interval,
)


class FcuObservationTest(unittest.TestCase):
    def test_interval_rejects_first_duplicate_and_large_gap(self):
        self.assertIsNone(valid_interval(None, 1.0))
        self.assertIsNone(valid_interval(1.0, 1.0))
        self.assertIsNone(valid_interval(1.0, 1.8))
        self.assertAlmostEqual(valid_interval(1.0, 1.1), 0.1)

    def test_mavros_flu_flow_is_converted_back_to_sensor_frd(self):
        x, y = legacy_flow_rate_to_sensor_frd(0.2, -0.3, 0.1)
        self.assertAlmostEqual(x, 0.02)
        self.assertAlmostEqual(y, 0.03)

    def test_mavlink1_pixels_are_converted_back_to_sensor_frd(self):
        x, y = legacy_pixel_flow_to_sensor_frd(13.0, 6.5, 130.0, 130.0)
        self.assertAlmostEqual(x, math.atan2(13.0, 130.0))
        self.assertAlmostEqual(y, -math.atan2(6.5, 130.0))

    def test_fcu_gyro_is_integrated_and_converted_to_frd(self):
        samples = [
            (1.0, 1.0, 2.0, 3.0),
            (1.1, 1.0, 2.0, 3.0),
            (1.2, 1.0, 2.0, 3.0),
        ]
        result = integrate_flu_gyro_as_sensor_frd(samples, 1.05, 1.15)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0], 0.1)
        self.assertAlmostEqual(result[1], -0.2)
        self.assertAlmostEqual(result[2], -0.3)

    def test_gyro_integration_rejects_uncovered_interval(self):
        samples = [(1.0, 0.0, 0.0, 0.0), (1.1, 0.0, 0.0, 0.0)]
        self.assertIsNone(integrate_flu_gyro_as_sensor_frd(samples, 0.9, 1.05))
        self.assertTrue(math.isnan(float("nan")))

    def test_gyro_integration_falls_back_to_arrival_clock(self):
        source_samples = [
            (10.0, 1.0, 2.0, 3.0),
            (10.1, 1.0, 2.0, 3.0),
        ]
        arrival_samples = [
            (20.0, 1.0, 2.0, 3.0),
            (20.1, 1.0, 2.0, 3.0),
            (20.2, 1.0, 2.0, 3.0),
        ]
        result, used_fallback = integrate_flu_gyro_with_arrival_fallback(
            source_samples,
            arrival_samples,
            9.95,
            10.05,
            20.15,
        )
        self.assertTrue(used_fallback)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0], 0.1)
        self.assertAlmostEqual(result[1], -0.2)
        self.assertAlmostEqual(result[2], -0.3)


if __name__ == "__main__":
    unittest.main()
