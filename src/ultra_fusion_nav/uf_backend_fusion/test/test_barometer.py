import unittest

from uf_backend_fusion.barometer import LocalBarometerSegment


class LocalBarometerSegmentTest(unittest.TestCase):
    def _filled_segment(self):
        segment = LocalBarometerSegment(
            baseline_window_s=2.0,
            minimum_baseline_samples=5,
            minimum_baseline_span_s=0.8,
            maximum_sample_age_s=0.3,
        )
        for index in range(11):
            self.assertTrue(segment.add_sample(1.0 + index * 0.1, 100000.0))
        return segment

    def test_segment_uses_only_recent_pre_activation_pressure(self):
        segment = self._filled_segment()
        measurement = segment.measurement(2.0, 7.5, True)

        self.assertIsNotNone(measurement)
        self.assertAlmostEqual(measurement.height_m, 7.5, places=9)
        self.assertEqual(measurement.segment_id, 1)

    def test_pressure_drop_produces_positive_relative_height(self):
        segment = self._filled_segment()
        self.assertIsNotNone(segment.measurement(2.0, 5.0, True))
        segment.add_sample(2.1, 99988.143)
        measurement = segment.measurement(2.1, 100.0, True)

        self.assertAlmostEqual(measurement.height_m, 6.0, places=2)
        self.assertEqual(measurement.segment_id, 1)

    def test_unknown_pressure_variance_uses_conservative_local_default(self):
        segment = self._filled_segment()
        segment.add_sample(2.1, 99988.143, float("nan"))
        measurement = segment.measurement(2.1, 5.0, True)

        self.assertIsNotNone(measurement)
        self.assertGreaterEqual(measurement.variance_m2, 0.25)

    def test_recovery_ends_segment_and_new_activation_uses_new_datum(self):
        segment = self._filled_segment()
        first = segment.measurement(2.0, 5.0, True)
        self.assertIsNotNone(first)
        self.assertIsNone(segment.measurement(2.0, 5.0, False))
        self.assertFalse(segment.active)
        for index in range(1, 12):
            segment.add_sample(2.0 + index * 0.1, 99000.0)
        second = segment.measurement(3.1, 12.0, True)

        self.assertIsNotNone(second)
        self.assertAlmostEqual(second.height_m, 12.0, places=6)
        self.assertEqual(second.segment_id, 2)

    def test_same_pressure_sample_is_consumed_once(self):
        segment = self._filled_segment()
        self.assertIsNotNone(segment.measurement(2.0, 5.0, True))
        self.assertIsNone(segment.measurement(2.0, 5.0, True))
        self.assertEqual(segment.last_reason, "pressure_sample_already_consumed")

    def test_reset_discards_old_pressure_history(self):
        segment = self._filled_segment()
        self.assertIsNotNone(segment.measurement(2.0, 5.0, True))
        segment.reset("epoch_reset")

        self.assertFalse(segment.active)
        self.assertEqual(len(segment.samples), 0)
        self.assertIsNone(segment.measurement(2.1, 7.0, True))
        self.assertEqual(segment.last_reason, "pressure_or_state_unavailable")


if __name__ == "__main__":
    unittest.main()
