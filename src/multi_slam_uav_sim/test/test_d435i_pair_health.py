
import unittest

from multi_slam_uav_sim.d435i_pair_health import ExactStampPairHealth


class ExactStampPairHealthTest(unittest.TestCase):

    def test_interleaved_callbacks_keep_every_exact_pair(self):
        tracker = ExactStampPairHealth(max_pending=20, max_intervals=20)
        events = [
            ("color", 100, 1.000),
            ("color", 200, 1.010),
            ("depth", 100, 1.020),
            ("color", 300, 1.030),
            ("depth", 200, 1.040),
            ("depth", 300, 1.050),
        ]
        matched = [tracker.observe(*event) for event in events]
        self.assertEqual(matched, [False, False, True, False, True, True])
        self.assertEqual(tracker.pair_sequence, 3)
        self.assertEqual(tracker.last_pair_stamp_ns, 300)
        self.assertEqual(tracker.last_stamp_delta_ms, 0.0)
        self.assertEqual(len(tracker.intervals), 2)
        self.assertFalse(tracker.color_arrivals)
        self.assertFalse(tracker.depth_arrivals)

    def test_duplicate_callbacks_do_not_double_count(self):
        tracker = ExactStampPairHealth()
        self.assertFalse(tracker.observe("color", 100, 1.0))
        self.assertTrue(tracker.observe("depth", 100, 1.1))
        self.assertFalse(tracker.observe("color", 100, 1.2))
        self.assertFalse(tracker.observe("depth", 100, 1.3))
        self.assertEqual(tracker.pair_sequence, 1)

    def test_transport_records_preserve_sequence_and_detect_gaps(self):
        tracker = ExactStampPairHealth()
        self.assertTrue(tracker.observe_pair(100, 1.0, sequence=41))
        self.assertTrue(tracker.observe_pair(200, 1.1, sequence=42))
        self.assertTrue(tracker.observe_pair(300, 1.3, sequence=44))
        self.assertEqual(tracker.pair_sequence, 44)
        self.assertEqual(tracker.observed_pair_count, 3)
        self.assertEqual(tracker.pair_sequence_gaps, 1)
        self.assertAlmostEqual(tracker.intervals[0], 0.1)
        self.assertAlmostEqual(tracker.intervals[1], 0.2)

    def test_source_sequence_drop_ratio_uses_bounded_recent_window(self):
        tracker = ExactStampPairHealth(source_window=4)
        records = (
            (100, 1.0, 1, 10),
            (200, 1.1, 2, 11),
            (300, 1.2, 3, 13),
            (400, 1.3, 4, 15),
            (500, 1.4, 5, 16),
        )
        for stamp, arrival, sequence, source in records:
            tracker.observe_pair(
                stamp, arrival, sequence=sequence, source_sequence=source)
        self.assertEqual(tracker.source_sequence_gaps, 2)
        self.assertAlmostEqual(tracker.source_drop_ratio, 2.0 / 6.0)

    def test_batched_callbacks_report_long_run_pair_rate(self):
        tracker = ExactStampPairHealth(max_pending=120, max_intervals=120)
        stamp = 0
        for batch in range(40):
            base = float(batch) * 0.1
            for index in range(3):
                stamp += 1
                tracker.observe("color", stamp, base + index * 0.001)
            for index in range(3):
                key = stamp - 2 + index
                tracker.observe("depth", key, base + 0.010 + index * 0.001)
        rate_hz = 1.0 / (sum(tracker.intervals) / len(tracker.intervals))
        self.assertEqual(tracker.pair_sequence, 120)
        self.assertAlmostEqual(rate_hz, 30.0, delta=0.5)

    def test_unmatched_cache_is_bounded(self):
        tracker = ExactStampPairHealth(max_pending=5)
        for key in range(20):
            tracker.observe("color", key, float(key))
        self.assertEqual(sorted(tracker.color_arrivals), [15, 16, 17, 18, 19])
        self.assertLessEqual(len(tracker.color_arrivals), 5)

    def test_invalid_stream_is_rejected(self):
        tracker = ExactStampPairHealth()
        with self.assertRaises(ValueError):
            tracker.observe("infrared", 1, 0.0)


if __name__ == "__main__":
    unittest.main()

