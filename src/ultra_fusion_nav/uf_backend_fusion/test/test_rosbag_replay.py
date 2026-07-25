import unittest

from uf_backend_fusion.rosbag_factors import (
    _align_relative_flow_clock,
    frd_to_enu_delta,
    optical_flow_decision,
    scheduler_decision,
)


class RosbagReplayHelpersTest(unittest.TestCase):
    def test_relative_flow_clock_is_aligned_only_when_span_is_plausible(self):
        streams = {
            "lio": [
                {"stamp_s": 2000.0},
                {"stamp_s": 2020.0},
            ],
            "flow": [
                {"stamp_s": 10.0},
                {"stamp_s": 20.0},
            ],
        }
        offset = _align_relative_flow_clock(streams)
        self.assertAlmostEqual(offset, 1990.0)
        self.assertEqual(streams["flow"][0]["stamp_s"], 2000.0)

    def test_longer_unmatched_flow_span_is_not_shifted(self):
        streams = {
            "lio": [{"stamp_s": 1000.0}, {"stamp_s": 1010.0}],
            "flow": [{"stamp_s": 10.0}, {"stamp_s": 30.0}],
        }
        self.assertEqual(_align_relative_flow_clock(streams), 0.0)
        self.assertEqual(streams["flow"][0]["stamp_s"], 10.0)

    def test_flow_axis_and_scheduler_decision_are_explicit(self):
        east, north = frd_to_enu_delta(1.0, 0.0, 0.0)
        self.assertAlmostEqual(east, 1.0)
        self.assertAlmostEqual(north, 0.0)
        decision = scheduler_decision(0.9)
        self.assertFalse(decision["factor_enabled"])
        self.assertEqual(decision["reliability_weight"], 0.0)

    def test_zero_quality_flow_is_hard_disabled(self):
        decision = optical_flow_decision(
            0.10,
            {"quality": 0.0},
            ["low_quality_extension"],
            0.0,
        )
        self.assertFalse(decision["factor_enabled"])
        self.assertEqual(decision["reliability_weight"], 0.0)
        self.assertIn("quality_below_minimum_extension", decision["reasons"])

    def test_valid_quality_flow_keeps_continuous_weight(self):
        decision = optical_flow_decision(0.10, {"quality": 80.0}, [], 80.0)
        self.assertTrue(decision["factor_enabled"])
        self.assertGreater(decision["reliability_weight"], 0.0)


if __name__ == "__main__":
    unittest.main()
