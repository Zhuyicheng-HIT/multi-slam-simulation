import unittest

from uf_reliability.scoring import (
    gnss_score, imu_score, lidar_score, optical_flow_score, vision_score,
)


class ScoringTest(unittest.TestCase):
    def test_lidar_degradation_increases(self):
        good = lidar_score([10, 20, 30, 40, 50, 60], [0.1, 0.2, 0.7], 0.0, 1000)[0]
        bad = lidar_score([1e-8, 1e-8, 1e-6, 10, 20, 30], [0.0, 0.0, 1.0], 0.8, 80)[0]
        self.assertLess(good, 0.3)
        self.assertGreater(bad, 0.8)

    def test_gnss_jump_and_outage_increase(self):
        good = gnss_score(1.0, 0.5, 0.2)[0]
        jump = gnss_score(1.0, 0.5, 9.0)[0]
        outage_result = gnss_score(0.0, 0.5, -1.0)
        outage = outage_result[0]
        self.assertLess(good, 0.1)
        self.assertGreater(jump, 0.5)
        self.assertGreater(outage, 0.2)
        self.assertAlmostEqual(outage_result[1]["evidence_weight_coverage"], 0.45)
        self.assertIn("incomplete_paper_evidence", outage_result[2])

    def test_imu_saturation_increases(self):
        self.assertLess(imu_score(1.0, 0.1, False)[0], 0.1)
        self.assertGreater(imu_score(0.0, 10.0, True)[0], 0.9)

    def test_optical_flow_low_quality_increases(self):
        good = optical_flow_score(0.10, 0.11, 220, 3.0)[0]
        bad = optical_flow_score(1.0, 0.0, 5, 3.0)[0]
        self.assertLess(good, 0.1)
        self.assertGreater(bad, 0.8)

    def test_vision_holes_and_blur_increase(self):
        good = vision_score(150, 150, 1.0, 0.2, 0.98)[0]
        bad = vision_score(10, 150, 0.1, 8.0, 0.10)[0]
        self.assertLess(good, 0.1)
        self.assertGreater(bad, 0.6)

    def test_missing_imu_residual_preserves_paper_weights(self):
        score, evidence, reasons = imu_score(0.0, -1.0, False)
        self.assertAlmostEqual(score, 0.35)
        self.assertAlmostEqual(evidence["evidence_weight_coverage"], 0.55)
        self.assertEqual(evidence["score_complete"], 0.0)
        self.assertIn("incomplete_paper_evidence", reasons)

    def test_missing_flow_prediction_is_not_treated_as_zero_motion(self):
        score, evidence, reasons = optical_flow_score(0.8, None, 255, 3.0)
        self.assertAlmostEqual(score, 0.0)
        self.assertAlmostEqual(evidence["evidence_weight_coverage"], 0.40)
        self.assertEqual(evidence["increment_term_eq22_adapted"], -1.0)
        self.assertIn("increment_prediction_unavailable_eq22_adapted", reasons)

    def test_missing_vision_reprojection_preserves_paper_weights(self):
        _, evidence, reasons = vision_score(150, 150, 1.0, -1.0, 1.0)
        self.assertAlmostEqual(evidence["evidence_weight_coverage"], 0.75)
        self.assertEqual(evidence["score_complete"], 0.0)
        self.assertIn("incomplete_paper_evidence", reasons)


if __name__ == "__main__":
    unittest.main()
