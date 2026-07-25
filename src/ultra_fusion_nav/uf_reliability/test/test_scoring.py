import unittest

from uf_reliability.scoring import (
    gnss_integrity_quality, gnss_score, imu_score, lidar_score,
    optical_flow_displacement_frd, optical_flow_score, vision_score,
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

    def test_gnss_hard_jump_gate_forces_factor_failure(self):
        score, evidence, reasons = gnss_score(
            1.0, 0.5, 9.0, hard_jump=True
        )
        self.assertEqual(score, 1.0)
        self.assertEqual(evidence["jump_hard_gate"], 1.0)
        self.assertIn("jump_hard_gate_eq23", reasons)

    def test_fcu_gnss_metadata_refines_fix_quality(self):
        good, evidence, reasons = gnss_integrity_quality(6, 10, 1.2)
        weak, _, weak_reasons = gnss_integrity_quality(2, 4, 6.0)
        self.assertGreater(good, 0.95)
        self.assertEqual(evidence["fix_type"], 6.0)
        self.assertNotIn("few_satellites", reasons)
        self.assertLess(weak, 0.4)
        self.assertIn("weak_fcu_fix_type", weak_reasons)
        self.assertIn("few_satellites", weak_reasons)

    def test_missing_fcu_gnss_metadata_is_explicit(self):
        quality, evidence, reasons = gnss_integrity_quality(None, None, None)
        self.assertIsNone(quality)
        self.assertEqual(evidence["fix_type"], -1.0)
        self.assertIn("fcu_gnss_metadata_unavailable", reasons)

    def test_imu_saturation_increases(self):
        self.assertLess(imu_score(1.0, 0.1, False)[0], 0.1)
        self.assertGreater(imu_score(0.0, 10.0, True)[0], 0.9)

    def test_optical_flow_low_quality_increases(self):
        good = optical_flow_score(0.10, 0.11, 220, 3.0)[0]
        bad = optical_flow_score(1.0, 0.0, 5, 3.0)[0]
        self.assertLess(good, 0.1)
        self.assertGreater(bad, 0.8)

    def test_optical_flow_uses_vector_increment_residual(self):
        result = optical_flow_score((1.0, 0.0), (0.0, 1.0), 255, 3.0)
        self.assertEqual(result[1]["increment_term_eq22_adapted"], 1.0)

    def test_optical_flow_rad_geometry_recovers_frd_displacement(self):
        displacement = optical_flow_displacement_frd(0.03, 0.12, 0.01, 0.02, 2.0)
        self.assertAlmostEqual(displacement[0], 0.20)
        self.assertAlmostEqual(displacement[1], -0.04)

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
