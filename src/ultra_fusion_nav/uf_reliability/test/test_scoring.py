import unittest

from uf_reliability.scoring import (
    augment_lidar_score, gnss_integrity_quality, gnss_score, imu_score, lidar_score,
    lidar_factor_score, lidar_innovation_score, lidar_map_score,
    optical_flow_displacement_frd, optical_flow_score, vision_score,
)


class ScoringTest(unittest.TestCase):
    def test_lidar_degradation_increases(self):
        good = lidar_score([10, 20, 30, 40, 50, 60], [0.1, 0.2, 0.7], 0.0, 1000)[0]
        bad = lidar_score([1e-8, 1e-8, 1e-6, 10, 20, 30], [0.0, 0.0, 1.0], 0.8, 80)[0]
        self.assertLess(good, 0.3)
        self.assertGreater(bad, 0.8)

    def test_lidar_map_protection_extensions_raise_dynamic_score(self):
        paper_result = lidar_score(
            [10, 20, 30, 40, 50, 60], [0.1, 0.2, 0.7], 0.0, 800
        )
        clean = augment_lidar_score(
            paper_result, 0.03, 1.0, 0.0, 0.02, 0.95, 0.90,
        )
        dynamic = augment_lidar_score(
            paper_result, 0.10, 0.8, 0.20, 0.20, 0.55, 0.35,
        )

        self.assertGreater(dynamic[0], clean[0] + 0.20)
        self.assertEqual(
            clean[1]["paper_score_eq19"], dynamic[1]["paper_score_eq19"]
        )
        self.assertGreater(dynamic[1]["extension_score_normalized"], 0.9)
        self.assertIn("map_protection_degraded_extension", dynamic[2])

    def test_approximate_lidar_geometry_is_soft_only(self):
        poor_geometry = lidar_score(
            [1.0e-8, 1.0e-8, 1.0e-6, 10.0, 20.0, 30.0],
            [0.0, 0.0, 1.0], 0.8, 80,
        )
        consistent_prediction = lidar_innovation_score(0.02, 0.01)
        result = lidar_factor_score(
            poor_geometry, consistent_prediction, approximate_geometry=True
        )

        self.assertLess(result[0], 0.30)
        self.assertEqual(result[1]["hard_gate_allowed"], 0.0)
        self.assertEqual(result[1]["score_complete"], 1.0)
        self.assertIn("approximate_geometry_soft_only", result[2])

    def test_native_geometry_and_prediction_can_authorise_hard_gate(self):
        poor_geometry = lidar_score(
            [1.0e-8, 1.0e-8, 1.0e-6, 10.0, 20.0, 30.0],
            [0.0, 0.0, 1.0], 0.8, 80,
        )
        inconsistent_prediction = lidar_innovation_score(1.0, 0.8)
        result = lidar_factor_score(
            poor_geometry, inconsistent_prediction, approximate_geometry=False
        )

        self.assertGreater(result[0], 0.80)
        self.assertEqual(result[1]["hard_gate_allowed"], 1.0)

    def test_map_quality_diagnostic_is_not_double_counted(self):
        first = lidar_map_score(
            0.05, 0.8, 0.05, 0.10, 0.85,
            map_quality_diagnostic=0.1,
        )
        second = lidar_map_score(
            0.05, 0.8, 0.05, 0.10, 0.85,
            map_quality_diagnostic=0.9,
        )

        self.assertAlmostEqual(first[0], second[0])
        self.assertNotEqual(
            first[1]["map_quality_diagnostic"],
            second[1]["map_quality_diagnostic"],
        )
        self.assertIn("map_risk_not_used_for_lidar_pose_factor", first[2])

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
