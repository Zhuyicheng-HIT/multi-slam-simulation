import unittest
from types import SimpleNamespace

import numpy as np

from uf_reliability.reliability_monitor import (
    VISION_FACTOR_SCORE_TOPIC,
    VISION_HEALTH_TOPIC,
    conservative_partial_score,
    depth_valid_ratio,
    flow_observation_valid,
    nonnegative_diagnostic_value,
    score_operationally_usable,
    visual_factor_track_metrics,
)


class ReliabilityMonitorHelpersTest(unittest.TestCase):
    @staticmethod
    def message(value, status_name="unified_backend_fusion"):
        return SimpleNamespace(status=[SimpleNamespace(
            name=status_name,
            values=[SimpleNamespace(
                key="imu_preintegration_residual_mahalanobis",
                value=value,
            )],
        )])

    def test_reads_backend_preintegration_residual(self):
        value = nonnegative_diagnostic_value(
            self.message("1.25"),
            "unified_backend_fusion",
            "imu_preintegration_residual_mahalanobis",
        )
        self.assertEqual(value, 1.25)

    def test_rejects_missing_stale_sentinel_and_nonfinite_values(self):
        for value in ("-1", "nan", "not-a-number"):
            result = nonnegative_diagnostic_value(
                self.message(value),
                "unified_backend_fusion",
                "imu_preintegration_residual_mahalanobis",
            )
            self.assertIsNone(result)
        self.assertIsNone(nonnegative_diagnostic_value(
            self.message("1.0", status_name="other_node"),
            "unified_backend_fusion",
            "imu_preintegration_residual_mahalanobis",
        ))

    def test_rotation_hard_gate_marks_flow_unavailable(self):
        enabled_gate = SimpleNamespace(hard_disabled=False)
        disabled_gate = SimpleNamespace(hard_disabled=True)
        self.assertTrue(flow_observation_valid(True, 0.05, enabled_gate))
        self.assertFalse(flow_observation_valid(True, 0.35, disabled_gate))
        self.assertFalse(flow_observation_valid(True, None, enabled_gate))
        self.assertFalse(flow_observation_valid(False, 0.05, enabled_gate))

    def test_partial_imu_evidence_is_provisional_after_warmup(self):
        evidence = {"evidence_weight_coverage": 0.55}
        self.assertFalse(score_operationally_usable(
            True, evidence, 19, 20, 0.55
        ))
        self.assertTrue(score_operationally_usable(
            True, evidence, 20, 20, 0.55
        ))
        self.assertFalse(score_operationally_usable(
            False, evidence, 20, 20, 0.55
        ))
        self.assertFalse(score_operationally_usable(
            True, evidence, 20, 20, 1.0
        ))

    def test_partial_gnss_score_caps_reliability_by_evidence_coverage(self):
        score = conservative_partial_score(0.02, 0.45)
        self.assertAlmostEqual(score, 0.559, places=6)
        self.assertLessEqual(1.0 - score, 0.45)
        self.assertAlmostEqual(conservative_partial_score(0.02, 1.0), 0.02)

    @staticmethod
    def visual_track(**overrides):
        values = {
            "previous_x": 0.1,
            "previous_y": 0.2,
            "current_x": 0.11,
            "current_y": 0.22,
            "track_age": 3,
            "klt_inlier": True,
            "geometric_inlier": True,
            "depth_valid": True,
            "inverse_depth": 0.5,
            "reprojection_error_px": 1.0,
            "grid_cell": 2,
        }
        values.update(overrides)
        return SimpleNamespace(**values)

    def test_visual_factor_score_uses_backend_selected_tracks(self):
        message = SimpleNamespace(tracks=[
            self.visual_track(grid_cell=2, reprojection_error_px=1.0),
            self.visual_track(grid_cell=10, reprojection_error_px=2.0),
            self.visual_track(depth_valid=False, grid_cell=11),
            self.visual_track(geometric_inlier=False, grid_cell=12),
            self.visual_track(track_age=1, grid_cell=13),
        ])
        metrics = visual_factor_track_metrics(message)
        self.assertEqual(metrics["raw_track_count"], 5)
        self.assertEqual(metrics["geometry_eligible_count"], 3)
        self.assertEqual(metrics["selected_track_count"], 2)
        self.assertAlmostEqual(metrics["selected_track_ratio"], 0.4)
        self.assertAlmostEqual(metrics["depth_valid_ratio"], 2.0 / 3.0)
        self.assertAlmostEqual(metrics["spatial_distribution"], 2.0 / 64.0)
        self.assertAlmostEqual(metrics["mean_reprojection_error_px"], 1.5)

    def test_visual_factor_selection_rejects_nonfinite_geometry(self):
        message = SimpleNamespace(tracks=[
            self.visual_track(previous_x=float("nan")),
            self.visual_track(inverse_depth=float("inf")),
            self.visual_track(inverse_depth=0.0),
        ])
        metrics = visual_factor_track_metrics(message)
        self.assertEqual(metrics["selected_track_count"], 0)

    def test_rgbd_direct_factor_selection_does_not_require_pnp_inlier(self):
        message = SimpleNamespace(tracks=[
            self.visual_track(geometric_inlier=False, grid_cell=2),
            self.visual_track(geometric_inlier=False, grid_cell=10),
            self.visual_track(geometric_inlier=False, depth_valid=False),
        ])
        paper = visual_factor_track_metrics(message, require_pnp=True)
        direct = visual_factor_track_metrics(message, require_pnp=False)
        self.assertEqual(paper["selected_track_count"], 0)
        self.assertEqual(direct["selected_track_count"], 2)
        self.assertEqual(direct["geometry_eligible_count"], 3)
        self.assertEqual(direct["mean_reprojection_error_px"], -1.0)

    def test_camera_health_and_factor_candidate_topics_are_distinct(self):
        self.assertEqual(VISION_HEALTH_TOPIC, "/reliability/vision_score")
        self.assertEqual(
            VISION_FACTOR_SCORE_TOPIC,
            "/reliability/vision_factor_score",
        )
        self.assertNotEqual(VISION_HEALTH_TOPIC, VISION_FACTOR_SCORE_TOPIC)

    def test_camera_depth_health_uses_the_supported_d435_range(self):
        values = np.asarray([0, 200, 300, 6000, 6001], dtype=np.uint16)
        message = SimpleNamespace(
            height=1,
            width=len(values),
            encoding="16UC1",
            data=values.tobytes(),
        )
        self.assertAlmostEqual(depth_valid_ratio(message, 0.30, 6.0), 0.4)


if __name__ == "__main__":
    unittest.main()
