import math
import unittest
from types import SimpleNamespace

import numpy as np

from multi_slam_uav_sim.external_nav_accuracy import ExternalNavAccuracy
from multi_slam_uav_sim.simulation_performance_monitor import (
    TopicWindow,
    diagnostic_timing_values,
)


class ExternalNavAccuracyTest(unittest.TestCase):
    def test_performance_monitor_accepts_backend_timing_diagnostics(self):
        message = SimpleNamespace(status=[SimpleNamespace(
            name="unified_backend_fusion",
            values=[
                SimpleNamespace(key="backend_solve_mean_ms", value="16.5"),
                SimpleNamespace(key="backend_solve_max_ms", value="25.0"),
                SimpleNamespace(key="callback_ms", value="18.2"),
                SimpleNamespace(key="window_states", value="8"),
                SimpleNamespace(key="backend_solve_ms", value="not-a-number"),
            ],
        )])

        values = diagnostic_timing_values(message)

        self.assertEqual(values, {
            "unified_backend_fusion/backend_solve_mean_ms": 16.5,
            "unified_backend_fusion/backend_solve_max_ms": 25.0,
            "unified_backend_fusion/callback_ms": 18.2,
        })

    def test_topic_window_exposes_source_to_arrival_rate_mismatch(self):
        window = TopicWindow()
        for index in range(10):
            window.add(arrival_s=index * 0.1, source_stamp_s=index * 0.05)

        summary = window.summary(now_s=0.9, window_s=2.0)

        self.assertAlmostEqual(summary["rate_hz"], 10.0)
        self.assertAlmostEqual(summary["source_stamp_rate_hz"], 20.0)
        self.assertAlmostEqual(summary["source_to_arrival_rate_ratio"], 2.0)
        self.assertAlmostEqual(summary["arrival_interval_median_ms"], 100.0)

    def test_rigid_alignment_recovers_rotation_without_hiding_scale(self):
        estimate = np.asarray([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
        angle = math.radians(30.0)
        rotation = np.asarray([
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ])
        truth = (rotation @ (1.2 * estimate).T).T + np.asarray([3.0, -2.0])

        aligned, recovered_rotation, _, scale = ExternalNavAccuracy._align_xy(
            estimate, truth)

        self.assertAlmostEqual(
            math.degrees(math.atan2(recovered_rotation[1, 0], recovered_rotation[0, 0])),
            30.0,
            places=6,
        )
        self.assertAlmostEqual(scale, 1.2, places=6)
        self.assertGreater(float(np.max(np.linalg.norm(aligned - truth, axis=1))), 0.1)


if __name__ == "__main__":
    unittest.main()
