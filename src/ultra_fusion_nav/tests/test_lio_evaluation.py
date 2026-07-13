import importlib.util
from pathlib import Path
import unittest

import numpy as np


def load_evaluator():
    script = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_lio_trajectory.py"
    spec = importlib.util.spec_from_file_location("evaluate_lio_trajectory", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVALUATOR = load_evaluator()


class LioEvaluationTest(unittest.TestCase):
    def test_rigid_alignment_removes_global_frame_difference(self):
        truth = np.asarray([
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ])
        estimate = truth.copy()
        estimate[:, 1] = 10.0
        estimate[:, 2] = truth[:, 1]

        report = EVALUATOR.evaluate(estimate, truth)

        self.assertLess(report["ate_rmse_m"], 1.0e-9)
        self.assertLess(report["rpe_translation_rmse_m"], 1.0e-9)

    def test_timestamp_gate_rejects_unrelated_poses(self):
        estimate = np.asarray([[0.0, 0, 0, 0, 0, 0, 0, 1]], dtype=float)
        truth = np.asarray([[1.0, 0, 0, 0, 0, 0, 0, 1]], dtype=float)

        self.assertEqual(EVALUATOR.match_rows(estimate, truth, 0.05), [])


if __name__ == "__main__":
    unittest.main()
