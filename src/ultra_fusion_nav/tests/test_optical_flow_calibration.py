import importlib.util
from pathlib import Path
import unittest

import numpy as np


def load_calibrator():
    script = Path(__file__).resolve().parents[1] / "scripts" / "calibrate_optical_flow_lio.py"
    spec = importlib.util.spec_from_file_location("calibrate_optical_flow_lio", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CALIBRATOR = load_calibrator()


class OpticalFlowCalibrationTest(unittest.TestCase):
    def test_finds_expected_axis_mapping_and_scale(self):
        pairs = []
        for index in range(1, 30):
            body = np.asarray([0.01 * index, 0.003 * (index % 5 - 2)])
            flow = np.asarray([-body[1] / 20.0, body[0] / 20.0])
            pairs.append({
                "flow_x_m_raw": flow[0],
                "flow_y_m_raw": flow[1],
                "body_dx_m": body[0],
                "body_dy_m": body[1],
            })
        best = CALIBRATOR.fit_candidates(pairs)[0]
        self.assertTrue(best["swap_xy"])
        self.assertEqual(best["sign_x"], 1.0)
        self.assertEqual(best["sign_y"], -1.0)
        self.assertAlmostEqual(best["scale"], 20.0, places=8)
        self.assertLess(best["rmse_m"], 1.0e-10)

    def test_interpolates_yaw_across_wrap(self):
        samples = [(0.0, 0.0, 0.0, 3.0), (1.0, 1.0, 0.0, -3.0)]
        value = CALIBRATOR.interpolate_odom(samples, 0.5, max_gap_s=1.1)
        self.assertIsNotNone(value)
        self.assertAlmostEqual(value[0], 0.5)
        self.assertGreater(abs(value[2]), 3.0)

    def test_lio_pairing_uses_wall_output_interval_not_sim_integration(self):
        odom = [
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 0.0),
            (2.0, 2.0, 0.0, 0.0),
        ]
        flows = [(
            2.0,
            0.5,
            1.0,
            0.0,
            0.5,
            180,
            2.0,
            0.0,
            0.0,
            0.0,
        )]
        pairs = CALIBRATOR.build_pairs(odom, flows, min_motion_m=0.0, max_gap_s=1.1)
        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0]["body_dx_m"], 1.0)
        self.assertAlmostEqual(pairs[0]["flow_y_m_raw"], 1.0)


if __name__ == "__main__":
    unittest.main()
