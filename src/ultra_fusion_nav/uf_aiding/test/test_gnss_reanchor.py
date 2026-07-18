import unittest

import numpy as np

from uf_aiding.gnss_reanchor import (
    ACTIVE, OUTAGE, REANCHORING, REJECTED_JUMP, SmoothGnssReanchor,
)


class SmoothGnssReanchorTest(unittest.TestCase):
    def _initialize(self, reanchor):
        result = None
        for index in range(5):
            result = reanchor.update(
                index * 0.2,
                [100.0 + index * 0.1, 20.0, 3.0],
                [index * 0.1, 0.0, 3.0],
                0.05,
            )
        self.assertEqual(result.state, REANCHORING)
        result = reanchor.update(4.0, [102.0, 20.0, 3.0], [2.0, 0.0, 3.0], 0.05)
        self.assertEqual(result.state, ACTIVE)
        return result

    def test_jump_is_rejected_without_output(self):
        reanchor = SmoothGnssReanchor()
        self._initialize(reanchor)
        result = reanchor.update(4.2, [120.0, 20.0, 3.0], [2.2, 0.0, 3.0], 0.8)
        self.assertFalse(result.accepted)
        self.assertIsNone(result.position)

        result = reanchor.update(4.3, [120.0, 20.0, 3.0], [2.3, 0.0, 3.0], 0.05)
        self.assertEqual(result.state, REJECTED_JUMP)
        self.assertFalse(result.accepted)
        self.assertGreater(result.innovation_m, 3.0)

    def test_outage_requires_stable_reacquisition_and_ramps_blend(self):
        reanchor = SmoothGnssReanchor(reanchor_duration_s=2.0)
        self._initialize(reanchor)
        result = reanchor.update(6.0, None, [3.0, 0.0, 3.0], 1.0, valid_fix=False)
        self.assertEqual(result.state, OUTAGE)

        for index in range(4):
            result = reanchor.update(
                6.2 + index * 0.2,
                [203.0 + index * 0.1, -30.0, 3.0],
                [3.0 + index * 0.1, 0.0, 3.0],
                0.05,
            )
            self.assertFalse(result.accepted)
        result = reanchor.update(7.0, [203.4, -30.0, 3.0], [3.4, 0.0, 3.0], 0.05)
        self.assertEqual(result.state, REANCHORING)
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.blend, 0.0)
        self.assertLess(float(np.linalg.norm(result.position - np.asarray([3.4, 0.0, 3.0]))), 1.0e-9)

        result = reanchor.update(8.0, [203.9, -30.0, 3.0], [3.9, 0.0, 3.0], 0.05)
        self.assertAlmostEqual(result.blend, 0.5)
        result = reanchor.update(9.0, [204.4, -30.0, 3.0], [4.4, 0.0, 3.0], 0.05)
        self.assertEqual(result.state, ACTIVE)
        self.assertAlmostEqual(result.blend, 1.0)

    def test_unstable_reacquisition_never_unlocks(self):
        reanchor = SmoothGnssReanchor(anchor_stability_m=0.5)
        self._initialize(reanchor)
        reanchor.update(6.0, None, [3.0, 0.0, 3.0], 1.0, valid_fix=False)
        for index in range(10):
            offset = 100.0 if index % 2 else 120.0
            result = reanchor.update(
                6.2 + index * 0.2,
                [offset, 20.0, 3.0],
                [3.0 + index * 0.1, 0.0, 3.0],
                0.05,
            )
            self.assertFalse(result.accepted)
            self.assertIsNone(result.position)


if __name__ == "__main__":
    unittest.main()
