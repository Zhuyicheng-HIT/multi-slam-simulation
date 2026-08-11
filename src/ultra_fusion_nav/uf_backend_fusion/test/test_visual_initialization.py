from types import SimpleNamespace
import unittest

import numpy as np

from uf_backend_fusion.manifold import so3_exp
from uf_backend_fusion.visual_initialization import (
    OnlineVisualTimeCalibrator,
    VisualInitializationGate,
)


class VisualInitializationTest(unittest.TestCase):
    @staticmethod
    def _rate(stamp_s):
        return (
            0.45
            + 0.20 * np.sin(3.7 * stamp_s)
            + 0.13 * np.sin(8.3 * stamp_s + 0.4)
        )

    def test_camera_imu_time_offset_locks_after_stable_excitation(self):
        true_offset_s = 0.020
        imu_samples = [
            SimpleNamespace(
                stamp_s=float(stamp_s),
                angular_velocity=np.asarray([
                    self._rate(stamp_s - true_offset_s), 0.0, 0.0
                ]),
            )
            for stamp_s in np.arange(0.0, 6.0, 0.005)
        ]
        calibrator = OnlineVisualTimeCalibrator(
            window_s=6.0,
            minimum_pairs=8,
            offset_range_s=0.040,
            offset_step_s=0.005,
            minimum_correlation=0.80,
            minimum_correlation_margin=0.0001,
            history_length=4,
            lock_count=3,
            stability_tolerance_s=0.006,
        )
        update = calibrator.last_update
        for midpoint_s in np.arange(0.5, 5.0, 0.20):
            dt_s = 0.10
            rate = self._rate(midpoint_s)
            update = calibrator.update(
                midpoint_s - 0.5 * dt_s,
                midpoint_s + 0.5 * dt_s,
                so3_exp(np.asarray([rate * dt_s, 0.0, 0.0])),
                imu_samples,
            )
        self.assertTrue(update.locked)
        self.assertAlmostEqual(update.time_offset_s, true_offset_s, delta=0.006)
        self.assertGreater(update.correlation, 0.95)

    def test_invalid_pnp_rotation_is_rejected(self):
        calibrator = OnlineVisualTimeCalibrator(minimum_pairs=3)
        with self.assertRaisesRegex(ValueError, r"SO\(3\)"):
            calibrator.update(1.0, 1.1, np.zeros((3, 3)), [])

    def test_visual_gate_requires_time_lock_and_consecutive_batches(self):
        gate = VisualInitializationGate(
            minimum_batches=3, require_time_lock=True
        )
        waiting = gate.observe(geometrically_valid=True, time_locked=False)
        self.assertFalse(waiting.ready)
        self.assertEqual(waiting.reason, "waiting_for_visual_time_lock")
        first = gate.observe(geometrically_valid=True, time_locked=True)
        second = gate.observe(geometrically_valid=True, time_locked=True)
        third = gate.observe(geometrically_valid=True, time_locked=True)
        self.assertFalse(first.ready)
        self.assertFalse(second.ready)
        self.assertTrue(third.ready)

    def test_visual_gate_resets_after_inconsistent_batch(self):
        gate = VisualInitializationGate(
            minimum_batches=2, require_time_lock=False
        )
        gate.observe(geometrically_valid=True, time_locked=False)
        rejected = gate.observe(geometrically_valid=False, time_locked=False)
        self.assertEqual(rejected.consecutive_batches, 0)
        self.assertFalse(rejected.ready)


if __name__ == "__main__":
    unittest.main()
