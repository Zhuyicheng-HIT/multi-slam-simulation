from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from uf_backend_fusion.manifold import so3_exp
from uf_backend_fusion.visual_initialization import (
    OnlineVisualTimeCalibrator,
    VisualInitializationGate,
)
from uf_backend_fusion.spatiotemporal_calibration import TimeOffsetCandidate


class VisualInitializationTest(unittest.TestCase):
    def test_tiny_rotation_does_not_dilute_time_calibration_window(self):
        calibrator = OnlineVisualTimeCalibrator(
            minimum_pairs=3,
            minimum_interval_rotation_rad=0.001,
        )

        update = calibrator.update(
            1.0,
            1.1,
            np.eye(3),
            (),
            np.eye(3),
        )

        self.assertEqual(update.reason, "insufficient_visual_interval_rotation")
        self.assertEqual(len(calibrator.motion_intervals), 0)

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
                # solvePnP maps previous camera points into current camera
                # coordinates, the inverse of the physical camera rotation.
                so3_exp(np.asarray([-rate * dt_s, 0.0, 0.0])),
                imu_samples,
            )
        self.assertTrue(update.locked)
        self.assertAlmostEqual(update.time_offset_s, true_offset_s, delta=0.006)
        self.assertGreater(update.correlation, 0.95)

    def test_invalid_pnp_rotation_is_rejected(self):
        calibrator = OnlineVisualTimeCalibrator(minimum_pairs=3)
        with self.assertRaisesRegex(ValueError, r"SO\(3\)"):
            calibrator.update(1.0, 1.1, np.zeros((3, 3)), [])

    def test_time_offset_at_search_boundary_is_not_locked(self):
        calibrator = OnlineVisualTimeCalibrator(
            minimum_pairs=3,
            offset_range_s=0.020,
            offset_step_s=0.005,
            minimum_correlation=0.50,
            minimum_correlation_margin=0.01,
            lock_count=1,
            history_length=1,
        )
        candidate = TimeOffsetCandidate(
            True, 0.020, 0.95, 0.20, 8, "candidate_ready"
        )
        with patch(
            "uf_backend_fusion.visual_initialization.estimate_interval_time_offset",
            return_value=candidate,
        ):
            update = calibrator.update(
                1.0, 1.1, so3_exp(np.asarray([0.1, 0.0, 0.0])), []
            )
        self.assertFalse(update.accepted)
        self.assertFalse(update.locked)
        self.assertEqual(update.reason, "visual_time_offset_search_boundary")

    def test_time_lock_requires_consecutive_accepted_candidates(self):
        calibrator = OnlineVisualTimeCalibrator(
            minimum_pairs=3,
            offset_range_s=0.040,
            offset_step_s=0.005,
            minimum_correlation=0.50,
            minimum_correlation_margin=0.01,
            lock_count=3,
            history_length=4,
        )
        accepted = TimeOffsetCandidate(
            True, 0.010, 0.95, 0.20, 8, "candidate_ready"
        )
        ambiguous = TimeOffsetCandidate(
            True, 0.010, 0.95, 0.001, 8, "candidate_ready"
        )
        sequence = [accepted, ambiguous, accepted, accepted]
        with patch(
            "uf_backend_fusion.visual_initialization.estimate_interval_time_offset",
            side_effect=sequence,
        ):
            updates = [
                calibrator.update(
                    float(index), float(index) + 0.1,
                    so3_exp(np.asarray([0.1, 0.0, 0.0])), [],
                )
                for index in range(1, 5)
            ]
        self.assertTrue(all(not update.locked for update in updates))
        self.assertEqual(len(calibrator.offset_history), 2)

    def test_time_lock_votes_must_come_from_independent_windows(self):
        calibrator = OnlineVisualTimeCalibrator(
            minimum_pairs=3,
            offset_range_s=0.040,
            offset_step_s=0.005,
            minimum_correlation=0.50,
            minimum_correlation_margin=0.01,
            lock_count=3,
            history_length=4,
            minimum_lock_candidate_separation_s=1.0,
        )
        candidate = TimeOffsetCandidate(
            True, 0.010, 0.95, 0.20, 8, "candidate_ready"
        )
        with patch(
            "uf_backend_fusion.visual_initialization.estimate_interval_time_offset",
            return_value=candidate,
        ):
            first = calibrator.update(
                1.0, 1.1, so3_exp(np.asarray([-0.1, 0.0, 0.0])), []
            )
            repeated = calibrator.update(
                1.2, 1.3, so3_exp(np.asarray([-0.1, 0.0, 0.0])), []
            )
            second = calibrator.update(
                2.1, 2.2, so3_exp(np.asarray([-0.1, 0.0, 0.0])), []
            )
            third = calibrator.update(
                3.2, 3.3, so3_exp(np.asarray([-0.1, 0.0, 0.0])), []
            )
        self.assertTrue(first.accepted)
        self.assertFalse(repeated.accepted)
        self.assertEqual(
            repeated.reason, "visual_time_offset_vote_not_independent"
        )
        self.assertTrue(second.accepted)
        self.assertTrue(third.locked)

    def test_static_visual_noise_cannot_lock_time_offset(self):
        calibrator = OnlineVisualTimeCalibrator(
            minimum_pairs=3,
            offset_range_s=0.040,
            offset_step_s=0.005,
            minimum_correlation=0.50,
            minimum_correlation_margin=0.01,
            lock_count=1,
            history_length=1,
            minimum_accumulated_rotation_rad=0.10,
        )
        candidate = TimeOffsetCandidate(
            True, 0.010, 0.95, 0.20, 8, "candidate_ready"
        )
        with patch(
            "uf_backend_fusion.visual_initialization.estimate_interval_time_offset",
            return_value=candidate,
        ):
            update = calibrator.update(
                1.0, 1.1, so3_exp(np.asarray([-0.001, 0.0, 0.0])), []
            )
        self.assertFalse(update.accepted)
        self.assertFalse(update.locked)
        self.assertEqual(
            update.reason, "insufficient_visual_interval_rotation"
        )

    def test_camera_extrinsic_maps_signed_rotation_into_body_axes(self):
        rotation_body_camera = np.asarray([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ])
        calibrator = OnlineVisualTimeCalibrator(
            minimum_pairs=3,
            minimum_accumulated_rotation_rad=0.01,
        )
        body_rotation = so3_exp(np.asarray([0.03, -0.01, 0.02]))
        pnp_rotation = (
            rotation_body_camera.T
            @ body_rotation.T
            @ rotation_body_camera
        )
        with patch(
            "uf_backend_fusion.visual_initialization.estimate_interval_time_offset",
            return_value=TimeOffsetCandidate(
                False, 0.0, -1.0, 0.0, 0, "insufficient_samples"
            ),
        ) as estimate:
            calibrator.update(
                1.0, 1.1, pnp_rotation, [], rotation_body_camera
            )
        interval = estimate.call_args.args[0][-1]
        np.testing.assert_allclose(
            interval[2], np.asarray([0.03, -0.01, 0.02]), atol=1.0e-9
        )

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
