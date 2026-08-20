import json
import math
import unittest

from multi_slam_uav_sim.relocalization_motion import (
    MotionCommand,
    MotionStatus,
    body_offset_to_local,
    decode_motion_command,
    decode_motion_status,
    encode_motion_command,
    encode_motion_status,
    motion_observations,
)


class RelocalizationMotionTest(unittest.TestCase):
    def test_hold_is_one_stationary_observation(self):
        self.assertEqual(
            motion_observations("hold"),
            (motion_observations("hold")[0],),
        )
        observation = motion_observations("hold")[0]
        self.assertEqual(
            (observation.forward_m, observation.left_m, observation.yaw_offset_rad),
            (0.0, 0.0, 0.0),
        )

    def test_yaw_scan_covers_a_circle_and_returns_to_anchor_heading(self):
        observations = motion_observations("yaw_scan", yaw_step_deg=45.0)
        self.assertEqual(len(observations), 8)
        self.assertTrue(all(
            observation.forward_m == 0.0 and observation.left_m == 0.0
            for observation in observations
        ))
        self.assertAlmostEqual(observations[0].yaw_offset_rad, math.pi / 4.0)
        self.assertAlmostEqual(observations[-1].yaw_offset_rad, 0.0, places=12)

    def test_circle_is_tangent_to_anchor_and_returns_to_it(self):
        radius = 0.6
        observations = motion_observations("circle", radius_m=radius)
        self.assertEqual(len(observations), 4)
        for observation in observations:
            self.assertAlmostEqual(
                math.hypot(observation.forward_m, observation.left_m - radius),
                radius,
            )
            self.assertEqual(observation.yaw_offset_rad, 0.0)
        self.assertAlmostEqual(observations[-1].forward_m, 0.0, places=12)
        self.assertAlmostEqual(observations[-1].left_m, 0.0, places=12)

    def test_figure_eight_has_opposed_lobes_and_returns_to_anchor(self):
        observations = motion_observations("figure8", radius_m=0.6)
        self.assertEqual(len(observations), 4)
        self.assertGreater(observations[0].forward_m, 0.0)
        self.assertLess(observations[2].forward_m, 0.0)
        self.assertAlmostEqual(observations[1].forward_m, 0.0, places=12)
        self.assertAlmostEqual(observations[-1].forward_m, 0.0, places=12)
        self.assertTrue(all(
            observation.yaw_offset_rad == 0.0 for observation in observations
        ))

    def test_body_offset_rotates_into_fcu_local_frame(self):
        x, y = body_offset_to_local(
            2.0, 3.0, math.pi / 2.0, forward_m=1.0, left_m=0.5
        )
        self.assertAlmostEqual(x, 1.5)
        self.assertAlmostEqual(y, 4.0)

    def test_command_and_status_round_trip(self):
        command = MotionCommand(42, "circle", 2, 4)
        self.assertEqual(decode_motion_command(encode_motion_command(command)), command)
        status = MotionStatus(42, "circle", 2, 4, "settled", "ok", 0.8, 3.2)
        self.assertEqual(decode_motion_status(encode_motion_status(status)), status)

    def test_command_rejects_inconsistent_step_count(self):
        payload = json.dumps({
            "sequence_id": 1,
            "profile": "yaw_scan",
            "step_index": 0,
            "step_count": 1,
        })
        with self.assertRaisesRegex(ValueError, "step_count"):
            decode_motion_command(payload)


if __name__ == "__main__":
    unittest.main()
