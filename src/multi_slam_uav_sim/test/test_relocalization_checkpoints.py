import unittest

from multi_slam_uav_sim.relocalization_checkpoints import (
    MissionCheckpoint,
    decode_checkpoint,
    encode_checkpoint,
    parse_checkpoint_indices,
)


class RelocalizationCheckpointTest(unittest.TestCase):
    def test_checkpoint_indices_are_positive_unique_and_increasing(self):
        self.assertEqual(parse_checkpoint_indices("3, 7,11"), (3, 7, 11))

        for invalid in ("", "0,2", "2,2", "3,1", "one,2"):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                parse_checkpoint_indices(invalid)

    def test_checkpoint_payload_round_trip_is_structured_and_finite(self):
        checkpoint = MissionCheckpoint(
            index=4,
            label="S pass 1/1",
            distance_m=8.0,
            position=(1.0, -2.0, 5.5),
        )

        self.assertEqual(decode_checkpoint(encode_checkpoint(checkpoint)), checkpoint)

    def test_checkpoint_payload_rejects_invalid_geometry(self):
        with self.assertRaises(ValueError):
            encode_checkpoint(
                MissionCheckpoint(0, "route", 1.0, (0.0, 0.0, 5.0)))
        with self.assertRaises(ValueError):
            decode_checkpoint(
                '{"index":1,"label":"route","distance_m":1.0,'
                '"position":[0,1]}')


if __name__ == "__main__":
    unittest.main()
