import unittest

from mavros_msgs.srv import StreamRate

from multi_slam_uav_sim.mavros_stream_requester import (
    fallback_stream_ids,
    fallback_stream_rate,
)


class MavrosStreamRequesterTest(unittest.TestCase):
    def test_minimal_mode_needs_no_legacy_stream_after_highres_success(self):
        self.assertEqual(fallback_stream_ids(True, True), [])

    def test_minimal_mode_falls_back_only_to_raw_sensors(self):
        self.assertEqual(
            fallback_stream_ids(True, False),
            [StreamRate.Request.STREAM_RAW_SENSORS],
        )

    def test_full_mode_avoids_duplicate_raw_imu_after_highres_success(self):
        streams = fallback_stream_ids(False, True)
        self.assertNotIn(StreamRate.Request.STREAM_RAW_SENSORS, streams)
        self.assertIn(StreamRate.Request.STREAM_POSITION, streams)

    def test_full_mode_keeps_all_legacy_fallbacks_after_highres_failure(self):
        streams = fallback_stream_ids(False, False)
        self.assertIn(StreamRate.Request.STREAM_RAW_SENSORS, streams)
        self.assertIn(StreamRate.Request.STREAM_POSITION, streams)

    def test_raw_sensor_fallback_preserves_configured_imu_rate(self):
        self.assertEqual(
            fallback_stream_rate(
                StreamRate.Request.STREAM_RAW_SENSORS, 20, 100.0
            ),
            100,
        )
        self.assertEqual(
            fallback_stream_rate(StreamRate.Request.STREAM_POSITION, 20, 100.0),
            20,
        )


if __name__ == "__main__":
    unittest.main()
