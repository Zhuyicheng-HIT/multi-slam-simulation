import unittest

from multi_slam_uav_sim.micolink_protocol import (
    MICOLINK_MTF01_DEVICE_ID,
    MICOLINK_RANGE_SENSOR_MESSAGE_ID,
    MicoLinkParser,
    MicoLinkSensorClock,
    Mtf01RangeFlow,
    decode_range_flow_payload,
    encode_range_flow_frame,
    flow_velocity_to_integrated_radians,
    integrated_radians_to_flow_velocity,
    sensor_interval_seconds,
)


CAPTURED_COM17_FRAME = bytes.fromhex(
    "EF 0F 00 51 35 14 A0 56 11 00 14 00 00 00 "
    "FF 00 01 FF 00 00 00 00 36 01 FF FF E7"
)


class MicoLinkProtocolTest(unittest.TestCase):
    def test_decodes_real_com17_mtf01_frame(self):
        parser = MicoLinkParser()
        frames = parser.feed(CAPTURED_COM17_FRAME)
        self.assertEqual(len(frames), 1)
        frame = frames[0]
        self.assertEqual(frame.device_id, MICOLINK_MTF01_DEVICE_ID)
        self.assertEqual(frame.system_id, 0)
        self.assertEqual(frame.message_id, MICOLINK_RANGE_SENSOR_MESSAGE_ID)
        self.assertEqual(frame.sequence, 0x35)
        observation = decode_range_flow_payload(frame.payload)
        self.assertEqual(observation.time_ms, 1136288)
        self.assertEqual(observation.distance_mm, 20)
        self.assertEqual(observation.strength, 255)
        self.assertEqual(observation.tof_status, 1)
        self.assertEqual(observation.flow_velocity_x, 0)
        self.assertEqual(observation.flow_velocity_y, 0)
        self.assertEqual(observation.flow_quality, 54)
        self.assertEqual(observation.flow_status, 1)

    def test_frame_roundtrip_matches_fixed_27_byte_wire_format(self):
        observation = Mtf01RangeFlow(
            time_ms=1234,
            distance_mm=1750,
            strength=255,
            precision=0,
            tof_status=1,
            reserved1=255,
            flow_velocity_x=-123,
            flow_velocity_y=456,
            flow_quality=82,
            flow_status=1,
            reserved2=0xFFFF,
        )
        encoded = encode_range_flow_frame(observation, sequence=7)
        self.assertEqual(len(encoded), 27)
        decoded = MicoLinkParser().feed(encoded)[0]
        self.assertEqual(decode_range_flow_payload(decoded.payload), observation)

    def test_stream_parser_resynchronizes_across_chunks_and_noise(self):
        parser = MicoLinkParser()
        self.assertEqual(parser.feed(b"noise" + CAPTURED_COM17_FRAME[:11]), [])
        frames = parser.feed(CAPTURED_COM17_FRAME[11:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(parser.discarded_bytes, 5)

    def test_stream_parser_rejects_bad_checksum(self):
        damaged = bytearray(CAPTURED_COM17_FRAME)
        damaged[-1] ^= 0x01
        parser = MicoLinkParser()
        self.assertEqual(parser.feed(damaged), [])
        self.assertEqual(parser.checksum_errors, 1)

    def test_flow_rate_conversion_uses_micolink_units(self):
        encoded = integrated_radians_to_flow_velocity(0.0123, -0.0046, 0.01)
        self.assertEqual(encoded, (123, -46))
        recovered = flow_velocity_to_integrated_radians(*encoded, 0.01)
        self.assertAlmostEqual(recovered[0], 0.0123)
        self.assertAlmostEqual(recovered[1], -0.0046)

    def test_sensor_time_supports_32_bit_wrap(self):
        self.assertAlmostEqual(sensor_interval_seconds(0xFFFFFFFA, 4), 0.010)
        self.assertIsNone(sensor_interval_seconds(100, 100))
        clock = MicoLinkSensorClock(initial_time_ms=0xFFFFFFFE)
        self.assertEqual(clock.advance(3000), 1)


if __name__ == "__main__":
    unittest.main()
