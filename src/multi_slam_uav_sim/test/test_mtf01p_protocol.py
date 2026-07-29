import math
import struct
import unittest

from multi_slam_uav_sim.mtf01p_protocol import (
    DISTANCE_SENSOR_MESSAGE_ID,
    OPTICAL_FLOW_MESSAGE_ID,
    SensorClock,
    decode_distance_sensor_payload,
    decode_optical_flow_payload,
    focal_length_px,
    integrated_radians_to_pixels,
    mavros_payload_bytes,
    pixels_to_integrated_radians,
    sensor_frd_to_ros_flu,
)


class Mtf01pProtocolTest(unittest.TestCase):
    def test_mavlink_apm_message_ids(self):
        self.assertEqual(OPTICAL_FLOW_MESSAGE_ID, 100)
        self.assertEqual(DISTANCE_SENSOR_MESSAGE_ID, 132)

    def test_100px_42deg_focal_length(self):
        self.assertAlmostEqual(
            focal_length_px(100, math.radians(42.0)), 130.254, places=3
        )

    def test_mavlink1_pixel_roundtrip_is_quantized_but_consistent(self):
        focal = focal_length_px()
        original = (0.035, -0.021)
        pixels = integrated_radians_to_pixels(*original, focal, focal)
        recovered = pixels_to_integrated_radians(*pixels, focal, focal)
        self.assertAlmostEqual(recovered[0], original[0], delta=0.5 / focal)
        self.assertAlmostEqual(recovered[1], original[1], delta=0.5 / focal)

    def test_aircraft_frd_to_ros_flu_flips_y(self):
        self.assertEqual(sensor_frd_to_ros_flu(3.0, -4.0), (3.0, 4.0))

    def test_sensor_clock_uses_integration_intervals(self):
        clock = SensorClock(initial_time_usec=1000)
        self.assertEqual(clock.advance(33333), 34333)
        self.assertEqual(clock.advance(10000), 44333)

    def test_mavros_payload64_decodes_optical_flow_base_fields(self):
        payload = struct.pack("<QfffhhBB", 123456, 1.5, -2.5, 3.0, 12, -7, 0, 201)
        padded = payload + bytes((-len(payload)) % 8)
        payload64 = struct.unpack(f"<{len(padded) // 8}Q", padded)
        decoded = decode_optical_flow_payload(
            mavros_payload_bytes(payload64, len(payload))
        )
        self.assertEqual(decoded["time_usec"], 123456)
        self.assertEqual(decoded["flow_x"], 12)
        self.assertEqual(decoded["flow_y"], -7)
        self.assertEqual(decoded["quality"], 201)

    def test_distance_sensor_base_fields_decode(self):
        payload = struct.pack("<IHHHBBBB", 321, 8, 1200, 245, 0, 0, 25, 255)
        decoded = decode_distance_sensor_payload(payload)
        self.assertEqual(decoded["time_boot_ms"], 321)
        self.assertEqual(decoded["current_distance_cm"], 245)
        self.assertEqual(decoded["orientation"], 25)


if __name__ == "__main__":
    unittest.main()
