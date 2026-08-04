import math
import unittest

from uf_sensor_pipeline.nmea0183 import (
    GgaObservation,
    KNOT_TO_MPS,
    RmcObservation,
    nmea_checksum,
    parse_sentence,
    utc_datetime,
)


def sentence(payload):
    return f"${payload}*{nmea_checksum(payload):02X}\r\n"


class Nmea0183Test(unittest.TestCase):
    def test_documented_rmc_fields_and_units(self):
        payload = (
            "GNRMC,024813.640,A,3158.4608,N,11848.3737,E,"
            "10.05,324.27,150706,,,A"
        )
        observation, checksum_valid = parse_sentence(sentence(payload))

        self.assertTrue(checksum_valid)
        self.assertIsInstance(observation, RmcObservation)
        self.assertTrue(observation.valid)
        self.assertAlmostEqual(observation.latitude_deg, 31.9743466667, places=8)
        self.assertAlmostEqual(observation.longitude_deg, 118.8062283333, places=8)
        self.assertAlmostEqual(observation.speed_mps, 10.05 * KNOT_TO_MPS)
        self.assertAlmostEqual(observation.course_deg, 324.27)
        self.assertEqual(utc_datetime(observation).isoformat(), "2006-07-15T02:48:13.640000+00:00")

    def test_document_checksum_mismatch_is_detected(self):
        documented = (
            "$GNRMC,024813.640,A,3158.4608,N,11848.3737,E,"
            "10.05,324.27,150706,,,A*50\r\n"
        )
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            parse_sentence(documented)

        observation, checksum_valid = parse_sentence(
            documented, strict_checksum=False
        )
        self.assertIsInstance(observation, RmcObservation)
        self.assertFalse(checksum_valid)

    def test_gga_provides_integrity_and_ellipsoid_altitude(self):
        payload = (
            "GNGGA,024813.640,3158.4608,N,11848.3737,E,"
            "1,12,0.8,24.3,M,-2.1,M,,"
        )
        observation, checksum_valid = parse_sentence(sentence(payload))

        self.assertTrue(checksum_valid)
        self.assertIsInstance(observation, GgaObservation)
        self.assertEqual(observation.fix_quality, 1)
        self.assertEqual(observation.satellite_count, 12)
        self.assertAlmostEqual(observation.hdop, 0.8)
        self.assertAlmostEqual(observation.altitude_ellipsoid_m, 22.2)

    def test_invalid_rmc_can_omit_position(self):
        payload = "GNRMC,024813.640,V,,,,,,,150706,,,N"
        observation, _ = parse_sentence(sentence(payload))

        self.assertFalse(observation.valid)
        self.assertTrue(math.isnan(observation.latitude_deg))
        self.assertTrue(math.isnan(observation.longitude_deg))


if __name__ == "__main__":
    unittest.main()
