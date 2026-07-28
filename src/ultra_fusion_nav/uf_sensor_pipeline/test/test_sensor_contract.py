import unittest

from uf_sensor_pipeline.sensor_contract_monitor import normalize_modalities


class SensorContractTest(unittest.TestCase):
    def test_four_source_profile_is_ordered_and_unique(self):
        active = normalize_modalities(
            ["lidar", "imu", "gnss", "optical_flow", "imu"]
        )

        self.assertEqual(active, ("lidar", "imu", "gnss", "optical_flow"))

    def test_empty_or_unknown_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_modalities([])
        with self.assertRaises(ValueError):
            normalize_modalities(["lidar", "vision"])


if __name__ == "__main__":
    unittest.main()
