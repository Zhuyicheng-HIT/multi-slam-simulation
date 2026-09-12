import unittest

from sensor_msgs.msg import Imu

from uf_sensor_pipeline.fault_models import standardize_imu_acceleration


class SensorRelayManagerTest(unittest.TestCase):
    def test_imu_relay_unit_conversion_preserves_unknown_covariance(self):
        message = Imu()
        message.linear_acceleration.x = 1.0
        message.linear_acceleration.y = -2.0
        message.linear_acceleration.z = 0.5
        message.linear_acceleration_covariance[0] = -1.0
        output = standardize_imu_acceleration(message, 9.80665)
        self.assertAlmostEqual(output.linear_acceleration.x, 9.80665)
        self.assertAlmostEqual(output.linear_acceleration.y, -19.6133)
        self.assertAlmostEqual(output.linear_acceleration.z, 4.903325)
        self.assertEqual(output.linear_acceleration_covariance[0], -1.0)

    def test_imu_relay_unit_conversion_scales_known_covariance(self):
        message = Imu()
        message.linear_acceleration_covariance[0] = 2.0
        output = standardize_imu_acceleration(message, 2.0)
        self.assertEqual(output.linear_acceleration_covariance[0], 8.0)


if __name__ == "__main__":
    unittest.main()
