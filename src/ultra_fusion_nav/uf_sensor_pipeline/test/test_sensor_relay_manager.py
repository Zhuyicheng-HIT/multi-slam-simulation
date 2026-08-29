import math
import unittest

from sensor_msgs.msg import Imu

from uf_sensor_pipeline import fault_models
from uf_sensor_pipeline.fault_models import standardize_imu_acceleration


class SensorRelayManagerTest(unittest.TestCase):
    @staticmethod
    def _body_transform(message, scale, rotation, frame_id="base_link"):
        transform = getattr(fault_models, "standardize_imu_to_body", None)
        if transform is None:
            raise AssertionError(
                "standardize_imu_to_body must implement the real MID360 mount contract"
            )
        return transform(message, scale, rotation, frame_id)

    def test_static_gravity_uses_positive_body_from_mid360_pitch(self):
        angle = math.radians(15.0)
        rotation = (
            math.cos(angle), 0.0, math.sin(angle),
            0.0, 1.0, 0.0,
            -math.sin(angle), 0.0, math.cos(angle),
        )
        message = Imu()
        message.header.frame_id = "livox_frame"
        message.header.stamp.sec = 42
        message.header.stamp.nanosec = 123
        message.linear_acceleration.x = -math.sin(angle)
        message.linear_acceleration.z = math.cos(angle)

        output = self._body_transform(message, 9.80665, rotation)

        self.assertAlmostEqual(output.linear_acceleration.x, 0.0, places=6)
        self.assertAlmostEqual(output.linear_acceleration.y, 0.0, places=6)
        self.assertAlmostEqual(output.linear_acceleration.z, 9.80665, places=6)
        self.assertEqual(output.header.frame_id, "base_link")
        self.assertEqual((output.header.stamp.sec, output.header.stamp.nanosec), (42, 123))
        self.assertEqual(message.header.frame_id, "livox_frame")
        self.assertAlmostEqual(message.linear_acceleration.x, -math.sin(angle))

    def test_raw_mid360_has_no_orientation_and_output_marks_it_unavailable(self):
        message = Imu()
        message.orientation.x = 0.2
        message.orientation.y = -0.1
        message.orientation.z = 0.3
        message.orientation.w = 0.0

        output = self._body_transform(
            message,
            9.80665,
            (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        )

        self.assertEqual(
            (output.orientation.x, output.orientation.y,
             output.orientation.z, output.orientation.w),
            (0.0, 0.0, 0.0, 1.0),
        )
        self.assertEqual(output.orientation_covariance[0], -1.0)
        self.assertEqual(message.orientation.x, 0.2)

    def test_vectors_and_covariances_rotate_into_body_flu(self):
        rotation = (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        message = Imu()
        message.linear_acceleration.x = 1.0
        message.angular_velocity.x = 2.0
        message.linear_acceleration_covariance = [
            1.0, 0.0, 0.0,
            0.0, 4.0, 0.0,
            0.0, 0.0, 9.0,
        ]
        message.angular_velocity_covariance = [
            2.0, 0.0, 0.0,
            0.0, 3.0, 0.0,
            0.0, 0.0, 5.0,
        ]

        output = self._body_transform(message, 2.0, rotation)

        self.assertAlmostEqual(output.linear_acceleration.x, 0.0)
        self.assertAlmostEqual(output.linear_acceleration.y, 2.0)
        self.assertAlmostEqual(output.angular_velocity.x, 0.0)
        self.assertAlmostEqual(output.angular_velocity.y, 2.0)
        self.assertEqual(
            list(output.linear_acceleration_covariance),
            [16.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 36.0],
        )
        self.assertEqual(
            list(output.angular_velocity_covariance),
            [3.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 5.0],
        )

    def test_unknown_covariance_sentinels_survive_body_rotation(self):
        message = Imu()
        message.linear_acceleration_covariance[0] = -1.0
        message.angular_velocity_covariance[0] = -1.0
        rotation = (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        output = self._body_transform(message, 9.80665, rotation)

        self.assertEqual(list(output.linear_acceleration_covariance),
                         list(message.linear_acceleration_covariance))
        self.assertEqual(list(output.angular_velocity_covariance),
                         list(message.angular_velocity_covariance))

    def test_rejects_non_rotation_matrix(self):
        with self.assertRaisesRegex(ValueError, "proper orthonormal rotation"):
            self._body_transform(Imu(), 1.0, (2.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0))

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
