import math
import unittest

import cv2
import numpy as np

from multi_slam_uav_sim.optical_flow_model import (
    compensated_planar_velocity,
    integrate_gyro,
    pixel_flow_to_radians,
    ros_flu_gyro_to_sensor_frd,
    sensor_velocity_frd,
    sensor_displacement_frd,
    synthesize_optical_flow,
    synthesize_optical_flow_from_displacement,
    track_lk_flow,
)


class OpticalFlowModelTest(unittest.TestCase):
    def test_pixel_flow_uses_camera_focal_length_without_empirical_scale(self):
        flow = pixel_flow_to_radians(10.0, -5.0, 500.0, 400.0)
        self.assertAlmostEqual(flow[0], math.atan2(10.0, 500.0))
        self.assertAlmostEqual(flow[1], math.atan2(-5.0, 400.0))

    def test_gyro_is_converted_to_sensor_frd_and_integrated_in_window(self):
        self.assertEqual(ros_flu_gyro_to_sensor_frd((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))
        samples = [
            (0.0, 1.0, -2.0, -3.0),
            (0.05, 1.0, -2.0, -3.0),
            (0.10, 1.0, -2.0, -3.0),
        ]
        integral = integrate_gyro(samples, 0.0, 0.10)
        np.testing.assert_allclose(integral, (0.1, -0.2, -0.3), atol=1.0e-12)

    def test_compensated_velocity_matches_mavlink_axis_definition(self):
        velocity = compensated_planar_velocity(
            raw_flow_rad=(0.02, 0.04),
            gyro_rad=(0.01, 0.01),
            integration_s=0.10,
            distance_m=2.0,
        )
        self.assertAlmostEqual(velocity[0], 0.6)
        self.assertAlmostEqual(velocity[1], -0.2)

    def test_physics_sensor_round_trip_preserves_planar_velocity(self):
        velocity = sensor_velocity_frd(
            world_velocity=(1.0, -2.0, 0.0),
            body_to_world_quaternion=(0.0, 0.0, 0.0, 1.0),
            gyro_frd=(0.0, 0.0, 0.0),
            lever_arm_frd=(0.0, 0.0, 0.35),
        )
        self.assertEqual(velocity, (1.0, 2.0, 0.0))
        raw = synthesize_optical_flow(
            velocity_frd=velocity,
            gyro_integral_frd=(0.01, -0.02, 0.0),
            integration_s=0.1,
            distance_m=2.0,
        )
        recovered = compensated_planar_velocity(
            raw_flow_rad=raw,
            gyro_rad=(0.01, -0.02),
            integration_s=0.1,
            distance_m=2.0,
        )
        np.testing.assert_allclose(recovered, velocity[:2], atol=1.0e-12)

    def test_pose_window_synthesis_preserves_sensor_displacement(self):
        displacement = sensor_displacement_frd(
            start_pose=((0.0, 0.0, 2.0), (0.0, 0.0, 0.0, 1.0)),
            end_pose=((0.1, -0.2, 2.0), (0.0, 0.0, 0.0, 1.0)),
            lever_arm_frd=(0.0, 0.0, 0.35),
        )
        self.assertEqual(displacement, (0.1, 0.2, 0.0))
        raw = synthesize_optical_flow_from_displacement(
            displacement,
            gyro_integral_frd=(0.01, -0.02, 0.0),
            distance_m=2.0,
        )
        recovered = compensated_planar_velocity(
            raw_flow_rad=raw,
            gyro_rad=(0.01, -0.02),
            integration_s=0.1,
            distance_m=2.0,
        )
        np.testing.assert_allclose(recovered, (1.0, 2.0), atol=1.0e-12)

    def test_lk_tracker_recovers_known_image_translation(self):
        rng = np.random.default_rng(7)
        previous = np.zeros((240, 320), dtype=np.uint8)
        for _ in range(160):
            x = int(rng.integers(10, 310))
            y = int(rng.integers(10, 230))
            value = int(rng.integers(80, 255))
            cv2.circle(previous, (x, y), 2, value, -1)
        transform = np.asarray([[1.0, 0.0, 3.5], [0.0, 1.0, -2.25]], dtype=np.float32)
        current = cv2.warpAffine(previous, transform, (320, 240))
        result = track_lk_flow(previous, current, min_inliers=8)
        self.assertGreater(result.quality, 0)
        self.assertGreaterEqual(result.inlier_count, 20)
        self.assertAlmostEqual(result.dx_px, 3.5, delta=0.25)
        self.assertAlmostEqual(result.dy_px, -2.25, delta=0.25)

    def test_lk_tracker_rejects_textureless_image(self):
        image = np.full((120, 160), 127, dtype=np.uint8)
        result = track_lk_flow(image, image)
        self.assertEqual(result.quality, 0)
        self.assertEqual(result.inlier_count, 0)


if __name__ == "__main__":
    unittest.main()
