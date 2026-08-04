import math
import unittest

from uf_sensor_pipeline.gps_flow_fusion import (
    GpsFlowComplementaryFilter,
    LocalEnuProjector,
    compensated_flow_velocity_frd,
    velocity_frd_to_enu,
)


class GpsFlowFusionTest(unittest.TestCase):
    def test_local_projection_preserves_meter_scale_and_axes(self):
        latitude = 45.0
        longitude = 120.0
        projector = LocalEnuProjector(latitude, longitude, 100.0)
        north_shift_deg = math.degrees(10.0 / 6378137.0)
        east_shift_deg = math.degrees(5.0 / (6378137.0 * math.cos(math.radians(latitude))))

        east, north, up = projector.project(
            latitude + north_shift_deg, longitude + east_shift_deg, 102.0)

        self.assertAlmostEqual(east, 5.0, delta=0.03)
        self.assertAlmostEqual(north, 9.98, delta=0.03)
        self.assertAlmostEqual(up, 2.0, delta=0.03)

    def test_mavlink_flow_compensation_returns_sensor_frd_velocity(self):
        velocity = compensated_flow_velocity_frd(
            integrated_x=-0.01,
            integrated_y=0.02,
            integrated_xgyro=0.0,
            integrated_ygyro=0.0,
            integration_s=0.1,
            distance_m=2.0,
        )

        self.assertIsNotNone(velocity)
        self.assertAlmostEqual(velocity[0], 0.4)
        self.assertAlmostEqual(velocity[1], 0.2)

    def test_frd_right_axis_is_converted_to_ros_left_axis(self):
        east, north = velocity_frd_to_enu(1.0, 0.5, 0.0)
        self.assertAlmostEqual(east, 1.0)
        self.assertAlmostEqual(north, -0.5)

    def test_flow_predicts_between_absolute_gnss_updates(self):
        fusion = GpsFlowComplementaryFilter(
            gps_position_gain=0.5,
            flow_velocity_gain=1.0,
            gps_jump_gate_m=10.0,
            maximum_predict_step_s=2.0,
        )
        self.assertTrue(fusion.update_gnss((0.0, 0.0, 0.0), 1.0, 1.0).accepted)
        self.assertTrue(fusion.update_flow(1.0, 0.0, 0.0, 1.1, 1.0).accepted)

        fusion.predict_to(2.1)

        self.assertAlmostEqual(fusion.position[0], 1.0, places=6)
        self.assertAlmostEqual(fusion.position[1], 0.0, places=6)

    def test_gnss_jump_is_rejected_without_dragging_state(self):
        fusion = GpsFlowComplementaryFilter(gps_jump_gate_m=5.0)
        fusion.update_gnss((0.0, 0.0, 0.0), 1.0, 1.0)

        result = fusion.update_gnss((20.0, 0.0, 0.0), 1.0, 2.0)

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "gps_jump_gate")
        self.assertAlmostEqual(fusion.position[0], 0.0)


if __name__ == "__main__":
    unittest.main()
