import unittest


from multi_slam_uav_sim.gz_barometer_sim import GazeboBarometerBridge


class GazeboBarometerModelTest(unittest.TestCase):
    def test_reference_pressure_matches_gazebo_harmonic_model(self):
        pressure = GazeboBarometerBridge.pressure_from_height(584.0, 0.0)
        self.assertAlmostEqual(pressure, 94503.8145, places=3)

    def test_pressure_decreases_with_local_height(self):
        pressures = [
            GazeboBarometerBridge.pressure_from_height(584.0, height)
            for height in (0.0, 1.0, 5.0, 10.0, 20.0)
        ]
        self.assertEqual(pressures, sorted(pressures, reverse=True))
        self.assertAlmostEqual(pressures[1], 94492.4632, places=3)
        self.assertAlmostEqual(pressures[3], 94390.3510, places=3)

    def test_reference_altitude_is_a_fixed_datum(self):
        ground = GazeboBarometerBridge.pressure_from_height(584.0, 0.0)
        ten_m_above_ground = GazeboBarometerBridge.pressure_from_height(584.0, 10.0)
        shifted_datum = GazeboBarometerBridge.pressure_from_height(594.0, 0.0)
        self.assertAlmostEqual(ten_m_above_ground, shifted_datum, delta=0.1)
        self.assertLess(ten_m_above_ground, ground)


if __name__ == "__main__":
    unittest.main()
