import unittest

from uf_backend_fusion.z_gauge import LocalToGlobalZGauge


class LocalToGlobalZGaugeTest(unittest.TestCase):
    def _initialized(self):
        gauge = LocalToGlobalZGauge(initialization_samples=3)
        for stamp in (1.0, 1.4, 1.8):
            update = gauge.update(
                stamp, 2.0, 2.1, 0.09,
                source_healthy=True, lidar_z_weak=False,
            )
        self.assertTrue(update.initialized)
        self.assertAlmostEqual(gauge.offset_m, 0.1)
        return gauge

    def test_strong_lidar_holds_global_gauge(self):
        gauge = self._initialized()
        update = gauge.update(
            2.2, 2.0, 2.8, 0.09,
            source_healthy=True, lidar_z_weak=False,
        )

        self.assertFalse(update.active)
        self.assertAlmostEqual(update.offset_m, 0.1)

    def test_weak_lidar_uses_bounded_global_height_correction(self):
        gauge = self._initialized()
        update = gauge.update(
            2.2, 1.5, 2.1, 0.09,
            source_healthy=True, lidar_z_weak=True,
        )

        self.assertTrue(update.active)
        self.assertGreater(update.offset_m, 0.1)
        self.assertLessEqual(update.correction_m, 0.30)
        self.assertAlmostEqual(
            gauge.local_z(gauge.global_z(3.0)), 3.0, places=9
        )

    def test_unhealthy_global_source_never_moves_gauge(self):
        gauge = self._initialized()
        update = gauge.update(
            2.2, 1.0, 20.0, 0.09,
            source_healthy=False, lidar_z_weak=True,
        )

        self.assertFalse(update.active)
        self.assertAlmostEqual(update.offset_m, 0.1)


if __name__ == "__main__":
    unittest.main()
