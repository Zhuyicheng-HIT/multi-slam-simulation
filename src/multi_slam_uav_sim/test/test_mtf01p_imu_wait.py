from collections import deque
import time
import unittest

from multi_slam_uav_sim.mtf01p_mavlink_bridge import Mtf01pMavlinkBridge


class Mtf01pImuWaitTest(unittest.TestCase):
    @staticmethod
    def _bridge():
        bridge = object.__new__(Mtf01pMavlinkBridge)
        bridge.pending_gyro_flows = deque(maxlen=10)
        bridge.imu_samples = deque(maxlen=10)
        bridge.maximum_imu_wait_wall = 0.25
        bridge.align_sim_imu_clock = False
        bridge.imu_clock_offsets = deque(maxlen=10)
        bridge.counts = {"gyro_wait_expired": 0}
        bridge.published = []
        bridge._publish_flow = lambda flow, distance: bridge.published.append(
            (flow, distance)
        )
        return bridge

    def test_flow_waits_until_imu_covers_source_end(self):
        bridge = self._bridge()
        flow = {"stamp_ns": 10_000_000_000}
        bridge.imu_samples.append((9.99, 0.0, 0.0, 0.0))
        bridge.pending_gyro_flows.append((time.monotonic(), flow, 2.0))

        bridge._drain_pending_gyro_flows()
        self.assertEqual(bridge.published, [])

        bridge.imu_samples.append((10.01, 0.0, 0.0, 0.0))
        bridge._drain_pending_gyro_flows()
        self.assertEqual(bridge.published, [(flow, 2.0)])
        self.assertEqual(bridge.counts["gyro_wait_expired"], 0)

    def test_expired_flow_is_released_for_explicit_invalid_gyro(self):
        bridge = self._bridge()
        flow = {"stamp_ns": 10_000_000_000}
        bridge.pending_gyro_flows.append((time.monotonic() - 1.0, flow, 2.0))

        bridge._drain_pending_gyro_flows()

        self.assertEqual(bridge.published, [(flow, 2.0)])
        self.assertEqual(bridge.counts["gyro_wait_expired"], 1)

    def test_sim_clock_offset_is_applied_to_imu_coverage(self):
        bridge = self._bridge()
        bridge.align_sim_imu_clock = True
        bridge.imu_clock_offsets.extend([0.58] * 5)
        flow = {"stamp_ns": 10_000_000_000}
        bridge.imu_samples.append((9.43, 0.0, 0.0, 0.0))
        bridge.pending_gyro_flows.append((time.monotonic(), flow, 2.0))

        bridge._drain_pending_gyro_flows()

        self.assertEqual(bridge.published, [(flow, 2.0)])
        self.assertAlmostEqual(bridge._imu_time_offset_s(), 0.58)

    def test_sim_clock_offset_rejects_callback_latency(self):
        bridge = self._bridge()
        bridge.align_sim_imu_clock = True
        bridge.imu_clock_offsets = deque(
            [0.50] * 10 + [0.53] * 80 + [0.60] * 10,
            maxlen=200,
        )

        self.assertAlmostEqual(bridge._imu_time_offset_s(), 0.50)
        p05, p50, p95 = bridge._imu_time_offset_stats()
        self.assertAlmostEqual(p05, 0.50)
        self.assertAlmostEqual(p50, 0.53)
        self.assertAlmostEqual(p95, 0.60)


if __name__ == "__main__":
    unittest.main()
