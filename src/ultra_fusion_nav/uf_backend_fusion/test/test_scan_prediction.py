import math
import unittest

import numpy as np

from uf_backend_fusion.imu_preintegration import ImuSample
from uf_backend_fusion.imu_preintegration import preintegrate_manifold
from uf_backend_fusion.scan_prediction import (
    build_scan_prediction,
    consume_cached_prediction,
    interpolate_scan_pose,
    prediction_reusable,
    scan_request_ready,
    scan_request_stale,
    slerp_quaternion_xyzw,
)


class ScanPredictionTest(unittest.TestCase):
    @staticmethod
    def _stationary_samples(start_s=10.0, end_s=10.1, count=11):
        return [
            ImuSample(
                start_s + index * (end_s - start_s) / (count - 1),
                (0.0, 0.0, 9.81),
                (0.0, 0.0, 0.0),
            )
            for index in range(count)
        ]

    def test_prediction_integrates_interval_once_and_reuses_measurement(self):
        calls = []

        from uf_backend_fusion.imu_preintegration import preintegrate_manifold

        def counted(*args, **kwargs):
            calls.append((args[1], args[2]))
            return preintegrate_manifold(*args, **kwargs)

        state = np.zeros(15, dtype=float)
        prediction = build_scan_prediction(
            7,
            10.0,
            10.0,
            10.1,
            state,
            self._stationary_samples(),
            preintegrator=counted,
        )

        self.assertTrue(prediction.valid, prediction.reason)
        self.assertEqual(calls, [(10.0, 10.1)])
        self.assertIsNotNone(prediction.measurement)
        np.testing.assert_allclose(prediction.start_state, state)
        np.testing.assert_allclose(prediction.begin_state, state)
        np.testing.assert_allclose(prediction.end_state[:9], 0.0, atol=1.0e-9)

    def test_scan_request_waits_for_previous_native_factor_consumption(self):
        self.assertFalse(scan_request_ready(-1, 1))
        self.assertTrue(scan_request_ready(0, 1))
        self.assertFalse(scan_request_ready(3, 5))
        self.assertTrue(scan_request_ready(4, 5))

    def test_scan_request_stale_only_after_terminal_consumption(self):
        self.assertFalse(scan_request_stale(3, 4))
        self.assertTrue(scan_request_stale(4, 4))
        self.assertTrue(scan_request_stale(5, 4))

    def test_preintegration_normalizes_near_equal_interval_boundaries(self):
        samples = [
            ImuSample(10.0 - 5.0e-10, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
            ImuSample(10.05, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
            ImuSample(10.1 + 5.0e-10, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0)),
        ]

        measurement = preintegrate_manifold(samples, 10.0, 10.1)

        self.assertTrue(measurement.valid, measurement.reason)
        self.assertAlmostEqual(measurement.dt_s, 0.1)

    def test_prediction_propagates_across_dropped_lidar_packet(self):
        calls = []

        from uf_backend_fusion.imu_preintegration import preintegrate_manifold

        def counted(*args, **kwargs):
            calls.append((args[1], args[2]))
            return preintegrate_manifold(*args, **kwargs)

        prediction = build_scan_prediction(
            8,
            10.0,
            10.04,
            10.14,
            np.zeros(15),
            self._stationary_samples(10.0, 10.14, 15),
            maximum_begin_gap_s=0.02,
            preintegrator=counted,
        )

        self.assertTrue(prediction.valid, prediction.reason)
        self.assertEqual(calls, [(10.0, 10.14), (10.0, 10.04)])
        np.testing.assert_allclose(prediction.begin_state[:9], 0.0, atol=1.0e-9)
        self.assertAlmostEqual(prediction.measurement.dt_s, 0.14)

    def test_short_imu_outage_inflates_covariance_and_degrades_quality(self):
        state = np.zeros(15, dtype=float)
        samples = self._stationary_samples(10.0, 10.2, 2)
        prediction = build_scan_prediction(
            19,
            10.0,
            10.0,
            10.2,
            state,
            samples,
            nominal_imu_gap_s=0.10,
            maximum_imu_gap_s=0.30,
        )

        self.assertTrue(prediction.valid, prediction.reason)
        self.assertEqual(prediction.reason, "imu_gap_degraded")
        self.assertAlmostEqual(prediction.quality, 0.5)
        baseline = preintegrate_manifold(
            samples, 10.0, 10.2, max_gap_s=0.30
        )
        self.assertTrue(
            np.all(
                np.asarray(prediction.measurement.covariance)
                >= np.asarray(baseline.covariance)
            )
        )

    def test_prediction_rejects_scan_that_precedes_committed_state(self):
        prediction = build_scan_prediction(
            8,
            10.0,
            9.97,
            10.07,
            np.zeros(15),
            self._stationary_samples(9.97, 10.07, 11),
            maximum_begin_gap_s=0.02,
        )

        self.assertFalse(prediction.valid)
        self.assertEqual(
            prediction.reason, "scan_begin_precedes_last_committed_state"
        )

    def test_cached_measurement_requires_same_committed_start_state(self):
        state = np.zeros(15, dtype=float)
        prediction = build_scan_prediction(
            9, 10.0, 10.0, 10.1, state, self._stationary_samples()
        )
        reusable, reason = prediction_reusable(
            prediction,
            sequence=9,
            previous_stamp_s=10.0,
            scan_end_s=10.1,
            current_previous_state=state,
        )
        self.assertTrue(reusable, reason)

        changed = state.copy()
        changed[0] = 1.0e-3
        reusable, reason = prediction_reusable(
            prediction,
            sequence=9,
            previous_stamp_s=10.0,
            scan_end_s=10.1,
            current_previous_state=changed,
        )
        self.assertFalse(reusable)
        self.assertEqual(reason, "start_state_changed")

    def test_interpolation_matches_linear_translation_and_shortest_arc_slerp(self):
        half_yaw = math.pi / 4.0
        end = np.array([0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)])
        position, quaternion = interpolate_scan_pose(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [2.0, 4.0, 6.0],
            end,
            0.5,
        )

        np.testing.assert_allclose(position, [1.0, 2.0, 3.0])
        expected = np.array([
            0.0, 0.0, math.sin(math.pi / 8.0), math.cos(math.pi / 8.0)
        ])
        np.testing.assert_allclose(quaternion, expected, atol=1.0e-12)
        np.testing.assert_allclose(
            slerp_quaternion_xyzw([0, 0, 0, 1], -end, 0.5),
            expected,
            atol=1.0e-12,
        )

    def test_interpolation_rejects_out_of_scan_time(self):
        with self.assertRaises(ValueError):
            interpolate_scan_pose(
                [0, 0, 0], [0, 0, 0, 1],
                [1, 0, 0], [0, 0, 0, 1], 1.01,
            )

    def test_cached_prediction_is_consumed_once(self):
        state = np.zeros(15, dtype=float)
        prediction = build_scan_prediction(
            17, 10.0, 10.0, 10.1, state, self._stationary_samples()
        )
        cache = {17: prediction}

        consumed, reason = consume_cached_prediction(
            cache,
            sequence=17,
            previous_stamp_s=10.0,
            scan_end_s=10.1,
            current_previous_state=state,
        )

        self.assertEqual(reason, "ok")
        self.assertIs(consumed, prediction)
        self.assertIs(consumed.measurement, prediction.measurement)
        self.assertEqual(cache, {})
        second, second_reason = consume_cached_prediction(
            cache,
            sequence=17,
            previous_stamp_s=10.0,
            scan_end_s=10.1,
            current_previous_state=state,
        )
        self.assertIsNone(second)
        self.assertEqual(second_reason, "cache_miss")

    def test_changed_backend_state_discards_cached_prediction(self):
        state = np.zeros(15, dtype=float)
        prediction = build_scan_prediction(
            18, 10.0, 10.0, 10.1, state, self._stationary_samples()
        )
        changed = state.copy()
        changed[0] = 0.01
        cache = {18: prediction}

        consumed, reason = consume_cached_prediction(
            cache,
            sequence=18,
            previous_stamp_s=10.0,
            scan_end_s=10.1,
            current_previous_state=changed,
        )

        self.assertIsNone(consumed)
        self.assertEqual(reason, "start_state_changed")
        self.assertEqual(cache, {})


if __name__ == "__main__":
    unittest.main()
