import math
import unittest

import numpy as np

from uf_backend_fusion.imu_preintegration import (
    ImuSample,
    ManifoldPreintegratedImu,
    preintegrate_manifold,
)
from uf_backend_fusion.live_propagation import (
    auxiliary_keyframe_admission,
    backend_process_covariance,
    backend_state_transition,
    live_propagation_admission,
    make_optimization_anchor,
    propagate_optimization_anchor,
    state_covariance_to_odometry_covariances,
    unified_odom_publication_decision,
)


def measurement_with_covariance(covariance):
    zeros = (0.0,) * 9
    return ManifoldPreintegratedImu(
        True,
        "ok",
        0.1,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        tuple(float(value) for value in np.asarray(covariance).ravel()),
        1,
        0.1,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        zeros,
        zeros,
        zeros,
        zeros,
        zeros,
    )


class LivePropagationTest(unittest.TestCase):
    def test_fixed_rate_owner_prevents_propagation_optimized_timestamp_race(self):
        # Legacy dual publishing lets a newer IMU stamp suppress the optimizer
        # output that commits immediately afterwards.
        self.assertEqual(
            unified_odom_publication_decision(
                "legacy_hybrid", "optimized", 9.95, 10.0
            ),
            (False, "nonmonotonic_output"),
        )
        # Fixed-rate mode makes that optimized state an anchor-only update;
        # the next timer tick remains the sole monotonic odometry writer.
        self.assertEqual(
            unified_odom_publication_decision(
                "fixed_rate_propagated", "optimized", 9.95, 10.0
            ),
            (False, "source_not_owner"),
        )
        self.assertEqual(
            unified_odom_publication_decision(
                "fixed_rate_propagated", "imu_propagated", 10.10, 10.0
            ),
            (True, "ready"),
        )
        self.assertEqual(
            unified_odom_publication_decision(
                "lidar_event_propagated", "optimized", 10.10, 10.0
            ),
            (False, "source_not_owner"),
        )
        self.assertEqual(
            unified_odom_publication_decision(
                "lidar_event_propagated", "imu_propagated", 10.10, 10.0
            ),
            (True, "ready"),
        )

    def test_auxiliary_keyframe_requires_native_outage_and_fresh_imu(self):
        arguments = dict(
            now_s=10.0,
            latest_imu_stamp_s=9.98,
            last_state_stamp_s=9.70,
            latest_native_arrival_s=9.50,
            lidar_silence_timeout_s=0.35,
            minimum_state_interval_s=0.20,
            maximum_imu_age_s=0.20,
        )
        self.assertEqual(
            auxiliary_keyframe_admission(**arguments), (True, "ready")
        )
        self.assertEqual(
            auxiliary_keyframe_admission(
                **{**arguments, "latest_native_arrival_s": 9.80}
            ),
            (False, "lidar_recent"),
        )
        self.assertEqual(
            auxiliary_keyframe_admission(
                **{**arguments, "latest_imu_stamp_s": 9.75}
            ),
            (False, "imu_stale"),
        )
        self.assertEqual(
            auxiliary_keyframe_admission(
                **{**arguments, "last_state_stamp_s": 9.85}
            ),
            (False, "imu_not_advanced"),
        )

    def test_admission_requires_lidar_silence_and_monotonic_output(self):
        arguments = dict(
            now_s=10.0,
            latest_imu_stamp_s=9.98,
            target_stamp_s=9.98,
            anchor_stamp_s=9.70,
            last_output_stamp_s=9.80,
            latest_lidar_activity_s=9.60,
            lidar_silence_timeout_s=0.25,
            maximum_output_age_s=0.20,
            minimum_output_interval_s=0.08,
            maximum_imu_age_s=0.20,
        )
        self.assertEqual(
            live_propagation_admission(**arguments), (True, "ready")
        )
        self.assertEqual(
            live_propagation_admission(
                **{**arguments, "latest_lidar_activity_s": 9.90}
            ),
            (False, "lidar_recent"),
        )
        self.assertEqual(
            live_propagation_admission(
                **{
                    **arguments,
                    "last_output_stamp_s": 9.70,
                    "latest_lidar_activity_s": 9.90,
                }
            ),
            (True, "ready"),
        )
        self.assertEqual(
            live_propagation_admission(
                **{**arguments, "last_output_stamp_s": 9.95}
            ),
            (False, "output_not_advanced"),
        )

    def test_continuous_output_ignores_lidar_recency_but_keeps_timestamp_gate(self):
        arguments = dict(
            now_s=10.0,
            latest_imu_stamp_s=9.98,
            target_stamp_s=9.98,
            anchor_stamp_s=9.70,
            last_output_stamp_s=9.80,
            latest_lidar_activity_s=9.99,
            lidar_silence_timeout_s=0.25,
            maximum_output_age_s=0.20,
            minimum_output_interval_s=0.08,
            maximum_imu_age_s=0.20,
            continuous_output=True,
        )
        self.assertEqual(
            live_propagation_admission(**arguments), (True, "ready")
        )
        self.assertEqual(
            live_propagation_admission(
                **{
                    **arguments,
                    "anchor_stamp_s": 9.94,
                    "last_output_stamp_s": 9.80,
                }
            ),
            (True, "ready"),
        )
        self.assertEqual(
            live_propagation_admission(
                **{**arguments, "last_output_stamp_s": 9.95}
            ),
            (False, "output_not_advanced"),
        )

    def test_anchor_is_immutable_deep_copy(self):
        state = np.zeros(15)
        covariance = np.eye(15)
        anchor = make_optimization_anchor(10.0, state, covariance, 1)
        state[0] = 100.0
        covariance[0, 0] = 100.0
        self.assertEqual(anchor.state[0], 0.0)
        self.assertEqual(anchor.covariance[0], 1.0)
        self.assertEqual(anchor.reset_counter, 0)
        with self.assertRaisesRegex(ValueError, "positive semidefinite"):
            invalid = np.eye(15)
            invalid[0, 0] = -1.0
            make_optimization_anchor(10.0, np.zeros(15), invalid, 1)

    def test_static_imu_propagation_preserves_pose_and_produces_psd(self):
        samples = [
            ImuSample(stamp, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
            for stamp in (10.0, 10.05, 10.10)
        ]
        measurement = preintegrate_manifold(samples, 10.0, 10.1)
        self.assertTrue(measurement.valid)
        anchor = make_optimization_anchor(
            10.0, np.zeros(15), np.eye(15) * 0.01, 7
        )
        propagated = propagate_optimization_anchor(anchor, 10.1, measurement)
        state = np.asarray(propagated.state)
        covariance = np.asarray(propagated.covariance).reshape(15, 15)
        np.testing.assert_allclose(state[:9], np.zeros(9), atol=1.0e-10)
        np.testing.assert_allclose(covariance, covariance.T, atol=1.0e-12)
        self.assertGreaterEqual(float(np.min(np.linalg.eigvalsh(covariance))), -1.0e-12)
        self.assertEqual(propagated.anchor_generation, 7)
        self.assertEqual(propagated.anchor_reset_counter, 0)

    def test_transition_contains_position_velocity_and_right_local_rotation(self):
        measurement = measurement_with_covariance(np.eye(15))
        transition = backend_state_transition(np.zeros(15), measurement)
        np.testing.assert_allclose(transition[0:3, 6:9], np.eye(3) * 0.1)
        np.testing.assert_allclose(transition[3:6, 3:6], np.eye(3))
        np.testing.assert_allclose(transition[6:9, 6:9], np.eye(3))

    def test_process_covariance_reorders_preintegration_blocks(self):
        diagonal = np.arange(1.0, 16.0)
        measurement = measurement_with_covariance(np.diag(diagonal))
        state = np.zeros(15)
        state[5] = math.pi / 2.0
        process = backend_process_covariance(state, measurement)
        np.testing.assert_allclose(np.diag(process)[0:3], [2.0, 1.0, 3.0])
        np.testing.assert_allclose(np.diag(process)[3:6], [7.0, 8.0, 9.0])
        np.testing.assert_allclose(np.diag(process)[6:9], [5.0, 4.0, 6.0])
        np.testing.assert_allclose(np.diag(process)[9:15], diagonal[9:15])

    def test_ros_covariance_maps_right_local_rotation_and_body_velocity(self):
        state = np.zeros(15)
        state[5] = math.pi / 2.0
        state[6] = 2.0
        covariance = np.diag(np.arange(1.0, 16.0))
        pose, velocity = state_covariance_to_odometry_covariances(
            state, covariance
        )
        np.testing.assert_allclose(np.diag(pose)[0:3], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(np.diag(pose)[3:6], [5.0, 4.0, 6.0])
        self.assertTrue(np.all(np.isfinite(velocity)))
        self.assertGreaterEqual(float(np.min(np.linalg.eigvalsh(velocity))), -1.0e-12)


if __name__ == "__main__":
    unittest.main()
