import unittest
from types import SimpleNamespace

import numpy as np

from uf_backend_fusion.native_lidar import (
    EXPECTED_POSE_STATE_ORDER,
    NativeFactorBuffer,
    native_factor_from_message,
    quaternion_xyzw_to_rpy,
    right_perturbation_jacobian_rpy,
    rpy_to_quaternion_xyzw,
    with_yaw_reference,
)


def make_message(stamp_s=10.0):
    hessian = np.zeros((12, 12), dtype=float)
    hessian[:6, :6] = np.diag([4.0, 3.0, 2.0, 1.0, 0.8, 0.6])
    gradient = np.zeros(12, dtype=float)
    gradient[:6] = [0.2, -0.1, 0.05, 0.01, 0.02, -0.03]
    return SimpleNamespace(
        correspondences_valid=True,
        approximate=False,
        jacobian_columns=12,
        matched_points=3,
        candidate_points=5,
        jacobian_state_order=list(EXPECTED_POSE_STATE_ORDER) + [
            f"extrinsic_{index}" for index in range(6)
        ],
        state_hessian=hessian.reshape(-1).tolist(),
        state_gradient=gradient.tolist(),
        linearization_position=[1.0, 2.0, 3.0],
        linearization_quaternion=[0.0, 0.0, 0.0, 1.0],
        residuals=[0.1, -0.2, 0.05],
        measurement_variance=0.001,
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=int(stamp_s),
                nanosec=int(round((stamp_s - int(stamp_s)) * 1.0e9)),
            )
        ),
        scan_sequence=7,
        source="fast_lio_ikfom",
        map_frame="camera_init",
        state_frame="body",
    )


class NativeLidarConversionTest(unittest.TestCase):
    def test_identity_orientation_preserves_pose_normal_equation(self):
        msg = make_message()
        factor = native_factor_from_message(msg)

        expected_hessian = np.asarray(msg.state_hessian).reshape(12, 12)[:6, :6]
        expected_gradient = np.asarray(msg.state_gradient)[:6]
        np.testing.assert_allclose(factor.pose_hessian, expected_hessian)
        np.testing.assert_allclose(factor.pose_gradient, expected_gradient)
        np.testing.assert_allclose(factor.linearization_pose, [1, 2, 3, 0, 0, 0])
        self.assertAlmostEqual(factor.residual_squared, 0.0525)

    def test_rpy_quaternion_round_trip_and_yaw_unwrap(self):
        original = np.asarray([0.2, -0.3, 3.10])
        recovered = quaternion_xyzw_to_rpy(rpy_to_quaternion_xyzw(original))
        np.testing.assert_allclose(recovered, original, atol=1.0e-12)
        msg = make_message()
        msg.linearization_quaternion = list(rpy_to_quaternion_xyzw([0.1, -0.2, -3.12]))
        factor = with_yaw_reference(native_factor_from_message(msg), 3.10)
        self.assertGreater(factor.linearization_pose[5], 3.10)
        self.assertLess(factor.linearization_pose[5] - 3.10, 0.1)

    def test_nonzero_rpy_transforms_right_tangent_normal_equation(self):
        rpy = np.asarray([0.3, -0.4, 0.7])
        msg = make_message()
        msg.linearization_quaternion = list(rpy_to_quaternion_xyzw(rpy))
        factor = native_factor_from_message(msg)
        right_hessian = np.asarray(msg.state_hessian).reshape(12, 12)[:6, :6]
        right_gradient = np.asarray(msg.state_gradient)[:6]
        transform = np.eye(6)
        transform[3:, 3:] = right_perturbation_jacobian_rpy(rpy)
        np.testing.assert_allclose(
            factor.pose_hessian, transform.T @ right_hessian @ transform
        )
        np.testing.assert_allclose(
            factor.pose_gradient, transform.T @ right_gradient
        )

    def test_buffer_pairs_nearest_stamp_and_drops_stale_packets(self):
        first = native_factor_from_message(make_message(10.000))
        second = native_factor_from_message(make_message(10.100))
        buffer = NativeFactorBuffer(max_size=4)
        buffer.push(second)
        buffer.push(first)

        matched = buffer.pop_nearest(10.002, 0.005)

        self.assertIsNotNone(matched)
        self.assertAlmostEqual(matched.stamp_s, 10.0)
        self.assertIsNone(buffer.pop_nearest(10.200, 0.005))
        self.assertEqual(len(buffer), 0)

    def test_rejects_incompatible_pose_state_order(self):
        msg = make_message()
        msg.jacobian_state_order[0] = "rotation_first"
        with self.assertRaisesRegex(ValueError, "state order"):
            native_factor_from_message(msg)


if __name__ == "__main__":
    unittest.main()
