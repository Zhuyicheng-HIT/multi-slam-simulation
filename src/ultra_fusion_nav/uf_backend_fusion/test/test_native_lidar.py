import math
import unittest
from types import SimpleNamespace

import numpy as np

from uf_backend_fusion.native_lidar import (
    EXPECTED_POSE_STATE_ORDER,
    NativeFactorBuffer,
    lidar_pose_observability,
    lidar_reliability_layers,
    lidar_vertical_observability,
    native_factor_from_message,
    point_plane_residual_jacobian,
    quaternion_xyzw_to_rpy,
    right_perturbation_jacobian_rpy,
    rpy_to_quaternion_xyzw,
    rpy_to_rotation_matrix,
    transform_native_factor_map,
    validate_native_frame_contract,
    with_yaw_reference,
)
from uf_backend_fusion.manifold import state_plus


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
        reset_counter=3,
        source="fast_lio_ikfom",
        map_frame="camera_init",
        state_frame="body",
        sensor_frame="lidar",
        lidar_points_xyz=[],
        plane_normals_xyz=[],
        plane_points_xyz=[],
        lidar_to_body_translation=[0.0, 0.0, 0.0],
        lidar_to_body_quaternion=[0.0, 0.0, 0.0, 1.0],
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
        self.assertEqual(factor.stamp_ns, 10_000_000_000)
        self.assertEqual(factor.reset_counter, 3)
        self.assertTrue(factor.correspondences_valid)

    def test_reset_counter_defaults_for_old_packets_and_rejects_invalid_range(self):
        message = make_message()
        del message.reset_counter
        self.assertEqual(native_factor_from_message(message).reset_counter, 0)

        message.reset_counter = -1
        with self.assertRaisesRegex(ValueError, "uint32"):
            native_factor_from_message(message)

    def test_directional_observability_preserves_strong_subspace(self):
        message = make_message()
        hessian = np.zeros((12, 12), dtype=float)
        hessian[:6, :6] = np.diag([4.0, 3.0, 2.0, 1.0e-8, 0.8, 1.0e-9])
        message.state_hessian = hessian.ravel().tolist()
        factor = native_factor_from_message(message)

        observability = lidar_pose_observability(factor)

        self.assertEqual(observability.effective_rank, 4)
        self.assertEqual(observability.translation_rank, 3)
        self.assertEqual(observability.rotation_rank, 1)
        self.assertTrue(math.isinf(observability.condition_number))
        self.assertEqual(len(observability.weakest_direction), 6)

    def test_vertical_observability_exposes_plane_support_and_pose_coupling(self):
        message = make_message()
        hessian = np.zeros((12, 12), dtype=float)
        hessian[2, 2] = 10.0
        hessian[4, 4] = 10.0
        hessian[2, 4] = hessian[4, 2] = 9.0
        hessian[0, 0] = 4.0
        hessian[1, 1] = 6.0
        hessian[3, 3] = 2.0
        hessian[5, 5] = 3.0
        message.state_hessian = hessian.ravel().tolist()
        message.plane_normals_xyz = [
            0.0, 0.0, 1.0,
            0.0, 0.6, 0.8,
            1.0, 0.0, 0.0,
        ]
        message.lidar_points_xyz = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        message.plane_points_xyz = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        factor = native_factor_from_message(message)

        vertical = lidar_vertical_observability(factor)

        self.assertAlmostEqual(vertical.raw_information, 10_000.0)
        self.assertAlmostEqual(vertical.profile_information, 1_900.0)
        self.assertAlmostEqual(vertical.coupling_retention_ratio, 0.19)
        self.assertAlmostEqual(vertical.normal_z_energy_fraction, 0.5)
        self.assertAlmostEqual(vertical.horizontal_plane_fraction, 2.0 / 3.0)

        layers = lidar_reliability_layers(
            factor,
            vertical,
            position_innovation_m=0.1,
            yaw_innovation_rad=0.05,
            position_innovation_scale_m=1.0,
            yaw_innovation_scale_rad=0.5,
        )
        self.assertEqual(layers.health_degradation, 0.0)
        self.assertAlmostEqual(layers.consistency_degradation, 0.1)
        self.assertTrue(all(
            0.0 <= value <= 1.0
            for value in layers.isotropic_information_support_xyz
        ))
        self.assertTrue(all(
            combined >= observable
            for combined, observable in zip(
                layers.combined_degradation_xyz,
                layers.observability_degradation_xyz,
            )
        ))
        np.testing.assert_allclose(
            vertical.axis_raw_information, [4000.0, 6000.0, 10000.0]
        )
        np.testing.assert_allclose(
            vertical.axis_profile_information, [4000.0, 6000.0, 1900.0]
        )
        np.testing.assert_allclose(
            vertical.axis_coupling_retention_ratio, [1.0, 1.0, 0.19]
        )
        np.testing.assert_allclose(
            vertical.axis_relative_support,
            [2.0 / 3.0, 1.0, 1900.0 / 6000.0],
        )
        np.testing.assert_allclose(
            np.asarray(vertical.translation_profile_information).reshape(3, 3),
            np.diag([4000.0, 6000.0, 1900.0]),
        )
        np.testing.assert_allclose(
            vertical.translation_normalized_eigenvalues,
            [1900.0 / 6000.0, 2.0 / 3.0, 1.0],
        )
        np.testing.assert_allclose(
            vertical.weakest_translation_direction, [0.0, 0.0, 1.0]
        )

    def test_facade_only_geometry_reports_weak_map_z_without_disabling_xy(self):
        message = make_message()
        hessian = np.zeros((12, 12), dtype=float)
        hessian[:6, :6] = np.diag([100.0, 80.0, 0.10, 5.0, 5.0, 5.0])
        message.state_hessian = hessian.ravel().tolist()
        message.plane_normals_xyz = [
            1.0, 0.0, 0.0,
            0.8, 0.6, 0.0,
            0.0, 1.0, 0.0,
        ]
        message.lidar_points_xyz = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            1.0, 1.0, 0.0,
        ]
        message.plane_points_xyz = list(message.lidar_points_xyz)

        factor = native_factor_from_message(message)
        directional = lidar_vertical_observability(factor)

        np.testing.assert_allclose(
            directional.axis_profile_information,
            [100000.0, 80000.0, 100.0],
        )
        np.testing.assert_allclose(
            directional.axis_relative_support, [1.0, 0.8, 0.001]
        )
        np.testing.assert_allclose(
            directional.translation_normalized_eigenvalues,
            [0.001, 0.8, 1.0],
        )
        np.testing.assert_allclose(
            directional.weakest_translation_direction, [0.0, 0.0, 1.0]
        )
        self.assertEqual(directional.horizontal_plane_fraction, 0.0)
        layers = lidar_reliability_layers(factor, directional)
        self.assertAlmostEqual(layers.observability_degradation_xyz[2], 0.9)
        self.assertLess(
            layers.observability_degradation_xyz[0],
            layers.observability_degradation_xyz[2],
        )

    def test_trigger_only_frame_is_valid_without_a_lidar_factor(self):
        msg = make_message(10.125)
        msg.correspondences_valid = False
        msg.matched_points = 0
        msg.residuals = []
        factor = native_factor_from_message(msg)

        self.assertFalse(factor.correspondences_valid)
        self.assertEqual(factor.stamp_ns, 10_125_000_000)
        self.assertEqual(factor.matched_points, 0)
        np.testing.assert_array_equal(factor.pose_hessian, np.zeros((6, 6)))

    def test_frame_contract_rejects_silent_coordinate_relabeling(self):
        factor = native_factor_from_message(make_message())
        validate_native_frame_contract(factor, "camera_init", "body")
        with self.assertRaisesRegex(ValueError, "map frame"):
            validate_native_frame_contract(factor, "map", "body")
        with self.assertRaisesRegex(ValueError, "state frame"):
            validate_native_frame_contract(factor, "camera_init", "base_link")

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
        np.testing.assert_allclose(factor.pose_hessian_right, right_hessian)
        np.testing.assert_allclose(factor.pose_gradient_right, right_gradient)

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

    def test_raw_correspondences_relinearize_at_backend_pose(self):
        msg = make_message()
        msg.lidar_points_xyz = [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]
        msg.plane_normals_xyz = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.plane_points_xyz = [2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 1.0, 2.0, 4.0]
        msg.residuals = [0.0, 1.0, 2.0]
        factor = native_factor_from_message(msg)

        residual, jacobian = point_plane_residual_jacobian(
            factor, [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]
        )

        np.testing.assert_allclose(residual, [0.0, 1.0, 2.0])
        np.testing.assert_allclose(jacobian[:, :3], np.eye(3))
        self.assertEqual(jacobian.shape, (3, 6))

    def test_body_tilt_and_fixed_lidar_pitch_are_both_applied(self):
        msg = make_message()
        lidar_points = np.asarray([
            [2.0, 0.3, -0.2],
            [0.4, 1.7, 0.5],
            [-0.6, 0.2, 2.2],
        ])
        lidar_pitch = rpy_to_rotation_matrix([0.0, math.radians(10.0), 0.0])
        body_pose = np.asarray([1.2, -0.7, 2.8, 0.18, -0.14, 0.35])
        body_rotation = rpy_to_rotation_matrix(body_pose[3:])
        world_points = (lidar_points @ lidar_pitch.T) @ body_rotation.T + body_pose[:3]
        normals = np.eye(3)
        msg.lidar_points_xyz = lidar_points.reshape(-1).tolist()
        msg.plane_normals_xyz = normals.reshape(-1).tolist()
        msg.plane_points_xyz = world_points.reshape(-1).tolist()
        msg.residuals = [0.0, 0.0, 0.0]
        msg.lidar_to_body_quaternion = list(
            rpy_to_quaternion_xyzw([0.0, math.radians(10.0), 0.0])
        )
        factor = native_factor_from_message(msg)

        residual, jacobian = point_plane_residual_jacobian(factor, body_pose)
        np.testing.assert_allclose(residual, np.zeros(3), atol=1.0e-12)
        self.assertTrue(np.all(np.isfinite(jacobian)))

        level_body_pose = body_pose.copy()
        level_body_pose[3:5] = 0.0
        uncompensated_tilt, _ = point_plane_residual_jacobian(
            factor, level_body_pose
        )
        self.assertGreater(float(np.linalg.norm(uncompensated_tilt)), 0.1)

        msg.lidar_to_body_quaternion = [0.0, 0.0, 0.0, 1.0]
        missing_mount_pitch = native_factor_from_message(msg)
        uncompensated_mount, _ = point_plane_residual_jacobian(
            missing_mount_pitch, body_pose
        )
        self.assertGreater(float(np.linalg.norm(uncompensated_mount)), 0.1)

    def test_point_plane_jacobian_matches_right_local_finite_difference(self):
        msg = make_message()
        msg.lidar_points_xyz = [
            2.0, 0.3, -0.2,
            0.4, 1.7, 0.5,
            -0.6, 0.2, 2.2,
        ]
        msg.plane_normals_xyz = [
            0.8, -0.3, 0.5,
            -0.2, 0.9, 0.4,
            0.3, 0.1, -0.95,
        ]
        msg.plane_points_xyz = [
            1.2, -0.4, 0.7,
            -0.3, 1.1, 0.2,
            0.6, -0.8, 1.5,
        ]
        msg.lidar_to_body_translation = [0.15, -0.04, 0.09]
        msg.lidar_to_body_quaternion = list(
            rpy_to_quaternion_xyzw([0.08, -0.12, 0.05])
        )
        factor = native_factor_from_message(msg)
        pose = np.asarray([0.4, -0.2, 0.3, 0.15, -0.1, 0.25])
        residual, analytic = point_plane_residual_jacobian(factor, pose)
        state = np.zeros(15)
        state[:6] = pose
        numerical = np.zeros_like(analytic)
        epsilon = 1.0e-7
        for column in range(6):
            increment = np.zeros(15)
            increment[column] = epsilon
            plus = state_plus(state, increment)
            minus = state_plus(state, -increment)
            plus_residual, _ = point_plane_residual_jacobian(
                factor, plus[:6]
            )
            minus_residual, _ = point_plane_residual_jacobian(
                factor, minus[:6]
            )
            numerical[:, column] = (
                plus_residual - minus_residual
            ) / (2.0 * epsilon)

        self.assertTrue(np.all(np.isfinite(residual)))
        np.testing.assert_allclose(
            analytic, numerical, atol=2.0e-8, rtol=2.0e-8
        )

    def test_map_alignment_preserves_point_plane_residuals(self):
        msg = make_message()
        msg.lidar_points_xyz = [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]
        msg.plane_normals_xyz = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.plane_points_xyz = [2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 1.0, 2.0, 4.0]
        msg.residuals = [0.0, 1.0, 2.0]
        factor = native_factor_from_message(msg)
        alignment = np.eye(4)
        alignment[:3, :3] = rpy_to_rotation_matrix([0.1, -0.2, 0.4])
        alignment[:3, 3] = [8.0, -3.0, 1.5]

        transformed = transform_native_factor_map(factor, alignment)
        residual_before, _ = point_plane_residual_jacobian(
            factor, factor.linearization_pose
        )
        residual_after, _ = point_plane_residual_jacobian(
            transformed, transformed.linearization_pose
        )

        np.testing.assert_allclose(residual_after, residual_before, atol=1.0e-10)
        np.testing.assert_allclose(
            transformed.linearization_pose[:3],
            alignment[:3, :3] @ factor.linearization_pose[:3] + alignment[:3, 3],
        )

    def test_rejects_incompatible_pose_state_order(self):
        msg = make_message()
        msg.jacobian_state_order[0] = "rotation_first"
        with self.assertRaisesRegex(ValueError, "state order"):
            native_factor_from_message(msg)


if __name__ == "__main__":
    unittest.main()
