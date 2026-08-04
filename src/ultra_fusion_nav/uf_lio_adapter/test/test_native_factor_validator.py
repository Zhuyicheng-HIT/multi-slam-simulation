import unittest
from types import SimpleNamespace

import numpy as np

from uf_lio_adapter.native_factor_validator import analyze_factor


def make_factor():
    points = np.asarray([[1.0, 0.5, -0.2], [-0.4, 1.2, 0.8]])
    normals = np.asarray([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    position = np.asarray([1.0, 2.0, 3.0])
    sensor_translation = np.asarray([0.1, 0.2, 0.3])
    points_global = points + sensor_translation + position
    residuals = np.asarray([0.12, -0.07])
    plane_points = points_global - residuals[:, None] * normals
    points_body = points + sensor_translation
    jacobian = np.zeros((2, 12), dtype=np.float64)
    jacobian[:, :3] = normals
    jacobian[:, 3:6] = np.cross(points_body, normals)
    hessian = jacobian.T @ jacobian
    gradient = jacobian.T @ residuals
    return SimpleNamespace(
        matched_points=2,
        candidate_points=10,
        jacobian_columns=12,
        lidar_points_xyz=points.reshape(-1).tolist(),
        plane_normals_xyz=normals.reshape(-1).tolist(),
        plane_points_xyz=plane_points.reshape(-1).tolist(),
        residuals=residuals.tolist(),
        jacobian=jacobian.reshape(-1).tolist(),
        state_hessian=hessian.reshape(-1).tolist(),
        state_gradient=gradient.tolist(),
        pose_covariance=np.eye(6).reshape(-1).tolist(),
        linearization_position=position.tolist(),
        linearization_quaternion=[0.0, 0.0, 0.0, 1.0],
        lidar_to_body_translation=sensor_translation.tolist(),
        lidar_to_body_quaternion=[0.0, 0.0, 0.0, 1.0],
        measurement_variance=0.001,
        jacobian_state_order=[str(i) for i in range(12)],
        correspondences_valid=True,
        approximate=False,
        source="fast_lio_ikfom",
        map_frame="camera_init",
        sensor_frame="mid360_link",
        state_frame="body",
    )


class NativeFactorValidatorTest(unittest.TestCase):
    def test_recomputes_geometry_and_normal_equation(self):
        result = analyze_factor(make_factor())
        self.assertTrue(result["valid"], result["errors"])
        self.assertLess(result["geometry_residual_abs_max"], 1.0e-12)
        self.assertLess(result["hessian_relative_error"], 1.0e-12)
        self.assertLess(result["gradient_relative_error"], 1.0e-12)
        self.assertLess(result["pose_jacobian_relative_error"], 1.0e-12)
        self.assertEqual(len(result["pose_hessian_eigenvalues"]), 6)
        self.assertGreaterEqual(result["pose_hessian_min_eigenvalue"], -1.0e-12)
        self.assertGreaterEqual(result["pose_hessian_min_eigenvalue_per_match"], 0.0)
        self.assertGreaterEqual(result["pose_hessian_condition_number"], 1.0)
        self.assertAlmostEqual(result["residual_mean_m"], 0.095)
        self.assertAlmostEqual(result["residual_median_m"], 0.095)
        self.assertEqual(len(result["normal_covariance_eigenvalues"]), 3)
        self.assertGreaterEqual(result["axial_penalty"], 0.0)
        self.assertLessEqual(result["axial_penalty"], 1.0)
        self.assertGreater(result["spatial_coverage"], 0.0)
        self.assertTrue(result["debug_jacobian_available"])
        self.assertEqual(result["jacobian_model"], "fixed_extrinsic")

    def test_optional_debug_jacobian_is_reconstructed_from_geometry(self):
        msg = make_factor()
        msg.jacobian = []

        result = analyze_factor(msg)

        self.assertTrue(result["valid"], result["errors"])
        self.assertFalse(result["debug_jacobian_available"])
        self.assertEqual(result["jacobian_model"], "fixed_extrinsic")
        self.assertLess(result["hessian_relative_error"], 1.0e-12)
        self.assertLess(result["gradient_relative_error"], 1.0e-12)

    def test_reconstructs_online_extrinsic_jacobian_branch(self):
        msg = make_factor()
        points = np.asarray(msg.lidar_points_xyz).reshape(-1, 3)
        normals = np.asarray(msg.plane_normals_xyz).reshape(-1, 3)
        translation = np.asarray(msg.lidar_to_body_translation)
        points_body = points + translation
        jacobian = np.concatenate((
            normals,
            np.cross(points_body, normals),
            np.cross(points, normals),
            normals,
        ), axis=1)
        residuals = np.asarray(msg.residuals)
        msg.jacobian = []
        msg.state_hessian = (jacobian.T @ jacobian).reshape(-1).tolist()
        msg.state_gradient = (jacobian.T @ residuals).tolist()

        result = analyze_factor(msg)

        self.assertTrue(result["valid"], result["errors"])
        self.assertEqual(result["jacobian_model"], "estimated_extrinsic")

    def test_rejects_nonempty_partial_debug_jacobian(self):
        msg = make_factor()
        msg.jacobian = msg.jacobian[:-1]

        result = analyze_factor(msg)

        self.assertFalse(result["valid"])
        self.assertIn("jacobian length mismatch", result["errors"])

    def test_rejects_corrupted_hessian(self):
        msg = make_factor()
        msg.state_hessian[0] += 0.1
        result = analyze_factor(msg)
        self.assertFalse(result["valid"])
        self.assertIn("state_hessian != J^T J", result["errors"])

    def test_rejects_jacobian_with_wrong_point_to_plane_geometry(self):
        msg = make_factor()
        jacobian = np.asarray(msg.jacobian).reshape(2, 12)
        jacobian[0, 3] += 0.1
        msg.jacobian = jacobian.reshape(-1).tolist()
        msg.state_hessian = (jacobian.T @ jacobian).reshape(-1).tolist()
        msg.state_gradient = (jacobian.T @ np.asarray(msg.residuals)).tolist()

        result = analyze_factor(msg)

        self.assertFalse(result["valid"])
        self.assertIn(
            "pose Jacobian does not match point-to-plane geometry", result["errors"]
        )


if __name__ == "__main__":
    unittest.main()
