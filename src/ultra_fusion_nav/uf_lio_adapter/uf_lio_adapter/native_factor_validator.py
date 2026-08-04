import json
import os
from pathlib import Path

# This diagnostic performs many small matrix products. Multi-threaded BLAS is
# slower here and can disturb the simulation that it is meant to observe.
for _thread_env in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[_thread_env] = "1"

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from fast_lio.msg import NativeLidarFactor
except ImportError as exc:  # pragma: no cover - exercised without the FAST-LIO overlay
    NativeLidarFactor = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


def _relative_error(actual, expected):
    return float(np.linalg.norm(actual - expected) / max(1.0, np.linalg.norm(expected)))


def analyze_factor(msg, *, geometry_tolerance=1.0e-7, normal_equation_tolerance=1.0e-7,
                   jacobian_tolerance=1.0e-7):
    """Validate one native factor and recompute its exported normal equation."""
    matched = int(msg.matched_points)
    result = {
        "valid": False,
        "errors": [],
        "matched_points": matched,
        "candidate_points": int(msg.candidate_points),
    }

    if not bool(msg.correspondences_valid):
        result["errors"].append("correspondences_valid is false")
    if bool(msg.approximate):
        result["errors"].append("factor is marked approximate")
    if int(msg.jacobian_columns) != 12:
        result["errors"].append("jacobian_columns != 12")
    if result["candidate_points"] < matched:
        result["errors"].append("matched_points exceeds candidate_points")
    if not str(msg.source) or not str(msg.map_frame) or not str(msg.sensor_frame):
        result["errors"].append("source/frame metadata is incomplete")
    if not str(msg.state_frame):
        result["errors"].append("state_frame is empty")
    if len(msg.jacobian_state_order) != 12:
        result["errors"].append("jacobian_state_order length != 12")

    expected_lengths = {
        "lidar_points_xyz": matched * 3,
        "plane_normals_xyz": matched * 3,
        "plane_points_xyz": matched * 3,
        "residuals": matched,
        "state_hessian": 144,
        "state_gradient": 12,
        "pose_covariance": 36,
    }
    for field, expected in expected_lengths.items():
        if len(getattr(msg, field)) != expected:
            result["errors"].append(f"{field} length mismatch")
    jacobian_length = len(msg.jacobian)
    expected_jacobian_length = matched * 12
    if jacobian_length not in (0, expected_jacobian_length):
        result["errors"].append("jacobian length mismatch")
    if result["errors"]:
        return result

    points = np.asarray(msg.lidar_points_xyz, dtype=np.float64).reshape(matched, 3)
    normals = np.asarray(msg.plane_normals_xyz, dtype=np.float64).reshape(matched, 3)
    plane_points = np.asarray(msg.plane_points_xyz, dtype=np.float64).reshape(matched, 3)
    residuals = np.asarray(msg.residuals, dtype=np.float64)
    debug_jacobian = (
        np.asarray(msg.jacobian, dtype=np.float64).reshape(matched, 12)
        if jacobian_length else None
    )
    hessian = np.asarray(msg.state_hessian, dtype=np.float64).reshape(12, 12)
    gradient = np.asarray(msg.state_gradient, dtype=np.float64)
    covariance = np.asarray(msg.pose_covariance, dtype=np.float64).reshape(6, 6)
    position = np.asarray(msg.linearization_position, dtype=np.float64)
    quaternion = np.asarray(msg.linearization_quaternion, dtype=np.float64)
    sensor_translation = np.asarray(msg.lidar_to_body_translation, dtype=np.float64)
    sensor_quaternion = np.asarray(msg.lidar_to_body_quaternion, dtype=np.float64)
    arrays = (points, normals, plane_points, residuals, hessian, gradient,
              covariance, position, quaternion, sensor_translation,
              sensor_quaternion)
    if debug_jacobian is not None:
        arrays = (*arrays, debug_jacobian)
    if not all(np.all(np.isfinite(array)) for array in arrays):
        result["errors"].append("factor contains non-finite values")
        return result

    normal_norms = np.linalg.norm(normals, axis=1)
    quaternion_norm = float(np.linalg.norm(quaternion))
    sensor_quaternion_norm = float(np.linalg.norm(sensor_quaternion))
    if np.any(normal_norms <= 1.0e-12):
        result["errors"].append("zero plane normal")
    if abs(quaternion_norm - 1.0) > 1.0e-5:
        result["errors"].append("linearization quaternion is not unit length")
    if abs(sensor_quaternion_norm - 1.0) > 1.0e-5:
        result["errors"].append("LiDAR-to-body quaternion is not unit length")

    # The exported plane point is constructed so n dot (p_global - plane_point)
    # equals the signed point-to-plane residual. Reconstructing p_global needs
    # the body pose and the LiDAR-to-body transform exported in the packet.
    def quaternion_to_matrix(q):
        x, y, z, w = q / np.linalg.norm(q)
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    body_rotation = quaternion_to_matrix(quaternion)
    sensor_rotation = quaternion_to_matrix(sensor_quaternion)
    points_global = (body_rotation @ (sensor_rotation @ points.T + sensor_translation[:, None]))
    points_global = points_global.T + position
    geometry_residuals = np.einsum("ij,ij->i", normals, points_global - plane_points)

    # Ultra-Fusion Eq. (18), adapted to FAST-LIO's right SO(3) perturbation:
    # J_i = [n_W^T, (p_body x R_WB^T n_W)^T].
    points_body = (sensor_rotation @ points.T + sensor_translation[:, None]).T
    normals_body = (body_rotation.T @ normals.T).T
    expected_pose_jacobian = np.concatenate(
        (normals, np.cross(points_body, normals_body)), axis=1
    )
    # FAST-LIO either keeps its LiDAR-body extrinsic fixed (last six columns
    # zero) or estimates it online. Reconstruct both exact source branches and
    # choose the one matching the exported normal equation. This keeps the
    # per-point debug Jacobian optional without weakening J^T J / J^T r checks.
    normals_sensor = (sensor_rotation.T @ normals_body.T).T
    estimated_extrinsic_jacobian = np.concatenate((
        expected_pose_jacobian,
        np.cross(points, normals_sensor),
        normals_body,
    ), axis=1)
    fixed_extrinsic_jacobian = np.concatenate((
        expected_pose_jacobian,
        np.zeros((matched, 6), dtype=np.float64),
    ), axis=1)
    candidates = {
        "fixed_extrinsic": fixed_extrinsic_jacobian,
        "estimated_extrinsic": estimated_extrinsic_jacobian,
    }
    candidate_errors = {
        name: (
            _relative_error(candidate.T @ candidate, hessian)
            + _relative_error(candidate.T @ residuals, gradient)
        )
        for name, candidate in candidates.items()
    }
    jacobian_model = min(candidate_errors, key=candidate_errors.get)
    reconstructed_jacobian = candidates[jacobian_model]
    pose_jacobian = (
        debug_jacobian[:, :6]
        if debug_jacobian is not None else reconstructed_jacobian[:, :6]
    )
    pose_jacobian_relative_error = _relative_error(
        pose_jacobian, expected_pose_jacobian
    )
    pose_jacobian_abs_max = float(
        np.max(np.abs(pose_jacobian - expected_pose_jacobian))
    ) if matched else 0.0

    debug_jacobian_relative_error = (
        _relative_error(debug_jacobian, reconstructed_jacobian)
        if debug_jacobian is not None else 0.0
    )
    expected_hessian = reconstructed_jacobian.T @ reconstructed_jacobian
    expected_gradient = reconstructed_jacobian.T @ residuals
    hessian_relative_error = _relative_error(hessian, expected_hessian)
    gradient_relative_error = _relative_error(gradient, expected_gradient)
    hessian_symmetry_error = _relative_error(hessian, hessian.T)
    covariance_symmetry_error = _relative_error(covariance, covariance.T)
    hessian_eigenvalues = np.linalg.eigvalsh((hessian + hessian.T) * 0.5)
    pose_hessian = hessian[:6, :6]
    pose_hessian_eigenvalues = np.linalg.eigvalsh((pose_hessian + pose_hessian.T) * 0.5)
    pose_min_eigenvalue = max(0.0, float(pose_hessian_eigenvalues[0]))
    pose_max_eigenvalue = float(pose_hessian_eigenvalues[-1])
    pose_condition_number = pose_max_eigenvalue / max(1.0e-12, pose_min_eigenvalue)
    covariance_diagonal = np.diag(covariance)
    geometry_error = float(np.max(np.abs(geometry_residuals - residuals))) if matched else 0.0
    residual_magnitudes = np.abs(residuals)
    if matched:
        normal_information = normals.T @ normals / matched
        normal_eigenvalues = np.maximum(
            np.linalg.eigvalsh((normal_information + normal_information.T) * 0.5),
            0.0,
        )
        centered_points = points - np.median(points, axis=0)
        occupied_octants = {
            (point[0] >= 0.0, point[1] >= 0.0, point[2] >= 0.0)
            for point in centered_points
        }
        spatial_coverage = len(occupied_octants) / 8.0
    else:
        normal_eigenvalues = np.zeros(3, dtype=np.float64)
        spatial_coverage = 0.0
    pose_support_floor = max(1.0e-12, 0.05 * pose_max_eigenvalue)
    axial_penalty = float(
        np.mean(1.0 - np.minimum(1.0, pose_hessian_eigenvalues / pose_support_floor))
    )

    result.update({
        "residual_rms": float(np.sqrt(np.mean(residuals * residuals))) if matched else 0.0,
        "residual_abs_max": float(np.max(np.abs(residuals))) if matched else 0.0,
        "residual_mean_m": float(np.mean(residual_magnitudes)) if matched else 0.0,
        "residual_median_m": float(np.median(residual_magnitudes)) if matched else 0.0,
        "residual_p95_m": float(np.percentile(residual_magnitudes, 95)) if matched else 0.0,
        "geometry_residual_abs_max": geometry_error,
        "pose_jacobian_relative_error": pose_jacobian_relative_error,
        "pose_jacobian_abs_max": pose_jacobian_abs_max,
        "jacobian_frobenius": float(np.linalg.norm(reconstructed_jacobian)),
        "debug_jacobian_available": debug_jacobian is not None,
        "debug_jacobian_relative_error": debug_jacobian_relative_error,
        "jacobian_model": jacobian_model,
        "hessian_trace": float(np.trace(hessian)),
        "hessian_min_eigenvalue": float(hessian_eigenvalues[0]),
        "pose_hessian_min_eigenvalue": pose_min_eigenvalue,
        "pose_hessian_min_eigenvalue_per_match": pose_min_eigenvalue / max(1, matched),
        "pose_hessian_max_eigenvalue": pose_max_eigenvalue,
        "pose_hessian_condition_number": pose_condition_number,
        "pose_hessian_eigenvalues": [float(value) for value in pose_hessian_eigenvalues],
        "normal_covariance_eigenvalues": [float(value) for value in normal_eigenvalues],
        "axial_penalty": axial_penalty,
        "spatial_coverage": spatial_coverage,
        "hessian_relative_error": hessian_relative_error,
        "hessian_symmetry_error": hessian_symmetry_error,
        "gradient_l2": float(np.linalg.norm(gradient)),
        "gradient_relative_error": gradient_relative_error,
        "covariance_trace": float(np.trace(covariance)),
        "covariance_diagonal_min": float(np.min(covariance_diagonal)),
        "covariance_symmetry_error": covariance_symmetry_error,
        "normal_norm_min": float(np.min(normal_norms)) if matched else 0.0,
        "normal_norm_max": float(np.max(normal_norms)) if matched else 0.0,
        "measurement_variance": float(msg.measurement_variance),
    })
    if geometry_error > geometry_tolerance:
        result["errors"].append("point-to-plane residual geometry mismatch")
    if pose_jacobian_relative_error > jacobian_tolerance:
        result["errors"].append("pose Jacobian does not match point-to-plane geometry")
    if debug_jacobian_relative_error > jacobian_tolerance:
        result["errors"].append("debug Jacobian does not match reconstructed geometry")
    if hessian_relative_error > normal_equation_tolerance:
        result["errors"].append("state_hessian != J^T J")
    if gradient_relative_error > normal_equation_tolerance:
        result["errors"].append("state_gradient != J^T r")
    if hessian_symmetry_error > normal_equation_tolerance:
        result["errors"].append("state_hessian is not symmetric")
    if float(hessian_eigenvalues[0]) < -normal_equation_tolerance:
        result["errors"].append("state_hessian is not positive semidefinite")
    if covariance_symmetry_error > normal_equation_tolerance:
        result["errors"].append("pose_covariance is not symmetric")
    if float(np.min(covariance_diagonal)) < -normal_equation_tolerance:
        result["errors"].append("pose_covariance has negative diagonal")
    result["valid"] = not result["errors"]
    return result


class NativeFactorValidator(Node):
    """Validate native FAST-LIO point-to-plane packets and their dynamics."""

    def __init__(self):
        super().__init__("native_factor_validator")
        if NativeLidarFactor is None:
            raise RuntimeError(
                "fast_lio/msg/NativeLidarFactor is unavailable; source the FAST-LIO overlay"
            ) from _IMPORT_ERROR
        self.declare_parameter("topic", "/fast_lio/native_lidar_factor")
        self.declare_parameter("summary_period_s", 1.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("summary_path", "")
        self.declare_parameter("min_dynamic_packets", 2)
        self.topic = str(self.get_parameter("topic").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.summary_path = str(self.get_parameter("summary_path").value)
        self.min_dynamic_packets = int(self.get_parameter("min_dynamic_packets").value)
        self.received = 0
        self.valid = 0
        self.invalid = 0
        self.dynamic_packets = 0
        self.previous_metrics = None
        self.first_sequence = None
        self.last_sequence = None
        self.metric_min = {}
        self.metric_max = {}
        self.last_summary_time = self.get_clock().now().nanoseconds * 1.0e-9
        self.output = None
        if self.output_path:
            path = Path(self.output_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.output = path.open("w", encoding="utf-8")
        self.create_subscription(
            NativeLidarFactor, self.topic, self._factor, qos_profile_sensor_data
        )
        self.summary_period_s = float(self.get_parameter("summary_period_s").value)

    def _factor(self, msg):
        self.received += 1
        self.last_sequence = int(msg.scan_sequence)
        if self.first_sequence is None:
            self.first_sequence = self.last_sequence
        result = analyze_factor(msg)
        if result["valid"]:
            self.valid += 1
            dynamic_keys = (
                "matched_points", "residual_rms", "jacobian_frobenius",
                "hessian_trace", "pose_hessian_min_eigenvalue",
                "pose_hessian_min_eigenvalue_per_match",
                "pose_hessian_condition_number", "gradient_l2", "covariance_trace",
            )
            if self.previous_metrics is not None and any(
                abs(result[key] - self.previous_metrics[key]) > 1.0e-12
                for key in dynamic_keys
            ):
                self.dynamic_packets += 1
            self.previous_metrics = {key: result[key] for key in dynamic_keys}
            for key in dynamic_keys:
                self.metric_min[key] = min(result[key], self.metric_min.get(key, result[key]))
                self.metric_max[key] = max(result[key], self.metric_max.get(key, result[key]))
        else:
            self.invalid += 1
            self.get_logger().error(
                f"invalid native factor sequence={self.last_sequence}: "
                + "; ".join(result["errors"])
            )

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        result.update({"stamp_ns": stamp_ns, "scan_sequence": self.last_sequence})
        if self.output is not None:
            self.output.write(json.dumps(result, sort_keys=True) + "\n")
            self.output.flush()
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now < self.last_summary_time:
            self.last_summary_time = now
        if now - self.last_summary_time >= self.summary_period_s:
            self.last_summary_time = now
            self.get_logger().info(
                f"native factors received={self.received} valid={self.valid} "
                f"invalid={self.invalid} dynamic_packets={self.dynamic_packets} "
                f"latest_matches={int(msg.matched_points)} "
                f"residual_rms={result.get('residual_rms', 0.0):.6g} "
                f"hessian_trace={result.get('hessian_trace', 0.0):.6g}"
            )

    def _summary(self):
        return {
            "received": self.received,
            "valid": self.valid,
            "invalid": self.invalid,
            "valid_ratio": float(self.valid / self.received) if self.received else 0.0,
            "dynamic_packets": self.dynamic_packets,
            "dynamic_passed": self.dynamic_packets >= self.min_dynamic_packets,
            "min_dynamic_packets": self.min_dynamic_packets,
            "first_sequence": self.first_sequence,
            "last_sequence": self.last_sequence,
            "metric_min": self.metric_min,
            "metric_max": self.metric_max,
        }

    def destroy_node(self):
        if self.summary_path:
            path = Path(self.summary_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(self._summary(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if self.output is not None:
            self.output.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NativeFactorValidator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
