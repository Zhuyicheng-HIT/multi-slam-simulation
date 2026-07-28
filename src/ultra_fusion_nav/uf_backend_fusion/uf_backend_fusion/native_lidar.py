"""FAST-LIO native point-to-plane factor conversion helpers.

The FAST-LIO packet uses a right SO(3) perturbation for its rotational
Jacobian.  The Stage 7 backend stores ZYX roll/pitch/yaw coordinates, so the
normal equation must be transformed before it can be inserted into the
window.  Only the pose block is consumed; online extrinsics remain fixed.
"""

from dataclasses import dataclass, replace
import math
from typing import Sequence

import numpy as np


EXPECTED_POSE_STATE_ORDER = (
    "map_position_x",
    "map_position_y",
    "map_position_z",
    "body_rotation_x",
    "body_rotation_y",
    "body_rotation_z",
)


@dataclass(frozen=True)
class NativeLidarPoseNormal:
    stamp_ns: int
    stamp_s: float
    scan_sequence: int
    matched_points: int
    candidate_points: int
    linearization_pose: np.ndarray
    pose_hessian: np.ndarray
    pose_gradient: np.ndarray
    residual_squared: float
    measurement_variance: float
    source: str
    map_frame: str
    state_frame: str
    sensor_frame: str
    correspondences_valid: bool
    lidar_points: np.ndarray | None = None
    plane_normals: np.ndarray | None = None
    plane_points: np.ndarray | None = None
    lidar_to_body_rotation: np.ndarray | None = None
    lidar_to_body_translation: np.ndarray | None = None
    pose_hessian_right: np.ndarray | None = None
    pose_gradient_right: np.ndarray | None = None


def _finite_vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (size,) or np.any(~np.isfinite(array)):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return array


def quaternion_xyzw_to_rpy(values: Sequence[float]) -> np.ndarray:
    x, y, z, w = _finite_vector(values, 4, "quaternion")
    norm = float(np.linalg.norm([x, y, z, w]))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm must be positive")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.asarray([roll, pitch, yaw], dtype=float)


def rpy_to_quaternion_xyzw(values: Sequence[float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = _finite_vector(values, 3, "roll/pitch/yaw")
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_to_rotation_matrix(values: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = _finite_vector(values, 3, "roll/pitch/yaw")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def quaternion_xyzw_to_rotation_matrix(values: Sequence[float]) -> np.ndarray:
    return rpy_to_rotation_matrix(quaternion_xyzw_to_rpy(values))


def point_plane_residual_jacobian(
    factor: NativeLidarPoseNormal, pose: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """Relinearize FAST-LIO correspondences at a backend pose.

    The rotation perturbation is right-multiplicative, matching FAST-LIO's
    point-to-plane Jacobian and the manifold backend's local SO(3) update.
    """
    pose = _finite_vector(pose, 6, "LiDAR factor pose")
    if any(
        value is None
        for value in (
            factor.lidar_points,
            factor.plane_normals,
            factor.plane_points,
            factor.lidar_to_body_rotation,
            factor.lidar_to_body_translation,
        )
    ):
        raise ValueError("raw point-to-plane correspondences are unavailable")
    lidar_points = np.asarray(factor.lidar_points, dtype=float)
    normals = np.asarray(factor.plane_normals, dtype=float)
    plane_points = np.asarray(factor.plane_points, dtype=float)
    if (
        lidar_points.shape != (factor.matched_points, 3)
        or normals.shape != lidar_points.shape
        or plane_points.shape != lidar_points.shape
    ):
        raise ValueError("raw point-to-plane arrays have incompatible shapes")
    rotation = rpy_to_rotation_matrix(pose[3:6])
    lidar_to_body_rotation = np.asarray(
        factor.lidar_to_body_rotation, dtype=float
    )
    lidar_to_body_translation = np.asarray(
        factor.lidar_to_body_translation, dtype=float
    )
    body_points = (
        lidar_points @ lidar_to_body_rotation.T + lidar_to_body_translation
    )
    world_points = body_points @ rotation.T + pose[:3]
    residual = np.sum(normals * (world_points - plane_points), axis=1)
    normals_body = normals @ rotation
    rotation_jacobian = np.cross(body_points, normals_body)
    jacobian = np.concatenate((normals, rotation_jacobian), axis=1)
    if np.any(~np.isfinite(residual)) or np.any(~np.isfinite(jacobian)):
        raise ValueError("point-to-plane residual linearization is non-finite")
    return residual, jacobian


def right_perturbation_jacobian_rpy(values: Sequence[float]) -> np.ndarray:
    """Map additive ZYX RPY increments to FAST-LIO's body-right tangent."""
    roll, pitch, _ = _finite_vector(values, 3, "roll/pitch/yaw")
    cosine_pitch = math.cos(pitch)
    if abs(cosine_pitch) < 1.0e-4:
        raise ValueError("RPY linearization is too close to pitch gimbal lock")
    return np.asarray([
        [1.0, 0.0, -math.sin(pitch)],
        [0.0, math.cos(roll), math.sin(roll) * cosine_pitch],
        [0.0, -math.sin(roll), math.cos(roll) * cosine_pitch],
    ], dtype=float)


def unwrap_angle(reference: float, wrapped: float) -> float:
    delta = math.atan2(math.sin(wrapped - reference), math.cos(wrapped - reference))
    return float(reference + delta)


def with_yaw_reference(
    factor: NativeLidarPoseNormal, reference_yaw: float,
) -> NativeLidarPoseNormal:
    pose = factor.linearization_pose.copy()
    pose[5] = unwrap_angle(float(reference_yaw), float(pose[5]))
    return replace(factor, linearization_pose=pose)


def native_factor_from_message(msg) -> NativeLidarPoseNormal:
    """Validate one FAST-LIO frame trigger and its optional point-plane factor."""
    if bool(msg.approximate):
        raise ValueError("approximate LiDAR packets cannot enter the native backend")
    columns = int(msg.jacobian_columns)
    if columns != 12:
        raise ValueError("native LiDAR Jacobian must have 12 columns")
    matched = int(msg.matched_points)
    candidate = int(msg.candidate_points)
    correspondences_valid = bool(msg.correspondences_valid)
    if matched < 0 or candidate < matched:
        raise ValueError("native LiDAR match counts are inconsistent")
    if correspondences_valid and matched <= 0:
        raise ValueError("valid native LiDAR correspondences must be non-empty")
    if not correspondences_valid and matched != 0:
        raise ValueError("trigger-only native LiDAR frames must have zero matches")
    state_order = tuple(str(value) for value in msg.jacobian_state_order)
    if len(state_order) != 12 or state_order[:6] != EXPECTED_POSE_STATE_ORDER:
        raise ValueError("native LiDAR pose state order is incompatible")

    position = _finite_vector(msg.linearization_position, 3, "linearization position")
    rpy = quaternion_xyzw_to_rpy(msg.linearization_quaternion)
    measurement_variance = float(msg.measurement_variance)
    if not math.isfinite(measurement_variance) or measurement_variance <= 0.0:
        raise ValueError("native LiDAR measurement variance must be positive")
    stamp_sec = int(msg.header.stamp.sec)
    stamp_nanosec = int(msg.header.stamp.nanosec)
    if stamp_nanosec < 0 or stamp_nanosec >= 1_000_000_000:
        raise ValueError("native LiDAR timestamp nanoseconds are invalid")
    stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
    if stamp_ns <= 0:
        raise ValueError("native LiDAR timestamp must be positive")
    map_frame = str(msg.map_frame)
    state_frame = str(msg.state_frame)
    sensor_frame = str(getattr(msg, "sensor_frame", ""))
    if not map_frame or not state_frame or not sensor_frame:
        raise ValueError("native LiDAR coordinate frames must be non-empty")

    full_hessian = np.asarray(msg.state_hessian, dtype=float)
    full_gradient = np.asarray(msg.state_gradient, dtype=float)
    if full_hessian.shape != (144,) or full_gradient.shape != (12,):
        raise ValueError("native LiDAR normal equation has the wrong shape")
    full_hessian = full_hessian.reshape(12, 12)
    if np.any(~np.isfinite(full_hessian)) or np.any(~np.isfinite(full_gradient)):
        raise ValueError("native LiDAR normal equation must be finite")
    symmetry_error = float(np.linalg.norm(full_hessian - full_hessian.T))
    if symmetry_error > 1.0e-6 * max(1.0, float(np.linalg.norm(full_hessian))):
        raise ValueError("native LiDAR Hessian is not symmetric")

    if not correspondences_valid:
        return NativeLidarPoseNormal(
            stamp_ns=stamp_ns,
            stamp_s=stamp_ns * 1.0e-9,
            scan_sequence=int(msg.scan_sequence),
            matched_points=0,
            candidate_points=candidate,
            linearization_pose=np.concatenate((position, rpy)),
            pose_hessian=np.zeros((6, 6), dtype=float),
            pose_gradient=np.zeros(6, dtype=float),
            residual_squared=0.0,
            measurement_variance=measurement_variance,
            source=str(msg.source),
            map_frame=map_frame,
            state_frame=state_frame,
            sensor_frame=sensor_frame,
            correspondences_valid=False,
            pose_hessian_right=np.zeros((6, 6), dtype=float),
            pose_gradient_right=np.zeros(6, dtype=float),
        )

    coordinate_transform = np.eye(6, dtype=float)
    coordinate_transform[3:, 3:] = right_perturbation_jacobian_rpy(rpy)
    pose_hessian_right = 0.5 * (full_hessian[:6, :6] + full_hessian[:6, :6].T)
    pose_gradient_right = full_gradient[:6]
    pose_hessian = coordinate_transform.T @ pose_hessian_right @ coordinate_transform
    pose_gradient = coordinate_transform.T @ pose_gradient_right
    minimum_eigenvalue = float(np.linalg.eigvalsh(pose_hessian)[0])
    if minimum_eigenvalue < -1.0e-6 * max(1.0, float(np.linalg.norm(pose_hessian))):
        raise ValueError("native LiDAR pose Hessian is not positive semidefinite")

    residuals = np.asarray(msg.residuals, dtype=float)
    if residuals.shape != (matched,) or np.any(~np.isfinite(residuals)):
        raise ValueError("native LiDAR residual vector has the wrong shape")
    raw_values = (
        np.asarray(getattr(msg, "lidar_points_xyz", ()), dtype=float),
        np.asarray(getattr(msg, "plane_normals_xyz", ()), dtype=float),
        np.asarray(getattr(msg, "plane_points_xyz", ()), dtype=float),
    )
    raw_present = any(value.size > 0 for value in raw_values)
    if raw_present and any(value.shape != (matched * 3,) for value in raw_values):
        raise ValueError("native LiDAR raw correspondence arrays have the wrong shape")
    lidar_points = raw_values[0].reshape(matched, 3) if raw_present else None
    plane_normals = raw_values[1].reshape(matched, 3) if raw_present else None
    plane_points = raw_values[2].reshape(matched, 3) if raw_present else None
    if raw_present:
        lidar_to_body_rotation = quaternion_xyzw_to_rotation_matrix(
            msg.lidar_to_body_quaternion
        )
        lidar_to_body_translation = _finite_vector(
            msg.lidar_to_body_translation, 3, "LiDAR-to-body translation"
        )
    else:
        lidar_to_body_rotation = None
        lidar_to_body_translation = None
    return NativeLidarPoseNormal(
        stamp_ns=stamp_ns,
        stamp_s=stamp_ns * 1.0e-9,
        scan_sequence=int(msg.scan_sequence),
        matched_points=matched,
        candidate_points=candidate,
        linearization_pose=np.concatenate((position, rpy)),
        pose_hessian=pose_hessian,
        pose_gradient=pose_gradient,
        residual_squared=float(residuals @ residuals),
        measurement_variance=measurement_variance,
        source=str(msg.source),
        map_frame=map_frame,
        state_frame=state_frame,
        sensor_frame=sensor_frame,
        correspondences_valid=True,
        lidar_points=lidar_points,
        plane_normals=plane_normals,
        plane_points=plane_points,
        lidar_to_body_rotation=lidar_to_body_rotation,
        lidar_to_body_translation=lidar_to_body_translation,
        pose_hessian_right=pose_hessian_right.copy(),
        pose_gradient_right=pose_gradient_right.copy(),
    )


def validate_native_frame_contract(
    factor: NativeLidarPoseNormal,
    expected_map_frame: str,
    expected_state_frame: str,
) -> None:
    """Reject silent coordinate relabeling at the frontend/backend boundary."""
    if factor.map_frame != str(expected_map_frame):
        raise ValueError(
            f"native LiDAR map frame {factor.map_frame!r} does not match "
            f"backend frame {expected_map_frame!r}"
        )
    if factor.state_frame != str(expected_state_frame):
        raise ValueError(
            f"native LiDAR state frame {factor.state_frame!r} does not match "
            f"backend body frame {expected_state_frame!r}"
        )


class NativeFactorBuffer:
    """Small ordered cache used to pair cross-topic factor and odometry stamps."""

    def __init__(self, max_size: int = 128):
        if max_size < 1:
            raise ValueError("native factor buffer size must be positive")
        self.max_size = int(max_size)
        self._items: list[NativeLidarPoseNormal] = []

    def __len__(self) -> int:
        return len(self._items)

    def push(self, factor: NativeLidarPoseNormal) -> None:
        self._items.append(factor)
        self._items.sort(key=lambda item: item.stamp_s)
        if len(self._items) > self.max_size:
            self._items = self._items[-self.max_size:]

    def pop_nearest(self, stamp_s: float, tolerance_s: float) -> NativeLidarPoseNormal | None:
        stamp_s = float(stamp_s)
        tolerance_s = float(tolerance_s)
        if tolerance_s < 0.0:
            raise ValueError("native factor timestamp tolerance must be non-negative")
        self._items = [
            item for item in self._items if item.stamp_s >= stamp_s - tolerance_s
        ]
        if not self._items:
            return None
        distances = [abs(item.stamp_s - stamp_s) for item in self._items]
        index = int(np.argmin(distances))
        if distances[index] > tolerance_s:
            return None
        return self._items.pop(index)
