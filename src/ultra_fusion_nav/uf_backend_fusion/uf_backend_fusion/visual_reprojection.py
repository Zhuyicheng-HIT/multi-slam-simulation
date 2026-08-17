"""Paper-aligned RGB-D inverse-depth reprojection factor primitives.

The compact Stage3 backend does not keep landmarks in its state.  Depth is
therefore treated as a measured inverse-depth anchor with propagated pixel
variance.  Both body poses remain optimized in the same fixed-lag window as
IMU, LiDAR, GNSS and optical flow.
"""

from dataclasses import dataclass
import math

import numpy as np

from .manifold import STATE_SIZE
from .native_lidar import rpy_to_rotation_matrix


@dataclass(frozen=True)
class VisualTrackBatch:
    anchor_normalized: np.ndarray
    current_normalized: np.ndarray
    inverse_depth: np.ndarray
    variance: np.ndarray
    rotation_body_camera: np.ndarray = None
    translation_body_camera: np.ndarray = None

    def __post_init__(self):
        anchor = np.asarray(self.anchor_normalized, dtype=float)
        current = np.asarray(self.current_normalized, dtype=float)
        inverse_depth = np.asarray(self.inverse_depth, dtype=float).reshape(-1)
        if anchor.ndim != 2 or anchor.shape[1] != 2 or current.shape != anchor.shape:
            raise ValueError("visual normalized observations must be Nx2")
        if inverse_depth.shape != (anchor.shape[0],):
            raise ValueError("inverse depth count must match observations")
        variance = np.asarray(self.variance, dtype=float)
        if variance.ndim == 0:
            variance = np.full(anchor.shape[0] * 2, float(variance))
        elif variance.shape == (anchor.shape[0],):
            variance = np.repeat(variance, 2)
        if variance.shape != (anchor.shape[0] * 2,):
            raise ValueError("visual variance must be scalar, N, or 2N")
        rotation = (
            np.eye(3) if self.rotation_body_camera is None
            else np.asarray(self.rotation_body_camera, dtype=float)
        )
        translation = (
            np.zeros(3) if self.translation_body_camera is None
            else np.asarray(self.translation_body_camera, dtype=float)
        )
        if rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError(
                "camera extrinsic must be a 3x3 rotation and 3-vector")
        if (
            anchor.shape[0] == 0 or np.any(
                ~np.isfinite(anchor)) or np.any(
                ~np.isfinite(current)) or np.any(
                ~np.isfinite(inverse_depth)) or np.any(
                    inverse_depth <= 0.0) or np.any(
                        ~np.isfinite(variance)) or np.any(
                            variance <= 0.0) or np.any(
                                ~np.isfinite(rotation)) or np.any(
                                    ~np.isfinite(translation))):
            raise ValueError(
                "visual track batch must be finite and physically valid")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
            raise ValueError("camera extrinsic rotation must be orthonormal")
        object.__setattr__(self, "anchor_normalized", anchor)
        object.__setattr__(self, "current_normalized", current)
        object.__setattr__(self, "inverse_depth", inverse_depth)
        object.__setattr__(self, "variance", variance)
        object.__setattr__(self, "rotation_body_camera", rotation)
        object.__setattr__(self, "translation_body_camera", translation)

    @property
    def track_count(self):
        return int(self.inverse_depth.size)


@dataclass(frozen=True)
class RgbdDepthTrackBatch:
    """Matched metric depth pairs that complement 2-D reprojection."""

    anchor_normalized: np.ndarray
    anchor_depth_m: np.ndarray
    current_depth_m: np.ndarray
    variance_m2: np.ndarray
    rotation_body_camera: np.ndarray = None
    translation_body_camera: np.ndarray = None

    def __post_init__(self):
        anchor = np.asarray(self.anchor_normalized, dtype=float)
        anchor_depth = np.asarray(self.anchor_depth_m, dtype=float).reshape(-1)
        current_depth = np.asarray(self.current_depth_m, dtype=float).reshape(-1)
        variance = np.asarray(self.variance_m2, dtype=float).reshape(-1)
        count = anchor.shape[0] if anchor.ndim == 2 else 0
        rotation = (
            np.eye(3) if self.rotation_body_camera is None
            else np.asarray(self.rotation_body_camera, dtype=float)
        )
        translation = (
            np.zeros(3) if self.translation_body_camera is None
            else np.asarray(self.translation_body_camera, dtype=float)
        )
        if (
            anchor.shape != (count, 2) or count == 0
            or anchor_depth.shape != (count,)
            or current_depth.shape != (count,)
            or variance.shape != (count,)
            or rotation.shape != (3, 3) or translation.shape != (3,)
            or np.any(~np.isfinite(anchor))
            or np.any(~np.isfinite(anchor_depth))
            or np.any(~np.isfinite(current_depth))
            or np.any(~np.isfinite(variance))
            or np.any(anchor_depth <= 0.0)
            or np.any(current_depth <= 0.0)
            or np.any(variance <= 0.0)
            or np.any(~np.isfinite(rotation))
            or np.any(~np.isfinite(translation))
        ):
            raise ValueError("RGB-D depth tracks must be finite and positive")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
            raise ValueError("camera extrinsic rotation must be orthonormal")
        object.__setattr__(self, "anchor_normalized", anchor)
        object.__setattr__(self, "anchor_depth_m", anchor_depth)
        object.__setattr__(self, "current_depth_m", current_depth)
        object.__setattr__(self, "variance_m2", variance)
        object.__setattr__(self, "rotation_body_camera", rotation)
        object.__setattr__(self, "translation_body_camera", translation)

    @property
    def track_count(self):
        return int(self.anchor_depth_m.size)


@dataclass(frozen=True)
class VisualLinearizationCheck:
    """Pre-solve consistency of one measured track batch and two states."""

    valid: bool
    reason: str
    valid_track_count: int
    total_track_count: int
    valid_track_ratio: float
    reprojection_rmse_normalized: float
    reprojection_rmse_px: float
    jacobian_rank: int
    jacobian_condition_number: float
    whitened_nis_per_dof: float
    information_trace: float
    information_max_eigenvalue: float


def visual_pose_observability(points3d, rotation, translation):
    """Return dimensionless six-DoF rank and condition for RGB-D PnP geometry."""
    points = np.asarray(points3d, dtype=float)
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float).reshape(-1)
    if (
        points.ndim != 2 or points.shape[1] != 3 or len(points) < 3
        or rotation.shape != (3, 3) or translation.shape != (3,)
        or np.any(~np.isfinite(points)) or np.any(~np.isfinite(rotation))
        or np.any(~np.isfinite(translation))
    ):
        return 0, math.inf
    current = points @ rotation.T + translation
    valid = np.isfinite(current).all(axis=1) & (current[:, 2] > 1.0e-4)
    current = current[valid]
    if len(current) < 3:
        return 0, math.inf
    median_depth = float(np.median(current[:, 2]))
    if not math.isfinite(median_depth) or median_depth <= 0.0:
        return 0, math.inf
    x, y, z = current.T
    inverse_z = 1.0 / z
    projection = np.zeros((len(current), 2, 3), dtype=float)
    projection[:, 0, 0] = inverse_z
    projection[:, 1, 1] = inverse_z
    projection[:, 0, 2] = -x * inverse_z * inverse_z
    projection[:, 1, 2] = -y * inverse_z * inverse_z
    skew = np.zeros((len(current), 3, 3), dtype=float)
    skew[:, 0, 1] = -z
    skew[:, 0, 2] = y
    skew[:, 1, 0] = z
    skew[:, 1, 2] = -x
    skew[:, 2, 0] = -y
    skew[:, 2, 1] = x
    rotation_columns = np.einsum(
        "nij,njk->nik", projection, -skew, optimize=True
    )
    translation_columns = projection * median_depth
    jacobian = np.concatenate(
        (rotation_columns, translation_columns), axis=2
    ).reshape(-1, 6)
    information = jacobian.T @ jacobian / max(1, len(current))
    eigenvalues = np.linalg.eigvalsh(information)
    maximum = float(eigenvalues[-1]) if eigenvalues.size else 0.0
    tolerance = max(1.0e-12, maximum * 1.0e-9)
    rank = int(np.count_nonzero(eigenvalues > tolerance))
    if rank < 6:
        return rank, math.inf
    return rank, maximum / float(eigenvalues[0])


def _project_with_jacobian(point_camera, minimum_depth=1.0e-4):
    x, y, z = np.asarray(point_camera, dtype=float)
    if not math.isfinite(z) or z <= minimum_depth:
        return None, None
    prediction = np.asarray([x / z, y / z])
    jacobian = np.asarray([
        [1.0 / z, 0.0, -x / (z * z)],
        [0.0, 1.0 / z, -y / (z * z)],
    ])
    return prediction, jacobian


def visual_reprojection_residual_jacobians(
        anchor_state, current_state, tracks, minimum_depth=1.0e-4):
    """Return residual and right-local pose Jacobians for Eq. (10)."""
    anchor_state = np.asarray(anchor_state, dtype=float)
    current_state = np.asarray(current_state, dtype=float)
    if anchor_state.shape != (
        STATE_SIZE,
    ) or current_state.shape != (
        STATE_SIZE,
    ):
        raise ValueError("visual factor requires two 15-state vectors")
    rotation_anchor = rpy_to_rotation_matrix(anchor_state[3:6])
    rotation_current = rpy_to_rotation_matrix(current_state[3:6])
    rotation_body_camera = tracks.rotation_body_camera
    translation_body_camera = tracks.translation_body_camera
    bearings = np.column_stack((
        tracks.anchor_normalized,
        np.ones(tracks.track_count, dtype=float),
    ))
    point_camera_anchor = bearings / tracks.inverse_depth[:, None]
    point_body_anchor = (
        point_camera_anchor @ rotation_body_camera.T
        + translation_body_camera
    )
    point_world = point_body_anchor @ rotation_anchor.T + anchor_state[:3]
    point_body_current = (
        point_world - current_state[:3]
    ) @ rotation_current
    body_to_camera = rotation_body_camera.T
    point_camera_current = (
        point_body_current - translation_body_camera
    ) @ rotation_body_camera
    depth = point_camera_current[:, 2]
    valid = np.isfinite(depth) & (depth > minimum_depth)
    valid_indices = np.flatnonzero(valid)
    if valid_indices.size == 0:
        return (
            np.empty(0), np.empty((0, STATE_SIZE)),
            np.empty((0, STATE_SIZE)), np.empty(0, dtype=int),
        )
    point_camera_current = point_camera_current[valid]
    point_body_anchor = point_body_anchor[valid]
    point_body_current = point_body_current[valid]
    depth = point_camera_current[:, 2]
    inverse_z = 1.0 / depth
    prediction = point_camera_current[:, :2] * inverse_z[:, None]
    projection_jacobian = np.zeros((valid_indices.size, 2, 3), dtype=float)
    projection_jacobian[:, 0, 0] = inverse_z
    projection_jacobian[:, 1, 1] = inverse_z
    projection_jacobian[:, 0, 2] = (
        -point_camera_current[:, 0] * inverse_z * inverse_z
    )
    projection_jacobian[:, 1, 2] = (
        -point_camera_current[:, 1] * inverse_z * inverse_z
    )

    def skew_batch(points):
        matrices = np.zeros((points.shape[0], 3, 3), dtype=float)
        matrices[:, 0, 1] = -points[:, 2]
        matrices[:, 0, 2] = points[:, 1]
        matrices[:, 1, 0] = points[:, 2]
        matrices[:, 1, 2] = -points[:, 0]
        matrices[:, 2, 0] = -points[:, 1]
        matrices[:, 2, 1] = points[:, 0]
        return matrices

    current_to_camera = body_to_camera @ rotation_current.T
    anchor_rotation = np.einsum(
        "ij,njk->nik",
        -current_to_camera @ rotation_anchor,
        skew_batch(point_body_anchor),
        optimize=True,
    )
    current_rotation = np.einsum(
        "ij,njk->nik",
        body_to_camera,
        skew_batch(point_body_current),
        optimize=True,
    )
    jacobian_anchor = np.zeros(
        (valid_indices.size, 2, STATE_SIZE), dtype=float
    )
    jacobian_current = np.zeros_like(jacobian_anchor)
    jacobian_anchor[:, :, :3] = np.einsum(
        "nij,jk->nik", projection_jacobian, current_to_camera,
        optimize=True,
    )
    jacobian_anchor[:, :, 3:6] = np.einsum(
        "nij,njk->nik", projection_jacobian, anchor_rotation,
        optimize=True,
    )
    jacobian_current[:, :, :3] = -jacobian_anchor[:, :, :3]
    jacobian_current[:, :, 3:6] = np.einsum(
        "nij,njk->nik", projection_jacobian, current_rotation,
        optimize=True,
    )
    return (
        (prediction - tracks.current_normalized[valid]).reshape(-1),
        jacobian_anchor.reshape(-1, STATE_SIZE),
        jacobian_current.reshape(-1, STATE_SIZE),
        valid_indices,
    )


def visual_reprojection_residual(anchor_state, current_state, tracks):
    return visual_reprojection_residual_jacobians(
        anchor_state, current_state, tracks
    )[0]


def rgbd_depth_residual_jacobians(
        anchor_state, current_state, tracks, minimum_depth=1.0e-4):
    """Return line-of-sight metric-depth residuals and pose Jacobians.

    Reprojection already contributes the two angular image rows.  This adds
    only predicted camera Z minus the measured current depth, completing a
    sparse 3-D RGB-D observation without double-counting pixel coordinates.
    """
    if not isinstance(tracks, RgbdDepthTrackBatch):
        raise ValueError("RGB-D depth factor has the wrong track type")
    anchor_state = np.asarray(anchor_state, dtype=float)
    current_state = np.asarray(current_state, dtype=float)
    if anchor_state.shape != (STATE_SIZE,) or current_state.shape != (STATE_SIZE,):
        raise ValueError("RGB-D depth factor requires two 15-state vectors")
    rotation_anchor = rpy_to_rotation_matrix(anchor_state[3:6])
    rotation_current = rpy_to_rotation_matrix(current_state[3:6])
    rotation_body_camera = tracks.rotation_body_camera
    translation_body_camera = tracks.translation_body_camera
    bearings = np.column_stack((
        tracks.anchor_normalized,
        np.ones(tracks.track_count, dtype=float),
    ))
    point_camera_anchor = bearings * tracks.anchor_depth_m[:, None]
    point_body_anchor = (
        point_camera_anchor @ rotation_body_camera.T
        + translation_body_camera
    )
    point_world = point_body_anchor @ rotation_anchor.T + anchor_state[:3]
    point_body_current = (
        point_world - current_state[:3]
    ) @ rotation_current
    point_camera_current = (
        point_body_current - translation_body_camera
    ) @ rotation_body_camera
    valid = (
        np.isfinite(point_camera_current).all(axis=1)
        & (point_camera_current[:, 2] > minimum_depth)
    )
    valid_indices = np.flatnonzero(valid)
    if valid_indices.size == 0:
        return (
            np.empty(0), np.empty((0, STATE_SIZE)),
            np.empty((0, STATE_SIZE)), np.empty(0, dtype=int),
        )

    def skew_batch(points):
        matrices = np.zeros((points.shape[0], 3, 3), dtype=float)
        matrices[:, 0, 1] = -points[:, 2]
        matrices[:, 0, 2] = points[:, 1]
        matrices[:, 1, 0] = points[:, 2]
        matrices[:, 1, 2] = -points[:, 0]
        matrices[:, 2, 0] = -points[:, 1]
        matrices[:, 2, 1] = points[:, 0]
        return matrices

    body_to_camera = rotation_body_camera.T
    current_to_camera = body_to_camera @ rotation_current.T
    anchor_rotation = np.einsum(
        "ij,njk->nik",
        -current_to_camera @ rotation_anchor,
        skew_batch(point_body_anchor[valid]),
        optimize=True,
    )
    current_rotation = np.einsum(
        "ij,njk->nik",
        body_to_camera,
        skew_batch(point_body_current[valid]),
        optimize=True,
    )
    anchor_jacobian = np.zeros((valid_indices.size, STATE_SIZE), dtype=float)
    current_jacobian = np.zeros_like(anchor_jacobian)
    anchor_jacobian[:, :3] = current_to_camera[2]
    anchor_jacobian[:, 3:6] = anchor_rotation[:, 2, :]
    current_jacobian[:, :3] = -current_to_camera[2]
    current_jacobian[:, 3:6] = current_rotation[:, 2, :]
    residual = (
        point_camera_current[valid, 2] - tracks.current_depth_m[valid]
    )
    return residual, anchor_jacobian, current_jacobian, valid_indices


def validate_visual_linearization(
    anchor_state,
    current_state,
    tracks,
    focal_x_px,
    focal_y_px,
    *,
    maximum_reprojection_rmse_px=6.0,
    minimum_valid_track_ratio=0.8,
    minimum_jacobian_rank=6,
    maximum_jacobian_condition_number=5.0e4,
):
    """Check a factor at the current window linearization before staging it.

    The frontend PnP/RANSAC check proves that the RGB-D tracks are internally
    geometric.  This second check proves that the same tracks are compatible
    with the already active multi-sensor states.  It is an innovation check,
    not a replacement timestamp or a relaxed optimization-integrity limit.
    The default six-pixel bound is twice the frontend's three-pixel PnP RANSAC
    threshold so ordinary estimator correction remains possible while gross
    cross-modal disagreement is rejected before a transaction is opened.
    """
    focal_x_px = float(focal_x_px)
    focal_y_px = float(focal_y_px)
    maximum_reprojection_rmse_px = float(maximum_reprojection_rmse_px)
    minimum_valid_track_ratio = float(minimum_valid_track_ratio)
    minimum_jacobian_rank = int(minimum_jacobian_rank)
    maximum_jacobian_condition_number = float(
        maximum_jacobian_condition_number
    )
    if (
        not isinstance(tracks, VisualTrackBatch)
        or not math.isfinite(focal_x_px) or focal_x_px <= 0.0
        or not math.isfinite(focal_y_px) or focal_y_px <= 0.0
        or not math.isfinite(maximum_reprojection_rmse_px)
        or maximum_reprojection_rmse_px <= 0.0
        or not 0.0 < minimum_valid_track_ratio <= 1.0
        or not 1 <= minimum_jacobian_rank <= 6
        or not math.isfinite(maximum_jacobian_condition_number)
        or maximum_jacobian_condition_number <= 1.0
    ):
        raise ValueError("visual linearization check configuration is invalid")
    residual, anchor_jacobian, current_jacobian, valid = (
        visual_reprojection_residual_jacobians(
            anchor_state, current_state, tracks
        )
    )
    valid_count = int(valid.size)
    total_count = tracks.track_count
    valid_ratio = valid_count / max(1, total_count)

    def result(
        valid_result, reason, normalized=math.inf, pixels=math.inf,
        rank=0, condition=math.inf, nis_per_dof=math.inf,
        information_trace=0.0, information_max_eigenvalue=0.0,
    ):
        return VisualLinearizationCheck(
            bool(valid_result), str(reason), valid_count, total_count,
            float(valid_ratio), float(normalized), float(pixels), int(rank),
            float(condition), float(nis_per_dof), float(information_trace),
            float(information_max_eigenvalue),
        )

    if valid_count == 0 or residual.size != valid_count * 2:
        return result(False, "no_projectable_tracks")
    if valid_ratio < minimum_valid_track_ratio:
        return result(False, "insufficient_projectable_track_ratio")
    if (
        np.any(~np.isfinite(residual))
        or np.any(~np.isfinite(anchor_jacobian))
        or np.any(~np.isfinite(current_jacobian))
    ):
        return result(False, "nonfinite_residual_or_jacobian")
    residual_2d = residual.reshape(-1, 2)
    normalized_rmse = float(np.sqrt(np.mean(np.sum(residual_2d ** 2, axis=1))))
    pixel_residual = residual_2d * np.asarray([focal_x_px, focal_y_px])
    pixel_rmse = float(np.sqrt(np.mean(np.sum(pixel_residual ** 2, axis=1))))
    relative_jacobian = current_jacobian[:, :6] - anchor_jacobian[:, :6]
    variance = tracks.variance.reshape(-1, 2)[valid].reshape(-1)
    whitened_nis_per_dof = float(np.mean(residual * residual / variance))
    representative_depth = float(np.median(1.0 / tracks.inverse_depth[valid]))
    scaled_jacobian = relative_jacobian.copy()
    scaled_jacobian[:, :3] *= representative_depth
    whitened_jacobian = scaled_jacobian / np.sqrt(variance)[:, None]
    information = (
        whitened_jacobian.T @ whitened_jacobian / max(1, valid_count)
    )
    eigenvalues = np.linalg.eigvalsh(information)
    maximum_eigenvalue = float(eigenvalues[-1])
    tolerance = max(1.0e-12, maximum_eigenvalue * 1.0e-9)
    rank = int(np.count_nonzero(eigenvalues > tolerance))
    condition = (
        maximum_eigenvalue / float(eigenvalues[0])
        if rank == 6 else math.inf
    )
    information_trace = float(np.trace(information))
    if not math.isfinite(normalized_rmse) or not math.isfinite(pixel_rmse):
        return result(
            False, "nonfinite_reprojection_rmse", rank=rank,
            condition=condition, nis_per_dof=whitened_nis_per_dof,
            information_trace=information_trace,
            information_max_eigenvalue=maximum_eigenvalue,
        )
    if rank < minimum_jacobian_rank:
        return result(
            False, "insufficient_visual_jacobian_rank",
            normalized_rmse, pixel_rmse, rank, condition,
            whitened_nis_per_dof, information_trace, maximum_eigenvalue,
        )
    if condition > maximum_jacobian_condition_number:
        return result(
            False, "ill_conditioned_visual_jacobian",
            normalized_rmse, pixel_rmse, rank, condition,
            whitened_nis_per_dof, information_trace, maximum_eigenvalue,
        )
    if pixel_rmse > maximum_reprojection_rmse_px:
        return result(
            False, "state_innovation_reprojection_rmse",
            normalized_rmse, pixel_rmse, rank, condition,
            whitened_nis_per_dof, information_trace, maximum_eigenvalue,
        )
    return result(
        True, "linearization_valid", normalized_rmse, pixel_rmse, rank,
        condition, whitened_nis_per_dof, information_trace,
        maximum_eigenvalue,
    )
