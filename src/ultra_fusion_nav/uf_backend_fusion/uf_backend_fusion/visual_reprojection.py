"""Paper-aligned RGB-D inverse-depth reprojection factor primitives.

The compact Stage3 backend does not keep landmarks in its state.  Depth is
therefore treated as a measured inverse-depth anchor with propagated pixel
variance.  Both body poses remain optimized in the same fixed-lag window as
IMU, LiDAR, GNSS and optical flow.
"""

from dataclasses import dataclass
import math

import numpy as np

from .manifold import STATE_SIZE, skew
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
    residuals = []
    jacobian_anchor = []
    jacobian_current = []
    valid_indices = []
    for index in range(tracks.track_count):
        bearing = np.r_[tracks.anchor_normalized[index], 1.0]
        point_camera_anchor = bearing / tracks.inverse_depth[index]
        point_body_anchor = (
            rotation_body_camera @ point_camera_anchor +
            translation_body_camera)
        point_world = rotation_anchor @ point_body_anchor + anchor_state[:3]
        point_body_current = rotation_current.T @ (
            point_world - current_state[:3]
        )
        point_camera_current = rotation_body_camera.T @ (
            point_body_current - translation_body_camera
        )
        prediction, projection_jacobian = _project_with_jacobian(
            point_camera_current, minimum_depth
        )
        if prediction is None:
            continue
        body_to_camera = rotation_body_camera.T
        current_to_camera = body_to_camera @ rotation_current.T
        anchor_pose = np.zeros((3, STATE_SIZE))
        current_pose = np.zeros((3, STATE_SIZE))
        anchor_pose[:, :3] = current_to_camera
        anchor_pose[:, 3:6] = (
            current_to_camera @ (-rotation_anchor @ skew(point_body_anchor))
        )
        current_pose[:, :3] = -current_to_camera
        current_pose[:, 3:6] = body_to_camera @ skew(point_body_current)
        residuals.append(prediction - tracks.current_normalized[index])
        jacobian_anchor.append(projection_jacobian @ anchor_pose)
        jacobian_current.append(projection_jacobian @ current_pose)
        valid_indices.append(index)
    if not residuals:
        return (
            np.empty(0), np.empty((0, STATE_SIZE)),
            np.empty((0, STATE_SIZE)), np.empty(0, dtype=int),
        )
    return (
        np.concatenate(residuals), np.vstack(jacobian_anchor),
        np.vstack(jacobian_current), np.asarray(valid_indices, dtype=int),
    )


def visual_reprojection_residual(anchor_state, current_state, tracks):
    return visual_reprojection_residual_jacobians(
        anchor_state, current_state, tracks
    )[0]


def validate_visual_linearization(
    anchor_state,
    current_state,
    tracks,
    focal_x_px,
    focal_y_px,
    *,
    maximum_reprojection_rmse_px=6.0,
    minimum_valid_track_ratio=0.8,
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
    if (
        not isinstance(tracks, VisualTrackBatch)
        or not math.isfinite(focal_x_px) or focal_x_px <= 0.0
        or not math.isfinite(focal_y_px) or focal_y_px <= 0.0
        or not math.isfinite(maximum_reprojection_rmse_px)
        or maximum_reprojection_rmse_px <= 0.0
        or not 0.0 < minimum_valid_track_ratio <= 1.0
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

    def result(valid_result, reason, normalized=math.inf, pixels=math.inf,
               rank=0):
        return VisualLinearizationCheck(
            bool(valid_result), str(reason), valid_count, total_count,
            float(valid_ratio), float(normalized), float(pixels), int(rank),
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
    rank = int(np.linalg.matrix_rank(relative_jacobian))
    if not math.isfinite(normalized_rmse) or not math.isfinite(pixel_rmse):
        return result(False, "nonfinite_reprojection_rmse", rank=rank)
    if rank < 3:
        return result(
            False, "insufficient_visual_jacobian_rank",
            normalized_rmse, pixel_rmse, rank,
        )
    if pixel_rmse > maximum_reprojection_rmse_px:
        return result(
            False, "state_innovation_reprojection_rmse",
            normalized_rmse, pixel_rmse, rank,
        )
    return result(
        True, "linearization_valid", normalized_rmse, pixel_rmse, rank
    )
