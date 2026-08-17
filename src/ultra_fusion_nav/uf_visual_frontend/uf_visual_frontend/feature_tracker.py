"""Deterministic KLT/FB/depth/PnP processing independent of ROS."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class TrackResult:
    previous_pixels: np.ndarray
    current_pixels: np.ndarray
    feature_ids: np.ndarray
    ages: np.ndarray
    forward_backward_error: np.ndarray
    depth_m: np.ndarray
    depth_sigma_m: np.ndarray
    depth_valid: np.ndarray
    current_depth_m: np.ndarray
    current_depth_sigma_m: np.ndarray
    current_depth_valid: np.ndarray
    geometric_inlier: np.ndarray
    reprojection_error: np.ndarray
    rotation: object
    translation: object
    pnp_inlier_ratio: float
    pnp_information_rank: int
    pnp_condition_number: float


def pnp_observability(points3d, rotation, translation):
    """Return a dimensionless 6-DoF PnP information rank and condition.

    Translation columns are scaled by the median scene depth so their units are
    comparable with the rotation columns.  The result measures geometry, not
    motion magnitude, and remains meaningful for stationary RGB-D frames.
    """
    points = np.asarray(points3d, dtype=float)
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float).reshape(-1)
    if (
        points.ndim != 2 or points.shape[1] != 3 or len(points) < 3
        or rotation.shape != (3, 3) or translation.shape != (3,)
        or np.any(~np.isfinite(points)) or np.any(~np.isfinite(rotation))
        or np.any(~np.isfinite(translation))
    ):
        return 0, np.inf
    current = points @ rotation.T + translation
    valid = np.isfinite(current).all(axis=1) & (current[:, 2] > 1.0e-4)
    current = current[valid]
    if len(current) < 3:
        return 0, np.inf
    median_depth = float(np.median(current[:, 2]))
    if not np.isfinite(median_depth) or median_depth <= 0.0:
        return 0, np.inf

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
        return rank, np.inf
    minimum = float(eigenvalues[0])
    return rank, maximum / minimum


class RgbdFeatureTracker:
    def __init__(self, max_features=240, minimum_distance_px=12.0,
                 fb_threshold_px=1.0, pnp_reprojection_px=3.0,
                 minimum_pnp_points=8, depth_scale=0.001,
                 minimum_depth_m=0.30, maximum_depth_m=6.0,
                 depth_neighborhood_radius_px=1,
                 depth_minimum_support=3,
                 depth_minimum_inlier_ratio=0.60,
                 depth_inlier_absolute_tolerance_m=0.03,
                 depth_inlier_relative_tolerance=0.03,
                 depth_noise_floor_m=0.005):
        self.max_features = int(max_features)
        self.minimum_distance_px = float(minimum_distance_px)
        self.fb_threshold_px = float(fb_threshold_px)
        self.pnp_reprojection_px = float(pnp_reprojection_px)
        self.minimum_pnp_points = int(minimum_pnp_points)
        self.depth_scale = float(depth_scale)
        self.minimum_depth_m = float(minimum_depth_m)
        self.maximum_depth_m = float(maximum_depth_m)
        self.depth_neighborhood_radius_px = max(
            0, int(depth_neighborhood_radius_px))
        self.depth_minimum_support = max(1, int(depth_minimum_support))
        self.depth_minimum_inlier_ratio = min(
            1.0, max(0.0, float(depth_minimum_inlier_ratio)))
        self.depth_inlier_absolute_tolerance_m = max(
            0.0, float(depth_inlier_absolute_tolerance_m))
        self.depth_inlier_relative_tolerance = max(
            0.0, float(depth_inlier_relative_tolerance))
        self.depth_noise_floor_m = max(0.0, float(depth_noise_floor_m))
        self.previous_gray = None
        self.previous_depth = None
        self.points = np.empty((0, 2), np.float32)
        self.ids = np.empty(0, np.int64)
        self.ages = np.empty(0, np.int32)
        self.next_id = 1

    def _sample_depth(self, depth, pixels):
        values = np.asarray(depth)
        scale = self.depth_scale if values.dtype == np.uint16 else 1.0
        height, width = values.shape[:2]
        rounded = np.rint(pixels).astype(int)
        samples = np.full(len(pixels), np.nan, np.float32)
        sigma = np.full(len(pixels), np.nan, np.float32)
        valid = np.zeros(len(pixels), bool)
        radius = self.depth_neighborhood_radius_px
        for index, (x, y) in enumerate(rounded):
            if x < 0 or x >= width or y < 0 or y >= height:
                continue
            x0, x1 = max(0, x - radius), min(width, x + radius + 1)
            y0, y1 = max(0, y - radius), min(height, y + radius + 1)
            neighborhood = (
                values[y0:y1, x0:x1].reshape(-1).astype(np.float64) * scale
            )
            neighborhood = neighborhood[
                np.isfinite(neighborhood)
                & (neighborhood >= self.minimum_depth_m - 1.0e-9)
                & (neighborhood <= self.maximum_depth_m + 1.0e-9)
            ]
            if neighborhood.size < self.depth_minimum_support:
                continue
            median = float(np.median(neighborhood))
            tolerance = max(
                self.depth_inlier_absolute_tolerance_m,
                self.depth_inlier_relative_tolerance * median,
            )
            residual = np.abs(neighborhood - median)
            inliers = neighborhood[residual <= tolerance]
            required = max(
                self.depth_minimum_support,
                int(np.ceil(
                    self.depth_minimum_inlier_ratio * neighborhood.size)),
            )
            if inliers.size < required:
                continue
            estimate = float(np.median(inliers))
            mad = float(np.median(np.abs(inliers - estimate)))
            samples[index] = estimate
            sigma[index] = max(self.depth_noise_floor_m, 1.4826 * mad)
            valid[index] = True
        return samples, valid, sigma

    def _valid_depth_mask(self, depth):
        """Return pixels whose measured depth is inside the RGB-D range."""
        values = np.asarray(depth)
        scale = self.depth_scale if values.dtype == np.uint16 else 1.0
        depth_m = values.astype(np.float64, copy=False) * scale
        valid = (
            np.isfinite(depth_m)
            & (depth_m >= self.minimum_depth_m - 1.0e-9)
            & (depth_m <= self.maximum_depth_m + 1.0e-9)
        )
        return valid

    def _replenish(self, gray, occupied, valid_depth_mask=None):
        if valid_depth_mask is None:
            mask = np.full(gray.shape, 255, np.uint8)
        else:
            valid_depth_mask = np.asarray(valid_depth_mask, dtype=bool)
            if valid_depth_mask.shape != gray.shape:
                raise ValueError("depth mask and grayscale image shapes differ")
            mask = (valid_depth_mask.astype(np.uint8) * 255)
        for point in occupied:
            cv2.circle(mask, tuple(np.rint(point).astype(int)),
                       int(self.minimum_distance_px), 0, -1)
        needed = max(0, self.max_features - len(occupied))
        if not needed:
            return np.empty((0, 2), np.float32)
        corners = cv2.goodFeaturesToTrack(
            gray, needed, 0.01, self.minimum_distance_px, mask=mask,
            blockSize=7, useHarrisDetector=False,
        )
        return np.empty(
            (0, 2), np.float32) if corners is None else corners[:, 0]

    def process(self, gray, depth, camera_matrix):
        gray = np.asarray(gray, dtype=np.uint8)
        depth = np.asarray(depth)
        camera_matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
        valid_depth_mask = self._valid_depth_mask(depth)
        if self.previous_gray is None:
            self.points = self._replenish(
                gray, np.empty((0, 2)), valid_depth_mask
            )
            count = len(self.points)
            self.ids = np.arange(
                self.next_id,
                self.next_id + count,
                dtype=np.int64)
            self.next_id += count
            self.ages = np.ones(count, np.int32)
            self.previous_gray, self.previous_depth = gray.copy(), depth.copy()
            return None
        if not len(self.points):
            forward = np.empty((0, 2), np.float32)
            status = np.empty(0, bool)
            fb_error = np.empty(0, np.float32)
        else:
            forward, forward_status, _ = cv2.calcOpticalFlowPyrLK(
                self.previous_gray, gray, self.points.reshape(-1, 1, 2), None,
                winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            )
            backward, backward_status, _ = cv2.calcOpticalFlowPyrLK(
                gray, self.previous_gray, forward, None, winSize=(
                    21, 21), maxLevel=3, criteria=(
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01), )
            forward = forward[:, 0]
            backward = backward[:, 0]
            fb_error = np.linalg.norm(backward - self.points, axis=1)
            status = (
                forward_status[:, 0].astype(bool) & backward_status[:, 0].astype(bool)
                & np.isfinite(fb_error) & (fb_error <= self.fb_threshold_px)
            )
        previous = self.points[status]
        current = forward[status]
        ids = self.ids[status]
        ages = self.ages[status] + 1
        fb = fb_error[status]
        depth_m, depth_valid, depth_sigma_m = self._sample_depth(
            self.previous_depth, previous)
        geometric = np.zeros(len(current), bool)
        reprojection = np.full(len(current), -1.0, np.float32)
        rotation = translation = None
        pnp_inlier_ratio = 0.0
        pnp_information_rank = 0
        pnp_condition_number = np.inf
        candidates = np.flatnonzero(depth_valid)
        if len(candidates) >= self.minimum_pnp_points:
            normalized = cv2.undistortPoints(
                previous[candidates].reshape(-1, 1, 2), camera_matrix, None
            )[:, 0]
            points3d = np.c_[normalized *
                             depth_m[candidates, None], depth_m[candidates]]
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                points3d.astype(np.float32), current[candidates].astype(np.float32),
                camera_matrix, None, iterationsCount=100,
                reprojectionError=self.pnp_reprojection_px, confidence=0.999,
                flags=cv2.SOLVEPNP_EPNP,
            )
            if ok and inliers is not None:
                local = inliers[:, 0]
                accepted = candidates[local]
                geometric[accepted] = True
                projected, _ = cv2.projectPoints(
                    points3d, rvec, tvec, camera_matrix, None)
                reprojection[candidates] = np.linalg.norm(
                    projected[:, 0] - current[candidates], axis=1)
                rotation, translation = cv2.Rodrigues(rvec)[0], tvec[:, 0]
                pnp_inlier_ratio = len(local) / max(1, len(candidates))
                pnp_information_rank, pnp_condition_number = pnp_observability(
                    points3d[local], rotation, translation
                )
        current_depth_m, current_depth_valid, current_depth_sigma_m = (
            self._sample_depth(depth, current)
        )
        result = TrackResult(
            previous,
            current,
            ids,
            ages,
            fb,
            depth_m,
            depth_sigma_m,
            depth_valid,
            current_depth_m,
            current_depth_sigma_m,
            current_depth_valid,
            geometric,
            reprojection,
            rotation,
            translation,
            pnp_inlier_ratio,
            pnp_information_rank,
            pnp_condition_number)
        retained = current[current_depth_valid]
        retained_ids = ids[current_depth_valid]
        retained_ages = ages[current_depth_valid]
        new_points = self._replenish(gray, retained, valid_depth_mask)
        new_ids = np.arange(
            self.next_id,
            self.next_id +
            len(new_points),
            dtype=np.int64)
        self.next_id += len(new_points)
        self.points = np.vstack((retained, new_points)).astype(np.float32)
        self.ids = np.r_[retained_ids, new_ids]
        self.ages = np.r_[retained_ages, np.ones(len(new_points), np.int32)]
        self.previous_gray, self.previous_depth = gray.copy(), depth.copy()
        return result


def grid_uniformity(points, width, height, rows=8, columns=8):
    if not len(points) or width <= 0 or height <= 0:
        return 0.0
    cells = np.c_[
        np.clip((points[:, 0] * columns / width).astype(int), 0, columns - 1),
        np.clip((points[:, 1] * rows / height).astype(int), 0, rows - 1),
    ]
    return float(
        len(np.unique(cells[:, 1] * columns + cells[:, 0])) / (rows * columns))
