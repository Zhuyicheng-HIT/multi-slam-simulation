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
    depth_valid: np.ndarray
    geometric_inlier: np.ndarray
    reprojection_error: np.ndarray
    rotation: object
    translation: object


class RgbdFeatureTracker:
    def __init__(self, max_features=240, minimum_distance_px=12.0,
                 fb_threshold_px=1.0, pnp_reprojection_px=3.0,
                 minimum_pnp_points=8, depth_scale=0.001,
                 minimum_depth_m=0.15, maximum_depth_m=12.0):
        self.max_features = int(max_features)
        self.minimum_distance_px = float(minimum_distance_px)
        self.fb_threshold_px = float(fb_threshold_px)
        self.pnp_reprojection_px = float(pnp_reprojection_px)
        self.minimum_pnp_points = int(minimum_pnp_points)
        self.depth_scale = float(depth_scale)
        self.minimum_depth_m = float(minimum_depth_m)
        self.maximum_depth_m = float(maximum_depth_m)
        self.previous_gray = None
        self.previous_depth = None
        self.points = np.empty((0, 2), np.float32)
        self.ids = np.empty(0, np.int64)
        self.ages = np.empty(0, np.int32)
        self.next_id = 1

    def _depth_metres(self, depth):
        values = np.asarray(depth)
        scale = self.depth_scale if values.dtype == np.uint16 else 1.0
        return values.astype(np.float32) * scale

    def _sample_depth(self, depth, pixels):
        values = self._depth_metres(depth)
        height, width = values.shape[:2]
        rounded = np.rint(pixels).astype(int)
        valid_bounds = (
            (rounded[:, 0] >= 0) & (rounded[:, 0] < width)
            & (rounded[:, 1] >= 0) & (rounded[:, 1] < height)
        )
        samples = np.full(len(pixels), np.nan, np.float32)
        indices = np.flatnonzero(valid_bounds)
        samples[indices] = values[rounded[indices, 1], rounded[indices, 0]]
        valid = (
            np.isfinite(samples) & (samples >= self.minimum_depth_m)
            & (samples <= self.maximum_depth_m)
        )
        return samples, valid

    def _replenish(self, gray, occupied):
        mask = np.full(gray.shape, 255, np.uint8)
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
        camera_matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
        if self.previous_gray is None:
            self.points = self._replenish(gray, np.empty((0, 2)))
            count = len(self.points)
            self.ids = np.arange(
                self.next_id,
                self.next_id + count,
                dtype=np.int64)
            self.next_id += count
            self.ages = np.ones(count, np.int32)
            self.previous_gray, self.previous_depth = gray.copy(), np.asarray(depth).copy()
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
        depth_m, depth_valid = self._sample_depth(
            self.previous_depth, previous)
        geometric = np.zeros(len(current), bool)
        reprojection = np.full(len(current), -1.0, np.float32)
        rotation = translation = None
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
        result = TrackResult(
            previous,
            current,
            ids,
            ages,
            fb,
            depth_m,
            depth_valid,
            geometric,
            reprojection,
            rotation,
            translation)
        retained = current
        new_points = self._replenish(gray, retained)
        new_ids = np.arange(
            self.next_id,
            self.next_id +
            len(new_points),
            dtype=np.int64)
        self.next_id += len(new_points)
        self.points = np.vstack((retained, new_points)).astype(np.float32)
        self.ids = np.r_[ids, new_ids]
        self.ages = np.r_[ages, np.ones(len(new_points), np.int32)]
        self.previous_gray, self.previous_depth = gray.copy(), np.asarray(depth).copy()
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
