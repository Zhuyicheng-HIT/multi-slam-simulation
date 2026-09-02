"""Bounded SE(3) interpolation for offline per-point LiDAR deskew."""

import csv
from pathlib import Path

import numpy as np

from .association import PoseSample


class TrajectoryContractError(ValueError):
    """Raised when a requested point timestamp has no valid pose bracket."""


def _normalize_quaternions(quaternions):
    values = np.asarray(quaternions, dtype=np.float64)
    norms = np.linalg.norm(values, axis=-1, keepdims=True)
    if np.any(~np.isfinite(values)) or np.any(norms <= 1.0e-12):
        raise TrajectoryContractError("pose_nonfinite_or_zero_quaternion")
    return values / norms


def _rotate_vectors(vectors, quaternions):
    vectors = np.asarray(vectors, dtype=np.float64)
    quaternions = np.asarray(quaternions, dtype=np.float64)
    vector_part = quaternions[..., :3]
    scalar_part = quaternions[..., 3:4]
    first_cross = np.cross(vector_part, vectors)
    return vectors + 2.0 * (
        scalar_part * first_cross + np.cross(vector_part, first_cross)
    )


class PoseTrajectory:
    def __init__(self, samples, max_bracket_span_ns):
        self.samples = tuple(samples)
        self.max_bracket_span_ns = int(max_bracket_span_ns)
        if self.max_bracket_span_ns <= 0:
            raise TrajectoryContractError("max_bracket_span_ns must be positive")
        if not self.samples:
            raise TrajectoryContractError("pose_missing")
        stamps = np.asarray([item.stamp_ns for item in self.samples], dtype=np.int64)
        if np.any(stamps[1:] <= stamps[:-1]):
            raise TrajectoryContractError("pose_timestamp_regression")
        if any(not item.finite() for item in self.samples):
            raise TrajectoryContractError("pose_nonfinite")
        self.stamps = stamps
        self.epochs = np.asarray([item.epoch for item in self.samples], dtype=np.int64)
        self.translations = np.asarray(
            [item.translation for item in self.samples], dtype=np.float64
        )
        self.quaternions = _normalize_quaternions(
            [item.quaternion_xyzw for item in self.samples]
        )

    def interpolate_many(self, stamp_ns, epoch=None):
        queries = np.asarray(stamp_ns, dtype=np.int64).reshape(-1)
        right = np.searchsorted(self.stamps, queries, side="left")
        exact = (right < len(self.stamps))
        exact[exact] &= self.stamps[right[exact]] == queries[exact]
        if np.any((right == 0) & ~exact):
            raise TrajectoryContractError("missing_left_state")
        if np.any(right >= len(self.stamps)):
            raise TrajectoryContractError("missing_right_state")

        left = np.where(exact, right, right - 1)
        right_index = right
        if epoch is not None:
            expected = int(epoch)
            if np.any(self.epochs[left] != expected) or np.any(self.epochs[right_index] != expected):
                raise TrajectoryContractError("epoch_mismatch")
        if np.any(self.epochs[left] != self.epochs[right_index]):
            raise TrajectoryContractError("epoch_mismatch")

        spans = self.stamps[right_index] - self.stamps[left]
        if np.any((~exact) & (spans > self.max_bracket_span_ns)):
            raise TrajectoryContractError("bracket_span_exceeds_limit")
        alpha = np.zeros(len(queries), dtype=np.float64)
        interpolated = ~exact
        alpha[interpolated] = (
            (queries[interpolated] - self.stamps[left[interpolated]]) /
            spans[interpolated].astype(np.float64)
        )
        translations = (
            self.translations[left] * (1.0 - alpha[:, None]) +
            self.translations[right_index] * alpha[:, None]
        )

        q0 = self.quaternions[left]
        q1 = self.quaternions[right_index].copy()
        dots = np.sum(q0 * q1, axis=1)
        negative = dots < 0.0
        q1[negative] *= -1.0
        dots[negative] *= -1.0
        dots = np.clip(dots, -1.0, 1.0)
        quaternions = np.empty_like(q0)
        near = dots > 0.9995
        quaternions[near] = (
            q0[near] * (1.0 - alpha[near, None]) +
            q1[near] * alpha[near, None]
        )
        far = ~near
        if np.any(far):
            theta = np.arccos(dots[far])
            sine = np.sin(theta)
            left_weight = np.sin((1.0 - alpha[far]) * theta) / sine
            right_weight = np.sin(alpha[far] * theta) / sine
            quaternions[far] = (
                q0[far] * left_weight[:, None] +
                q1[far] * right_weight[:, None]
            )
        return translations, _normalize_quaternions(quaternions)

    def interpolate(self, stamp_ns, epoch=None):
        translations, quaternions = self.interpolate_many([stamp_ns], epoch)
        return translations[0], quaternions[0]


def deskew_lidar_points_to_map(
    points, point_stamp_ns, trajectory, epoch,
    pose_child_from_scan_translation, pose_child_from_scan_quaternion
):
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] not in (3, 4):
        raise TrajectoryContractError("points must be Nx3 or Nx4")
    stamps = np.asarray(point_stamp_ns, dtype=np.int64).reshape(-1)
    if len(stamps) != len(values):
        raise TrajectoryContractError("point timestamp count mismatch")
    translation, quaternion = trajectory.interpolate_many(stamps, epoch)
    extrinsic_translation = np.asarray(
        pose_child_from_scan_translation, dtype=np.float64
    ).reshape(3)
    extrinsic_quaternion = _normalize_quaternions(
        np.asarray(pose_child_from_scan_quaternion, dtype=np.float64).reshape(1, 4)
    )[0]
    body_points = _rotate_vectors(values[:, :3], extrinsic_quaternion)
    body_points += extrinsic_translation
    map_points = _rotate_vectors(body_points, quaternion) + translation
    output = values.copy()
    output[:, :3] = map_points
    return output


def write_pose_trajectory(path, samples):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_ns", "epoch", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
        for pose in samples:
            writer.writerow([
                int(pose.stamp_ns), int(pose.epoch), *pose.translation,
                *pose.quaternion_xyzw,
            ])


def load_pose_trajectory(path):
    samples = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            samples.append(PoseSample(
                int(row["stamp_ns"]),
                int(row["epoch"]),
                tuple(float(row[name]) for name in ("tx", "ty", "tz")),
                tuple(float(row[name]) for name in ("qx", "qy", "qz", "qw")),
            ))
    return samples
