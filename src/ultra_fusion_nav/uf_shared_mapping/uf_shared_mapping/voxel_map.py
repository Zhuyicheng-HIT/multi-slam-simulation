"""Bounded source-aware voxel map for LiDAR geometry and RGB-D colour."""

from dataclasses import dataclass
import heapq
import json
from pathlib import Path

import numpy as np


@dataclass
class Voxel:
    position: np.ndarray
    color: np.ndarray
    lidar_count: int = 0
    rgbd_count: int = 0
    color_count: int = 0
    color_weight: float = 0.0
    last_stamp_s: float = 0.0


class SourceAwareVoxelMap:
    def __init__(self, voxel_size_m=0.10, conflict_distance_m=0.18,
                 maximum_voxels=500000, minimum_visual_reliability=0.35,
                 occlusion_azimuth_bin_deg=0.5,
                 occlusion_elevation_bin_deg=0.5,
                 occlusion_neighbor_bins=0,
                 occlusion_margin_m=0.40,
                 low_height_max_m=1.5,
                 high_height_min_m=2.5):
        self.voxel_size_m = float(voxel_size_m)
        self.conflict_distance_m = float(conflict_distance_m)
        self.maximum_voxels = int(maximum_voxels)
        self.minimum_visual_reliability = float(minimum_visual_reliability)
        self.occlusion_azimuth_bin_rad = np.deg2rad(
            float(occlusion_azimuth_bin_deg))
        self.occlusion_elevation_bin_rad = np.deg2rad(
            float(occlusion_elevation_bin_deg))
        self.occlusion_neighbor_bins = max(0, int(occlusion_neighbor_bins))
        self.occlusion_margin_m = float(occlusion_margin_m)
        self.low_height_max_m = float(low_height_max_m)
        self.high_height_min_m = float(high_height_min_m)
        if self.voxel_size_m <= 0.0 or self.conflict_distance_m <= 0.0:
            raise ValueError("voxel and conflict distances must be positive")
        if (self.occlusion_azimuth_bin_rad <= 0.0 or
                self.occlusion_elevation_bin_rad <= 0.0 or
                self.occlusion_margin_m < 0.0):
            raise ValueError("occlusion bins must be positive and margin non-negative")
        if self.low_height_max_m >= self.high_height_min_m:
            raise ValueError("low height maximum must be below high height minimum")
        self.voxels = {}
        self.metrics = {
            "lidar_points": 0, "rgbd_points": 0, "rgbd_conflicts": 0,
            "rgbd_supplements": 0, "rgbd_consistent": 0, "evictions": 0,
            "rgbd_occlusion_candidates": 0, "rgbd_occluded": 0,
        }

    def _key(self, point):
        return tuple(
            np.floor(
                np.asarray(point) /
                self.voxel_size_m).astype(
                np.int64))

    def _evict_if_needed(self):
        excess = len(self.voxels) - self.maximum_voxels
        if excess <= 0:
            return
        oldest = heapq.nsmallest(
            excess,
            self.voxels.items(),
            key=lambda item: (item[1].last_stamp_s, item[0]),
        )
        for key, _ in oldest:
            del self.voxels[key]
            self.metrics["evictions"] += 1

    @staticmethod
    def _group_sums(keys, values):
        """Aggregate rows by voxel key without per-point Python updates."""
        unique, inverse, counts = np.unique(
            keys, axis=0, return_inverse=True, return_counts=True)
        sums = np.zeros((len(unique), values.shape[1]), dtype=np.float64)
        np.add.at(sums, inverse, values)
        return unique, inverse, counts, sums

    def _angular_bins(self, points, origin):
        relative = np.asarray(points, dtype=np.float64) - origin
        ranges = np.linalg.norm(relative, axis=1)
        horizontal = np.hypot(relative[:, 0], relative[:, 1])
        azimuth = np.arctan2(relative[:, 1], relative[:, 0])
        elevation = np.arctan2(relative[:, 2], horizontal)
        azimuth_bins = max(
            1, int(np.ceil(2.0 * np.pi / self.occlusion_azimuth_bin_rad)))
        elevation_bins = max(
            1, int(np.ceil(np.pi / self.occlusion_elevation_bin_rad)))
        azimuth_index = np.floor(
            (azimuth + np.pi) / self.occlusion_azimuth_bin_rad
        ).astype(np.int64) % azimuth_bins
        elevation_index = np.clip(
            np.floor(
                (elevation + 0.5 * np.pi) /
                self.occlusion_elevation_bin_rad
            ).astype(np.int64),
            0,
            elevation_bins - 1,
        )
        return (azimuth_index, elevation_index, ranges,
                azimuth_bins, elevation_bins)

    def _occlusion_mask(self, points, sensor_origin, occlusion_points):
        """Reject RGB-D returns hidden behind the latest LiDAR surface.

        Azimuth and elevation are both retained, so an overhead return cannot
        erase or occlude a low obstacle merely because they share an XY ray.
        The check is conservative: only a nearby angular bin with a LiDAR
        return at least ``occlusion_margin_m`` closer is considered blocking.
        """
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        lidar = np.asarray(occlusion_points, dtype=np.float64).reshape(-1, 3)
        origin = np.asarray(sensor_origin, dtype=np.float64).reshape(3)
        lidar = lidar[np.all(np.isfinite(lidar), axis=1)]
        if not len(points) or not len(lidar) or not np.all(np.isfinite(origin)):
            return np.zeros(len(points), dtype=bool)

        laz, lel, lranges, az_bins, el_bins = self._angular_bins(lidar, origin)
        valid_lidar = lranges > 1.0e-6
        laz, lel, lranges = laz[valid_lidar], lel[valid_lidar], lranges[valid_lidar]
        if not len(lranges):
            return np.zeros(len(points), dtype=bool)
        stride = el_bins + 1
        lidar_keys = laz * stride + lel
        unique_keys, inverse = np.unique(lidar_keys, return_inverse=True)
        nearest_lidar = np.full(len(unique_keys), np.inf, dtype=np.float64)
        np.minimum.at(nearest_lidar, inverse, lranges)

        raz, rel, rranges, _, _ = self._angular_bins(points, origin)
        nearest = np.full(len(points), np.inf, dtype=np.float64)
        radius = self.occlusion_neighbor_bins
        for daz in range(-radius, radius + 1):
            query_azimuth = (raz + daz) % az_bins
            for dele in range(-radius, radius + 1):
                query_elevation = rel + dele
                in_bounds = (
                    (query_elevation >= 0) & (query_elevation < el_bins)
                )
                query_keys = query_azimuth * stride + np.clip(
                    query_elevation, 0, el_bins - 1)
                positions = np.searchsorted(unique_keys, query_keys)
                matches = in_bounds & (positions < len(unique_keys))
                matched_rows = np.flatnonzero(matches)
                if len(matched_rows):
                    matched_positions = positions[matched_rows]
                    equal = unique_keys[matched_positions] == query_keys[matched_rows]
                    matched_rows = matched_rows[equal]
                    matched_positions = matched_positions[equal]
                    nearest[matched_rows] = np.minimum(
                        nearest[matched_rows], nearest_lidar[matched_positions])
        return np.isfinite(nearest) & (
            rranges > nearest + self.occlusion_margin_m)

    def integrate_lidar(self, points, stamp_s=0.0):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.all(np.isfinite(points), axis=1)]
        if not len(points):
            return
        keys = np.floor(points / self.voxel_size_m).astype(np.int64)
        unique, _, counts, sums = self._group_sums(keys, points)
        for key_values, count, point_sum in zip(unique, counts, sums):
            key = tuple(int(value) for value in key_values)
            voxel = self.voxels.get(key)
            if voxel is None:
                voxel = Voxel(
                    point_sum / count,
                    np.zeros(3),
                    last_stamp_s=float(stamp_s))
                self.voxels[key] = voxel
            else:
                total = voxel.lidar_count + int(count)
                voxel.position = (
                    voxel.position * voxel.lidar_count + point_sum) / total
                voxel.lidar_count = total
            if voxel.lidar_count == 0:
                voxel.lidar_count = int(count)
            voxel.last_stamp_s = float(stamp_s)
        self.metrics["lidar_points"] += int(len(points))
        self._evict_if_needed()

    def integrate_rgbd(self, points, colors, reliability, stamp_s=0.0,
                       sensor_origin=None, occlusion_points=None):
        reliability = float(reliability)
        if reliability < self.minimum_visual_reliability:
            return 0
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        colors = np.asarray(colors, dtype=float).reshape(-1, 3)
        if len(points) != len(colors):
            raise ValueError("point and color counts must match")
        valid = np.all(np.isfinite(points), axis=1) & np.all(
            np.isfinite(colors), axis=1
        )
        points = points[valid]
        colors = np.clip(colors[valid], 0, 255)
        self.metrics["rgbd_points"] += int(points.shape[0])
        if not len(points):
            return 0

        if sensor_origin is not None and occlusion_points is not None:
            self.metrics["rgbd_occlusion_candidates"] += int(len(points))
            occluded = self._occlusion_mask(
                points, sensor_origin, occlusion_points)
            rejected = int(np.count_nonzero(occluded))
            self.metrics["rgbd_occluded"] += rejected
            self.metrics["rgbd_conflicts"] += rejected
            points, colors = points[~occluded], colors[~occluded]
            if not len(points):
                return 0

        keys = np.floor(points / self.voxel_size_m).astype(np.int64)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        lidar_positions = np.zeros((len(unique), 3), dtype=np.float64)
        has_lidar = np.zeros(len(unique), dtype=bool)
        for index, key_values in enumerate(unique):
            key = tuple(int(value) for value in key_values)
            voxel = self.voxels.get(key)
            if voxel is not None and voxel.lidar_count:
                has_lidar[index] = True
                lidar_positions[index] = voxel.position
        exact_lidar = has_lidar[inverse]
        deltas = points - lidar_positions[inverse]
        exact_conflict = exact_lidar & (
            np.einsum("ij,ij->i", deltas, deltas) >
            self.conflict_distance_m * self.conflict_distance_m
        )
        self.metrics["rgbd_conflicts"] += int(np.count_nonzero(exact_conflict))
        points, colors, keys = (
            points[~exact_conflict], colors[~exact_conflict], keys[~exact_conflict]
        )
        if not len(points):
            return 0

        unique, grouped_inverse, counts, point_sums = self._group_sums(
            keys, points)
        color_sums = np.zeros((len(unique), 3), dtype=np.float64)
        np.add.at(color_sums, grouped_inverse, colors)
        for key_values, count, point_sum, color_sum in zip(
                unique, counts, point_sums, color_sums):
            key = tuple(int(value) for value in key_values)
            voxel = self.voxels.get(key)
            if voxel is None:
                voxel = Voxel(
                    point_sum / count,
                    np.zeros(3),
                    last_stamp_s=float(stamp_s))
                self.voxels[key] = voxel
                self.metrics["rgbd_supplements"] += 1
            elif voxel.lidar_count:
                self.metrics["rgbd_consistent"] += int(count)
            total = voxel.rgbd_count + int(count)
            if voxel.lidar_count == 0:
                voxel.position = (
                    voxel.position * voxel.rgbd_count + point_sum) / total
            voxel.rgbd_count = total
            batch_weight = max(0.05, reliability) * int(count)
            batch_color = color_sum / count
            previous_weight = voxel.color_weight
            voxel.color = (
                voxel.color * previous_weight + batch_color * batch_weight
            ) / (previous_weight + batch_weight)
            voxel.color_weight += batch_weight
            voxel.color_count += int(count)
            voxel.last_stamp_s = float(stamp_s)
        self._evict_if_needed()
        return int(len(points))

    def arrays(self, source="joint"):
        selected = []
        for voxel in self.voxels.values():
            if source == "lidar" and not voxel.lidar_count:
                continue
            if source == "rgbd" and not voxel.rgbd_count:
                continue
            selected.append(voxel)
        if not selected:
            return np.empty((0, 3)), np.empty((0, 3), np.uint8)
        return (
            np.vstack([voxel.position for voxel in selected]),
            np.vstack([voxel.color for voxel in selected]).astype(np.uint8),
        )

    def summary(self):
        lidar = sum(voxel.lidar_count > 0 for voxel in self.voxels.values())
        rgbd = sum(voxel.rgbd_count > 0 for voxel in self.voxels.values())
        joint = sum(voxel.lidar_count > 0 and voxel.rgbd_count > 0
                    for voxel in self.voxels.values())
        lidar_only = sum(voxel.lidar_count > 0 and not voxel.rgbd_count
                         for voxel in self.voxels.values())
        supplementary = sum(not voxel.lidar_count and voxel.rgbd_count > 0
                            for voxel in self.voxels.values())
        low = int(sum(
            voxel.position[2] <= self.low_height_max_m
            for voxel in self.voxels.values()))
        high = int(sum(
            voxel.position[2] >= self.high_height_min_m
            for voxel in self.voxels.values()))
        middle = len(self.voxels) - low - high
        return {
            **self.metrics,
            "voxel_count": len(self.voxels), "lidar_voxels": lidar,
            "rgbd_voxels": rgbd, "joint_voxels": joint,
            "lidar_only_voxels": lidar_only,
            "supplementary_rgbd_voxels": supplementary,
            "low_height_voxels": low,
            "middle_height_voxels": middle,
            "high_height_voxels": high,
            "color_coverage_ratio": joint / max(1, lidar),
            "volume_growth_ratio": supplementary / max(1, lidar),
            "conflict_ratio": self.metrics["rgbd_conflicts"] / max(1, self.metrics["rgbd_points"]),
            "occlusion_rejection_ratio": self.metrics["rgbd_occluded"] /
            max(1, self.metrics["rgbd_occlusion_candidates"]),
        }


def write_ascii_pcd(path, points, colors):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points).reshape(-1, 3)
    colors = np.asarray(colors).reshape(-1, 3)
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\n")
        stream.write("SIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
        stream.write(
            f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n")
        for point, color in zip(points, colors):
            rgb = (int(color[0]) << 16) | (int(color[1]) << 8) | int(color[2])
            stream.write(
                f"{point[0]:.7g} {point[1]:.7g} {point[2]:.7g} {rgb}\n")


def write_summary(path, summary):
    Path(path).write_text(
        json.dumps(
            summary,
            indent=2,
            sort_keys=True) +
        "\n",
        encoding="utf-8")
