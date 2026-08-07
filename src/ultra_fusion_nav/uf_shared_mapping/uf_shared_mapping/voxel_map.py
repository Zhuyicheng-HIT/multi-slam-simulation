"""Bounded source-aware voxel map for LiDAR geometry and RGB-D colour."""

from dataclasses import dataclass
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
    last_stamp_s: float = 0.0


class SourceAwareVoxelMap:
    def __init__(self, voxel_size_m=0.10, conflict_distance_m=0.18,
                 maximum_voxels=500000, minimum_visual_reliability=0.35):
        self.voxel_size_m = float(voxel_size_m)
        self.conflict_distance_m = float(conflict_distance_m)
        self.maximum_voxels = int(maximum_voxels)
        self.minimum_visual_reliability = float(minimum_visual_reliability)
        if self.voxel_size_m <= 0.0 or self.conflict_distance_m <= 0.0:
            raise ValueError("voxel and conflict distances must be positive")
        self.voxels = {}
        self.metrics = {
            "lidar_points": 0, "rgbd_points": 0, "rgbd_conflicts": 0,
            "rgbd_supplements": 0, "rgbd_consistent": 0, "evictions": 0,
        }

    def _key(self, point):
        return tuple(
            np.floor(
                np.asarray(point) /
                self.voxel_size_m).astype(
                np.int64))

    def _evict_if_needed(self):
        while len(self.voxels) > self.maximum_voxels:
            key = min(
                self.voxels,
                key=lambda item: self.voxels[item].last_stamp_s)
            del self.voxels[key]
            self.metrics["evictions"] += 1

    def integrate_lidar(self, points, stamp_s=0.0):
        for point in np.asarray(points, dtype=float).reshape(-1, 3):
            if not np.all(np.isfinite(point)):
                continue
            key = self._key(point)
            voxel = self.voxels.get(key)
            if voxel is None:
                voxel = Voxel(
                    point.copy(),
                    np.zeros(3),
                    last_stamp_s=float(stamp_s))
                self.voxels[key] = voxel
            total = voxel.lidar_count + 1
            voxel.position = (
                voxel.position * voxel.lidar_count + point) / total
            voxel.lidar_count = total
            voxel.last_stamp_s = float(stamp_s)
            self.metrics["lidar_points"] += 1
        self._evict_if_needed()

    def integrate_rgbd(self, points, colors, reliability, stamp_s=0.0):
        reliability = float(reliability)
        if reliability < self.minimum_visual_reliability:
            return 0
        accepted = 0
        for point, color in zip(
                np.asarray(points, dtype=float).reshape(-1, 3),
                np.asarray(colors, dtype=float).reshape(-1, 3)):
            if not np.all(
                    np.isfinite(point)) or not np.all(
                    np.isfinite(color)):
                continue
            self.metrics["rgbd_points"] += 1
            key = self._key(point)
            voxel = self.voxels.get(key)
            if voxel is not None and voxel.lidar_count > 0:
                if np.linalg.norm(
                        point - voxel.position) > self.conflict_distance_m:
                    self.metrics["rgbd_conflicts"] += 1
                    continue
                self.metrics["rgbd_consistent"] += 1
            elif voxel is None:
                voxel = Voxel(
                    point.copy(),
                    np.zeros(3),
                    last_stamp_s=float(stamp_s))
                self.voxels[key] = voxel
                self.metrics["rgbd_supplements"] += 1
            total = voxel.rgbd_count + 1
            if voxel.lidar_count == 0:
                voxel.position = (
                    voxel.position * voxel.rgbd_count + point) / total
            voxel.rgbd_count = total
            color_weight = max(0.05, reliability)
            previous_weight = float(voxel.color_count)
            voxel.color = (
                voxel.color * previous_weight + np.clip(color, 0, 255) * color_weight
            ) / (previous_weight + color_weight)
            voxel.color_count += 1
            voxel.last_stamp_s = float(stamp_s)
            accepted += 1
        self._evict_if_needed()
        return accepted

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
        return {
            **self.metrics,
            "voxel_count": len(self.voxels), "lidar_voxels": lidar,
            "rgbd_voxels": rgbd, "joint_voxels": joint,
            "lidar_only_voxels": lidar_only,
            "supplementary_rgbd_voxels": supplementary,
            "color_coverage_ratio": joint / max(1, lidar),
            "volume_growth_ratio": supplementary / max(1, lidar),
            "conflict_ratio": self.metrics["rgbd_conflicts"] / max(1, self.metrics["rgbd_points"]),
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
