"""Evidence-preserving offline voxel aggregation and conservative cleanup."""

from dataclasses import dataclass, field
import itertools

import numpy as np


@dataclass(frozen=True)
class MaintenanceConfig:
    voxel_size_m: float = 0.12
    minimum_scan_support: int = 2
    stable_support_scans: int = 4
    minimum_component_voxels: int = 3
    isolation_neighbor_threshold: int = 1
    maximum_provenance_scan_ids: int = 8
    maximum_pose_bracket_ns: int = 250_000_000
    ghosting_voxel_size_m: float = 0.06
    structural_sample_limit: int = 20000


@dataclass
class VoxelEvidence:
    point_sum: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float64))
    point_count: int = 0
    scan_ids: set = field(default_factory=set)
    scan_support_count: int = 0
    first_scan_id: int = None
    last_scan_id: int = None

    @property
    def scan_support(self):
        return self.scan_support_count

    @property
    def centroid(self):
        return self.point_sum / max(1, self.point_count)


class EvidenceVoxelMap:
    def __init__(self, config=None):
        self.config = config or MaintenanceConfig()
        if self.config.voxel_size_m <= 0.0:
            raise ValueError("voxel_size_m must be positive")
        self.voxels = {}
        self.last_decisions = {}

    def _key(self, point):
        return tuple(np.floor(point[:3] / self.config.voxel_size_m).astype(np.int64))

    def add_scan(self, scan_id, points):
        array = np.asarray(points, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] not in (3, 4):
            raise ValueError("points must be an Nx3 or Nx4 array")
        finite = array[np.all(np.isfinite(array[:, :3]), axis=1)]
        if not len(finite):
            return
        values = np.zeros((len(finite), 4), dtype=np.float64)
        values[:, :3] = finite[:, :3]
        if finite.shape[1] == 4:
            valid_intensity = np.isfinite(finite[:, 3])
            values[valid_intensity, 3] = finite[valid_intensity, 3]
        keys = np.floor(
            finite[:, :3] / self.config.voxel_size_m
        ).astype(np.int64)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        counts = np.bincount(inverse, minlength=len(unique_keys))
        sums = np.column_stack([
            np.bincount(inverse, weights=values[:, column], minlength=len(unique_keys))
            for column in range(4)
        ])
        for index, key_values in enumerate(unique_keys):
            key = tuple(int(value) for value in key_values)
            evidence = self.voxels.setdefault(key, VoxelEvidence())
            evidence.point_sum += sums[index]
            evidence.point_count += int(counts[index])
            if evidence.last_scan_id != scan_id:
                evidence.scan_support_count += 1
                if len(evidence.scan_ids) < self.config.maximum_provenance_scan_ids:
                    evidence.scan_ids.add(int(scan_id))
            evidence.first_scan_id = scan_id if evidence.first_scan_id is None else min(evidence.first_scan_id, scan_id)
            evidence.last_scan_id = scan_id if evidence.last_scan_id is None else max(evidence.last_scan_id, scan_id)

    def all_points(self):
        ordered = sorted(self.voxels)
        if not ordered:
            return np.empty((0, 4), dtype=np.float64)
        return np.asarray([self.voxels[key].centroid for key in ordered], dtype=np.float64)

    @staticmethod
    def _neighbor_keys(key):
        for delta in itertools.product((-1, 0, 1), repeat=3):
            if delta != (0, 0, 0):
                yield tuple(key[index] + delta[index] for index in range(3))

    def _components(self, keys):
        remaining = set(keys)
        while remaining:
            seed = remaining.pop()
            component = {seed}
            frontier = [seed]
            while frontier:
                current = frontier.pop()
                for neighbor in self._neighbor_keys(current):
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        component.add(neighbor)
                        frontier.append(neighbor)
            yield component

    def cleaned_points(self):
        metrics = {
            "input_voxels": len(self.voxels),
            "removed_low_support": 0,
            "removed_isolated": 0,
            "removed_small_component": 0,
            "static_voxels": 0,
        }
        admitted = set()
        decisions = {}
        for key, evidence in self.voxels.items():
            if evidence.scan_support < self.config.minimum_scan_support:
                metrics["removed_low_support"] += 1
                decisions[key] = "removed_low_support"
            else:
                admitted.add(key)
                decisions[key] = "candidate"

        neighbor_counts = {
            key: sum(neighbor in admitted for neighbor in self._neighbor_keys(key))
            for key in admitted
        }
        for key in list(admitted):
            evidence = self.voxels[key]
            isolated = (
                neighbor_counts[key] < self.config.isolation_neighbor_threshold
            )
            not_stable = (
                evidence.scan_support < self.config.stable_support_scans
            )
            if isolated and not_stable:
                admitted.remove(key)
                metrics["removed_isolated"] += 1
                decisions[key] = "removed_isolated"

        for component in list(self._components(admitted)):
            stable = any(
                self.voxels[key].scan_support >= self.config.stable_support_scans
                for key in component
            )
            if len(component) < self.config.minimum_component_voxels and not stable:
                for key in component:
                    admitted.remove(key)
                    decisions[key] = "removed_small_component"
                metrics["removed_small_component"] += len(component)

        ordered = sorted(admitted)
        output = np.array([self.voxels[key].centroid for key in ordered], dtype=np.float64)
        if not ordered:
            output = np.empty((0, 4), dtype=np.float64)
        metrics["static_voxels"] = len(ordered)
        metrics["retained_voxel_ratio"] = (
            len(ordered) / len(self.voxels) if self.voxels else 1.0
        )
        for key in ordered:
            decisions[key] = "static_retained"
        self.last_decisions = decisions
        return output, metrics
