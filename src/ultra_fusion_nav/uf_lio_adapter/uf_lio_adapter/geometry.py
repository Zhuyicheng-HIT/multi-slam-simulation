import math
import struct
from collections import deque

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def cloud_xyz(msg, max_points=None):
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"}.issubset(fields) or msg.point_step <= 0:
        return np.empty((0, 3), dtype=np.float64)
    formats = {PointField.FLOAT32: "f", PointField.FLOAT64: "d"}
    if any(fields[name].datatype not in formats for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)
    count = min(int(msg.width) * int(msg.height), len(msg.data) // int(msg.point_step))
    stride = max(1, math.ceil(count / max_points)) if max_points else 1
    prefix = ">" if msg.is_bigendian else "<"
    points = []
    for index in range(0, count, stride):
        base = index * int(msg.point_step)
        point = []
        for name in ("x", "y", "z"):
            field = fields[name]
            point.append(struct.unpack_from(prefix + formats[field.datatype], msg.data, base + field.offset)[0])
        if all(math.isfinite(value) for value in point):
            points.append(point)
    return np.asarray(points, dtype=np.float64)


def voxel_centroids(points, voxel_size):
    accumulators = {}
    for point in points:
        key = tuple(np.floor(point / voxel_size).astype(np.int64))
        if key not in accumulators:
            accumulators[key] = [point.copy(), 1]
        else:
            accumulators[key][0] += point
            accumulators[key][1] += 1
    return {key: total / count for key, (total, count) in accumulators.items()}


class TemporalVoxelFilter:
    """Classify registered-cloud voxels by persistence in a fixed world frame."""

    def __init__(self, window_frames=5, min_static_support=2, neighbor_radius=1):
        if window_frames < 1:
            raise ValueError("window_frames must be positive")
        if min_static_support < 1:
            raise ValueError("min_static_support must be positive")
        if neighbor_radius < 0:
            raise ValueError("neighbor_radius must be non-negative")
        self.window_frames = int(window_frames)
        self.min_static_support = int(min_static_support)
        self.neighbor_radius = int(neighbor_radius)
        self.history = deque(maxlen=self.window_frames)

    def _supported(self, key, frame):
        radius = self.neighbor_radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    if (key[0] + dx, key[1] + dy, key[2] + dz) in frame:
                        return True
        return False

    @staticmethod
    def _array(values):
        if not values:
            return np.empty((0, 3), dtype=np.float64)
        return np.asarray(values, dtype=np.float64).reshape((-1, 3))

    def classify(self, points, voxel_size):
        centroids = voxel_centroids(points, voxel_size) if len(points) else {}
        current_keys = set(centroids)
        previous = self.history[-1] if self.history else set()
        window_warm = len(self.history) >= self.window_frames
        static = []
        dynamic = []
        uncertain = []
        repeatable = 0

        for key, centroid in centroids.items():
            support = sum(self._supported(key, frame) for frame in self.history)
            if previous and self._supported(key, previous):
                repeatable += 1
            if support >= self.min_static_support:
                static.append(centroid)
            elif window_warm and support == 0:
                dynamic.append(centroid)
            else:
                uncertain.append(centroid)

        self.history.append(current_keys)
        return {
            "static_points": self._array(static),
            "dynamic_points": self._array(dynamic),
            "uncertain_points": self._array(uncertain),
            "feature_repeatability": repeatable / max(1, len(current_keys)),
            "window_warm": window_warm,
            "input_voxels": len(current_keys),
        }


def voxel_buckets(points, voxel_size):
    buckets = {}
    for point in points:
        key = tuple(np.floor(point / voxel_size).astype(np.int64))
        buckets.setdefault(key, []).append(point)
    return {key: np.asarray(values) for key, values in buckets.items()}


def local_plane(key, buckets):
    neighbors = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                points = buckets.get((key[0] + dx, key[1] + dy, key[2] + dz))
                if points is not None:
                    neighbors.append(points)
    if not neighbors:
        return None
    points = np.concatenate(neighbors, axis=0)
    if len(points) < 6:
        return None
    centroid = np.mean(points, axis=0)
    covariance = (points - centroid).T @ (points - centroid) / len(points)
    values, vectors = np.linalg.eigh(covariance)
    if values[-1] <= 1.0e-12 or values[0] / values[-1] > 0.12:
        return None
    return centroid, vectors[:, 0]


def _skew(point):
    x, y, z = point
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def geometry_diagnostics(current, previous, voxel_size=0.5):
    empty = {
        "matched_points": 0,
        "residual_mean_m": float("nan"),
        "residual_median_m": float("nan"),
        "residual_p95_m": float("nan"),
        "hessian_eigenvalues": np.zeros(6),
        "hessian_condition": float("inf"),
        "normal_covariance_eigenvalues": np.zeros(3),
        "axial_penalty": 1.0,
        "spatial_coverage": 0.0,
        "map_quality": 0.0,
    }
    if current is None or previous is None or len(current) == 0 or len(previous) == 0:
        return empty
    buckets = voxel_buckets(previous, voxel_size)
    planes = {}
    residuals = []
    matched = []
    normals = []
    hessian = np.zeros((6, 6), dtype=np.float64)
    for point in current:
        base = np.floor(point / voxel_size).astype(np.int64)
        best_plane = None
        best_distance = float("inf")
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    key = (base[0] + dx, base[1] + dy, base[2] + dz)
                    if key not in buckets:
                        continue
                    if key not in planes:
                        planes[key] = local_plane(key, buckets)
                    plane = planes[key]
                    if plane is None:
                        continue
                    centroid, normal = plane
                    distance = abs(float(normal @ (point - centroid)))
                    if distance < best_distance:
                        best_plane = plane
                        best_distance = distance
        if best_plane is None or best_distance > voxel_size:
            continue
        _, normal = best_plane
        residuals.append(best_distance)
        matched.append(point)
        normals.append(normal)
        jacobian = np.concatenate((normal, -normal @ _skew(point)))
        hessian += np.outer(jacobian, jacobian)
    if not residuals:
        return empty
    hessian += 1.0e-8 * np.eye(6)
    eigenvalues = np.maximum(np.linalg.eigvalsh(hessian), 0.0)
    condition = float(eigenvalues[-1] / max(eigenvalues[0], 1.0e-9))
    normals_array = np.asarray(normals)
    normal_covariance = normals_array.T @ normals_array / len(normals_array)
    normal_eigenvalues = np.maximum(np.linalg.eigvalsh(normal_covariance), 0.0)
    support_floor = max(1.0e-12, 0.05 * eigenvalues[-1])
    axial_penalty = float(np.mean(1.0 - np.minimum(1.0, eigenvalues / support_floor)))
    matched_array = np.asarray(matched)
    centered = matched_array - np.median(matched_array, axis=0)
    octants = {(point[0] >= 0.0, point[1] >= 0.0, point[2] >= 0.0) for point in centered}
    coverage = len(octants) / 8.0
    residuals = np.asarray(residuals)
    match_ratio = len(residuals) / max(1, len(current))
    quality = float(np.clip(match_ratio * np.exp(-np.percentile(residuals, 95) / voxel_size), 0.0, 1.0))
    return {
        "matched_points": len(residuals),
        "residual_mean_m": float(np.mean(residuals)),
        "residual_median_m": float(np.median(residuals)),
        "residual_p95_m": float(np.percentile(residuals, 95)),
        "hessian_eigenvalues": eigenvalues,
        "hessian_condition": condition,
        "normal_covariance_eigenvalues": normal_eigenvalues,
        "axial_penalty": axial_penalty,
        "spatial_coverage": coverage,
        "map_quality": quality,
    }


def xyz_cloud(points, header):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = False
    msg.data = b"".join(struct.pack("<fff", *point) for point in np.asarray(points, dtype=np.float32))
    return msg
