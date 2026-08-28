"""Conservative geometry-retention metrics for offline voxel cleanup."""

import numpy as np


def structural_retention_metrics(voxel_map):
    keys = sorted(voxel_map.voxels)
    if not keys:
        return {
            "sampled_voxels": 0,
            "ground_candidates": 0,
            "wall_candidates": 0,
            "ground_retention_ratio": 1.0,
            "wall_retention_ratio": 1.0,
            "stable_ground_candidates": 0,
            "stable_wall_candidates": 0,
            "stable_ground_retention_ratio": 1.0,
            "stable_wall_retention_ratio": 1.0,
        }
    limit = max(1, int(voxel_map.config.structural_sample_limit))
    stride = max(1, int(np.ceil(len(keys) / limit)))
    sampled = keys[::stride]
    ground = []
    wall = []
    for key in sampled:
        neighborhood_keys = [key] + [
            candidate for candidate in voxel_map._neighbor_keys(key)
            if candidate in voxel_map.voxels
        ]
        if len(neighborhood_keys) < 5:
            continue
        points = np.asarray(
            [voxel_map.voxels[candidate].centroid[:3] for candidate in neighborhood_keys]
        )
        covariance = np.cov(points, rowvar=False, bias=True)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 1.0e-12:
            continue
        planarity = (eigenvalues[1] - eigenvalues[0]) / eigenvalues[-1]
        if planarity < 0.35:
            continue
        normal = eigenvectors[:, 0]
        vertical = abs(float(normal[2]))
        if vertical >= 0.80:
            ground.append(key)
        elif vertical <= 0.30:
            wall.append(key)

    def retained_ratio(group):
        if not group:
            return 1.0
        retained = sum(
            voxel_map.last_decisions.get(key) == "static_retained" for key in group
        )
        return retained / len(group)

    stable_ground = [
        key for key in ground
        if voxel_map.voxels[key].scan_support >= voxel_map.config.stable_support_scans
    ]
    stable_wall = [
        key for key in wall
        if voxel_map.voxels[key].scan_support >= voxel_map.config.stable_support_scans
    ]

    return {
        "sampled_voxels": len(sampled),
        "ground_candidates": len(ground),
        "wall_candidates": len(wall),
        "ground_retention_ratio": retained_ratio(ground),
        "wall_retention_ratio": retained_ratio(wall),
        "stable_ground_candidates": len(stable_ground),
        "stable_wall_candidates": len(stable_wall),
        "stable_ground_retention_ratio": retained_ratio(stable_ground),
        "stable_wall_retention_ratio": retained_ratio(stable_wall),
    }
