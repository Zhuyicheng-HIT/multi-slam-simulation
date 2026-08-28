"""Loop-consistency and geometry-change metrics without external truth."""

import numpy as np
from scipy.spatial import cKDTree

from .optimizer import edge_error


def _residual_summary(poses, edges, family, phase):
    errors = [edge_error(edge, poses) for edge in edges]
    translation = np.asarray([np.linalg.norm(value[:3]) for value in errors])
    rotation = np.asarray([np.linalg.norm(value[3:]) for value in errors])
    return {
        f"{family}_translation_rmse_{phase}_m": float(
            np.sqrt(
                np.mean(
                    translation ** 2))) if len(translation) else 0.0,
        f"{family}_translation_max_{phase}_m": float(
            np.max(translation)) if len(translation) else 0.0,
        f"{family}_rotation_rmse_{phase}_rad": float(
                        np.sqrt(
                            np.mean(
                                rotation ** 2))) if len(rotation) else 0.0,
        f"{family}_rotation_max_{phase}_rad": float(
                                    np.max(rotation)) if len(rotation) else 0.0,
    }


def graph_residual_metrics(
        original_poses, corrected_poses, sequential_edges, loop_edges):
    metrics = {}
    metrics.update(
        _residual_summary(
            original_poses,
            sequential_edges,
            "sequential",
            "before"))
    metrics.update(
        _residual_summary(
            corrected_poses,
            sequential_edges,
            "sequential",
            "after"))
    metrics.update(
        _residual_summary(
            original_poses,
            loop_edges,
            "loop",
            "before"))
    metrics.update(
        _residual_summary(
            corrected_poses,
            loop_edges,
            "loop",
            "after"))
    if loop_edges:
        closure = max(
            loop_edges,
            key=lambda edge: (
                abs(edge.target_id - edge.source_id), -edge.audit_id),
        )
        before = edge_error(closure, original_poses)
        after = edge_error(closure, corrected_poses)
        metrics.update({
            "closure_source_id": int(closure.source_id),
            "closure_target_id": int(closure.target_id),
            "start_end_closure_translation_before_m": float(np.linalg.norm(before[:3])),
            "start_end_closure_translation_after_m": float(np.linalg.norm(after[:3])),
            "start_end_closure_rotation_before_rad": float(np.linalg.norm(before[3:])),
            "start_end_closure_rotation_after_rad": float(np.linalg.norm(after[3:])),
        })
    return metrics


def _sample(points, maximum=5000):
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < 3 or not np.all(
            np.isfinite(values[:, :3])):
        raise ValueError("point cloud must contain finite XYZ")
    values = values[:, :3]
    if len(values) <= maximum:
        return values
    indices = np.linspace(0, len(values) - 1, maximum, dtype=np.int64)
    return values[indices]


def _surface_thickness(points):
    sampled = _sample(points)
    if len(sampled) < 8:
        return np.zeros(len(sampled))
    tree = cKDTree(np.asarray(points, dtype=np.float64)[:, :3])
    _, indices = tree.query(sampled, k=min(16, len(points)))
    thickness = []
    for neighborhood in indices:
        local = np.asarray(points, dtype=np.float64)[neighborhood, :3]
        eigenvalues = np.linalg.eigvalsh(
            np.cov(local, rowvar=False, bias=True))
        thickness.append(np.sqrt(max(0.0, float(eigenvalues[0]))))
    return np.asarray(thickness)


def _voxel_set(points, voxel_size_m):
    keys = np.floor(
        np.asarray(
            points,
            dtype=np.float64)[
            :,
            :3] /
        voxel_size_m).astype(
        np.int64)
    return {tuple(row) for row in keys}


def compare_point_clouds(before_points, after_points, voxel_size_m=0.15):
    if voxel_size_m <= 0.0:
        raise ValueError("voxel_size_m must be positive")
    before = np.asarray(before_points, dtype=np.float64)
    after = np.asarray(after_points, dtype=np.float64)
    before_sample = _sample(before)
    after_sample = _sample(after)
    before_to_after = cKDTree(after[:, :3]).query(before_sample)[0]
    after_to_before = cKDTree(before[:, :3]).query(after_sample)[0]
    symmetric = np.concatenate((before_to_after, after_to_before))
    before_voxels = _voxel_set(before, voxel_size_m)
    after_voxels = _voxel_set(after, voxel_size_m)
    union = before_voxels | after_voxels
    before_thickness = _surface_thickness(before)
    after_thickness = _surface_thickness(after)
    return {
        "before_points": len(before),
        "after_points": len(after),
        "symmetric_nn_p50_m": float(np.percentile(symmetric, 50)),
        "symmetric_nn_p95_m": float(np.percentile(symmetric, 95)),
        "symmetric_nn_p99_m": float(np.percentile(symmetric, 99)),
        "voxel_jaccard": len(before_voxels & after_voxels) / max(1, len(union)),
        "surface_thickness_p50_before_m": float(np.percentile(before_thickness, 50)),
        "surface_thickness_p95_before_m": float(np.percentile(before_thickness, 95)),
        "surface_thickness_p50_after_m": float(np.percentile(after_thickness, 50)),
        "surface_thickness_p95_after_m": float(np.percentile(after_thickness, 95)),
    }
