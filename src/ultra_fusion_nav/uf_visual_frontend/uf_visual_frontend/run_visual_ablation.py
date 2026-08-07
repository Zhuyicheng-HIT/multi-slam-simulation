"""Deterministic factor-level A/B harness; does not replace flight validation."""

import argparse
import json
from pathlib import Path
import time

import numpy as np

from uf_backend_fusion.manifold import STATE_SIZE, so3_log
from uf_backend_fusion.manifold_window import ManifoldSlidingWindowBackend
from uf_backend_fusion.native_lidar import rpy_to_rotation_matrix
from uf_backend_fusion.visual_reprojection import (
    VisualTrackBatch, visual_reprojection_residual_jacobians,
)
from uf_shared_mapping.voxel_map import SourceAwareVoxelMap


def trajectory(count=8):
    states = []
    for index in range(count):
        state = np.zeros(STATE_SIZE)
        state[:3] = [0.25 * index, 0.12 * np.sin(0.5 * index), 1.5]
        state[5] = 0.035 * index
        states.append(state)
    return states


def consistent_tracks(previous, current, rng, count=80):
    anchors = rng.uniform([-0.45, -0.32], [0.45, 0.32], size=(count, 2))
    depth = rng.uniform(2.0, 6.0, size=count)
    seed = VisualTrackBatch(anchors, anchors, 1.0 / depth, 4.0e-6)
    residual, _, _, valid = visual_reprojection_residual_jacobians(
        previous, current, seed
    )
    anchors = anchors[valid]
    depth = depth[valid]
    current_pixels = anchors + residual.reshape(-1, 2)
    current_pixels += rng.normal(0.0, 4.0e-4, current_pixels.shape)
    return VisualTrackBatch(anchors, current_pixels, 1.0 / depth, 4.0e-6)


def solve(mode, seed):
    rng = np.random.default_rng(seed)
    truth = trajectory()
    backend = ManifoldSlidingWindowBackend(max_states=12, max_iterations=10)
    indices = []
    started = time.perf_counter()
    for state in truth:
        initial = state.copy()
        initial[:3] += rng.normal(0.0, 0.07, 3)
        initial[3:6] += rng.normal(0.0, 0.02, 3)
        indices.append(backend.add_state(initial))
    backend.add_prior(indices[0], truth[0], covariance=1.0e-7)
    for index, state in enumerate(truth):
        lidar_position = state[:3] + rng.normal(0.0, 0.035, 3)
        lidar_rotation = state[3:6] + rng.normal(0.0, 0.012, 3)
        backend.add_lidar_pose(
            indices[index],
            lidar_position,
            lidar_rotation,
            covariance=np.r_[
                np.full(
                    3,
                    0.08 ** 2),
                np.full(
                    3,
                    0.04 ** 2)])
        if index % 3 == 0:
            backend.add_gnss(
                indices[index], state[:3] + rng.normal(0.0, 0.08, 3),
                covariance=0.16 ** 2,
            )
        if index == 0:
            continue
        delta_world = truth[index][:3] - truth[index - 1][:3]
        backend.add_optical_flow(
            indices[index - 1], indices[index],
            delta_world + rng.normal(0.0, 0.012, 3),
            covariance=0.05 ** 2)
        if mode == "legacy_rtab_relative":
            previous_rotation = rpy_to_rotation_matrix(truth[index - 1][3:6])
            delta_body = previous_rotation.T @ delta_world
            relative_rotation = so3_log(
                previous_rotation.T @ rpy_to_rotation_matrix(truth[index][3:6])
            )
            # Reproducible small drift represents a frame-to-frame odometry
            # baseline.
            backend.add_legacy_visual_odometry(
                indices[index - 1], indices[index],
                delta_body + [0.0015, -0.0008, 0.0], relative_rotation,
                covariance=np.r_[np.full(3, 0.015 ** 2), np.full(3, 0.01 ** 2)],
            )
        elif mode == "paper_reprojection":
            backend.add_visual_reprojection(
                indices[index - 1], indices[index],
                consistent_tracks(truth[index - 1], truth[index], rng),
            )
    backend.optimize()
    elapsed = time.perf_counter() - started
    estimate = np.vstack(backend.states())
    reference = np.vstack(truth)
    translation = np.linalg.norm(estimate[:, :3] - reference[:, :3], axis=1)
    rotation = np.linalg.norm(estimate[:, 3:6] - reference[:, 3:6], axis=1)
    return {
        "mode": mode, "seed": seed,
        "translation_rmse_m": float(np.sqrt(np.mean(translation ** 2))),
        "rotation_rmse_rad": float(np.sqrt(np.mean(rotation ** 2))),
        "final_translation_error_m": float(translation[-1]),
        "initial_cost": backend.last_initial_cost,
        "final_cost": backend.last_cost,
        "solve_time_s": elapsed,
        "solver_rejected_steps": backend.last_rejected_steps,
    }


def mapping_ablation(seed):
    rng = np.random.default_rng(seed)
    mapping = SourceAwareVoxelMap(voxel_size_m=0.2, conflict_distance_m=0.25)
    x, y = np.meshgrid(np.linspace(-2, 2, 30), np.linspace(-2, 2, 30))
    lidar = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    mapping.integrate_lidar(lidar)
    rgbd = np.vstack((lidar + rng.normal(0.0, 0.015, lidar.shape),
                      np.c_[np.full(150, 2.8), rng.uniform(-2, 2, 150), rng.uniform(0, 2, 150)]))
    colors = np.tile([120, 180, 220], (len(rgbd), 1))
    mapping.integrate_rgbd(rgbd, colors, 0.9)
    return mapping.summary()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="visual_ablation.json")
    parser.add_argument("--runs", type=int, default=3)
    args = parser.parse_args()
    results = []
    for seed in range(args.runs):
        for mode in (
            "four_source",
            "legacy_rtab_relative",
                "paper_reprojection"):
            results.append(solve(mode, seed))
    document = {
        "scope": "deterministic factor-level ablation; not flight evidence",
        "runs": results,
        "paper_reprojection_plus_shared_map": [
            mapping_ablation(seed) for seed in range(
                args.runs)],
    }
    Path(
        args.output).write_text(
        json.dumps(
            document,
            indent=2) +
        "\n",
        encoding="utf-8")
    print(json.dumps(document, indent=2))


if __name__ == "__main__":
    main()
