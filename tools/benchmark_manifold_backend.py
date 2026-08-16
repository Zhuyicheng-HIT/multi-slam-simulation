#!/usr/bin/env python3
"""Deterministic microbenchmark for the five-source manifold backend."""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np


def load_backend(source_root):
    package = Path(source_root) / "src/ultra_fusion_nav/uf_backend_fusion"
    sys.path.insert(0, str(package))
    from uf_backend_fusion.imu_preintegration import (  # noqa: PLC0415
        ImuSample,
        preintegrate_manifold,
    )
    from uf_backend_fusion.manifold_window import (  # noqa: PLC0415
        ManifoldSlidingWindowBackend,
    )
    from uf_backend_fusion.native_lidar import (  # noqa: PLC0415
        NativeLidarPoseNormal,
    )
    from uf_backend_fusion.visual_reprojection import (  # noqa: PLC0415
        VisualTrackBatch,
        visual_reprojection_residual_jacobians,
    )
    return {
        "Backend": ManifoldSlidingWindowBackend,
        "ImuSample": ImuSample,
        "preintegrate": preintegrate_manifold,
        "LidarFactor": NativeLidarPoseNormal,
        "VisualTracks": VisualTrackBatch,
        "visual_linearize": visual_reprojection_residual_jacobians,
    }


def build_graph(
    api,
    state_count=8,
    lidar_points=256,
    visual_tracks=48,
    cpp_math_core_enabled=True,
):
    backend = api["Backend"](
        max_states=state_count,
        max_iterations=2,
        lm_max_trials=4,
        cpp_math_core_enabled=cpp_math_core_enabled,
    )
    states = []
    for index in range(state_count):
        state = np.zeros(15)
        state[:3] = [0.11 * index, 0.025 * np.sin(index), 0.01 * index]
        state[3:6] = [0.005 * index, -0.003 * index, 0.012 * index]
        states.append(state)
        backend.add_state(state)
    backend.add_prior(0, states[0], covariance=np.full(15, 1.0e-3))

    imu_samples = [
        api["ImuSample"](
            sample * 0.01,
            (0.02, -0.01, 9.81),
            (0.005, -0.003, 0.012),
        )
        for sample in range(11)
    ]
    imu = api["preintegrate"](imu_samples, 0.0, 0.1)
    rng = np.random.default_rng(7)
    lidar_body = rng.uniform([-2.0, -2.0, -1.0], [2.0, 2.0, 1.0],
                             size=(lidar_points, 3))
    normals = rng.normal(size=(lidar_points, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)

    anchors = rng.uniform(
        [-0.35, -0.25], [0.35, 0.25], size=(visual_tracks, 2)
    )
    inverse_depth = 1.0 / rng.uniform(1.5, 5.0, size=visual_tracks)
    for index in range(1, state_count):
        backend.add_imu_preintegrated(index - 1, index, imu)
        backend.add_gnss(
            index,
            states[index][:3] + [0.015, -0.01, 0.005],
            covariance=0.4,
        )
        backend.add_optical_flow_body(
            index - 1,
            index,
            [0.11, 0.0, 0.01],
            covariance=0.08,
        )
        plane_points = lidar_body + states[index][:3]
        lidar = api["LidarFactor"](
            stamp_ns=index * 100_000_000,
            stamp_s=index * 0.1,
            scan_sequence=index,
            reset_counter=0,
            matched_points=lidar_points,
            candidate_points=lidar_points,
            linearization_pose=states[index][:6],
            pose_hessian=np.eye(6),
            pose_gradient=np.zeros(6),
            residual_squared=0.0,
            measurement_variance=1.0e-3,
            source="benchmark",
            map_frame="map",
            state_frame="body",
            sensor_frame="lidar",
            correspondences_valid=True,
            lidar_points=lidar_body,
            plane_normals=normals,
            plane_points=plane_points,
            lidar_to_body_rotation=np.eye(3),
            lidar_to_body_translation=np.zeros(3),
        )
        backend.add_native_lidar_correspondences(index, lidar)
        seed = api["VisualTracks"](
            anchors, anchors, inverse_depth, 2.5e-5
        )
        predicted, _, _, _ = api["visual_linearize"](
            states[index - 1], states[index], seed
        )
        tracks = api["VisualTracks"](
            anchors,
            anchors + predicted.reshape(-1, 2),
            inverse_depth,
            2.5e-5,
        )
        backend.add_visual_reprojection(index - 1, index, tracks)
    return backend


def elapsed_ms(callback, repetitions):
    started = time.perf_counter()
    for _ in range(repetitions):
        callback()
    return 1000.0 * (time.perf_counter() - started) / repetitions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--linearizations", type=int, default=30)
    parser.add_argument("--cost-evaluations", type=int, default=100)
    parser.add_argument("--optimizations", type=int, default=5)
    parser.add_argument("--states", type=int, default=8)
    parser.add_argument("--lidar-points", type=int, default=256)
    parser.add_argument("--visual-tracks", type=int, default=48)
    parser.add_argument(
        "--cpp-math-core", choices=("on", "off"), default="on"
    )
    args = parser.parse_args()

    api = load_backend(args.source_root)
    cpp_enabled = args.cpp_math_core == "on"
    build = lambda: build_graph(
        api,
        state_count=args.states,
        lidar_points=args.lidar_points,
        visual_tracks=args.visual_tracks,
        cpp_math_core_enabled=cpp_enabled,
    )
    backend = build()
    if cpp_enabled and not backend.cpp_math_core_enabled:
        raise RuntimeError(
            "C++ math core was requested but the extension is unavailable"
        )
    backend._normal()
    result = {
        "source_root": str(Path(args.source_root).resolve()),
        "cpp_math_core_requested": cpp_enabled,
        "cpp_math_core_active": backend.cpp_math_core_enabled,
        "states": backend.state_count,
        "factors": backend.factor_count,
        "lidar_points_per_factor": args.lidar_points,
        "visual_tracks_per_factor": args.visual_tracks,
        "linearization_mean_ms": elapsed_ms(
            backend._normal, args.linearizations
        ),
        "cost_only_mean_ms": elapsed_ms(
            backend._cost, args.cost_evaluations
        ) if hasattr(backend, "_cost") else None,
        "optimization_mean_ms": elapsed_ms(
            lambda: build().optimize(), args.optimizations
        ),
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
