#!/usr/bin/env python3
"""Deterministic performance/occlusion benchmark for the online shared map."""

import argparse
import json
from pathlib import Path
import resource
import statistics
import sys
import time
import tracemalloc

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(
    ROOT / "src" / "ultra_fusion_nav" / "uf_shared_mapping"))

from uf_shared_mapping.voxel_map import SourceAwareVoxelMap  # noqa: E402


def visible_scene(seed):
    rng = np.random.default_rng(seed)
    azimuth = np.deg2rad(np.linspace(-60.0, 60.0, 240))
    elevation = np.deg2rad(np.linspace(-15.0, 45.0, 120))
    azimuth, elevation = np.meshgrid(azimuth, elevation, indexing="xy")
    azimuth, elevation = azimuth.ravel(), elevation.ravel()
    directions = np.c_[
        np.cos(elevation) * np.cos(azimuth),
        np.cos(elevation) * np.sin(azimuth),
        np.sin(elevation),
    ]
    ranges = 4.0 + 1.2 * np.sin(2.0 * azimuth) + 0.6 * np.cos(
        3.0 * elevation)
    lidar = directions * ranges[:, None]

    consistent_index = rng.choice(len(lidar), 15000, replace=False)
    consistent = directions[consistent_index] * (
        ranges[consistent_index] + rng.normal(0.0, 0.018, len(consistent_index))
    )[:, None]
    ghost_index = rng.choice(len(lidar), 5000, replace=False)
    ghost = directions[ghost_index] * (
        ranges[ghost_index] + 0.60 + rng.normal(0.0, 0.015, len(ghost_index))
    )[:, None]

    supplement_azimuth = rng.uniform(np.deg2rad(75.0), np.deg2rad(105.0), 4000)
    supplement_elevation = rng.uniform(
        np.deg2rad(-10.0), np.deg2rad(40.0), 4000)
    supplement_directions = np.c_[
        np.cos(supplement_elevation) * np.cos(supplement_azimuth),
        np.cos(supplement_elevation) * np.sin(supplement_azimuth),
        np.sin(supplement_elevation),
    ]
    supplement = supplement_directions * rng.uniform(3.0, 8.0, 4000)[:, None]
    rgbd = np.vstack([consistent, ghost, supplement])
    colors = rng.integers(0, 256, (len(rgbd), 3), dtype=np.uint8)
    labels = np.r_[
        np.zeros(len(consistent), dtype=np.int8),
        np.ones(len(ghost), dtype=np.int8),
        np.full(len(supplement), 2, dtype=np.int8),
    ]
    return lidar, rgbd, colors, labels


def run_once(lidar, rgbd, colors, occlusion):
    mapping = SourceAwareVoxelMap()
    wall_start = time.perf_counter()
    cpu_start = time.process_time()
    lidar_start = time.perf_counter()
    mapping.integrate_lidar(lidar, 1.0)
    lidar_ms = (time.perf_counter() - lidar_start) * 1000.0
    rgbd_start = time.perf_counter()
    accepted = mapping.integrate_rgbd(
        rgbd,
        colors,
        0.9,
        1.05,
        sensor_origin=[0.0, 0.0, 0.0] if occlusion else None,
        occlusion_points=lidar if occlusion else None,
    )
    rgbd_ms = (time.perf_counter() - rgbd_start) * 1000.0
    arrays_start = time.perf_counter()
    mapping.arrays("joint")
    arrays_ms = (time.perf_counter() - arrays_start) * 1000.0
    return {
        "lidar_ms": lidar_ms,
        "rgbd_ms": rgbd_ms,
        "arrays_ms": arrays_ms,
        "wall_ms": (time.perf_counter() - wall_start) * 1000.0,
        "cpu_ms": (time.process_time() - cpu_start) * 1000.0,
        "accepted": accepted,
        "summary": mapping.summary(),
    }


def timing_summary(runs, name):
    values = [float(run[name]) for run in runs]
    return {
        "mean_ms": statistics.fmean(values),
        "median_ms": statistics.median(values),
        "min_ms": min(values),
        "max_ms": max(values),
    }


def benchmark(seed, runs):
    lidar, rgbd, colors, labels = visible_scene(seed)
    control = [run_once(lidar, rgbd, colors, False) for _ in range(runs)]
    filtered = [run_once(lidar, rgbd, colors, True) for _ in range(runs)]

    mapping = SourceAwareVoxelMap()
    rejected = mapping._occlusion_mask(rgbd, [0.0, 0.0, 0.0], lidar)
    tracemalloc.start()
    run_once(lidar, rgbd, colors, True)
    _, peak_bytes = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return {
        "seed": seed,
        "runs": runs,
        "inputs": {
            "lidar": len(lidar),
            "rgbd": len(rgbd),
            "consistent": int(np.count_nonzero(labels == 0)),
            "occluded_ghost": int(np.count_nonzero(labels == 1)),
            "supplement": int(np.count_nonzero(labels == 2)),
        },
        "quality": {
            "consistent_reject_ratio": float(np.mean(rejected[labels == 0])),
            "ghost_reject_ratio": float(np.mean(rejected[labels == 1])),
            "supplement_reject_ratio": float(np.mean(rejected[labels == 2])),
            "control_summary": control[-1]["summary"],
            "filtered_summary": filtered[-1]["summary"],
        },
        "timing": {
            "control": {
                name: timing_summary(control, name)
                for name in ("lidar_ms", "rgbd_ms", "arrays_ms", "wall_ms", "cpu_ms")
            },
            "height_aware_occlusion": {
                name: timing_summary(filtered, name)
                for name in ("lidar_ms", "rgbd_ms", "arrays_ms", "wall_ms", "cpu_ms")
            },
        },
        "memory": {
            "tracemalloc_peak_bytes": peak_bytes,
            "process_max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=20260809)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be positive")
    report = benchmark(args.seed, args.runs)
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
