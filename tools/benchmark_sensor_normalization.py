#!/usr/bin/env python3
"""Deterministic synthetic benchmark for the sensor normalization hot path.

This intentionally exercises only message construction and the body filter;
it does not claim Gazebo or hardware performance.
"""
import argparse
import time

import numpy as np


def synthetic_cloud(points: int) -> np.ndarray:
    message = np.empty((points, 4), dtype=np.float32)
    indexes = np.arange(points)
    message[:, 0] = 1.0 + indexes % 80
    message[:, 1] = indexes % 17
    message[:, 2] = (indexes % 11) * 0.1
    message[:, 3] = 1.0
    return message


def vectorized_filter(message, bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    xyz = message[:, :3]
    distance_sq = np.einsum("ij,ij->i", xyz, xyz)
    range_keep = np.isfinite(distance_sq) & (distance_sq >= 0.1 ** 2) & (distance_sq <= 40.0 ** 2)
    body_keep = ~(
        (xyz[:, 0] >= min_x) & (xyz[:, 0] <= max_x) &
        (xyz[:, 1] >= min_y) & (xyz[:, 1] <= max_y) &
        (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)
    )
    return message[range_keep & body_keep]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--points", type=int, default=20000)
    parser.add_argument("--frames", type=int, default=100)
    args = parser.parse_args()
    message = synthetic_cloud(args.points)
    bounds = (-0.45, 0.45, -0.45, 0.45, -0.35, 0.15)
    for _ in range(5):
        vectorized_filter(message, bounds)
    started = time.perf_counter()
    removed = 0
    output_points = 0
    for _ in range(args.frames):
        output = vectorized_filter(message, bounds)
        removed += message.shape[0] - output.shape[0]
        output_points += output.shape[0]
    elapsed = time.perf_counter() - started
    frame_ms = elapsed * 1000.0 / max(1, args.frames)
    print(f"frames={args.frames} points={args.points} total_s={elapsed:.6f}")
    print(f"filter_ms_pure_python_process={frame_ms:.3f} effective_hz={1.0 / max(elapsed / max(1, args.frames), 1e-9):.2f}")
    print(f"body_removed={removed} output_points={output_points}")
    print("synthetic_input_rates_hz=lidar:10,imu:200,optical_flow:100")


if __name__ == "__main__":
    main()
