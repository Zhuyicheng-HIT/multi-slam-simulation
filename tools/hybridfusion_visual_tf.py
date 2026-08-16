#!/usr/bin/env python3
"""Emit tf2 static-transform arguments for a HybridFusion visual-frame PCD."""

import argparse
from pathlib import Path

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("transform", type=Path)
    parser.add_argument("--visual-child", default="hybridfusion_visual_map")
    args = parser.parse_args()

    with args.transform.open("r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle) or {}
    record = root.get("transform_lidar_to_visual") or {}
    matrix_values = np.asarray(record.get("matrix_row_major", []), dtype=float)
    if matrix_values.shape != (16,) or np.any(~np.isfinite(matrix_values)):
        raise ValueError("transform.yaml is missing a finite 4x4 matrix")
    transform_visual_lidar = matrix_values.reshape(4, 4)
    if not np.allclose(transform_visual_lidar[3], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError("HybridFusion transform is not a homogeneous SE(3) matrix")
    lidar_frame = str(record.get("child_frame", "")).strip()
    if not lidar_frame:
        raise ValueError("HybridFusion transform has no LiDAR child frame")

    transform_lidar_visual = np.linalg.inv(transform_visual_lidar)
    rotation = transform_lidar_visual[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    translation = transform_lidar_visual[:3, 3]

    values = [
        "--x", translation[0], "--y", translation[1], "--z", translation[2],
        "--qx", quaternion[0], "--qy", quaternion[1],
        "--qz", quaternion[2], "--qw", quaternion[3],
        "--frame-id", lidar_frame, "--child-frame-id", args.visual_child,
    ]
    for value in values:
        print(value if isinstance(value, str) else f"{value:.12g}")


if __name__ == "__main__":
    main()
