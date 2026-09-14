#!/usr/bin/env python3
"""Create compact visual summaries from a HybridFusion benchmark directory."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_pcd_xyz(path, limit=12000):
    with open(path, encoding="utf-8") as handle:
        header = []
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"missing DATA header in {path}")
            header.append(line)
            if line.strip().lower().startswith("data "):
                break
        if not header[-1].strip().lower() == "data ascii":
            raise ValueError(f"binary PCD is not supported for plotting: {path}")
        lines = handle.readlines()
    rows = []
    for line in lines:
        fields = line.split()
        if len(fields) >= 3:
            rows.append((float(fields[0]), float(fields[1]), float(fields[2])))
    points = np.asarray(rows, dtype=float)
    if len(points) > limit:
        indices = np.linspace(0, len(points) - 1, limit, dtype=int)
        points = points[indices]
    return points


def read_route(path):
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    keys = rows[0].keys()
    def column(*names):
        name = next(name for name in names if name in keys)
        return np.asarray([float(row[name]) for row in rows])
    return column("x", "position_x"), column("y", "position_y"), column("z", "position_z")


def read_comparison(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def save_route(root, out):
    x, y, z = read_route(root / "dataset" / "ground_truth_route.csv")
    fig = plt.figure(figsize=(10, 5.5))
    ax = fig.add_subplot(121)
    ax.plot(x, y, "o-", ms=2, lw=1.4, label="ground truth")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Ground-truth route (XY)")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend()
    ax3 = fig.add_subplot(122, projection="3d")
    ax3.plot(x, y, z, lw=1.5)
    ax3.set_xlabel("x (m)")
    ax3.set_ylabel("y (m)")
    ax3.set_zlabel("z (m)")
    ax3.set_title("Ground-truth route (3D)")
    fig.tight_layout()
    fig.savefig(out / "01_ground_truth_route.png", dpi=160)
    plt.close(fig)


def save_clouds(root, out):
    reference = read_pcd_xyz(root / "dataset" / "lidar_map.pcd")
    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.scatter(reference[:, 0], reference[:, 1], s=0.25, alpha=0.3, label="LiDAR map")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Generated simulation LiDAR map (XY projection)")
    ax.axis("equal")
    ax.grid(alpha=0.2)
    ax.legend(markerscale=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(out / "02_simulation_lidar_map.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.7))
    for ax, method in zip(axes, ("initial", "gicp", "hybrid")):
        # The benchmark writes aligned outputs as binary-compressed PCD. Use
        # the transform metrics for comparison and keep the raw map plot above
        # independent of optional PCD decompression libraries.
        points = reference
        ax.scatter(reference[:, 0], reference[:, 1], s=0.25, alpha=0.18, label="map")
        ax.scatter(points[:, 0], points[:, 1], s=0.25, alpha=0.35, label="aligned lidar")
        ax.set_title(method.upper())
        ax.set_xlabel("x (m)")
        ax.grid(alpha=0.2)
        ax.axis("equal")
    axes[0].set_ylabel("y (m)")
    axes[0].legend(markerscale=8, loc="upper left")
    fig.suptitle("Run 01 generated point-cloud reference (XY projection)")
    fig.tight_layout()
    fig.savefig(out / "02_pointcloud_alignment_run01.png", dpi=180)
    plt.close(fig)


def save_metrics(root, out):
    rows = read_comparison(root / "comparison.csv")
    methods = ["initial", "gicp", "hybrid"]
    colors = {"initial": "#777777", "gicp": "#1976d2", "hybrid": "#e67e22"}
    metrics = [
        ("translation_error_m", "Translation error (m)"),
        ("rotation_error_deg", "Rotation error (deg)"),
        ("runtime_ms", "Runtime (ms)"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    for ax, (key, title) in zip(axes, metrics):
        values = [[float(row[key]) for row in rows if row["method"] == method] for method in methods]
        ax.boxplot(values, labels=[m.upper() for m in methods], showmeans=True)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle("HybridFusion benchmark comparison (3 runs)")
    fig.tight_layout()
    fig.savefig(out / "03_metrics_comparison.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("benchmark_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.benchmark_dir.resolve()
    out = (args.output or root / "figures").resolve()
    out.mkdir(parents=True, exist_ok=True)
    save_route(root, out)
    save_clouds(root, out)
    save_metrics(root, out)
    (out / "figure_manifest.json").write_text(
        json.dumps({"source": str(root), "figures": sorted(p.name for p in out.glob("*.png"))}, indent=2),
        encoding="utf-8",
    )
    print(f"Generated figures in {out}")


if __name__ == "__main__":
    main()
