#!/usr/bin/env python3
"""Generate a deterministic building-scale cross-source map dataset.

The fixture executes a documented camera/LiDAR route through an analytic scene
with ground, façades, roofs, an annex, curbs and columns.  It is intended for
repeatable algorithm regression, not as a replacement for the live exporters.
"""

import argparse
import csv
import math
from pathlib import Path
import struct

import numpy as np
import yaml


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def transform(points, xyz_rpy):
    rotation = rotation_matrix(*xyz_rpy[3:])
    return points @ rotation.T + np.asarray(xyz_rpy[:3], dtype=np.float64)


def inverse_transform(points, xyz_rpy):
    rotation = rotation_matrix(*xyz_rpy[3:])
    return (points - np.asarray(xyz_rpy[:3], dtype=np.float64)) @ rotation


def packed_rgb(red, green, blue):
    value = (int(red) << 16) | (int(green) << 8) | int(blue)
    return struct.unpack("f", struct.pack("I", value))[0]


def write_pcd(path, points, colors):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\nFIELDS x y z rgb\n")
        stream.write("SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\nDATA ascii\n")
        for point, color in zip(points, colors):
            stream.write(
                f"{point[0]:.7f} {point[1]:.7f} {point[2]:.7f} "
                f"{packed_rgb(*color):.9e}\n")


def regular_values(start, stop, spacing):
    count = max(2, int(math.ceil((stop - start) / spacing)) + 1)
    return np.linspace(start, stop, count)


def plane_xy(x0, x1, y0, y1, z, spacing, normal, color):
    x, y = np.meshgrid(regular_values(x0, x1, spacing),
                       regular_values(y0, y1, spacing), indexing="xy")
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))
    normals = np.tile(np.asarray(normal), (len(points), 1))
    colors = np.tile(np.asarray(color, dtype=np.uint8), (len(points), 1))
    return points, normals, colors


def plane_xz(x0, x1, z0, z1, y, spacing, normal, color):
    x, z = np.meshgrid(regular_values(x0, x1, spacing),
                       regular_values(z0, z1, spacing), indexing="xy")
    points = np.column_stack((x.ravel(), np.full(x.size, y), z.ravel()))
    normals = np.tile(np.asarray(normal), (len(points), 1))
    colors = np.tile(np.asarray(color, dtype=np.uint8), (len(points), 1))
    return points, normals, colors


def plane_yz(y0, y1, z0, z1, x, spacing, normal, color):
    y, z = np.meshgrid(regular_values(y0, y1, spacing),
                       regular_values(z0, z1, spacing), indexing="xy")
    points = np.column_stack((np.full(y.size, x), y.ravel(), z.ravel()))
    normals = np.tile(np.asarray(normal), (len(points), 1))
    colors = np.tile(np.asarray(color, dtype=np.uint8), (len(points), 1))
    return points, normals, colors


def cylinder(cx, cy, radius, z0, z1, spacing, color):
    angles = regular_values(-math.pi, math.pi, spacing / radius)
    heights = regular_values(z0, z1, spacing)
    angle_grid, height_grid = np.meshgrid(angles, heights, indexing="xy")
    points = np.column_stack((
        cx + radius * np.cos(angle_grid).ravel(),
        cy + radius * np.sin(angle_grid).ravel(),
        height_grid.ravel(),
    ))
    normals = np.column_stack((
        np.cos(angle_grid).ravel(), np.sin(angle_grid).ravel(),
        np.zeros(angle_grid.size),
    ))
    colors = np.tile(np.asarray(color, dtype=np.uint8), (len(points), 1))
    return points, normals, colors


def build_scene(spacing):
    surfaces = [
        plane_xy(-8, 21, -11, 13, 0.0, spacing, (0, 0, 1), (105, 135, 92)),
        plane_xz(0, 12, 0, 6.5, -3, spacing, (0, -1, 0), (185, 171, 148)),
        plane_xz(0, 12, 0, 6.5, 5, spacing, (0, 1, 0), (169, 157, 139)),
        plane_yz(-3, 5, 0, 6.5, 0, spacing, (-1, 0, 0), (180, 164, 142)),
        plane_yz(-3, 5, 0, 6.5, 12, spacing, (1, 0, 0), (176, 160, 140)),
        plane_xy(0, 12, -3, 5, 6.5, spacing, (0, 0, 1), (144, 111, 83)),
        plane_xz(12, 17, 0, 3.6, -1, spacing, (0, -1, 0), (151, 168, 181)),
        plane_xz(12, 17, 0, 3.6, 4, spacing, (0, 1, 0), (147, 164, 178)),
        plane_yz(-1, 4, 0, 3.6, 17, spacing, (1, 0, 0), (141, 158, 174)),
        plane_xy(12, 17, -1, 4, 3.6, spacing, (0, 0, 1), (105, 119, 133)),
    ]
    # A low curb and front columns create stable XY boundaries and vertical detail.
    surfaces.extend([
        plane_xz(-1, 18, 0.0, 0.32, -5.2, spacing, (0, -1, 0), (130, 130, 130)),
        plane_xz(-1, 18, 0.0, 0.32, -4.8, spacing, (0, 1, 0), (130, 130, 130)),
        cylinder(2.0, -3.35, 0.24, 0.0, 4.8, spacing, (128, 122, 114)),
        cylinder(6.0, -3.35, 0.24, 0.0, 4.8, spacing, (128, 122, 114)),
        cylinder(10.0, -3.35, 0.24, 0.0, 4.8, spacing, (128, 122, 114)),
    ])
    points = np.concatenate([surface[0] for surface in surfaces])
    normals = np.concatenate([surface[1] for surface in surfaces])
    colors = np.concatenate([surface[2] for surface in surfaces])
    return points, normals, colors


def route(count):
    front_count = max(6, count * 2 // 3)
    side_count = max(4, count - front_count)
    front_x = np.linspace(-4.5, 17.5, front_count)
    front = np.column_stack((front_x, np.full(front_count, -8.2),
                             2.3 + 0.25 * np.sin(front_x * 0.25)))
    side_y = np.linspace(-7.0, 7.5, side_count)
    side = np.column_stack((np.full(side_count, 19.2), side_y,
                            2.5 + 0.2 * np.cos(side_y * 0.3)))
    return np.concatenate((front, side))


def visible_indices(points, normals, position, center, max_range, fov_deg):
    direction = center - position
    direction /= np.linalg.norm(direction)
    vectors = points - position
    distances = np.linalg.norm(vectors, axis=1)
    unit = vectors / np.maximum(distances[:, None], 1e-9)
    in_fov = unit @ direction >= math.cos(math.radians(fov_deg * 0.5))
    facing = np.einsum("ij,ij->i", normals, -unit) > -0.05
    return np.flatnonzero((distances >= 0.35) & (distances <= max_range) & in_fov & facing)


def write_route(path, positions, center):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("index", "stamp_s", "x", "y", "z", "yaw_rad", "pitch_rad"))
        for index, position in enumerate(positions):
            vector = center - position
            yaw = math.atan2(vector[1], vector[0])
            pitch = math.atan2(vector[2], math.hypot(vector[0], vector[1]))
            writer.writerow((index, f"{index * 0.8:.3f}", *[f"{v:.7f}" for v in position],
                             f"{yaw:.9f}", f"{pitch:.9f}"))


def generate(output_dir, preset, seed):
    rng = np.random.default_rng(seed)
    spacing = 0.42 if preset == "unit" else 0.16
    route_count = 10 if preset == "unit" else 30
    max_keyframe_points = 900 if preset == "unit" else 2600
    points, normals, colors = build_scene(spacing)
    positions = route(route_count)
    center = np.array((7.0, 0.5, 2.0))

    output_dir.mkdir(parents=True, exist_ok=True)
    keyframe_dir = output_dir / "visual_keyframes"
    keyframe_dir.mkdir(exist_ok=True)
    write_route(output_dir / "ground_truth_route.csv", positions, center)

    visual_seen = set()
    for index, position in enumerate(positions):
        visible = visible_indices(points, normals, position, center, 17.0, 86.0)
        if len(visible) > max_keyframe_points:
            visible = visible[np.linspace(0, len(visible) - 1,
                                          max_keyframe_points, dtype=int)]
        visual_seen.update(int(value) for value in visible)
        key_points = points[visible] + rng.normal(0.0, 0.012, (len(visible), 3))
        write_pcd(keyframe_dir / f"keyframe_{index:05d}.pcd",
                  key_points, colors[visible])

    visual_indices = np.array(sorted(visual_seen), dtype=int)
    # Remove a deterministic rear/roof subset to model front/side RGB-D visibility.
    visual_keep = ((np.arange(len(visual_indices)) * 17 + seed) % 11) != 0
    visual_indices = visual_indices[visual_keep]
    visual_points = points[visual_indices] + rng.normal(
        0.0, 0.014, (len(visual_indices), 3))
    visual_colors = colors[visual_indices]

    lidar_distance_mask = np.zeros(len(points), dtype=bool)
    for position in positions:
        lidar_distance_mask |= np.linalg.norm(points - position, axis=1) <= 20.0
    lidar_indices = np.flatnonzero(lidar_distance_mask)
    # Different angular/range sampling density while preserving genuine overlap.
    lidar_indices = lidar_indices[((lidar_indices * 13 + seed) % 5) != 0]
    lidar_world = points[lidar_indices] + rng.normal(0.0, 0.025, (len(lidar_indices), 3))
    lidar_colors = np.tile(np.array((225, 72, 58), dtype=np.uint8),
                           (len(lidar_world), 1))

    truth = np.array((1.20, -0.80, 0.22,
                      math.radians(1.0), math.radians(-1.5), math.radians(8.0)))
    initial = np.array((0.86, -0.46, 0.10,
                        0.0, 0.0, math.radians(4.5)))
    lidar_local = inverse_transform(lidar_world, truth)

    write_pcd(output_dir / "visual_map.pcd", visual_points, visual_colors)
    write_pcd(output_dir / "lidar_map.pcd", lidar_local, lidar_colors)
    manifest = {
        "dataset": {
            "id": f"hybridfusion_building_route_{preset}_seed{seed}",
            "visual_map": "visual_map.pcd",
            "lidar_map": "lidar_map.pcd",
            "visual_frame": "rtabmap_map",
            "lidar_frame": "camera_init",
            "initial_lidar_to_visual": [float(v) for v in initial],
            "truth_lidar_to_visual": [float(v) for v in truth],
            "route_file": "ground_truth_route.csv",
            "calibration_file": "camera_calibration.yaml",
            "scene": "building, annex, façades, roofs, ground, curb and columns",
            "visual_points": int(len(visual_points)),
            "lidar_points": int(len(lidar_local)),
            "keyframes": int(route_count),
            "generated_not_measured": True,
        }
    }
    (output_dir / "dataset.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    calibration = {
        "model": "D435i-compatible pinhole fixture",
        "width": 640, "height": 480,
        "K": [386.2, 0.0, 320.0, 0.0, 386.2, 240.0, 0.0, 0.0, 1.0],
        "depth_encoding": "16UC1",
        "depth_scale_m": 0.001,
        "rgb_frame": "front_d435i_color_optical_frame",
        "map_frame": "rtabmap_map",
        "timestamps": "ground_truth_route.csv stamp_s",
    }
    (output_dir / "camera_calibration.yaml").write_text(
        yaml.safe_dump(calibration, sort_keys=False), encoding="utf-8")
    print(
        f"dataset={output_dir} preset={preset} seed={seed} "
        f"visual_points={len(visual_points)} lidar_points={len(lidar_local)} "
        f"keyframes={route_count}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--preset", choices=("unit", "benchmark"), default="benchmark")
    parser.add_argument("--seed", type=int, default=20260805)
    arguments = parser.parse_args()
    generate(arguments.output.expanduser().resolve(), arguments.preset, arguments.seed)


if __name__ == "__main__":
    main()
