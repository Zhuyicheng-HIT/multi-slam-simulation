#!/usr/bin/env python3
"""Render a top-down coverage audit for the expanded S-curve world."""

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "multi_slam_uav_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from multi_slam_uav_sim.s_curve_path import generate_s_curve  # noqa: E402


def pose_xy(element):
    values = [float(value) for value in element.findtext("pose").split()]
    return values[0], values[1]


def children_with_prefix(root, tag, prefix):
    return [
        element for element in root.findall(f".//{tag}")
        if element.get("name", "").startswith(prefix)
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="/tmp/s_curve_world_audit.png")
    parser.add_argument("--json", default="/tmp/s_curve_world_audit.json")
    args = parser.parse_args()

    world = ET.parse(
        PACKAGE_ROOT / "worlds" / "simple_apm_rgbd_mid360.sdf"
    ).getroot()
    landmark_root = ET.parse(
        PACKAGE_ROOT / "models" / "s_curve_lidar_landmarks" / "model.sdf"
    ).getroot()
    x_lines = children_with_prefix(world, "visual", "x_grid_")
    y_lines = children_with_prefix(world, "visual", "y_grid_")
    flow_markers = children_with_prefix(world, "visual", "marker_")
    lidar_landmarks = landmark_root.findall(".//collision")
    lidar_xy = [pose_xy(item) for item in lidar_landmarks]
    route = generate_s_curve(12.0, 4.5, 5.0, 1.0, samples=241)

    fig, axis = plt.subplots(figsize=(9, 9), constrained_layout=True)
    axis.set_facecolor("#aeb7b3")
    for item in x_lines:
        x, _ = pose_xy(item)
        axis.plot([x, x], [-19, 19], color="#30363b", linewidth=0.45, alpha=0.75)
    for item in y_lines:
        _, y = pose_xy(item)
        axis.plot([-19, 19], [y, y], color="#30363b", linewidth=0.45, alpha=0.75)
    marker_xy = [pose_xy(item) for item in flow_markers]
    axis.scatter(
        [point[0] for point in marker_xy], [point[1] for point in marker_xy],
        marker="s", s=28, color="#f2c744", edgecolor="#202428",
        label="flow texture markers", zorder=3,
    )
    axis.scatter(
        [point[0] for point in lidar_xy], [point[1] for point in lidar_xy],
        marker="D", s=54, color="#d64b3c", edgecolor="#202428",
        label="LiDAR landmarks", zorder=4,
    )
    axis.plot(
        [point[0] for point in route], [point[1] for point in route],
        color="#1266a8", linewidth=2.2, label="long S route", zorder=5,
    )
    axis.scatter([0], [0], marker="^", s=90, color="#111111", label="home")
    axis.set_xlim(-21, 21)
    axis.set_ylim(-21, 21)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("local x (m)")
    axis.set_ylabel("local y (m)")
    axis.set_title("Expanded simulation world coverage audit")
    axis.legend(loc="upper left", framealpha=0.92)
    axis.grid(False)
    fig.savefig(args.output, dpi=170)

    route_max_distance = max(
        min(math.dist((x, y), landmark) for landmark in lidar_xy)
        for x, y, _ in route
    )
    audit_grid = [
        (x, y)
        for x in (-16.0, -8.0, 0.0, 8.0, 16.0)
        for y in (-16.0, -8.0, 0.0, 8.0, 16.0)
    ]
    expanded_max_distance = max(
        min(math.dist(point, landmark) for landmark in lidar_xy)
        for point in audit_grid
    )
    report = {
        "flow_grid_x_lines": len(x_lines),
        "flow_grid_y_lines": len(y_lines),
        "flow_markers": len(flow_markers),
        "lidar_landmarks": len(lidar_xy),
        "route_max_nearest_lidar_landmark_m": route_max_distance,
        "expanded_grid_max_nearest_lidar_landmark_m": expanded_max_distance,
        "texture_bounds_m": [-18.0, 18.0],
    }
    Path(args.json).write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
