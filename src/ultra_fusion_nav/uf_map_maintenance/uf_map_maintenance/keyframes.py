"""Deterministic offline keyframe export for PR18 relocalization smoke tests."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .builder import (
    _load_scan_to_pose_child_calibration,
    _read_poses,
    _rotation_matrix,
)
from .pcd import StreamingBinaryPcdWriter
from .trajectory import (
    PoseTrajectory,
    TrajectoryContractError,
    deskew_lidar_points_to_map,
    load_pose_trajectory,
)


def _rotation_distance(left, right):
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    left /= np.linalg.norm(left)
    right /= np.linalg.norm(right)
    return 2.0 * np.arccos(np.clip(abs(float(np.dot(left, right))), 0.0, 1.0))


def select_keyframe_indices(
    records, minimum_translation_m=1.0, minimum_rotation_rad=0.26,
    minimum_time_spacing_s=1.0
):
    if not records:
        return []
    selected = [0]
    for index in range(1, len(records)):
        previous = records[selected[-1]]
        current = records[index]
        elapsed = (current["stamp_ns"] - previous["stamp_ns"]) * 1.0e-9
        if elapsed < minimum_time_spacing_s:
            continue
        translation = np.linalg.norm(current["translation"] - previous["translation"])
        rotation = _rotation_distance(current["quaternion"], previous["quaternion"])
        if translation >= minimum_translation_m or rotation >= minimum_rotation_rad:
            selected.append(index)
    return selected


def export_keyframes(
    session_dir, pose_revision, trajectory_revision, output_dir,
    maximum_pose_bracket_ns=250_000_000,
    minimum_translation_m=1.0, minimum_rotation_rad=0.26,
    minimum_time_spacing_s=1.0
):
    session = Path(session_dir)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    descriptor_directory = output / "descriptor_clouds"
    map_directory = output / "map_clouds"
    descriptor_directory.mkdir()
    map_directory.mkdir()
    records = _read_poses(pose_revision)
    selected = select_keyframe_indices(
        records, minimum_translation_m, minimum_rotation_rad,
        minimum_time_spacing_s,
    )
    trajectory = PoseTrajectory(
        load_pose_trajectory(trajectory_revision), maximum_pose_bracket_ns
    )
    extrinsic_translation, extrinsic_quaternion, _ = (
        _load_scan_to_pose_child_calibration(session)
    )
    metadata_path = output / "keyframes.csv"
    rejected = {}
    exported = 0
    with metadata_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "keyframe_id", "scan_id", "stamp_ns", "epoch",
            "tx", "ty", "tz", "qx", "qy", "qz", "qw",
            "descriptor_pcd", "map_pcd", "points",
        ])
        for record_index in selected:
            record = records[record_index]
            scan_path = session / "scans" / f"{record['scan_id']:06d}.npz"
            with np.load(scan_path, allow_pickle=False) as archive:
                points = np.asarray(archive["points"], dtype=np.float64)
                offsets = np.asarray(archive["offset_time"], dtype=np.int64)
                timebase = int(np.asarray(archive["timebase"]).reshape(-1)[0])
            try:
                map_points = deskew_lidar_points_to_map(
                    points, offsets + timebase, trajectory, record["epoch"],
                    extrinsic_translation, extrinsic_quaternion,
                )
                translation, quaternion = trajectory.interpolate(
                    record["stamp_ns"], record["epoch"]
                )
            except TrajectoryContractError as error:
                reason = str(error)
                rejected[reason] = rejected.get(reason, 0) + 1
                continue
            body_points = map_points.copy()
            body_points[:, :3] -= translation
            body_points[:, :3] = body_points[:, :3] @ _rotation_matrix(quaternion)
            descriptor_relative = Path("descriptor_clouds") / f"{exported:06d}.pcd"
            map_relative = Path("map_clouds") / f"{exported:06d}.pcd"
            with StreamingBinaryPcdWriter(output / descriptor_relative) as pcd:
                pcd.append(body_points)
            with StreamingBinaryPcdWriter(output / map_relative) as pcd:
                pcd.append(map_points)
            writer.writerow([
                exported, record["scan_id"], record["stamp_ns"], record["epoch"],
                *translation, *quaternion, descriptor_relative, map_relative,
                len(points),
            ])
            exported += 1
    summary = {
        "input_scans": len(records),
        "selected_keyframe_candidates": len(selected),
        "exported_keyframes": exported,
        "rejected_keyframe_candidates": len(selected) - exported,
        "rejection_reasons": rejected,
        "minimum_translation_m": minimum_translation_m,
        "minimum_rotation_rad": minimum_rotation_rad,
        "minimum_time_spacing_s": minimum_time_spacing_s,
        "metadata": metadata_path.name,
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description="Export offline PR18-compatible LiDAR keyframes")
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--poses", required=True, type=Path)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--maximum-pose-bracket-ms", type=float, default=250.0)
    parser.add_argument("--minimum-translation-m", type=float, default=1.0)
    parser.add_argument("--minimum-rotation-deg", type=float, default=15.0)
    parser.add_argument("--minimum-time-spacing-s", type=float, default=1.0)
    arguments = parser.parse_args(argv)
    result = export_keyframes(
        arguments.session, arguments.poses, arguments.trajectory, arguments.output,
        int(arguments.maximum_pose_bracket_ms * 1.0e6),
        arguments.minimum_translation_m,
        np.deg2rad(arguments.minimum_rotation_deg),
        arguments.minimum_time_spacing_s,
    )
    print(json.dumps(result, sort_keys=True))
    return 0
