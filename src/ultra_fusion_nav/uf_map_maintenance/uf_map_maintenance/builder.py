"""Rebuild auditable raw, deskewed, voxelized, and cleaned map products."""

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import time

import numpy as np
import yaml

from .evaluation import structural_retention_metrics
from .manifest import validate_manifest
from .pcd import StreamingBinaryPcdWriter
from .trajectory import (
    PoseTrajectory,
    TrajectoryContractError,
    deskew_lidar_points_to_map,
    load_pose_trajectory,
)
from .voxel_map import EvidenceVoxelMap, MaintenanceConfig


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_poses(path):
    records = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            record = {
                "scan_id": int(row["scan_id"]),
                "stamp_ns": int(row["stamp_ns"]),
                "epoch": int(row["epoch"]),
                "translation": np.array([float(row[name]) for name in ("tx", "ty", "tz")]),
                "quaternion": np.array([float(row[name]) for name in ("qx", "qy", "qz", "qw")]),
            }
            if not np.all(np.isfinite(np.concatenate((record["translation"], record["quaternion"])))):
                raise ValueError("nonfinite pose")
            records.append(record)
    stamps = [record["stamp_ns"] for record in records]
    if stamps != sorted(stamps) or len(stamps) != len(set(stamps)):
        raise ValueError("pose timestamps must be strictly increasing")
    return records


def _rotation_matrix(quaternion):
    quaternion = quaternion.astype(np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = quaternion / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _scan_pose_transform(points, record, calibration_translation, calibration_quaternion):
    transformed = np.asarray(points, dtype=np.float64).copy()
    transformed[:, :3] = transformed[:, :3] @ _rotation_matrix(calibration_quaternion).T
    transformed[:, :3] += calibration_translation
    transformed[:, :3] = transformed[:, :3] @ _rotation_matrix(record["quaternion"]).T
    transformed[:, :3] += record["translation"]
    return transformed


def _packed_voxel_keys(points, voxel_size):
    if voxel_size <= 0:
        raise ValueError("ghosting voxel size must be positive")
    keys = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    if not len(keys):
        return []
    unique = np.unique(keys, axis=0)
    bias = 1 << 20
    shifted = unique + bias
    if np.any(shifted < 0) or np.any(shifted >= (1 << 21)):
        raise ValueError("map exceeds packed ghosting-voxel coordinate range")
    encoded = ((shifted[:, 0] << 42) | (shifted[:, 1] << 21) | shifted[:, 2])
    return (int(value) for value in encoded)


def _load_scan_to_pose_child_calibration(session):
    manifest_path = session / "manifest.json"
    if not manifest_path.exists():
        return np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), None
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    errors = validate_manifest(manifest, session)
    if errors:
        raise ValueError("invalid archive manifest: " + ",".join(errors))
    calibration = manifest["calibration"]
    if calibration.get("direction", "pose_child_from_scan") != "pose_child_from_scan":
        raise ValueError("unsupported calibration direction")
    return (
        np.asarray(calibration["translation"], dtype=np.float64),
        np.asarray(calibration["quaternion_xyzw"], dtype=np.float64),
        manifest,
    )


def _write_evidence(path, voxel_map):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "voxel_x", "voxel_y", "voxel_z", "centroid_x", "centroid_y",
            "centroid_z", "intensity", "point_count", "scan_support",
            "first_scan_id", "last_scan_id", "decision", "bounded_scan_ids",
        ])
        for key in sorted(voxel_map.voxels):
            evidence = voxel_map.voxels[key]
            writer.writerow([
                *key, *evidence.centroid, evidence.point_count,
                evidence.scan_support, evidence.first_scan_id, evidence.last_scan_id,
                voxel_map.last_decisions[key],
                ";".join(str(value) for value in sorted(evidence.scan_ids)),
            ])


def _write_single_pcd(path, points):
    with StreamingBinaryPcdWriter(path) as writer:
        writer.append(points)


def build_map_revision(session_dir, pose_revision, output_dir, config=None, trajectory_revision=None):
    started = time.perf_counter()
    session = Path(session_dir)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    config = config or MaintenanceConfig()
    pose_records = _read_poses(pose_revision)
    calibration_translation, calibration_quaternion, manifest = _load_scan_to_pose_child_calibration(session)
    if trajectory_revision is None:
        candidate = session / "poses" / "trajectory_original.csv"
        trajectory_revision = candidate if candidate.exists() else None
    trajectory = None
    if trajectory_revision is not None:
        trajectory = PoseTrajectory(load_pose_trajectory(trajectory_revision), config.maximum_pose_bracket_ns)

    scan_paths = [session / "scans" / f"{record['scan_id']:06d}.npz" for record in pose_records]
    missing = [str(path) for path in scan_paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing materialized scans: " + ",".join(missing[:3]))
    before = {str(path.relative_to(session)): _sha256(path) for path in scan_paths}
    voxel_map = EvidenceVoxelMap(config)
    input_points = 0
    deskewed_points = 0
    deskewed_scans = 0
    rejection_reasons = {}
    raw_ghosting_keys = set()
    deskewed_ghosting_keys = set()
    correction_samples = []

    raw_path = output / "raw_scan_pose_map.pcd"
    deskewed_path = output / "deskewed_map.pcd"
    with StreamingBinaryPcdWriter(raw_path) as raw_writer, StreamingBinaryPcdWriter(deskewed_path) as deskewed_writer:
        for record, scan_path in zip(pose_records, scan_paths):
            with np.load(scan_path, allow_pickle=False) as archive:
                points = np.asarray(archive["points"], dtype=np.float64)
                offsets = np.asarray(archive["offset_time"], dtype=np.int64) if "offset_time" in archive else None
                timebase = int(np.asarray(archive["timebase"]).reshape(-1)[0]) if "timebase" in archive else int(record["stamp_ns"])
            input_points += len(points)
            raw = _scan_pose_transform(points, record, calibration_translation, calibration_quaternion)
            raw_writer.append(raw)
            if trajectory is None or offsets is None:
                deskewed = raw
            else:
                try:
                    deskewed = deskew_lidar_points_to_map(
                        points, offsets + timebase, trajectory, record["epoch"],
                        calibration_translation, calibration_quaternion,
                    )
                except TrajectoryContractError as error:
                    reason = str(error)
                    rejection_reasons[reason] = rejection_reasons.get(reason, 0) + 1
                    continue
            deskewed_writer.append(deskewed)
            deskewed_scans += 1
            deskewed_points += len(deskewed)
            voxel_map.add_scan(record["scan_id"], deskewed)
            raw_ghosting_keys.update(_packed_voxel_keys(raw, config.ghosting_voxel_size_m))
            deskewed_ghosting_keys.update(_packed_voxel_keys(deskewed, config.ghosting_voxel_size_m))
            if trajectory is not None and offsets is not None and len(points):
                correction = np.linalg.norm(deskewed[:, :3] - raw[:, :3], axis=1)
                correction_samples.append(correction[::max(1, len(correction) // 128)])

    cleaned, metrics = voxel_map.cleaned_points()
    voxelized = voxel_map.all_points()
    _write_single_pcd(output / "voxelized_map.pcd", voxelized)
    _write_single_pcd(output / "cleaned_map.pcd", cleaned)
    legacy = output / "cleaned_global_map.pcd"
    try:
        os.link(output / "cleaned_map.pcd", legacy)
    except OSError:
        shutil.copyfile(output / "cleaned_map.pcd", legacy)
    _write_evidence(output / "voxel_evidence.csv", voxel_map)
    after = {str(path.relative_to(session)): _sha256(path) for path in scan_paths}

    corrections = np.concatenate(correction_samples) if correction_samples else np.empty(0)
    raw_ghost_voxels = len(raw_ghosting_keys)
    deskewed_ghost_voxels = len(deskewed_ghosting_keys)
    metrics.update(structural_retention_metrics(voxel_map))
    metrics.update({
        "pose_revision": str(Path(pose_revision).name),
        "pose_revision_sha256": _sha256(pose_revision),
        "trajectory_revision": Path(trajectory_revision).name if trajectory_revision else None,
        "trajectory_revision_sha256": _sha256(trajectory_revision) if trajectory_revision else None,
        "input_scans": len(pose_records),
        "input_points": input_points,
        "deskewed_scans": deskewed_scans,
        "deskew_rejected_scans": len(pose_records) - deskewed_scans,
        "deskew_rejection_reasons": rejection_reasons,
        "deskewed_points": deskewed_points,
        "deskew_mode": "per_point_pose_interpolation" if trajectory else "scan_pose_fallback",
        "maximum_pose_bracket_ns": config.maximum_pose_bracket_ns,
        "voxel_compression_ratio": 1.0 - len(voxelized) / max(1, deskewed_points),
        "cleanup_removed_voxel_ratio": 1.0 - len(cleaned) / max(1, len(voxelized)),
        "raw_ghosting_proxy_voxels": raw_ghost_voxels,
        "deskewed_ghosting_proxy_voxels": deskewed_ghost_voxels,
        "ghosting_proxy_reduction_ratio": ((raw_ghost_voxels - deskewed_ghost_voxels) / raw_ghost_voxels if raw_ghost_voxels else 0.0),
        "deskew_correction_p50_m": float(np.percentile(corrections, 50)) if len(corrections) else 0.0,
        "deskew_correction_p95_m": float(np.percentile(corrections, 95)) if len(corrections) else 0.0,
        "raw_scan_archive_immutable": before == after,
        "scan_sha256": before,
        "archive_manifest_validated": manifest is not None,
        "calibration_direction": "pose_child_from_scan",
        "elapsed_seconds": time.perf_counter() - started,
        "peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
        "product_bytes": {path.name: path.stat().st_size for path in (raw_path, deskewed_path, output / "voxelized_map.pcd", output / "cleaned_map.pcd")},
    })
    (output / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
    )
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description="Build four offline map products from a pose revision")
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--poses", required=True, type=Path)
    parser.add_argument("--trajectory", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path)
    arguments = parser.parse_args(argv)
    configuration = MaintenanceConfig()
    if arguments.config:
        configuration = MaintenanceConfig(**yaml.safe_load(arguments.config.read_text(encoding="utf-8"))["map"])
    result = build_map_revision(
        arguments.session, arguments.poses, arguments.output, configuration,
        trajectory_revision=arguments.trajectory,
    )
    print(json.dumps(result, sort_keys=True))
    return 0
