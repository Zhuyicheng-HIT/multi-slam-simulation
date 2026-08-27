"""Rebuild a cleaned global map from immutable scans and a pose revision."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from .manifest import validate_manifest
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


def _write_pcd(path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA ascii\n"
    )
    with Path(path).open("w", encoding="utf-8") as stream:
        stream.write(header)
        for point in points:
            stream.write("{:.9g} {:.9g} {:.9g} {:.9g}\n".format(*point))


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


def build_map_revision(session_dir, pose_revision, output_dir, config=None):
    session = Path(session_dir)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    config = config or MaintenanceConfig()
    pose_records = _read_poses(pose_revision)
    calibration_translation, calibration_quaternion, manifest = (
        _load_scan_to_pose_child_calibration(session)
    )
    scan_paths = [session / "scans" / f"{record['scan_id']:06d}.npz" for record in pose_records]
    before = {str(path.relative_to(session)): _sha256(path) for path in scan_paths}
    voxel_map = EvidenceVoxelMap(config)
    input_points = 0
    for record, scan_path in zip(pose_records, scan_paths):
        with np.load(scan_path, allow_pickle=False) as archive:
            points = np.asarray(archive["points"], dtype=np.float64)
        input_points += len(points)
        transformed = points.copy()
        calibration_rotation = _rotation_matrix(calibration_quaternion)
        transformed[:, :3] = points[:, :3] @ calibration_rotation.T
        transformed[:, :3] += calibration_translation
        transformed[:, :3] = transformed[:, :3] @ _rotation_matrix(record["quaternion"]).T
        transformed[:, :3] += record["translation"]
        voxel_map.add_scan(record["scan_id"], transformed)

    cleaned, metrics = voxel_map.cleaned_points()
    _write_pcd(output / "cleaned_global_map.pcd", cleaned)
    _write_evidence(output / "voxel_evidence.csv", voxel_map)
    after = {str(path.relative_to(session)): _sha256(path) for path in scan_paths}
    metrics.update({
        "pose_revision": str(Path(pose_revision).name),
        "pose_revision_sha256": _sha256(pose_revision),
        "input_scans": len(pose_records),
        "input_points": input_points,
        "raw_scan_archive_immutable": before == after,
        "scan_sha256": before,
        "archive_manifest_validated": manifest is not None,
        "calibration_direction": "pose_child_from_scan",
    })
    (output / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description="Build cleaned global map from a pose revision")
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--poses", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path)
    arguments = parser.parse_args(argv)
    configuration = MaintenanceConfig()
    if arguments.config:
        values = yaml.safe_load(arguments.config.read_text(encoding="utf-8"))["map"]
        configuration = MaintenanceConfig(**values)
    build_map_revision(arguments.session, arguments.poses, arguments.output, configuration)
    return 0
