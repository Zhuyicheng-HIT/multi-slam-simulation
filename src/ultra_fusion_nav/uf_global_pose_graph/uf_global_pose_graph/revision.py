"""Versioned corrected-pose output without mutating original archive poses."""

import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .se3 import compose, inverse, pose_matrix, transform_to_pose


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_records(path, has_scan_id):
    records = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            records.append({
                "scan_id": int(row["scan_id"]) if has_scan_id else None,
                "stamp_ns": int(row["stamp_ns"]),
                "epoch": int(row["epoch"]),
                "pose": pose_matrix(
                    [float(row[name]) for name in ("tx", "ty", "tz")],
                    [float(row[name]) for name in ("qx", "qy", "qz", "qw")],
                ),
            })
    if not records:
        raise ValueError("pose revision input is empty")
    stamps = [record["stamp_ns"] for record in records]
    if any(right <= left for left, right in zip(stamps[:-1], stamps[1:])):
        raise ValueError(
            "pose revision timestamps must be strictly increasing")
    return records


def _correction_interpolator(nodes, corrected_poses):
    stamps = np.asarray([node.stamp_ns for node in nodes], dtype=np.int64)
    corrections = [
        compose(corrected_poses[node.keyframe_id], inverse(node.original_pose))
        for node in nodes
    ]
    translations = np.asarray([value[:3, 3] for value in corrections])
    rotations = Rotation.from_matrix([value[:3, :3] for value in corrections])
    relative_time = (stamps - stamps[0]).astype(np.float64) * 1.0e-9
    slerp = Slerp(relative_time, rotations)

    def interpolate(stamp_ns):
        query = float(np.clip(
            (int(stamp_ns) - int(stamps[0])) * 1.0e-9,
            relative_time[0], relative_time[-1],
        ))
        translation = np.array([
            np.interp(query, relative_time, translations[:, axis]) for axis in range(3)
        ])
        output = np.eye(4)
        output[:3, :3] = slerp([query]).as_matrix()[0]
        output[:3, 3] = translation
        return output
    return interpolate


def _write_records(path, records, has_scan_id, correction_at):
    header = (["scan_id"] if has_scan_id else []) + [
        "stamp_ns", "epoch", "tx", "ty", "tz", "qx", "qy", "qz", "qw"
    ]
    with Path(path).open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for record in records:
            corrected = compose(
                correction_at(
                    record["stamp_ns"]),
                record["pose"])
            translation, quaternion = transform_to_pose(corrected)
            prefix = [record["scan_id"]] if has_scan_id else []
            writer.writerow(prefix + [
                record["stamp_ns"], record["epoch"],
                *(format(float(value), ".17g") for value in translation),
                *(format(float(value), ".17g") for value in quaternion),
            ])


def _write_keyframes(path, nodes, corrected_poses):
    with Path(path).open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "keyframe_id", "scan_id", "stamp_ns", "tx", "ty", "tz",
            "qx", "qy", "qz", "qw",
        ])
        for node in nodes:
            translation, quaternion = transform_to_pose(
                corrected_poses[node.keyframe_id])
            writer.writerow([
                node.keyframe_id, node.scan_id, node.stamp_ns,
                *(format(float(value), ".17g") for value in translation),
                *(format(float(value), ".17g") for value in quaternion),
            ])


def create_pose_revision(
    output_dir, revision_id, original_pose_path, original_trajectory_path,
    keyframe_nodes, corrected_poses, provenance,
):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    nodes = list(keyframe_nodes)
    if not nodes or nodes[0].keyframe_id not in corrected_poses:
        raise ValueError("corrected keyframe poses are incomplete")
    correction_at = _correction_interpolator(nodes, corrected_poses)
    poses_path = output / f"poses_{revision_id}.csv"
    trajectory_path = output / f"trajectory_{revision_id}.csv"
    keyframes_path = output / f"keyframes_{revision_id}.csv"
    _write_records(
        poses_path, _load_records(
            original_pose_path, True), True, correction_at
    )
    _write_records(
        trajectory_path, _load_records(original_trajectory_path, False), False,
        correction_at,
    )
    _write_keyframes(keyframes_path, nodes, corrected_poses)
    manifest_path = output / "pose_revision_manifest.json"
    manifest = {
        "schema_version": 1,
        "revision_id": str(revision_id),
        "correction_interpolation": "linear_translation_shortest_path_slerp",
        "first_keyframe_fixed": True,
        "inputs": {
            "original_pose_path": str(
                Path(original_pose_path).name),
            "original_pose_sha256": _sha256(original_pose_path),
            "original_trajectory_path": str(
                Path(original_trajectory_path).name),
            "original_trajectory_sha256": _sha256(original_trajectory_path),
            **dict(provenance),
        },
        "outputs": {
            "poses": poses_path.name,
            "poses_sha256": _sha256(poses_path),
            "trajectory": trajectory_path.name,
            "trajectory_sha256": _sha256(trajectory_path),
            "keyframes": keyframes_path.name,
            "keyframes_sha256": _sha256(keyframes_path),
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        "poses": poses_path,
        "trajectory": trajectory_path,
        "keyframes": keyframes_path,
        "manifest": manifest_path,
    }
