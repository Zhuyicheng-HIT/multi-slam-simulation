import csv
import hashlib
import json

import numpy as np

from uf_map_maintenance.builder import build_map_revision
from uf_map_maintenance.voxel_map import MaintenanceConfig


def write_poses(path, translations):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scan_id", "stamp_ns", "epoch", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
        for scan_id, translation in enumerate(translations):
            writer.writerow([scan_id, 1000 + scan_id, 1, *translation, 0, 0, 0, 1])


def test_corrected_pose_revision_rebuilds_without_mutating_scans(tmp_path):
    scans = tmp_path / "scans"
    scans.mkdir()
    for scan_id in range(2):
        np.savez(scans / f"{scan_id:06d}.npz", points=np.array([[0.1, 0.1, 0.1, 1.0]]))
    original = tmp_path / "original.csv"
    corrected = tmp_path / "corrected.csv"
    write_poses(original, [(0, 0, 0), (1, 0, 0)])
    write_poses(corrected, [(0, 0, 0), (0, 0, 0)])
    scan_hashes = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in scans.iterdir()
    }

    config = MaintenanceConfig(
        voxel_size_m=1.0,
        minimum_scan_support=1,
        stable_support_scans=1,
        minimum_component_voxels=1,
        isolation_neighbor_threshold=0,
    )
    original_result = build_map_revision(tmp_path, original, tmp_path / "out-original", config)
    corrected_result = build_map_revision(tmp_path, corrected, tmp_path / "out-corrected", config)

    assert original_result["static_voxels"] == 2
    assert corrected_result["static_voxels"] == 1
    assert original_result["pose_revision_sha256"] != corrected_result["pose_revision_sha256"]
    assert scan_hashes == {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in scans.iterdir()
    }
    metrics = json.loads((tmp_path / "out-corrected" / "metrics.json").read_text())
    assert metrics["raw_scan_archive_immutable"] is True
    assert (tmp_path / "out-corrected" / "voxel_evidence.csv").is_file()
