import csv
import hashlib
import json

import numpy as np

from uf_map_maintenance.builder import build_map_revision
from uf_map_maintenance.pcd import read_binary_pcd
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


def test_rebuild_emits_raw_deskewed_voxelized_and_cleaned_products(tmp_path):
    scans = tmp_path / "scans"
    poses = tmp_path / "poses"
    scans.mkdir()
    poses.mkdir()
    np.savez(
        scans / "000000.npz",
        points=np.array([[1.0, 0.0, 0.0, 10.0], [1.0, 0.0, 0.0, 20.0]]),
        offset_time=np.array([0, 100], dtype=np.uint32),
        timebase=np.array([0], dtype=np.uint64),
    )
    write_poses(poses / "original.csv", [(0, 0, 0)])
    with (poses / "trajectory_original.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_ns", "epoch", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
        writer.writerow([0, 1, 0, 0, 0, 0, 0, 0, 1])
        writer.writerow([100, 1, 1, 0, 0, 0, 0, 0, 1])

    config = MaintenanceConfig(
        voxel_size_m=0.25,
        minimum_scan_support=1,
        stable_support_scans=1,
        minimum_component_voxels=1,
        isolation_neighbor_threshold=0,
        maximum_pose_bracket_ns=100,
    )
    result = build_map_revision(
        tmp_path, poses / "original.csv", tmp_path / "products", config,
        trajectory_revision=poses / "trajectory_original.csv",
    )
    raw = read_binary_pcd(tmp_path / "products" / "raw_scan_pose_map.pcd")
    deskewed = read_binary_pcd(tmp_path / "products" / "deskewed_map.pcd")
    np.testing.assert_allclose(raw[:, 0], [1.0, 1.0])
    np.testing.assert_allclose(deskewed[:, 0], [1.0, 2.0])
    for name in ("voxelized_map.pcd", "cleaned_map.pcd", "cleaned_global_map.pcd"):
        assert (tmp_path / "products" / name).is_file()
    assert result["deskew_mode"] == "per_point_pose_interpolation"
    assert result["deskewed_scans"] == 1
    assert result["deskew_rejected_scans"] == 0
