import numpy as np

from uf_map_maintenance.voxel_map import EvidenceVoxelMap, MaintenanceConfig


def test_distinct_scan_support_and_duplicate_suppression():
    voxel_map = EvidenceVoxelMap(MaintenanceConfig(voxel_size_m=1.0))
    voxel_map.add_scan(1, np.array([[0.1, 0.1, 0.1], [0.2, 0.1, 0.1]]))
    voxel_map.add_scan(2, np.array([[0.3, 0.1, 0.1]]))
    evidence = next(iter(voxel_map.voxels.values()))
    assert evidence.point_count == 3
    assert evidence.scan_support == 2


def test_support_count_is_not_limited_by_bounded_provenance():
    config = MaintenanceConfig(voxel_size_m=1.0, maximum_provenance_scan_ids=2)
    voxel_map = EvidenceVoxelMap(config)
    for scan_id in range(10):
        voxel_map.add_scan(scan_id, np.array([[0.1, 0.1, 0.1]]))
    evidence = next(iter(voxel_map.voxels.values()))
    assert evidence.scan_support == 10
    assert len(evidence.scan_ids) == 2


def test_cleanup_removes_isolated_noise_and_preserves_stable_wall():
    config = MaintenanceConfig(
        voxel_size_m=1.0,
        minimum_scan_support=2,
        stable_support_scans=3,
        minimum_component_voxels=3,
    )
    voxel_map = EvidenceVoxelMap(config)
    wall = np.array([[0.1, y + 0.1, 0.1] for y in range(4)])
    for scan_id in range(3):
        voxel_map.add_scan(scan_id, wall)
    voxel_map.add_scan(99, np.array([[20.1, 20.1, 5.1]]))
    cleaned, metrics = voxel_map.cleaned_points()
    assert len(cleaned) == 4
    assert metrics["removed_low_support"] == 1
    assert metrics["static_voxels"] == 4


def test_small_low_support_floating_component_is_removed():
    config = MaintenanceConfig(
        voxel_size_m=1.0,
        minimum_scan_support=1,
        stable_support_scans=4,
        minimum_component_voxels=3,
    )
    voxel_map = EvidenceVoxelMap(config)
    voxel_map.add_scan(1, np.array([[3.1, 3.1, 4.1], [4.1, 3.1, 4.1]]))
    cleaned, metrics = voxel_map.cleaned_points()
    assert len(cleaned) == 0
    assert metrics["removed_small_component"] == 2


def test_all_points_exposes_deterministic_pre_cleanup_voxels():
    voxel_map = EvidenceVoxelMap(MaintenanceConfig(voxel_size_m=1.0))
    voxel_map.add_scan(1, np.array([[2.1, 0.1, 0.1, 2], [0.1, 0.1, 0.1, 1]]))
    points = voxel_map.all_points()
    np.testing.assert_allclose(points[:, 0], [0.1, 2.1])
    np.testing.assert_allclose(points[:, 3], [1, 2])
