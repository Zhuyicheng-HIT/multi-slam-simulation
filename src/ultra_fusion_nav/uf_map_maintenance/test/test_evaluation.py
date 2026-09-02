import numpy as np

from uf_map_maintenance.evaluation import structural_retention_metrics
from uf_map_maintenance.voxel_map import EvidenceVoxelMap, MaintenanceConfig


def test_structural_retention_keeps_supported_ground_and_wall():
    config = MaintenanceConfig(
        voxel_size_m=1.0,
        minimum_scan_support=2,
        stable_support_scans=3,
        minimum_component_voxels=3,
        isolation_neighbor_threshold=1,
        structural_sample_limit=1000,
    )
    voxel_map = EvidenceVoxelMap(config)
    ground = np.array([[x + 0.1, y + 0.1, 0.1] for x in range(5) for y in range(5)])
    wall = np.array([[0.1, y + 0.1, z + 0.1] for y in range(5) for z in range(1, 6)])
    structure = np.unique(np.vstack((ground, wall)), axis=0)
    for scan_id in range(3):
        voxel_map.add_scan(scan_id, structure)
    voxel_map.add_scan(99, np.array([[30.1, 30.1, 8.1]]))
    voxel_map.cleaned_points()

    metrics = structural_retention_metrics(voxel_map)
    assert metrics["sampled_voxels"] == len(voxel_map.voxels)
    assert metrics["ground_candidates"] > 0
    assert metrics["wall_candidates"] > 0
    assert metrics["ground_retention_ratio"] == 1.0
    assert metrics["wall_retention_ratio"] == 1.0
    assert metrics["stable_ground_retention_ratio"] == 1.0
    assert metrics["stable_wall_retention_ratio"] == 1.0
