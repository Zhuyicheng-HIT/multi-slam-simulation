import numpy as np
import unittest

from uf_shared_mapping.voxel_map import SourceAwareVoxelMap


def test_lidar_geometry_is_not_overwritten_by_rgbd():
    mapping = SourceAwareVoxelMap(voxel_size_m=0.5, conflict_distance_m=0.2)
    mapping.integrate_lidar([[0.1, 0.1, 0.1]])
    before = mapping.arrays("lidar")[0].copy()
    assert mapping.integrate_rgbd([[0.19, 0.1, 0.1]], [[255, 0, 0]], 1.0) == 1
    np.testing.assert_allclose(mapping.arrays("lidar")[0], before)
    assert mapping.summary()["joint_voxels"] == 1


def test_conflicting_rgbd_is_rejected_and_supplement_is_separate():
    mapping = SourceAwareVoxelMap(voxel_size_m=1.0, conflict_distance_m=0.1)
    mapping.integrate_lidar([[0.05, 0.05, 0.05]])
    assert mapping.integrate_rgbd([[0.8, 0.8, 0.8]], [[0, 255, 0]], 1.0) == 0
    assert mapping.summary()["rgbd_conflicts"] == 1
    assert mapping.integrate_rgbd([[2.0, 0.0, 0.0]], [[0, 0, 255]], 0.9) == 1
    assert mapping.summary()["supplementary_rgbd_voxels"] == 1


def test_low_reliability_does_not_mutate_map():
    mapping = SourceAwareVoxelMap(minimum_visual_reliability=0.5)
    mapping.integrate_rgbd([[1.0, 2.0, 3.0]], [[1, 2, 3]], 0.2)
    assert mapping.summary()["voxel_count"] == 0


class VoxelMapUnittest(unittest.TestCase):
    def test_lidar_geometry(self):
        test_lidar_geometry_is_not_overwritten_by_rgbd()

    def test_conflict_and_supplement(self):
        test_conflicting_rgbd_is_rejected_and_supplement_is_separate()

    def test_low_reliability(self):
        test_low_reliability_does_not_mutate_map()
