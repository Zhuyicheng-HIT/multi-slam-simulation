import numpy as np
import unittest

from sensor_msgs.msg import PointField
from uf_shared_mapping.shared_mapping_node import (
    structured_xyz_array,
    structured_xyzrgb_array,
)
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


def test_humble_structured_pointcloud_xyz_is_extracted():
    points = np.zeros(
        2,
        dtype={
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [0, 4, 8],
            "itemsize": 48,
        },
    )
    points["x"] = [1.0, 4.0]
    points["y"] = [2.0, 5.0]
    points["z"] = [3.0, 6.0]
    np.testing.assert_allclose(
        structured_xyz_array(points), [[1, 2, 3], [4, 5, 6]]
    )


def test_batched_keys_preserve_ordered_means_and_skip_nonfinite_rows():
    mapping = SourceAwareVoxelMap(voxel_size_m=1.0, conflict_distance_m=0.5)
    mapping.integrate_lidar([
        [0.1, 0.1, 0.1], [0.3, 0.1, 0.1], [np.nan, 0.0, 0.0]
    ])
    points, _ = mapping.arrays("lidar")
    np.testing.assert_allclose(points, [[0.2, 0.1, 0.1]])
    assert mapping.summary()["lidar_points"] == 2
    accepted = mapping.integrate_rgbd(
        [[0.2, 0.1, 0.1], [2.1, 0.0, 0.0], [3.0, np.inf, 0.0]],
        [[300, -10, 50], [10, 20, 30], [1, 2, 3]],
        1.0,
    )
    assert accepted == 2
    _, colors = mapping.arrays("rgbd")
    assert any(np.array_equal(color, [255, 0, 50]) for color in colors)
    assert mapping.summary()["rgbd_points"] == 2


def test_structured_xyzrgb_packs_expected_pointcloud_values():
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    rows = structured_xyzrgb_array(
        [[1.0, 2.0, 3.0]], [[0x12, 0x34, 0x56]], fields
    )
    np.testing.assert_allclose(
        [rows["x"][0], rows["y"][0], rows["z"][0]], [1.0, 2.0, 3.0]
    )
    assert int(rows["rgb"][0]) == 0x123456


class VoxelMapUnittest(unittest.TestCase):
    def test_lidar_geometry(self):
        test_lidar_geometry_is_not_overwritten_by_rgbd()

    def test_conflict_and_supplement(self):
        test_conflicting_rgbd_is_rejected_and_supplement_is_separate()

    def test_low_reliability(self):
        test_low_reliability_does_not_mutate_map()

    def test_structured_pointcloud(self):
        test_humble_structured_pointcloud_xyz_is_extracted()

    def test_batched_keys_and_nonfinite_rows(self):
        test_batched_keys_preserve_ordered_means_and_skip_nonfinite_rows()

    def test_structured_xyzrgb(self):
        test_structured_xyzrgb_packs_expected_pointcloud_values()
