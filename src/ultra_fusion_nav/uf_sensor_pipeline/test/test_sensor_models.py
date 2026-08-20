import struct
import unittest

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, NavSatFix, PointCloud2, PointField

from uf_sensor_pipeline.fault_models import (
    add_depth_holes,
    add_gnss_jump,
    add_moving_lidar_cluster,
    ensure_monotonic_stamp,
    shift_stamp,
)
from uf_sensor_pipeline.pointcloud_utils import filter_cloud


def cloud(points):
    msg = PointCloud2()
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.data = b"".join(struct.pack("<ffff", *point) for point in points)
    return msg


class SensorModelsTest(unittest.TestCase):
    def test_body_filter_preserves_full_point_records(self):
        msg = cloud([(0.2, 0.1, 0.0, 10.0), (1.0, 0.0, 0.0, 20.0)])

        output, removed_body, removed_range, total = filter_cloud(
            msg, (-0.25, 0.25, -0.25, 0.25, -0.05, 0.05), 0.1, 40.0
        )

        self.assertEqual((removed_body, removed_range, total), (1, 0, 2))
        self.assertEqual(output.width, 1)
        self.assertEqual(struct.unpack("<ffff", output.data), (1.0, 0.0, 0.0, 20.0))

    def test_body_filter_uses_closed_50_by_50_by_10_cm_envelope(self):
        msg = cloud([
            (0.25, 0.25, 0.0, 10.0),
            (0.251, 0.0, 0.0, 20.0),
            (0.20, 0.0, 0.051, 30.0),
        ])

        output, removed_body, removed_range, total = filter_cloud(
            msg, (-0.25, 0.25, -0.25, 0.25, -0.05, 0.05), 0.1, 40.0
        )

        self.assertEqual((removed_body, removed_range, total), (1, 0, 3))
        self.assertEqual(output.width, 2)

    def test_body_filter_applies_lidar_to_body_rotation(self):
        msg = cloud([(1.0, 0.0, 0.0, 10.0)])

        output, removed_body, removed_range, total = filter_cloud(
            msg,
            (0.90, 1.10, -0.10, 0.10, -0.20, -0.10),
            0.1,
            40.0,
            (0.984807753, 0.0, 0.173648178, 0.0, 1.0, 0.0, -0.173648178, 0.0, 0.984807753),
            (0.0, 0.0, 0.0),
        )

        self.assertEqual((removed_body, removed_range, total), (1, 0, 1))
        self.assertEqual(output.width, 0)

    def test_stamp_offset_handles_second_boundary(self):
        stamp = Time(sec=10, nanosec=900_000_000)

        shift_stamp(stamp, 0.2)

        self.assertEqual((stamp.sec, stamp.nanosec), (11, 100_000_000))

    def test_nonmonotonic_stamp_gets_minimal_repair(self):
        stamp = Time(sec=10, nanosec=99)

        last, repaired = ensure_monotonic_stamp(stamp, 10_000_000_100)

        self.assertTrue(repaired)
        self.assertEqual(last, 10_000_000_101)
        self.assertEqual((stamp.sec, stamp.nanosec), (10, 101))

    def test_gnss_jump_is_meter_scale(self):
        msg = NavSatFix()
        msg.latitude = 45.0
        msg.longitude = 120.0

        output = add_gnss_jump(msg, 10.0, 0.0)

        self.assertGreater(output.latitude, msg.latitude)
        self.assertAlmostEqual(output.longitude, msg.longitude, places=9)

    def test_depth_holes_are_repeatable(self):
        msg = Image()
        msg.encoding = "16UC1"
        msg.data = np.full(100, 1000, dtype=np.uint16).tobytes()

        first = add_depth_holes(msg, 0.5, np.random.default_rng(3))
        second = add_depth_holes(msg, 0.5, np.random.default_rng(3))

        self.assertEqual(first.data, second.data)
        self.assertGreater(np.count_nonzero(np.frombuffer(first.data, dtype=np.uint16) == 0), 30)

    def test_moving_lidar_cluster_preserves_input_and_uses_elapsed_time(self):
        msg = cloud([(1.0, 0.0, 0.0, 20.0), (2.0, 0.0, 0.0, 30.0)])

        first = add_moving_lidar_cluster(msg, 8, elapsed_s=0.0, speed_mps=1.0)
        second = add_moving_lidar_cluster(msg, 8, elapsed_s=1.0, speed_mps=1.0)

        self.assertEqual(first.width, 10)
        self.assertEqual(first.data[:len(msg.data)], msg.data)
        first_points = np.asarray([
            struct.unpack_from("<fff", first.data, index * first.point_step)
            for index in range(msg.width, first.width)
        ])
        second_points = np.asarray([
            struct.unpack_from("<fff", second.data, index * second.point_step)
            for index in range(msg.width, second.width)
        ])
        self.assertTrue(np.all(np.isfinite(first_points)))
        self.assertAlmostEqual(
            float(np.mean(second_points[:, 1]) - np.mean(first_points[:, 1])),
            1.0,
            places=5,
        )


if __name__ == "__main__":
    unittest.main()
