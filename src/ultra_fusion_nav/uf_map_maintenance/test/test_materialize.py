from types import SimpleNamespace

import numpy as np

from uf_map_maintenance.archive import bag_record_command
from uf_map_maintenance.materialize import (
    epoch_for_stamp,
    livox_message_metadata,
    livox_points_to_arrays,
)


def test_bag_command_streams_raw_scan_imu_and_pose_to_disk():
    command = bag_record_command(
        "raw", ["/livox/lidar", "/livox/imu", "/Odometry"], "sqlite3"
    )
    assert command[:7] == [
        "ros2", "bag", "record", "--storage", "sqlite3", "--output", "raw"
    ]
    assert command[-3:] == ["/livox/lidar", "/livox/imu", "/Odometry"]


def test_livox_materialization_preserves_raw_point_semantics():
    points = [
        SimpleNamespace(x=1.0, y=2.0, z=3.0, reflectivity=7, offset_time=42, line=3, tag=5),
        SimpleNamespace(x=float("nan"), y=0.0, z=0.0, reflectivity=8, offset_time=43, line=4, tag=6),
    ]
    xyz_i, offset_time, line, tag = livox_points_to_arrays(points)
    np.testing.assert_allclose(xyz_i, [[1.0, 2.0, 3.0, 7.0]])
    np.testing.assert_array_equal(offset_time, [42])
    np.testing.assert_array_equal(line, [3])
    np.testing.assert_array_equal(tag, [5])


def test_epoch_assignment_uses_latest_applied_past_event_only():
    events = [(100, 7), (300, 8)]
    assert epoch_for_stamp(50, events) == 0
    assert epoch_for_stamp(100, events) == 7
    assert epoch_for_stamp(299, events) == 7
    assert epoch_for_stamp(300, events) == 8


def test_livox_scan_level_timestamp_and_identity_are_preserved():
    message = SimpleNamespace(timebase=123456, lidar_id=7, rsvd=[1, 2, 3])
    metadata = livox_message_metadata(message)
    assert metadata == {"timebase": 123456, "lidar_id": 7, "rsvd": [1, 2, 3]}
