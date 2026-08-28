"""Materialize immutable Livox scans and corresponding poses from rosbag2."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import numpy as np

from .association import PoseSample, associate_scan_to_pose
from .manifest import sha256_file, write_manifest_atomic
from .trajectory import write_pose_trajectory


def livox_points_to_arrays(points):
    values = []
    offsets = []
    lines = []
    tags = []
    for point in points:
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            continue
        values.append([point.x, point.y, point.z, float(point.reflectivity)])
        offsets.append(point.offset_time)
        lines.append(point.line)
        tags.append(point.tag)
    return (
        np.asarray(values, dtype=np.float64).reshape((-1, 4)),
        np.asarray(offsets, dtype=np.uint32),
        np.asarray(lines, dtype=np.uint8),
        np.asarray(tags, dtype=np.uint8),
    )


def livox_message_metadata(message):
    return {
        "timebase": int(message.timebase),
        "lidar_id": int(message.lidar_id),
        "rsvd": [int(value) for value in message.rsvd],
    }


def _stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def epoch_for_stamp(stamp_ns, events):
    """Return the latest applied epoch at or before stamp_ns; never use future state."""
    index = bisect.bisect_right(events, (stamp_ns, math.inf)) - 1
    return int(events[index][1]) if index >= 0 else 0


def _reader(bag_directory, storage_id):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def _topic_types(reader):
    from rosidl_runtime_py.utilities import get_message

    return {
        item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
    }


def _read_poses(bag_directory, pose_topic, epoch_topic, storage_id):
    from rclpy.serialization import deserialize_message

    reader = _reader(bag_directory, storage_id)
    types = _topic_types(reader)
    if pose_topic not in types:
        raise RuntimeError("pose topic missing from archive: " + pose_topic)
    raw_poses = {}
    epoch_events = []
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic == epoch_topic and topic in types:
            message = deserialize_message(data, types[topic])
            if message.applied:
                stamp = _stamp_ns(message.header) or int(bag_stamp)
                epoch_id = int(message.session_id or message.reset_counter)
                epoch_events.append((stamp, epoch_id))
        elif topic == pose_topic:
            message = deserialize_message(data, types[topic])
            stamp = _stamp_ns(message.header) or int(bag_stamp)
            pose = message.pose.pose
            raw_poses[stamp] = (
                (pose.position.x, pose.position.y, pose.position.z),
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            )
    epoch_events = sorted(set(epoch_events))
    output = []
    for stamp in sorted(raw_poses):
        translation, quaternion = raw_poses[stamp]
        sample = PoseSample(
            stamp, epoch_for_stamp(stamp, epoch_events), translation, quaternion
        )
        if sample.finite():
            output.append(sample)
    return output


def write_full_pose_trajectory(path, poses):
    """Persist every finite odometry sample, not only scan associations."""
    write_pose_trajectory(path, poses)


def materialize_archive(
    archive_root,
    scan_topic="/livox/lidar",
    pose_topic="/Odometry",
    epoch_topic="/fusion/epoch",
    tolerance_ns=50_000_000,
    storage_id="sqlite3",
    pose_bag_directory=None,
):
    from rclpy.serialization import deserialize_message

    root = Path(archive_root)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "complete":
        raise RuntimeError("raw archive is not complete")
    scans_directory = root / "scans"
    poses_directory = root / "poses"
    if scans_directory.exists() or poses_directory.exists():
        raise RuntimeError("materialization output already exists")
    scans_directory.mkdir()
    poses_directory.mkdir()

    raw_directory = root / "raw"
    pose_directory = Path(pose_bag_directory) if pose_bag_directory else raw_directory
    poses = _read_poses(pose_directory, pose_topic, epoch_topic, storage_id)
    write_full_pose_trajectory(poses_directory / "trajectory_original.csv", poses)
    reader = _reader(raw_directory, storage_id)
    types = _topic_types(reader)
    if scan_topic not in types:
        raise RuntimeError("scan topic missing from archive: " + scan_topic)

    accepted = []
    rejected = []
    scan_id = 0
    previous_stamp = None
    pose_epoch_steps = [(pose.stamp_ns, pose.epoch) for pose in poses]
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != scan_topic:
            continue
        message = deserialize_message(data, types[topic])
        stamp = _stamp_ns(message.header)
        association = associate_scan_to_pose(
            stamp,
            epoch_for_stamp(stamp, pose_epoch_steps),
            poses,
            tolerance_ns,
            previous_scan_stamp_ns=previous_stamp,
        )
        previous_stamp = stamp
        if not association.accepted:
            rejected.append((stamp, association.reason, association.delta_ns))
            continue
        xyz_i, offset_time, line, tag = livox_points_to_arrays(message.points)
        if len(xyz_i) == 0:
            rejected.append((stamp, "empty_finite_scan", None))
            continue
        metadata = livox_message_metadata(message)
        np.savez(
            scans_directory / f"{scan_id:06d}.npz",
            points=xyz_i,
            offset_time=offset_time,
            line=line,
            tag=tag,
            source_stamp_ns=np.asarray([stamp], dtype=np.int64),
            timebase=np.asarray([metadata["timebase"]], dtype=np.uint64),
            lidar_id=np.asarray([metadata["lidar_id"]], dtype=np.uint8),
            rsvd=np.asarray(metadata["rsvd"], dtype=np.uint8),
        )
        accepted.append((scan_id, stamp, association.pose))
        scan_id += 1

    original = poses_directory / "original.csv"
    with original.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scan_id", "stamp_ns", "epoch", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
        for item_id, stamp, pose in accepted:
            writer.writerow([item_id, stamp, pose.epoch, *pose.translation, *pose.quaternion_xyzw])
    with (poses_directory / "rejected.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_ns", "reason", "delta_ns"])
        writer.writerows(rejected)

    manifest["materialization"].update({
        "status": "complete",
        "scan_topic": scan_topic,
        "pose_topic": pose_topic,
        "epoch_topic": epoch_topic,
        "association_tolerance_ns": tolerance_ns,
        "accepted_scans": len(accepted),
        "rejected_scans": len(rejected),
        "trajectory_samples": len(poses),
        "full_pose_trajectory": "poses/trajectory_original.csv",
    })
    manifest["artifacts"] = [
        {
            "path": str(path.relative_to(root)),
            "sha256": sha256_file(path),
            "bytes": path.stat().st_size,
        }
        for path in sorted(root.rglob("*"))
        if path.is_file() and path != manifest_path
    ]
    write_manifest_atomic(manifest, manifest_path)
    return manifest["materialization"]


def main(argv=None):
    parser = argparse.ArgumentParser(description="Materialize raw Livox scans and poses")
    parser.add_argument("--archive", required=True, type=Path)
    parser.add_argument("--scan-topic", default="/livox/lidar")
    parser.add_argument("--pose-topic", default="/Odometry")
    parser.add_argument("--epoch-topic", default="/fusion/epoch")
    parser.add_argument("--tolerance-ms", type=float, default=50.0)
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--pose-bag", type=Path)
    arguments = parser.parse_args(argv)
    materialize_archive(
        arguments.archive,
        arguments.scan_topic,
        arguments.pose_topic,
        arguments.epoch_topic,
        int(arguments.tolerance_ms * 1_000_000),
        arguments.storage_id,
        arguments.pose_bag,
    )
    return 0
