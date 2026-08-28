"""Stream a recorded PointCloud2 MID360 bag into an exact Livox CustomMsg bag."""

import argparse
import json
from pathlib import Path

from .manifest import sha256_file, write_manifest_atomic
from .pointcloud_adapter import decode_mid360_pointcloud2


def _stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _tree_artifacts(root):
    root = Path(root)
    return [
        {
            "path": str(path.relative_to(root)),
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        for path in sorted(root.rglob("*"))
        if path.is_file()
    ]


def prepare_output_path(output):
    """Validate output paths while leaving the rosbag URI itself absent."""
    output = Path(output).resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    temporary = output.with_name(output.name + ".incomplete")
    if temporary.exists():
        raise RuntimeError(f"incomplete output already exists: {temporary}")
    output.parent.mkdir(parents=True, exist_ok=True)
    return temporary


def normalize_bag(
    source, output, lidar_topic="/livox/lidar", imu_topic="/livox/imu",
    storage_id="sqlite3"
):
    import rosbag2_py
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import PointCloud2

    source = Path(source).resolve()
    output = Path(output).resolve()
    if not source.is_dir():
        raise RuntimeError(f"source bag directory does not exist: {source}")
    temporary = prepare_output_path(output)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {item.name: item for item in reader.get_all_topics_and_types()}
    if lidar_topic not in topics or topics[lidar_topic].type != "sensor_msgs/msg/PointCloud2":
        raise RuntimeError("source LiDAR topic is not sensor_msgs/msg/PointCloud2")
    if imu_topic not in topics:
        raise RuntimeError("source IMU topic is missing")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(temporary), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    qos = getattr(topics[lidar_topic], "offered_qos_profiles", "")
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=lidar_topic,
            type="livox_ros_driver2/msg/CustomMsg",
            serialization_format="cdr",
            offered_qos_profiles=qos,
        )
    )
    imu_metadata = topics[imu_topic]
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=imu_topic,
            type=imu_metadata.type,
            serialization_format="cdr",
            offered_qos_profiles=getattr(imu_metadata, "offered_qos_profiles", ""),
        )
    )

    counters = {
        "lidar_messages": 0,
        "imu_messages": 0,
        "source_points": 0,
        "output_points": 0,
        "rejected_nonfinite_xyz": 0,
        "zero_reflectivity_from_nonfinite": 0,
        "minimum_offset_ns": None,
        "maximum_offset_ns": None,
    }
    while reader.has_next():
        topic, serialized, bag_stamp = reader.read_next()
        if topic == imu_topic:
            writer.write(topic, serialized, bag_stamp)
            counters["imu_messages"] += 1
            continue
        if topic != lidar_topic:
            continue
        source_message = deserialize_message(serialized, PointCloud2)
        stamp_ns = _stamp_ns(source_message.header)
        scan = decode_mid360_pointcloud2(
            data=source_message.data,
            width=source_message.width,
            height=source_message.height,
            point_step=source_message.point_step,
            row_step=source_message.row_step,
            fields=source_message.fields,
            is_bigendian=source_message.is_bigendian,
            header_stamp_ns=stamp_ns,
        )
        output_message = CustomMsg()
        output_message.header = source_message.header
        output_message.timebase = stamp_ns
        output_message.point_num = scan.finite_points
        output_message.lidar_id = 0
        output_message.rsvd = [0, 0, 0]
        converted = []
        for index in range(scan.finite_points):
            point = CustomPoint()
            point.offset_time = int(scan.offset_time[index])
            point.x = float(scan.points_xyz[index, 0])
            point.y = float(scan.points_xyz[index, 1])
            point.z = float(scan.points_xyz[index, 2])
            point.reflectivity = int(scan.reflectivity[index])
            point.tag = int(scan.tag[index])
            point.line = int(scan.line[index])
            converted.append(point)
        output_message.points = converted
        writer.write(lidar_topic, serialize_message(output_message), bag_stamp)

        counters["lidar_messages"] += 1
        counters["source_points"] += scan.source_points
        counters["output_points"] += scan.finite_points
        counters["rejected_nonfinite_xyz"] += scan.rejected_nonfinite_xyz
        counters["zero_reflectivity_from_nonfinite"] += scan.zero_reflectivity_from_nonfinite
        if scan.offset_time.size:
            minimum = int(scan.offset_time.min())
            maximum = int(scan.offset_time.max())
            counters["minimum_offset_ns"] = minimum if counters["minimum_offset_ns"] is None else min(counters["minimum_offset_ns"], minimum)
            counters["maximum_offset_ns"] = maximum if counters["maximum_offset_ns"] is None else max(counters["maximum_offset_ns"], maximum)

    del writer
    manifest = {
        "schema_version": 1,
        "status": "complete",
        "source": str(source),
        "output": str(output),
        "lidar_topic": lidar_topic,
        "imu_topic": imu_topic,
        "source_lidar_type": "sensor_msgs/msg/PointCloud2",
        "output_lidar_type": "livox_ros_driver2/msg/CustomMsg",
        "time_contract": (
            "timebase=header_stamp_ns; "
            "offset_time=round(float64(point.timestamp)-float64(timebase))"
        ),
        "point_order_preserved": True,
        "counters": counters,
        "source_artifacts": _tree_artifacts(source),
        "output_artifacts": _tree_artifacts(temporary),
    }
    write_manifest_atomic(manifest, temporary / "normalization_manifest.json")
    temporary.rename(output)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description="Normalize recorded MID360 PointCloud2 to Livox CustomMsg")
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--storage-id", default="sqlite3")
    arguments = parser.parse_args(argv)
    result = normalize_bag(
        arguments.source, arguments.output, arguments.lidar_topic,
        arguments.imu_topic, arguments.storage_id
    )
    print(json.dumps(result["counters"], sort_keys=True))
    return 0
