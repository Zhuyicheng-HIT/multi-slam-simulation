#!/usr/bin/env python3
"""Export synchronized browser playback assets from an M2DGR ROS 2 bag."""

import argparse
import bisect
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def point_xyz(message):
    offsets = {field.name: field.offset for field in message.fields}
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": message.point_step,
        }
    )
    points = np.frombuffer(message.data, dtype=dtype)
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    return xyz[np.isfinite(xyz).all(axis=1)]


def sampled(points, limit):
    if len(points) <= limit:
        return points
    stride = math.ceil(len(points) / limit)
    return points[::stride][:limit]


def rotation_matrix(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def to_three(points):
    return np.column_stack((points[:, 0], points[:, 2], -points[:, 1])).astype("<f4")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output")
    parser.add_argument("--local-points", type=int, default=5000)
    parser.add_argument("--map-points", type=int, default=1400)
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {name: get_message(kind) for name, kind in types.items()}
    bridge = CvBridge()

    bag_start = None
    last_time = 0.0
    odometry = []
    lidar = []
    rgb_writer = None
    depth_writer = None
    rgb_frames = 0
    depth_frames = 0
    frame_size = None

    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        if bag_start is None:
            bag_start = stamp
        time = (stamp - bag_start) / 1e9
        last_time = max(last_time, time)
        if topic not in message_types:
            continue
        if topic == "/odom":
            message = deserialize_message(payload, message_types[topic])
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            odometry.append(
                (
                    time,
                    np.array([position.x, position.y, position.z], dtype=np.float32),
                    (orientation.x, orientation.y, orientation.z, orientation.w),
                )
            )
        elif topic == "/rslidar_points":
            message = deserialize_message(payload, message_types[topic])
            points = point_xyz(message)
            points = points[np.linalg.norm(points, axis=1) < 90.0]
            lidar.append((time, sampled(points, args.local_points).astype(np.float32)))
        elif topic == "/camera/color/image_raw":
            message = deserialize_message(payload, message_types[topic])
            image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if rgb_writer is None:
                frame_size = (image.shape[1], image.shape[0])
                rgb_writer = cv2.VideoWriter(
                    str(output / "camera-rgb.webm"),
                    cv2.VideoWriter_fourcc(*"VP80"),
                    15.0,
                    frame_size,
                )
            rgb_writer.write(image)
            rgb_frames += 1
        elif topic == "/camera/aligned_depth_to_color/image_raw":
            message = deserialize_message(payload, message_types[topic])
            depth = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            normalized = np.clip(depth.astype(np.float32) / 6000.0 * 255.0, 0, 255).astype(np.uint8)
            image = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
            image[depth == 0] = 0
            if depth_writer is None:
                depth_writer = cv2.VideoWriter(
                    str(output / "camera-depth.webm"),
                    cv2.VideoWriter_fourcc(*"VP80"),
                    15.0,
                    (image.shape[1], image.shape[0]),
                )
            depth_writer.write(image)
            depth_frames += 1

    if rgb_writer:
        rgb_writer.release()
    if depth_writer:
        depth_writer.release()
    if not odometry or not lidar:
        raise RuntimeError("bag must contain /odom and /rslidar_points")

    odom_times = [item[0] for item in odometry]
    origin = odometry[0][1]
    local_chunks = []
    map_chunks = []
    lidar_index = []
    local_offset = 0
    map_offset = 0
    for time, local_points in lidar:
        odom_index = min(bisect.bisect_left(odom_times, time), len(odometry) - 1)
        if odom_index and abs(odom_times[odom_index - 1] - time) < abs(odom_times[odom_index] - time):
            odom_index -= 1
        _, position, quaternion = odometry[odom_index]
        world = sampled(local_points, args.map_points) @ rotation_matrix(quaternion).T + position
        local_three = to_three(local_points)
        map_three = to_three(world - origin)
        local_chunks.append(local_three)
        map_chunks.append(map_three)
        lidar_index.append(
            {
                "time": round(time, 6),
                "localOffset": local_offset,
                "localCount": len(local_three),
                "mapOffset": map_offset,
                "mapCount": len(map_three),
            }
        )
        local_offset += len(local_three)
        map_offset += len(map_three)

    trajectory = []
    for time, position, _ in odometry:
        point = to_three(np.array([position - origin]))[0]
        trajectory.append([round(time, 6), *[round(float(value), 5) for value in point]])

    np.concatenate(local_chunks).astype("<f4").tofile(output / "lidar-local.bin")
    np.concatenate(map_chunks).astype("<f4").tofile(output / "lidar-map.bin")
    manifest = {
        "dataset": "M2DGR-Plus / Anomaly",
        "duration": round(last_time, 6),
        "cameraFps": 15.0,
        "rgbFrames": rgb_frames,
        "depthFrames": depth_frames,
        "lidarFrames": lidar_index,
        "trajectory": trajectory,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, separators=(",", ":")))
    print(json.dumps({key: manifest[key] for key in ["duration", "rgbFrames", "depthFrames"]}))
    print(f"lidar_frames={len(lidar_index)} local_points={local_offset} map_points={map_offset}")


if __name__ == "__main__":
    main()
