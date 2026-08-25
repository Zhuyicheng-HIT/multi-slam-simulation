#!/usr/bin/env python3
"""Export seekable R3LIVE camera and Livox assets without inventing a pose."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def sampled(points, limit):
    if len(points) <= limit:
        return points
    return points[:: math.ceil(len(points) / limit)][:limit]


def to_three(points):
    return np.column_stack((points[:, 0], points[:, 2], -points[:, 1])).astype("<f4")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output")
    parser.add_argument("--local-points", type=int, default=5000)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    image_type = get_message(types["/camera/image_color/compressed"])
    lidar_type = get_message(types["/livox/lidar"])
    writer = None
    bag_start = None
    last_time = 0.0
    camera_frames = 0
    local_chunks = []
    lidar_frames = []
    local_offset = 0

    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        if bag_start is None:
            bag_start = stamp
        time = (stamp - bag_start) / 1e9
        last_time = max(last_time, time)
        if topic == "/camera/image_color/compressed":
            message = deserialize_message(payload, image_type)
            image = cv2.imdecode(np.asarray(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                continue
            if writer is None:
                writer = cv2.VideoWriter(
                    str(output / "camera-rgb.webm"),
                    cv2.VideoWriter_fourcc(*"VP80"),
                    3339 / 101.8805942,
                    (image.shape[1], image.shape[0]),
                )
            writer.write(image)
            camera_frames += 1
        elif topic == "/livox/lidar":
            message = deserialize_message(payload, lidar_type)
            points = np.asarray([(point.x, point.y, point.z) for point in message.points], dtype=np.float32)
            points = points[np.isfinite(points).all(axis=1)]
            points = points[np.linalg.norm(points, axis=1) < 90.0]
            points = to_three(sampled(points, args.local_points))
            local_chunks.append(points)
            lidar_frames.append(
                {
                    "time": round(time, 6),
                    "localOffset": local_offset,
                    "localCount": len(points),
                    "mapOffset": 0,
                    "mapCount": 0,
                }
            )
            local_offset += len(points)

    if writer:
        writer.release()
    np.concatenate(local_chunks).astype("<f4").tofile(output / "lidar-local.bin")
    manifest = {
        "id": "r3live-degenerate-02",
        "dataset": "R3LIVE / degenerate_seq_02",
        "duration": round(last_time, 6),
        "cameraFps": 3339 / 101.8805942,
        "rgbFrames": camera_frames,
        "depthFrames": 0,
        "lidarFrames": lidar_frames,
        "trajectory": [],
        "mapAvailable": False,
        "mapReason": "数据集 RTK/GNSS/yaw 为零，且本次算法回放没有输出位姿",
        "topics": {
            "camera": "/camera/image_color/compressed",
            "lidar": "/livox/lidar",
            "pose": "unavailable",
        },
        "counts": {"LiDAR": 1019, "IMU": 20797, "RGB": 3339, "GNSS": 5093},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, separators=(",", ":")))
    print(json.dumps({"duration": manifest["duration"], "camera": camera_frames, "lidar": len(lidar_frames)}))


if __name__ == "__main__":
    main()
