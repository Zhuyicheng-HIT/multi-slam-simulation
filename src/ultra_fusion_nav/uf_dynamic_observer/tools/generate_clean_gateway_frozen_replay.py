#!/usr/bin/env python3

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg, CustomPoint
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu


SCENARIOS = [
    "static_baseline",
    "person_crossing",
    "multiple_targets",
    "small_fast_target",
    "slow_target",
    "opening_closing_door",
    "large_dynamic_occlusion",
    "radial_motion",
    "moving_then_stops",
    "occlusion_appear",
    "near_wall_motion",
    "far_sparse_target",
]
SCAN_COUNT = 70
SCAN_PERIOD_NS = 100_000_000
SCAN_DURATION_NS = 90_000_000
IMU_PERIOD_NS = 10_000_000
START_NS = 1_000_000_000


def stamp(nanoseconds):
    output = Time()
    output.sec = int(nanoseconds // 1_000_000_000)
    output.nanosec = int(nanoseconds % 1_000_000_000)
    return output


def trajectory(seconds):
    u = max(0.0, seconds - 2.0)
    if u <= 0.0:
        return (0.0, 0.0, 1.2, 0.0), (0.0, 0.0, 0.0), 0.0
    x = 0.20 * (u - 0.5 * (1.0 - math.exp(-2.0 * u)))
    y = 0.10 * (1.0 - math.cos(0.5 * u))
    yaw = 0.05 * (1.0 - math.cos(0.4 * u))
    acceleration = (0.40 * math.exp(-2.0 * u), 0.025 * math.cos(0.5 * u), 0.0)
    yaw_rate = 0.02 * math.sin(0.4 * u)
    return (x, y, 1.2, yaw), acceleration, yaw_rate


def add_static_environment(points):
    for x in [2.0 + 0.35 * index for index in range(24)]:
        for y in [-5.0, 5.0]:
            for z_index in range(12):
                points.append((x, y, -0.5 + 0.30 * z_index, False))
    for y_index in range(31):
        y = -5.0 + y_index / 3.0
        for z_index in range(12):
            points.append((10.0, y, -0.5 + 0.30 * z_index, False))
    for x_index in range(25):
        for y_index in range(25):
            points.append((1.0 + 0.38 * x_index, -4.5 + 0.38 * y_index, 0.0, False))
    for center_x, center_y in [(4.0, -2.0), (7.0, 2.0)]:
        for angle_index in range(20):
            angle = 2.0 * math.pi * angle_index / 20.0
            for z_index in range(12):
                points.append(
                    (
                        center_x + 0.35 * math.cos(angle),
                        center_y + 0.35 * math.sin(angle),
                        0.15 + 0.25 * z_index,
                        False,
                    )
                )


def add_box(points, x, y, half_width, height, dynamic, spacing=0.20):
    count = int(round(2.0 * half_width / spacing))
    for ix in range(count + 1):
        for iy in range(count + 1):
            for iz in range(int(round(height / spacing)) + 1):
                if ix not in (0, count) and iy not in (0, count):
                    continue
                points.append(
                    (
                        x - half_width + ix * spacing,
                        y - half_width + iy * spacing,
                        0.10 + iz * spacing,
                        dynamic,
                    )
                )


def world_scene(name, frame):
    points = []
    add_static_environment(points)
    active = frame >= 25
    progress = frame - 25
    if name == "person_crossing" and active:
        add_box(points, 5.0, -3.5 + 0.18 * progress, 0.25, 1.7, True)
    elif name == "multiple_targets" and active:
        add_box(points, 4.5, -3.5 + 0.18 * progress, 0.25, 1.7, True)
        add_box(points, 6.0, 3.5 - 0.18 * progress, 0.25, 1.7, True)
    elif name == "small_fast_target" and active:
        add_box(points, 4.0, -4.0 + 0.34 * progress, 0.12, 0.45, True, 0.12)
    elif name == "slow_target" and active:
        add_box(points, 5.0, -1.2 + 0.045 * progress, 0.30, 1.0, True)
    elif name == "opening_closing_door":
        moving = frame >= 30
        if frame < 45:
            angle = min(1.2, 0.08 * max(0, frame - 30))
        else:
            angle = max(0.0, 1.2 - 0.08 * (frame - 45))
        for ir in range(12):
            radius = 0.10 + ir * 0.10
            for iz in range(13):
                points.append(
                    (
                        6.0 + radius * math.cos(angle),
                        -1.0 + radius * math.sin(angle),
                        0.10 + iz * 0.18,
                        moving,
                    )
                )
    elif name == "large_dynamic_occlusion" and active:
        add_box(points, 4.0, -3.0 + 0.12 * progress, 1.2, 2.5, True, 0.25)
    elif name == "radial_motion" and active:
        x = 8.5 - 0.18 * min(progress, 18) + 0.20 * max(0, progress - 18)
        add_box(points, x, 0.6, 0.28, 1.5, True)
    elif name == "moving_then_stops" and active:
        add_box(points, 5.0, -3.0 + 0.18 * min(progress, 15), 0.28, 1.5, True)
    elif name == "occlusion_appear":
        add_box(points, 4.2, 0.0, 0.70, 2.5, False, 0.25)
        if (25 <= frame < 38) or frame >= 50:
            y = 1.0 - 0.12 * (frame - 25) if frame < 38 else -0.5 + 0.12 * (frame - 50)
            add_box(points, 5.3, y, 0.25, 1.5, True)
    elif name == "near_wall_motion" and active:
        add_box(points, 9.35, -3.2 + 0.15 * progress, 0.24, 1.5, True)
    elif name == "far_sparse_target" and active:
        add_box(points, 18.0, 5.5 + 0.04 * progress, 0.30, 1.3, True, 0.30)
    return points


def sensor_returns(name, frame, pose):
    px, py, pz, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    nearest = {}
    for x, y, z, dynamic in world_scene(name, frame):
        dx = x - px
        dy = y - py
        dz = z - pz
        sx = cosine * dx + sine * dy
        sy = -sine * dx + cosine * dy
        sz = dz
        distance = math.sqrt(sx * sx + sy * sy + sz * sz)
        if distance < 0.5 or distance > 35.0:
            continue
        azimuth = math.atan2(sy, sx)
        elevation = math.atan2(sz, math.hypot(sx, sy))
        cell = (round(azimuth / 0.009), round(elevation / 0.009))
        old = nearest.get(cell)
        if old is None or distance < old[0]:
            nearest[cell] = (distance, sx, sy, sz, dynamic)
    ordered = sorted(nearest.values(), key=lambda value: math.atan2(value[2], value[1]))
    return ordered


def create_lidar(name, frame, time_ns):
    pose, _, _ = trajectory((time_ns - START_NS) * 1.0e-9)
    returns = sensor_returns(name, frame, pose)
    message = CustomMsg()
    message.header.stamp = stamp(time_ns)
    message.header.frame_id = "mid360_link"
    message.timebase = time_ns
    message.lidar_id = 1
    message.rsvd = [0, 0, 0]
    dynamic_offsets = []
    for index, (_, x, y, z, dynamic) in enumerate(returns):
        point = CustomPoint()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        point.reflectivity = 80 if dynamic else 30
        point.tag = 0
        point.line = index % 4
        point.offset_time = int(index * SCAN_DURATION_NS // max(1, len(returns)))
        if dynamic:
            dynamic_offsets.append(point.offset_time)
        message.points.append(point)
    message.point_num = len(message.points)
    return message, dynamic_offsets, pose


def create_imu(time_ns):
    pose, acceleration, yaw_rate = trajectory((time_ns - START_NS) * 1.0e-9)
    yaw = pose[3]
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    ax, ay, _ = acceleration
    message = Imu()
    message.header.stamp = stamp(time_ns)
    message.header.frame_id = "body"
    message.linear_acceleration.x = cosine * ax + sine * ay
    message.linear_acceleration.y = -sine * ax + cosine * ay
    message.linear_acceleration.z = 9.80665
    message.angular_velocity.z = yaw_rate
    return message


def generate_scenario(root, name):
    bag_path = root / name / "input"
    bag_path.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="/frozen/livox/lidar",
            type="livox_ros_driver2/msg/CustomMsg",
            serialization_format="cdr",
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="/frozen/livox/imu",
            type="sensor_msgs/msg/Imu",
            serialization_format="cdr",
        )
    )
    events = []
    for imu_index in range((SCAN_COUNT * SCAN_PERIOD_NS + SCAN_DURATION_NS) // IMU_PERIOD_NS + 1):
        time_ns = START_NS + imu_index * IMU_PERIOD_NS
        events.append((time_ns, 0, create_imu(time_ns), "/frozen/livox/imu"))
    truth_scans = []
    for frame in range(SCAN_COUNT):
        time_ns = START_NS + frame * SCAN_PERIOD_NS
        message, dynamic_offsets, pose = create_lidar(name, frame, time_ns)
        events.append((time_ns, 1, message, "/frozen/livox/lidar"))
        truth_scans.append(
            {
                "stamp_ns": time_ns,
                "frame": frame,
                "pose_xyzyaw": list(pose),
                "point_count": len(message.points),
                "dynamic_offsets": dynamic_offsets,
            }
        )
    events.sort(key=lambda item: (item[0], item[1]))
    for time_ns, _, message, topic in events:
        writer.write(topic, serialize_message(message), time_ns)
    del writer
    truth_path = root / name / "truth.json"
    truth_path.write_text(
        json.dumps(
            {
                "scenario": name,
                "truth_role": "evaluator_only",
                "scan_count": SCAN_COUNT,
                "imu_rate_hz": 100,
                "lidar_rate_hz": 10,
                "low_altitude_near_constant_height": True,
                "scans": truth_scans,
            },
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    db3_path = next(bag_path.glob("*.db3"))
    digest = hashlib.sha256(db3_path.read_bytes()).hexdigest()
    return {
        "scenario": name,
        "bag": str(bag_path.relative_to(root)),
        "bag_sha256": digest,
        "truth": str(truth_path.relative_to(root)),
        "truth_sha256": hashlib.sha256(truth_path.read_bytes()).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    root = Path(args.output).resolve()
    if root.exists():
        if not args.force:
            raise SystemExit(f"output already exists: {root}")
        shutil.rmtree(root)
    root.mkdir(parents=True)
    records = [generate_scenario(root, name) for name in SCENARIOS]
    manifest = {
        "schema": "clean_gateway_frozen_replay_v1",
        "generator": Path(__file__).name,
        "scenarios": records,
    }
    manifest_path = root / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(str(manifest_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
