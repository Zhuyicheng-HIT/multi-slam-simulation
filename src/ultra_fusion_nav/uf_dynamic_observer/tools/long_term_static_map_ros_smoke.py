#!/usr/bin/env python3

import argparse
import json
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from uf_dynamic_interfaces.msg import PreviousFastLioState


def split_stamp(seconds):
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1.0e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


def scored_cloud(stamp_s, score=0.0, semantic=False):
    message = PointCloud2()
    message.header.stamp.sec, message.header.stamp.nanosec = split_stamp(stamp_s)
    message.header.frame_id = "camera_init"
    names = ["x", "y", "z", "intensity", "dynamic_score"]
    if semantic:
        names = ["x", "y", "z", "dynamic_confidence"]
    message.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(names)
    ]
    rows = []
    if semantic:
        rows.append((3.0, 0.0, 1.0, 0.95))
    else:
        rows.extend(
            [
                (3.0, 0.0, 1.0, 20.0, score),
                (3.0, 0.25, 1.0, 20.0, score),
                (3.0, -0.25, 1.0, 20.0, score),
            ]
        )
    message.height = 1
    message.width = len(rows)
    message.point_step = len(names) * 4
    message.row_step = message.width * message.point_step
    message.is_dense = True
    message.is_bigendian = False
    message.data = b"".join(struct.pack("<" + "f" * len(row), *row) for row in rows)
    return message


class Smoke(Node):
    def __init__(self):
        super().__init__("long_term_static_map_smoke")
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=128,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(
            PreviousFastLioState, "/clean_fast_lio/previous_state", reliable
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/dynamic_observer/scored_cloud", 10
        )
        self.semantic_pub = self.create_publisher(
            PointCloud2, "/semantic/dynamic_evidence", 10
        )
        self.create_subscription(
            PointCloud2, "/mapping/long_term_static/points", self._map, latched
        )
        self.create_subscription(
            PointCloud2,
            "/mapping/long_term_static/relocalization_points",
            self._relocalization,
            latched,
        )
        self.create_subscription(
            PointCloud2,
            "/mapping/long_term_static/loop_closure_points",
            self._loop,
            latched,
        )
        self.create_subscription(String, "/mapping/long_term_static/status", self._status, 20)
        self.map_points = 0
        self.relocalization_points = 0
        self.loop_points = 0
        self.statuses = []

    def _map(self, message):
        self.map_points = max(self.map_points, message.width * message.height)

    def _relocalization(self, message):
        self.relocalization_points = max(
            self.relocalization_points, message.width * message.height
        )

    def _loop(self, message):
        self.loop_points = max(self.loop_points, message.width * message.height)

    def _status(self, message):
        try:
            self.statuses.append(json.loads(message.data))
        except json.JSONDecodeError:
            self.statuses.append({"invalid_json": message.data})

    def publish_state(self, stamp_s, sequence, y):
        message = PreviousFastLioState()
        message.header.stamp.sec, message.header.stamp.nanosec = split_stamp(stamp_s)
        message.header.frame_id = "camera_init"
        message.map_frame = "camera_init"
        message.body_frame = "body"
        message.scan_sequence = sequence
        message.valid = True
        message.position = [0.0, y, 1.0]
        message.orientation_xyzw = [0.0, 0.0, 0.0, 1.0]
        message.velocity_map = [0.0, 0.0, 0.0]
        message.accel_bias = [0.0, 0.0, 0.0]
        message.gyro_bias = [0.0, 0.0, 0.0]
        self.state_pub.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()
    rclpy.init()
    node = Smoke()
    start = time.monotonic()
    while time.monotonic() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    base = node.get_clock().now().nanoseconds * 1.0e-9 + 0.2
    # Exercise explicit map-hold fail-open before a causal state exists.
    node.cloud_pub.publish(scored_cloud(base))
    for index in range(14):
        scan_stamp = base + 0.1 * (index + 1)
        node.publish_state(scan_stamp - 0.05, index + 1, -1.0 if index % 2 == 0 else 1.0)
        rclpy.spin_once(node, timeout_sec=0.03)
        node.cloud_pub.publish(scored_cloud(scan_stamp))
        if index == 8:
            node.semantic_pub.publish(scored_cloud(scan_stamp, semantic=True))
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.03)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.10)
        if node.map_points and node.relocalization_points and node.loop_points:
            break
    healthy = [item for item in node.statuses if item.get("health") == "HEALTHY"]
    held = [
        item
        for item in node.statuses
        if item.get("reason") == "previous_state_unavailable_map_held"
    ]
    future_flags = [item.get("future_pose_used") for item in node.statuses]
    output = {
        "map_points": node.map_points,
        "relocalization_points": node.relocalization_points,
        "loop_closure_points": node.loop_points,
        "healthy_statuses": len(healthy),
        "map_hold_fail_open_statuses": len(held),
        "all_future_pose_flags_false": bool(future_flags)
        and all(value is False for value in future_flags),
        "static_confirmed_only": bool(healthy)
        and all(item.get("output_policy") == "STATIC_CONFIRMED_ONLY" for item in healthy),
        "semantic_shadow_hits": max(
            [item.get("semantic_shadow_hits", 0) for item in node.statuses] or [0]
        ),
        "production_lidar_modified": any(
            item.get("production_lidar_modified") is True for item in node.statuses
        ),
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    node.destroy_node()
    rclpy.shutdown()
    passed = (
        output["map_points"] > 0
        and output["relocalization_points"] == output["map_points"]
        and output["loop_closure_points"] == output["map_points"]
        and output["healthy_statuses"] > 0
        and output["map_hold_fail_open_statuses"] > 0
        and output["all_future_pose_flags_false"]
        and output["static_confirmed_only"]
        and output["semantic_shadow_hits"] > 0
        and not output["production_lidar_modified"]
    )
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
