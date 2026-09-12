#!/usr/bin/env python3

import json
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from uf_dynamic_interfaces.msg import PreviousFastLioState


def stamp(nanoseconds):
    message = Time()
    message.sec = int(nanoseconds // 1_000_000_000)
    message.nanosec = int(nanoseconds % 1_000_000_000)
    return message


def point_signature(point):
    return (
        point.offset_time,
        point.x,
        point.y,
        point.z,
        point.reflectivity,
        point.tag,
        point.line,
    )


class GatewaySmoke(Node):
    def __init__(self):
        super().__init__("clean_gateway_smoke")
        reliable = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
        self.raw_pub = self.create_publisher(CustomMsg, "/livox/lidar", reliable)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", reliable)
        self.state_pub = self.create_publisher(
            PreviousFastLioState, "/clean_fast_lio/previous_state", reliable
        )
        self.create_subscription(
            CustomMsg, "/dynamic_observer/clean/livox", self._clean, reliable
        )
        self.create_subscription(
            String, "/dynamic_observer/clean/status", self._status, reliable
        )
        self.clean_by_stamp = {}
        self.status_by_stamp = {}
        self.raw_by_stamp = {}
        self.next_imu_ns = 1_000_000_000

    def _clean(self, message):
        key = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.clean_by_stamp[key] = message

    def _status(self, message):
        value = json.loads(message.data)
        self.status_by_stamp[value["source_stamp_ns"]] = value

    def publish_imu_until(self, end_ns):
        while self.next_imu_ns <= end_ns:
            message = Imu()
            message.header.stamp = stamp(self.next_imu_ns)
            message.header.frame_id = "body"
            message.linear_acceleration.z = 9.80665
            self.imu_pub.publish(message)
            self.next_imu_ns += 10_000_000

    def publish_state(self, state_ns, sequence):
        message = PreviousFastLioState()
        message.header.stamp = stamp(state_ns)
        message.header.frame_id = "camera_init"
        message.map_frame = "camera_init"
        message.body_frame = "body"
        message.scan_sequence = sequence
        message.reset_counter = 0
        message.valid = True
        message.orientation_xyzw[3] = 1.0
        self.state_pub.publish(message)

    @staticmethod
    def make_point(x, y, z, offset_ns, ordinal):
        output = CustomPoint()
        output.x = float(x)
        output.y = float(y)
        output.z = float(z)
        output.offset_time = int(offset_ns)
        output.reflectivity = (37 + ordinal) % 255
        output.tag = (2 + ordinal) % 255
        output.line = ordinal % 4
        return output

    def publish_scan(self, scan_ns, include_target, sequence):
        message = CustomMsg()
        message.header.stamp = stamp(scan_ns)
        message.header.frame_id = "mid360_link"
        message.timebase = scan_ns + 123
        message.lidar_id = 7
        message.rsvd = [9, 8, 7]
        points = []
        for y_index in range(-8, 9):
            for z_index in range(-4, 9):
                ordinal = len(points)
                points.append(
                    self.make_point(
                        8.0,
                        0.125 * y_index,
                        0.125 * z_index,
                        ordinal * 9_000_000 // 300,
                        ordinal,
                    )
                )
        if include_target:
            for y_index in range(-3, 4):
                for z_index in range(-3, 7):
                    ordinal = len(points)
                    points.append(
                        self.make_point(
                            4.0,
                            0.125 * y_index,
                            0.125 * z_index,
                            ordinal * 9_000_000 // 300,
                            ordinal,
                        )
                    )
        message.points = points
        message.point_num = len(points)
        self.raw_by_stamp[scan_ns] = message
        self.raw_pub.publish(message)

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)


def messages_identical(lhs, rhs):
    return (
        lhs.header == rhs.header
        and lhs.timebase == rhs.timebase
        and lhs.point_num == rhs.point_num
        and lhs.lidar_id == rhs.lidar_id
        and list(lhs.rsvd) == list(rhs.rsvd)
        and [point_signature(point) for point in lhs.points]
        == [point_signature(point) for point in rhs.points]
    )


def main():
    rclpy.init()
    node = GatewaySmoke()
    try:
        node.spin_for(0.3)
        fail_open_stamp = 1_200_000_000
        node.publish_scan(fail_open_stamp, False, 0)
        node.spin_for(0.3)

        imu_timeout_stamp = 1_300_000_000
        node.publish_state(imu_timeout_stamp - 10_000_000, 1)
        node.publish_scan(imu_timeout_stamp, False, 1)
        node.spin_for(0.3)

        queue_overflow_stamp = 1_500_000_000
        for index in range(10):
            node.publish_scan(queue_overflow_stamp + index * 10_000_000, False, index)
        node.spin_for(0.35)

        for sequence in range(12):
            scan_ns = 3_000_000_000 + sequence * 100_000_000
            node.publish_imu_until(scan_ns + 12_000_000)
            node.publish_state(scan_ns - 10_000_000, sequence)
            node.spin_for(0.02)
            node.publish_scan(scan_ns, sequence >= 7, sequence)
            node.spin_for(0.08)

        regressed_stamp = 3_500_000_000
        node.publish_scan(regressed_stamp, True, 99)
        node.spin_for(0.2)

        fail_open_message = node.clean_by_stamp.get(fail_open_stamp)
        fail_open_status = node.status_by_stamp.get(fail_open_stamp, {})
        imu_timeout_message = node.clean_by_stamp.get(imu_timeout_stamp)
        imu_timeout_status = node.status_by_stamp.get(imu_timeout_stamp, {})
        queue_overflow_message = node.clean_by_stamp.get(queue_overflow_stamp)
        queue_overflow_status = node.status_by_stamp.get(queue_overflow_stamp, {})
        healthy_statuses = [
            value
            for key, value in node.status_by_stamp.items()
            if key >= 3_000_000_000 and value.get("reason") == "ok"
        ]
        removed_statuses = [
            value for value in healthy_statuses if value.get("dynamic_removed", 0) > 0
        ]
        preservation_ok = True
        for key, clean in node.clean_by_stamp.items():
            raw = node.raw_by_stamp.get(key)
            if raw is None:
                continue
            raw_signatures = {point_signature(point) for point in raw.points}
            if any(point_signature(point) not in raw_signatures for point in clean.points):
                preservation_ok = False
                break
        regression_status = node.status_by_stamp.get(regressed_stamp, {})
        report = {
            "fail_open_exact_raw": (
                fail_open_message is not None
                and messages_identical(
                    node.raw_by_stamp[fail_open_stamp], fail_open_message
                )
            ),
            "fail_open_reason": fail_open_status.get("reason"),
            "imu_timeout_exact_raw": (
                imu_timeout_message is not None
                and messages_identical(
                    node.raw_by_stamp[imu_timeout_stamp], imu_timeout_message
                )
            ),
            "imu_timeout_reason": imu_timeout_status.get("reason"),
            "queue_overflow_exact_raw": (
                queue_overflow_message is not None
                and messages_identical(
                    node.raw_by_stamp[queue_overflow_stamp], queue_overflow_message
                )
            ),
            "queue_overflow_reason": queue_overflow_status.get("reason"),
            "healthy_scans": len(healthy_statuses),
            "dynamic_removal_scans": len(removed_statuses),
            "point_metadata_preserved": preservation_ok,
            "timestamp_regression_fail_open": regression_status.get("fail_open"),
            "timestamp_regression_reason": regression_status.get("reason"),
            "raw_publishers": node.count_publishers("/livox/lidar"),
            "clean_messages": len(node.clean_by_stamp),
            "status_messages": len(node.status_by_stamp),
        }
        print(json.dumps(report, sort_keys=True))
        passed = (
            report["fail_open_exact_raw"]
            and report["fail_open_reason"] == "previous_state_timeout"
            and report["imu_timeout_exact_raw"]
            and report["imu_timeout_reason"] == "imu_coverage_timeout"
            and report["queue_overflow_exact_raw"]
            and report["queue_overflow_reason"] == "queue_overflow"
            and report["healthy_scans"] >= 10
            and report["dynamic_removal_scans"] > 0
            and report["point_metadata_preserved"]
            and report["timestamp_regression_fail_open"] is True
            and report["timestamp_regression_reason"]
            == "input_timestamp_regression"
            and report["raw_publishers"] == 1
        )
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
