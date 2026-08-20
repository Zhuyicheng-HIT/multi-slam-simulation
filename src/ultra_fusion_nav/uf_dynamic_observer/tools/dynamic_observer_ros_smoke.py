#!/usr/bin/env python3

import json
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def time_message(nanoseconds):
    output = Time()
    output.sec = int(nanoseconds // 1_000_000_000)
    output.nanosec = int(nanoseconds % 1_000_000_000)
    return output


class SmokePublisher(Node):
    def __init__(self):
        super().__init__("dynamic_observer_smoke_publisher")
        self.odom_pub = self.create_publisher(
            Odometry, "/Odometry", qos_profile_sensor_data
        )
        self.imu_pub = self.create_publisher(
            Imu, "/livox/imu", qos_profile_sensor_data
        )
        self.lidar_pub = self.create_publisher(
            CustomMsg, "/livox/lidar", qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/dynamic_observer/statistics", self._statistics, 10
        )
        self.create_subscription(
            PointCloud2,
            "/dynamic_observer/scored_cloud",
            self._scored,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/dynamic_observer/dynamic_candidates",
            self._dynamic,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.02, self._tick)
        self.tick = 0
        self.scan = 0
        self.statistics_count = 0
        self.scored_points = 0
        self.max_dynamic_points = 0
        self.last_statistics = {}

    def _statistics(self, message):
        self.statistics_count += 1
        self.last_statistics = json.loads(message.data)

    def _scored(self, message):
        self.scored_points += int(message.width) * int(message.height)

    def _dynamic(self, message):
        self.max_dynamic_points = max(
            self.max_dynamic_points, int(message.width) * int(message.height)
        )

    def _point(self, x, y, z, offset_ns):
        point = CustomPoint()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        point.reflectivity = 40
        point.offset_time = int(offset_ns)
        return point

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        odom = Odometry()
        odom.header.stamp = time_message(now_ns)
        odom.header.frame_id = "camera_init"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        imu = Imu()
        imu.header.stamp = time_message(now_ns)
        imu.header.frame_id = "body"
        imu.linear_acceleration.z = 9.80665
        self.imu_pub.publish(imu)
        self.tick += 1
        if self.tick < 15 or self.tick % 5:
            return

        scan_start = now_ns - 30_000_000
        cloud = CustomMsg()
        cloud.header.stamp = time_message(scan_start)
        cloud.header.frame_id = "mid360_link"
        cloud.timebase = scan_start
        points = []
        for y_index in range(-8, 9):
            for z_index in range(-4, 9):
                offset = len(points) * 10_000_000 // 300
                points.append(
                    self._point(8.0, 0.125 * y_index, 0.125 * z_index, offset)
                )
        if self.scan >= 6:
            for y_index in range(-3, 4):
                for z_index in range(-3, 7):
                    offset = len(points) * 10_000_000 // 300
                    points.append(
                        self._point(
                            4.0,
                            0.125 * y_index,
                            0.125 * z_index,
                            offset,
                        )
                    )
        cloud.points = points
        cloud.point_num = len(points)
        self.lidar_pub.publish(cloud)
        self.scan += 1


def main():
    rclpy.init()
    node = SmokePublisher()
    deadline = time.monotonic() + 8.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                node.statistics_count >= 8
                and node.scored_points > 0
                and node.max_dynamic_points > 0
            ):
                break
        report = {
            "statistics_messages": node.statistics_count,
            "scored_points": node.scored_points,
            "max_dynamic_points": node.max_dynamic_points,
            "livox_publishers": node.count_publishers("/livox/lidar"),
            "last_statistics": node.last_statistics,
        }
        print(json.dumps(report, sort_keys=True))
        passed = (
            report["statistics_messages"] >= 8
            and report["scored_points"] > 0
            and report["max_dynamic_points"] > 0
            and report["last_statistics"].get(
                "fastlio_input_modified"
            ) is False
        )
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
