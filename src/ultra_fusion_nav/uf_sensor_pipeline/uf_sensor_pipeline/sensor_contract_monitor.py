import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2


class SensorContractMonitor(Node):
    def __init__(self):
        super().__init__("sensor_contract_monitor")
        defaults = {
            "lidar_topic": "/sensors/lidar/points",
            "imu_topic": "/sensors/imu",
            "gnss_topic": "/sensors/gnss/fix",
            "optical_flow_topic": "/sensors/optical_flow/rad",
            "depth_topic": "/sensors/rgbd/depth",
            "color_topic": "/sensors/rgbd/color",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("startup_grace_s", 12.0)
        self.declare_parameter("stale_after_s", 3.0)

        self.started = time.monotonic()
        self.stale_after = float(self.get_parameter("stale_after_s").value)
        self.streams = {}
        specs = [
            ("lidar", PointCloud2, "lidar_topic", 3.0),
            ("imu", Imu, "imu_topic", 30.0),
            ("gnss", NavSatFix, "gnss_topic", 0.5),
            ("optical_flow", OpticalFlowRad, "optical_flow_topic", 3.0),
            ("depth", Image, "depth_topic", 3.0),
            ("color", Image, "color_topic", 3.0),
        ]
        for modality, msg_type, parameter, minimum_rate in specs:
            self.streams[modality] = {
                "count": 0,
                "last_stamp": 0,
                "last_arrival": 0.0,
                "first_arrival": 0.0,
                "regressions": 0,
                "duplicates": 0,
                "empty_frames": 0,
                "zero_stamps": 0,
                "minimum_rate": minimum_rate,
                "topic": str(self.get_parameter(parameter).value),
            }
            self.create_subscription(
                msg_type,
                self.streams[modality]["topic"],
                lambda msg, key=modality: self._record(key, msg),
                qos_profile_sensor_data,
            )
        self.publisher = self.create_publisher(
            DiagnosticArray, "/sensor_contract/diagnostics", 10
        )
        self.create_timer(2.0, self._publish)

    def _record(self, modality, msg):
        stream = self.streams[modality]
        now = time.monotonic()
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp == 0:
            stream["zero_stamps"] += 1
        if stream["last_stamp"]:
            if stamp < stream["last_stamp"]:
                stream["regressions"] += 1
            elif stamp == stream["last_stamp"]:
                stream["duplicates"] += 1
        if not msg.header.frame_id:
            stream["empty_frames"] += 1
        stream["last_stamp"] = max(stream["last_stamp"], stamp)
        stream["count"] += 1
        stream["last_arrival"] = now
        if not stream["first_arrival"]:
            stream["first_arrival"] = now

    @staticmethod
    def _value(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _status(self, modality, stream, now):
        status = DiagnosticStatus()
        status.name = f"sensor_contract/{modality}"
        status.hardware_id = "simulation"
        elapsed = max(1.0e-6, now - stream["first_arrival"]) if stream["count"] else 0.0
        rate = stream["count"] / elapsed if elapsed else 0.0
        stale = stream["count"] == 0 or now - stream["last_arrival"] > self.stale_after
        problems = []
        if stale:
            problems.append("stale_or_missing")
        if rate and rate < stream["minimum_rate"]:
            problems.append("low_rate")
        if stream["regressions"]:
            problems.append("stamp_regression")
        if stream["zero_stamps"]:
            problems.append("zero_stamp")
        if stream["empty_frames"]:
            problems.append("empty_frame")
        grace = now - self.started < float(self.get_parameter("startup_grace_s").value)
        status.level = DiagnosticStatus.OK
        if problems:
            status.level = DiagnosticStatus.WARN if grace or problems == ["low_rate"] else DiagnosticStatus.ERROR
        status.message = "ok" if not problems else ",".join(problems)
        status.values = [
            self._value("topic", stream["topic"]),
            self._value("samples", stream["count"]),
            self._value("rate_hz", f"{rate:.3f}"),
            self._value("stamp_regressions", stream["regressions"]),
            self._value("stamp_duplicates", stream["duplicates"]),
            self._value("zero_stamps", stream["zero_stamps"]),
            self._value("empty_frames", stream["empty_frames"]),
        ]
        return status

    def _publish(self):
        now = time.monotonic()
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [self._status(name, stream, now) for name, stream in self.streams.items()]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SensorContractMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
