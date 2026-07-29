"""Direct MTF-01 MAVLink APM decoder for simulation and companion computers."""

from collections import deque
import json
import math
from pathlib import Path
import socket
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from pymavlink import mavutil
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import UInt8MultiArray

from .mtf01p_protocol import (
    DISTANCE_SENSOR_MESSAGE_ID,
    MTF01P_FOV_RAD,
    MTF01P_WIDTH_PX,
    OPTICAL_FLOW_MESSAGE_ID,
    SensorClock,
    focal_length_px,
    integrated_radians_to_pixels,
    pixels_to_integrated_radians,
)
from .optical_flow_model import integrate_gyro, ros_flu_gyro_to_sensor_frd


class Mtf01pMavlinkBridge(Node):
    """Decode MTF-01 MAVLink1 flow and range frames without an FCU hop."""

    def __init__(self):
        super().__init__("mtf01p_mavlink_bridge")
        defaults = {
            "mode": "sim",
            "input_topic": "/sim/optical_flow/rad_native",
            "flow_topic": "/sim/optical_flow/rad",
            "range_topic": "/sim/optical_flow/range",
            "raw_frame_topic": "/sim/mtf01/mavlink_frame",
            "imu_topic": "/mavros/imu/data_raw",
            "tcp_host": "127.0.0.1",
            "tcp_port": 5764,
            "source_system": 200,
            "source_component": 88,
            "frame_id": "mtf01_flow_sensor",
            "nominal_rate_hz": 100.0,
            "maximum_imu_gap_s": 0.12,
            "range_min_m": 0.02,
            "range_max_m": 8.0,
            "range_fov_rad": math.radians(6.0),
            "restamp_output": True,
            "report_path": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in ("sim", "tcp"):
            raise ValueError("mode must be sim or tcp")
        self.tcp_host = str(self.get_parameter("tcp_host").value)
        self.tcp_port = int(self.get_parameter("tcp_port").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.nominal_rate_hz = float(self.get_parameter("nominal_rate_hz").value)
        self.maximum_imu_gap = float(self.get_parameter("maximum_imu_gap_s").value)
        self.range_min = float(self.get_parameter("range_min_m").value)
        self.range_max = float(self.get_parameter("range_max_m").value)
        self.range_fov = float(self.get_parameter("range_fov_rad").value)
        self.report_path = str(self.get_parameter("report_path").value)

        self.focal_px = focal_length_px(MTF01P_WIDTH_PX, MTF01P_FOV_RAD)
        self.sensor_clock = SensorClock()
        self.parser = mavutil.mavlink.MAVLink(None)
        self.encoder = mavutil.mavlink.MAVLink(
            None, srcSystem=self.source_system, srcComponent=self.source_component
        )
        self.tcp_socket = None
        self.last_connect_attempt = 0.0
        self.imu_samples = deque(maxlen=2000)
        self.flow_arrivals = deque(maxlen=1000)
        self.last_flow_time_us = None
        self.device_epoch_us = None
        self.host_epoch_ns = None
        self.latest_range = None
        self.pending_flow = None
        self.last_frame_monotonic = None
        self.last_error = ""
        self.counts = {
            "sim_inputs": 0,
            "wire_bytes": 0,
            "valid_frames": 0,
            "bad_data": 0,
            "wrong_source": 0,
            "optical_flow": 0,
            "distance_sensor": 0,
            "published_flow": 0,
            "published_range": 0,
            "range_stale": 0,
            "timestamp_regressions": 0,
            "flow_frame_gaps": 0,
            "gyro_missing": 0,
        }

        self.flow_pub = self.create_publisher(
            OpticalFlowRad, str(self.get_parameter("flow_topic").value), qos_profile_sensor_data
        )
        self.range_pub = self.create_publisher(
            Range, str(self.get_parameter("range_topic").value), qos_profile_sensor_data
        )
        self.raw_pub = self.create_publisher(
            UInt8MultiArray, str(self.get_parameter("raw_frame_topic").value), qos_profile_sensor_data
        )
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/mtf01/mavlink_diagnostics", 10)
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self._on_imu, qos_profile_sensor_data
        )
        if self.mode == "sim":
            self.create_subscription(
                OpticalFlowRad, str(self.get_parameter("input_topic").value), self._on_sim_flow,
                qos_profile_sensor_data,
            )
        else:
            self.create_timer(0.005, self._read_tcp)
        self.create_timer(1.0, self._diagnostics)

    def _on_imu(self, msg):
        gyro_frd = ros_flu_gyro_to_sensor_frd(
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        )
        self.imu_samples.append((time.monotonic(), *gyro_frd))

    def _on_sim_flow(self, msg):
        self.counts["sim_inputs"] += 1
        integration_us = int(msg.integration_time_us)
        if integration_us <= 0 or not all(
            math.isfinite(float(value)) for value in (msg.integrated_x, msg.integrated_y, msg.distance)
        ):
            return
        sensor_time_us = self.sensor_clock.advance(integration_us)
        flow_x, flow_y = integrated_radians_to_pixels(
            msg.integrated_x, msg.integrated_y, self.focal_px, self.focal_px
        )
        quality = max(0, min(255, int(msg.quality)))
        current_cm = max(0, min(65535, int(round(msg.distance * 100.0))))
        # The physical MTF-01 in MAVLink APM mode reports zero velocity fields
        # and -1 ground_distance. Range comes only from DISTANCE_SENSOR.
        flow = mavutil.mavlink.MAVLink_optical_flow_message(
            sensor_time_us, 0, flow_x, flow_y, 0.0, 0.0, quality, -1.0
        )
        distance = mavutil.mavlink.MAVLink_distance_sensor_message(
            (sensor_time_us // 1000) & 0xFFFFFFFF,
            int(round(self.range_min * 100.0)), int(round(self.range_max * 100.0)), current_cm,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 0,
            mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 3,
        )
        self._accept_bytes(flow.pack(self.encoder, force_mavlink1=True))
        self._accept_bytes(distance.pack(self.encoder, force_mavlink1=True))

    def _connect_tcp(self):
        now = time.monotonic()
        if self.tcp_socket is not None or now - self.last_connect_attempt < 1.0:
            return
        self.last_connect_attempt = now
        try:
            self.tcp_socket = socket.create_connection((self.tcp_host, self.tcp_port), 0.25)
            self.tcp_socket.setblocking(False)
            self.last_error = ""
        except OSError as exc:
            self.last_error = str(exc)

    def _read_tcp(self):
        self._connect_tcp()
        if self.tcp_socket is None:
            return
        try:
            data = self.tcp_socket.recv(8192)
            if not data:
                self.tcp_socket.close()
                self.tcp_socket = None
                self.last_error = "serial TCP bridge closed the connection"
                return
        except BlockingIOError:
            return
        except OSError as exc:
            self.tcp_socket = None
            self.last_error = str(exc)
            return
        self._accept_bytes(data)

    def _accept_bytes(self, data):
        self.counts["wire_bytes"] += len(data)
        for byte in data:
            try:
                message = self.parser.parse_char(bytes((byte,)))
            except mavutil.mavlink.MAVError:
                self.counts["bad_data"] += 1
                continue
            if message is None:
                continue
            if message.get_type() == "BAD_DATA":
                self.counts["bad_data"] += 1
                continue
            self.counts["valid_frames"] += 1
            raw = UInt8MultiArray()
            raw.data = list(message.get_msgbuf())
            self.raw_pub.publish(raw)
            if (
                message.get_srcSystem() != self.source_system
                or message.get_srcComponent() != self.source_component
            ):
                self.counts["wrong_source"] += 1
                continue
            if message.get_msgId() == OPTICAL_FLOW_MESSAGE_ID:
                self._on_flow(message)
            elif message.get_msgId() == DISTANCE_SENSOR_MESSAGE_ID:
                self._on_range(message)

    def _stamp_ns(self, source_time_us):
        source_time_us = int(source_time_us)
        now_ns = self.get_clock().now().nanoseconds
        if self.device_epoch_us is None:
            self.device_epoch_us, self.host_epoch_ns = source_time_us, now_ns
        if source_time_us + 1_000_000 < self.device_epoch_us:
            self.counts["timestamp_regressions"] += 1
            self.device_epoch_us, self.host_epoch_ns = source_time_us, now_ns
        return self.host_epoch_ns + (source_time_us - self.device_epoch_us) * 1000

    def _on_range(self, message):
        self.counts["distance_sensor"] += 1
        source_time_us = int(message.time_boot_ms) * 1000
        distance_m = int(message.current_distance) * 0.01
        if not self.range_min <= distance_m <= self.range_max:
            return
        self.latest_range = (source_time_us, distance_m)
        output = Range()
        output.header.stamp = Time(nanoseconds=self._stamp_ns(source_time_us)).to_msg()
        output.header.frame_id = self.frame_id
        output.radiation_type = Range.INFRARED
        output.field_of_view = self.range_fov
        output.min_range = max(self.range_min, int(message.min_distance) * 0.01)
        output.max_range = min(self.range_max, int(message.max_distance) * 0.01)
        output.range = distance_m
        self.range_pub.publish(output)
        self.counts["published_range"] += 1
        if self.pending_flow is not None:
            pending_time_us, pending_flow = self.pending_flow
            if abs(pending_time_us - source_time_us) <= 20_000:
                self.pending_flow = None
                self._publish_flow(pending_flow, distance_m)

    def _on_flow(self, message):
        self.counts["optical_flow"] += 1
        source_time_us = int(message.time_usec)
        integration_s = 1.0 / max(self.nominal_rate_hz, 1.0)
        if self.last_flow_time_us is not None:
            interval_s = (source_time_us - self.last_flow_time_us) * 1.0e-6
            if interval_s > 0.0:
                integration_s = interval_s
                if interval_s > 1.5 / max(self.nominal_rate_hz, 1.0):
                    self.counts["flow_frame_gaps"] += 1
            else:
                self.counts["timestamp_regressions"] += 1
        self.last_flow_time_us = source_time_us
        stamp_ns = self._stamp_ns(source_time_us)
        flow_data = {
            "stamp_ns": stamp_ns,
            "flow_x": int(message.flow_x),
            "flow_y": int(message.flow_y),
            "quality": max(0, min(255, int(message.quality))),
            "integration_s": integration_s,
        }
        if self.latest_range is not None and abs(self.latest_range[0] - source_time_us) <= 20_000:
            self._publish_flow(flow_data, self.latest_range[1])
        else:
            if self.pending_flow is not None:
                self.counts["range_stale"] += 1
                self._publish_flow(self.pending_flow[1], None)
            self.pending_flow = (source_time_us, flow_data)

    def _publish_flow(self, flow_data, distance_m):
        gyro = integrate_gyro(
            list(self.imu_samples), time.monotonic() - flow_data["integration_s"], time.monotonic(),
            max_gap_s=self.maximum_imu_gap,
        )
        if gyro is None:
            gyro = (float("nan"), float("nan"), float("nan"))
            self.counts["gyro_missing"] += 1
        integrated_x, integrated_y = pixels_to_integrated_radians(
            flow_data["flow_x"], flow_data["flow_y"], self.focal_px, self.focal_px
        )
        output = OpticalFlowRad()
        output.header.stamp = Time(nanoseconds=flow_data["stamp_ns"]).to_msg()
        output.header.frame_id = self.frame_id
        output.integration_time_us = max(1, int(round(flow_data["integration_s"] * 1.0e6)))
        output.integrated_x, output.integrated_y = integrated_x, integrated_y
        output.integrated_xgyro, output.integrated_ygyro, output.integrated_zgyro = gyro
        output.quality = flow_data["quality"]
        # A delayed/missing range sample is an invalid float measurement, not None.
        output.distance = float("nan") if distance_m is None else float(distance_m)
        self.flow_pub.publish(output)
        self.counts["published_flow"] += 1
        self.last_frame_monotonic = time.monotonic()
        self.flow_arrivals.append(self.last_frame_monotonic)

    @staticmethod
    def _value(key, value):
        return KeyValue(key=str(key), value=str(value))

    def _diagnostics(self):
        fresh = self.last_frame_monotonic is not None and time.monotonic() - self.last_frame_monotonic < 0.5
        status = DiagnosticStatus()
        status.name = "mtf01/mavlink_apm"
        status.hardware_id = "mtf01_sim" if self.mode == "sim" else "mtf01_com_bridge"
        status.level = DiagnosticStatus.OK if fresh and self.counts["bad_data"] == 0 else DiagnosticStatus.ERROR
        status.message = "mavlink_stream_ok" if status.level == DiagnosticStatus.OK else "mavlink_stream_missing_or_invalid"
        status.values = [
            self._value("source", f"{self.source_system}:{self.source_component}"),
            self._value("published_flow", self.counts["published_flow"]),
            self._value("published_range", self.counts["published_range"]),
            self._value("bad_data", self.counts["bad_data"]),
            self._value("range_stale", self.counts["range_stale"]),
            self._value("gyro_missing", self.counts["gyro_missing"]),
            self._value("timestamp_regressions", self.counts["timestamp_regressions"]),
            self._value("flow_frame_gaps", self.counts["flow_frame_gaps"]),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status.append(status)
        self.diagnostics_pub.publish(diagnostics)
        self._write_report()

    def _write_report(self):
        if self.report_path:
            Path(self.report_path).write_text(
                json.dumps({"mode": self.mode, "counts": self.counts}, indent=2) + "\n",
                encoding="utf-8",
            )

    def close(self):
        if rclpy.ok():
            self._diagnostics()
        else:
            self._write_report()
        if self.tcp_socket is not None:
            self.tcp_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = Mtf01pMavlinkBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
