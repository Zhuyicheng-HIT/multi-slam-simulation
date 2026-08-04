from collections import deque
import json
import math
from pathlib import Path
import socket
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import UInt8MultiArray

from .micolink_protocol import (
    MICOLINK_HEADER,
    MICOLINK_MTF01_DEVICE_ID,
    MICOLINK_RANGE_SENSOR_MESSAGE_ID,
    MicoLinkParser,
    MicoLinkSensorClock,
    Mtf01RangeFlow,
    decode_range_flow_payload,
    encode_frame,
    encode_range_flow_frame,
    flow_velocity_to_integrated_radians,
    integrated_radians_to_flow_velocity,
    sensor_interval_seconds,
)
from .optical_flow_model import integrate_gyro, ros_flu_gyro_to_sensor_frd


class Mtf01MicoLinkBridge(Node):
    """Use the same MicoLink decoder for simulated and physical MTF-01 data."""

    def __init__(self):
        super().__init__("mtf01_micolink_bridge")
        defaults = {
            "mode": "sim",
            "input_topic": "/sim/optical_flow/rad_native",
            "flow_topic": "/sim/optical_flow/rad",
            "range_topic": "/sim/optical_flow/range_micolink",
            "raw_frame_topic": "/sim/mtf01/micolink_frame",
            "imu_topic": "/mavros/imu/data_raw",
            "tcp_host": "127.0.0.1",
            "tcp_port": 5764,
            "device_id": MICOLINK_MTF01_DEVICE_ID,
            "system_id": 0,
            "frame_id": "mtf01_flow_frd",
            "nominal_rate_hz": 100.0,
            "maximum_sensor_gap_s": 0.05,
            "maximum_imu_gap_s": 0.12,
            "range_min_m": 0.01,
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
        self.device_id = int(self.get_parameter("device_id").value)
        self.system_id = int(self.get_parameter("system_id").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.nominal_rate_hz = float(self.get_parameter("nominal_rate_hz").value)
        self.maximum_sensor_gap = float(
            self.get_parameter("maximum_sensor_gap_s").value
        )
        self.maximum_imu_gap = float(self.get_parameter("maximum_imu_gap_s").value)
        self.range_min = float(self.get_parameter("range_min_m").value)
        self.range_max = float(self.get_parameter("range_max_m").value)
        self.range_fov = float(self.get_parameter("range_fov_rad").value)
        self.restamp_output = bool(self.get_parameter("restamp_output").value)
        self.report_path = str(self.get_parameter("report_path").value)

        self.parser = MicoLinkParser()
        self.sensor_clock = MicoLinkSensorClock()
        self.sequence = 0
        self.last_sequence = None
        self.last_sensor_time_ms = None
        self.imu_arrival_samples = deque(maxlen=2000)
        self.frame_arrivals = deque(maxlen=1000)
        self.sensor_intervals = deque(maxlen=1000)
        self.last_frame_monotonic = None
        self.last_error = ""
        self.tcp_socket = None
        self.last_connect_attempt = 0.0
        self.counts = {
            "sim_inputs": 0,
            "wire_bytes": 0,
            "valid_frames": 0,
            "range_flow_frames": 0,
            "published_flow": 0,
            "published_range": 0,
            "sequence_gaps": 0,
            "sequence_duplicates": 0,
            "interval_repairs": 0,
            "gyro_missing": 0,
            "invalid_payload": 0,
        }

        self.flow_pub = self.create_publisher(
            OpticalFlowRad,
            str(self.get_parameter("flow_topic").value),
            qos_profile_sensor_data,
        )
        self.range_pub = self.create_publisher(
            Range,
            str(self.get_parameter("range_topic").value),
            qos_profile_sensor_data,
        )
        self.raw_pub = self.create_publisher(
            UInt8MultiArray,
            str(self.get_parameter("raw_frame_topic").value),
            qos_profile_sensor_data,
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/mtf01/micolink_diagnostics", 10
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._imu,
            qos_profile_sensor_data,
        )
        if self.mode == "sim":
            self.create_subscription(
                OpticalFlowRad,
                str(self.get_parameter("input_topic").value),
                self._sim_flow,
                qos_profile_sensor_data,
            )
        else:
            self.create_timer(0.005, self._tcp_tick)
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            f"MTF-01 MicoLink bridge mode={self.mode}, wire=EF/51/20-byte payload, "
            f"nominal_rate={self.nominal_rate_hz:.1f}Hz"
        )

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _imu(self, msg):
        arrival_s = self.get_clock().now().nanoseconds * 1.0e-9
        gyro_frd = ros_flu_gyro_to_sensor_frd(
            (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )
        )
        self.imu_arrival_samples.append((arrival_s, *gyro_frd))

    def _sim_flow(self, msg):
        self.counts["sim_inputs"] += 1
        integration_us = int(msg.integration_time_us)
        integration_s = integration_us * 1.0e-6
        values = (msg.integrated_x, msg.integrated_y, msg.distance, integration_s)
        if not all(math.isfinite(float(value)) for value in values) or integration_s <= 0.0:
            self.counts["invalid_payload"] += 1
            return
        flow_x, flow_y = integrated_radians_to_flow_velocity(
            msg.integrated_x, msg.integrated_y, integration_s
        )
        distance_valid = self.range_min <= float(msg.distance) <= self.range_max
        quality = max(0, min(255, int(msg.quality)))
        observation = Mtf01RangeFlow(
            time_ms=self.sensor_clock.advance(integration_us),
            distance_mm=int(round(float(msg.distance) * 1000.0)) if distance_valid else 0,
            strength=255 if distance_valid else 0,
            precision=0,
            tof_status=1 if distance_valid else 0,
            reserved1=255,
            flow_velocity_x=flow_x,
            flow_velocity_y=flow_y,
            flow_quality=quality,
            flow_status=1 if quality > 0 else 0,
            reserved2=0xFFFF,
        )
        frame = encode_range_flow_frame(
            observation,
            sequence=self.sequence,
            device_id=self.device_id,
            system_id=self.system_id,
        )
        self.sequence = (self.sequence + 1) & 0xFF
        now = self.get_clock().now()
        stamp = now.to_msg() if self.restamp_output else msg.header.stamp
        self._accept_bytes(frame, stamp, now.nanoseconds * 1.0e-9)

    def _connect_tcp(self):
        now = time.monotonic()
        if self.tcp_socket is not None or now - self.last_connect_attempt < 1.0:
            return
        self.last_connect_attempt = now
        try:
            connection = socket.create_connection(
                (self.tcp_host, self.tcp_port), timeout=0.25
            )
            connection.setblocking(False)
            self.tcp_socket = connection
            self.last_error = ""
            self.get_logger().info(
                f"Connected to Windows MTF-01 serial bridge at "
                f"{self.tcp_host}:{self.tcp_port}"
            )
        except OSError as exc:
            self.last_error = str(exc)

    def _close_tcp(self, reason):
        self.last_error = str(reason)
        if self.tcp_socket is not None:
            try:
                self.tcp_socket.close()
            except OSError:
                pass
        self.tcp_socket = None

    def _tcp_tick(self):
        self._connect_tcp()
        if self.tcp_socket is None:
            return
        try:
            data = self.tcp_socket.recv(8192)
            if not data:
                self._close_tcp("serial TCP bridge closed the connection")
                return
        except BlockingIOError:
            return
        except OSError as exc:
            self._close_tcp(exc)
            return
        now = self.get_clock().now()
        self._accept_bytes(data, now.to_msg(), now.nanoseconds * 1.0e-9)

    def _accept_bytes(self, data, stamp, arrival_s):
        self.counts["wire_bytes"] += len(data)
        for frame in self.parser.feed(data):
            self.counts["valid_frames"] += 1
            raw = UInt8MultiArray()
            raw.data = list(
                encode_frame(
                    frame.payload,
                    device_id=frame.device_id,
                    system_id=frame.system_id,
                    message_id=frame.message_id,
                    sequence=frame.sequence,
                )
            )
            self.raw_pub.publish(raw)
            if frame.message_id != MICOLINK_RANGE_SENSOR_MESSAGE_ID:
                continue
            try:
                observation = decode_range_flow_payload(frame.payload)
            except ValueError:
                self.counts["invalid_payload"] += 1
                continue
            self._publish_observation(frame.sequence, observation, stamp, arrival_s)

    def _publish_observation(self, sequence, observation, stamp, arrival_s):
        self.counts["range_flow_frames"] += 1
        if self.last_sequence is not None:
            sequence_advance = (sequence - self.last_sequence) & 0xFF
            if sequence_advance == 0:
                self.counts["sequence_duplicates"] += 1
            elif sequence_advance != 1:
                self.counts["sequence_gaps"] += sequence_advance - 1
        self.last_sequence = sequence

        previous_sensor_time_ms = self.last_sensor_time_ms
        sensor_interval_s = sensor_interval_seconds(
            previous_sensor_time_ms,
            observation.time_ms,
            nominal_rate_hz=self.nominal_rate_hz,
        )
        self.last_sensor_time_ms = observation.time_ms
        if (
            previous_sensor_time_ms is not None
            and sensor_interval_s is not None
            and sensor_interval_s <= self.maximum_sensor_gap
        ):
            self.sensor_intervals.append(sensor_interval_s)
        integration_s = sensor_interval_s
        if integration_s is None or integration_s > self.maximum_sensor_gap:
            integration_s = 1.0 / max(self.nominal_rate_hz, 1.0)
            self.counts["interval_repairs"] += 1

        integrated_x, integrated_y = flow_velocity_to_integrated_radians(
            observation.flow_velocity_x,
            observation.flow_velocity_y,
            integration_s,
        )
        gyro = integrate_gyro(
            list(self.imu_arrival_samples),
            arrival_s - integration_s,
            arrival_s,
            max_gap_s=self.maximum_imu_gap,
        )
        if gyro is None:
            gyro = (float("nan"), float("nan"), float("nan"))
            self.counts["gyro_missing"] += 1

        distance_valid = (
            observation.tof_status > 0
            and observation.distance_mm >= 10
            and observation.distance_mm <= int(round(self.range_max * 1000.0))
        )
        distance_m = (
            observation.distance_mm * 1.0e-3 if distance_valid else float("nan")
        )
        quality = observation.flow_quality if observation.flow_status > 0 else 0

        output = OpticalFlowRad()
        output.header.stamp = stamp
        output.header.frame_id = self.frame_id
        output.integration_time_us = max(1, int(round(integration_s * 1.0e6)))
        output.integrated_x = float(integrated_x)
        output.integrated_y = float(integrated_y)
        output.integrated_xgyro = float(gyro[0])
        output.integrated_ygyro = float(gyro[1])
        output.integrated_zgyro = float(gyro[2])
        output.temperature = 0
        output.quality = int(quality)
        output.time_delta_distance_us = 0
        output.distance = float(distance_m)
        self.flow_pub.publish(output)
        self.counts["published_flow"] += 1

        if distance_valid:
            range_msg = Range()
            range_msg.header.stamp = stamp
            range_msg.header.frame_id = self.frame_id
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = self.range_fov
            range_msg.min_range = self.range_min
            range_msg.max_range = self.range_max
            range_msg.range = distance_m
            self.range_pub.publish(range_msg)
            self.counts["published_range"] += 1

        self.last_frame_monotonic = time.monotonic()
        self.frame_arrivals.append(self.last_frame_monotonic)

    @staticmethod
    def _value(key, value):
        return KeyValue(key=str(key), value=str(value))

    def _report(self):
        arrival_rate_hz = 0.0
        if len(self.frame_arrivals) >= 2:
            duration = self.frame_arrivals[-1] - self.frame_arrivals[0]
            if duration > 0.0:
                arrival_rate_hz = (len(self.frame_arrivals) - 1) / duration
        sensor_rate_hz = 0.0
        if self.sensor_intervals:
            mean_interval = sum(self.sensor_intervals) / len(self.sensor_intervals)
            if mean_interval > 0.0:
                sensor_rate_hz = 1.0 / mean_interval
        return {
            "mode": self.mode,
            "wire_protocol": "MicoLink",
            "baud_rate": 115200,
            "frame": {
                "header": MICOLINK_HEADER,
                "header_hex": "0xEF",
                "message_id": MICOLINK_RANGE_SENSOR_MESSAGE_ID,
                "message_id_hex": "0x51",
                "payload_bytes": 20,
                "total_bytes": 27,
                "device_id": self.device_id,
                "system_id": self.system_id,
            },
            "tcp": {
                "host": self.tcp_host,
                "port": self.tcp_port,
                "connected": self.tcp_socket is not None,
            },
            "sensor_clock_rate_hz": sensor_rate_hz,
            "host_arrival_rate_hz": arrival_rate_hz,
            "counts": dict(self.counts),
            "parser": {
                "frames_decoded": self.parser.frames_decoded,
                "checksum_errors": self.parser.checksum_errors,
                "length_errors": self.parser.length_errors,
                "discarded_bytes": self.parser.discarded_bytes,
            },
            "last_error": self.last_error,
        }

    def _diagnostics(self):
        report = self._report()
        fresh = (
            self.last_frame_monotonic is not None
            and time.monotonic() - self.last_frame_monotonic < 0.5
        )
        status = DiagnosticStatus()
        status.name = "mtf01/micolink"
        status.hardware_id = "mtf01_sim" if self.mode == "sim" else "mtf01_com_bridge"
        if fresh and report["parser"]["checksum_errors"] == 0:
            status.level = DiagnosticStatus.OK
            status.message = "micolink_stream_ok"
        elif fresh:
            status.level = DiagnosticStatus.WARN
            status.message = "micolink_stream_has_errors"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "micolink_stream_missing"
        status.values = [
            self._value("mode", self.mode),
            self._value("wire_protocol", "MicoLink"),
            self._value(
                "sensor_clock_rate_hz", f"{report['sensor_clock_rate_hz']:.3f}"
            ),
            self._value(
                "host_arrival_rate_hz", f"{report['host_arrival_rate_hz']:.3f}"
            ),
            self._value("valid_frames", self.counts["valid_frames"]),
            self._value("checksum_errors", self.parser.checksum_errors),
            self._value("sequence_gaps", self.counts["sequence_gaps"]),
            self._value(
                "sequence_duplicates", self.counts["sequence_duplicates"]
            ),
            self._value("interval_repairs", self.counts["interval_repairs"]),
            self._value("gyro_missing", self.counts["gyro_missing"]),
            self._value("last_error", self.last_error or "none"),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status.append(status)
        self.diagnostic_pub.publish(diagnostics)
        if self.report_path:
            path = Path(self.report_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    def close(self):
        self._diagnostics()
        if self.tcp_socket is not None:
            self.tcp_socket.close()
            self.tcp_socket = None


def main(args=None):
    rclpy.init(args=args)
    node = Mtf01MicoLinkBridge()
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
