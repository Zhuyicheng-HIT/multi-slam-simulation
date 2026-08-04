import json
import math
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from pymavlink import mavutil
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .mtf01p_protocol import (
    DISTANCE_SENSOR_MESSAGE_ID,
    MTF01P_FOV_RAD,
    MTF01P_WIDTH_PX,
    OPTICAL_FLOW_MESSAGE_ID,
    SensorClock,
    compensated_planar_velocity,
    focal_length_px,
    integrated_radians_to_pixels,
)


class Mtf01pMavlinkSensor(Node):
    """Encode simulated flow exactly as an MTF01P mavlink_apm serial source."""

    def __init__(self):
        super().__init__("mtf01p_mavlink_sensor")
        defaults = {
            "input_topic": "/sim/optical_flow/rad",
            "connection_url": "tcp:127.0.0.1:5762",
            "source_system": 200,
            "source_component": 197,
            "image_width_px": MTF01P_WIDTH_PX,
            "field_of_view_rad": MTF01P_FOV_RAD,
            "minimum_distance_m": 0.08,
            "maximum_distance_m": 12.0,
            "report_path": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        width = int(self.get_parameter("image_width_px").value)
        fov = float(self.get_parameter("field_of_view_rad").value)
        self.fx_px = focal_length_px(width, fov)
        self.fy_px = self.fx_px
        self.connection_url = str(self.get_parameter("connection_url").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)
        self.minimum_distance = float(self.get_parameter("minimum_distance_m").value)
        self.maximum_distance = float(self.get_parameter("maximum_distance_m").value)
        self.report_path = str(self.get_parameter("report_path").value)
        self.sensor_clock = SensorClock()
        self.connection = None
        self.counts = {
            "input": 0,
            "sent_flow": 0,
            "sent_distance": 0,
            "received_routed": 0,
            "dropped": 0,
        }
        self.last_input_monotonic = None
        self.arrivals = []
        self.last_error = ""

        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/mtf01p/sensor_diagnostics", 10
        )
        self.create_subscription(
            OpticalFlowRad,
            self.get_parameter("input_topic").value,
            self._flow,
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._ensure_connection)
        self.create_timer(0.02, self._drain_incoming)
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            "MTF01P mavlink_apm emulator: MAVLink1 OPTICAL_FLOW(100) + "
            f"DISTANCE_SENSOR(132), sysid={self.source_system}, fx={self.fx_px:.3f}px"
        )

    def _ensure_connection(self):
        if self.connection is not None:
            return
        try:
            self.connection = mavutil.mavlink_connection(
                self.connection_url,
                source_system=self.source_system,
                source_component=self.source_component,
                dialect="ardupilotmega",
                autoreconnect=True,
                retries=1,
            )
            self.last_error = ""
            self.get_logger().info(f"MTF01P serial link connected: {self.connection_url}")
        except Exception as exc:
            self.last_error = str(exc)
            self.connection = None

    def _write_mavlink1(self, message):
        packet = message.pack(self.connection.mav, force_mavlink1=True)
        self.connection.write(packet)

    def _drain_incoming(self):
        if self.connection is None:
            return
        try:
            for _ in range(100):
                message = self.connection.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_type() != "BAD_DATA":
                    self.counts["received_routed"] += 1
        except Exception as exc:
            self.last_error = str(exc)
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None

    def _flow(self, msg):
        self.counts["input"] += 1
        now = time.monotonic()
        self.last_input_monotonic = now
        self.arrivals.append(now)
        self.arrivals = self.arrivals[-300:]
        if self.connection is None:
            self.counts["dropped"] += 1
            return

        integration_us = int(msg.integration_time_us)
        distance = float(msg.distance)
        values = (
            msg.integrated_x,
            msg.integrated_y,
            msg.integrated_xgyro,
            msg.integrated_ygyro,
            distance,
        )
        if integration_us <= 0 or not all(math.isfinite(float(value)) for value in values):
            self.counts["dropped"] += 1
            return

        sensor_time_us = self.sensor_clock.advance(integration_us)
        flow_x, flow_y = integrated_radians_to_pixels(
            msg.integrated_x, msg.integrated_y, self.fx_px, self.fy_px
        )
        velocity_x, velocity_y = compensated_planar_velocity(
            msg.integrated_x,
            msg.integrated_y,
            msg.integrated_xgyro,
            msg.integrated_ygyro,
            integration_us * 1.0e-6,
            distance,
        )
        quality = max(0, min(255, int(msg.quality)))
        distance_for_flow = distance if distance > 0.0 else -1.0
        distance_cm = max(0, min(65535, int(round(distance * 100.0))))
        minimum_cm = max(0, min(65535, int(round(self.minimum_distance * 100.0))))
        maximum_cm = max(minimum_cm, min(65535, int(round(self.maximum_distance * 100.0))))

        flow_message = mavutil.mavlink.MAVLink_optical_flow_message(
            sensor_time_us,
            0,
            flow_x,
            flow_y,
            float(velocity_x),
            float(velocity_y),
            quality,
            float(distance_for_flow),
        )
        distance_message = mavutil.mavlink.MAVLink_distance_sensor_message(
            (sensor_time_us // 1000) & 0xFFFFFFFF,
            minimum_cm,
            maximum_cm,
            distance_cm,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            0,
            mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            255,
        )
        try:
            self._write_mavlink1(flow_message)
            self.counts["sent_flow"] += 1
            self._write_mavlink1(distance_message)
            self.counts["sent_distance"] += 1
            self.last_error = ""
        except Exception as exc:
            self.last_error = str(exc)
            self.counts["dropped"] += 1
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None

    @staticmethod
    def _value(key, value):
        return KeyValue(key=key, value=str(value))

    def _report(self):
        rate_hz = 0.0
        if len(self.arrivals) >= 2 and self.arrivals[-1] > self.arrivals[0]:
            rate_hz = (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])
        return {
            "transport": "MTF01P mavlink_apm",
            "wire_protocol": "MAVLink1",
            "source_system": self.source_system,
            "source_component": self.source_component,
            "message_ids": {
                "optical_flow": OPTICAL_FLOW_MESSAGE_ID,
                "distance_sensor": DISTANCE_SENSOR_MESSAGE_ID,
            },
            "connection_url": self.connection_url,
            "connected": self.connection is not None,
            "focal_length_px": self.fx_px,
            "measured_input_rate_hz": rate_hz,
            "sensor_time_usec": self.sensor_clock.time_usec,
            "counts": dict(self.counts),
            "last_error": self.last_error,
        }

    def _diagnostics(self):
        report = self._report()
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "mtf01p/mavlink_sensor"
        status.hardware_id = "mtf01p_sim"
        fresh = (
            self.last_input_monotonic is not None
            and time.monotonic() - self.last_input_monotonic < 1.0
        )
        status.level = (
            DiagnosticStatus.OK
            if report["connected"] and fresh and self.counts["sent_flow"] > 0
            else DiagnosticStatus.ERROR
        )
        status.message = "routing_input_ok" if status.level == DiagnosticStatus.OK else "missing"
        status.values = [
            self._value("wire_protocol", report["wire_protocol"]),
            self._value("optical_flow_message_id", OPTICAL_FLOW_MESSAGE_ID),
            self._value("distance_sensor_message_id", DISTANCE_SENSOR_MESSAGE_ID),
            self._value("source_system", self.source_system),
            self._value("measured_input_rate_hz", f"{report['measured_input_rate_hz']:.3f}"),
            self._value("sent_flow", self.counts["sent_flow"]),
            self._value("sent_distance", self.counts["sent_distance"]),
            self._value("dropped", self.counts["dropped"]),
            self._value("last_error", self.last_error or "none"),
        ]
        output.status.append(status)
        self.diagnostic_pub.publish(output)
        if self.report_path:
            path = Path(self.report_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = Mtf01pMavlinkSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._diagnostics()
        if node.connection is not None:
            node.connection.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
