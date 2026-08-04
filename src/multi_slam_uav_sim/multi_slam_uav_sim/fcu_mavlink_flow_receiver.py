import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import Mavlink, OpticalFlow, OpticalFlowRad
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from .mtf01p_protocol import (
    DISTANCE_SENSOR_MESSAGE_ID,
    OPTICAL_FLOW_MESSAGE_ID,
    decode_distance_sensor_payload,
    decode_optical_flow_payload,
    focal_length_px,
    mavros_payload_bytes,
    pixels_to_integrated_radians,
    sensor_frd_to_ros_flu,
)


PARAMETERS_TO_VERIFY = (
    "SERIAL0_PROTOCOL",
    "SERIAL1_PROTOCOL",
    "MAV1_OPTIONS",
    "MAV2_OPTIONS",
    "FLOW_TYPE",
    "RNGFND1_TYPE",
)


class FcuMavlinkFlowReceiver(Node):
    """Decode MTF01P packets routed by ArduPilot into MAVROS' raw ROS endpoint."""

    def __init__(self):
        super().__init__("fcu_mavlink_flow_receiver")
        defaults = {
            "input_topic": "/uas1/mavlink_source",
            "parameter_service": "/mavros/param/get_parameters",
            "sensor_system_id": 200,
            "flow_output_topic": "/fcu/mavlink/optical_flow",
            "flow_rad_output_topic": "/fcu/mavlink/optical_flow_rad",
            "range_output_topic": "/fcu/mavlink/range",
            "report_path": "",
            "minimum_range_m": 0.08,
            "maximum_range_m": 12.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.sensor_system_id = int(self.get_parameter("sensor_system_id").value)
        self.report_path = str(self.get_parameter("report_path").value)
        self.minimum_range = float(self.get_parameter("minimum_range_m").value)
        self.maximum_range = float(self.get_parameter("maximum_range_m").value)
        self.parameters = {}
        self.parameter_request_pending = False
        self.last_error = ""
        self.last_source_time_us = None
        self.last_source_period_s = None
        self.last_output_stamp_ns = None
        self.source_periods_s = []
        self.arrivals = []
        self.last_raw_arrival = None
        self.last_flow_arrival = None
        self.counts = {
            "received_total": 0,
            "optical_flow": 0,
            "distance_sensor": 0,
            "wrong_source_flow": 0,
            "bad_framing": 0,
            "non_mavlink1": 0,
            "decode_errors": 0,
            "missing_timestamp": 0,
            "timestamp_regressions": 0,
            "published_flow": 0,
            "published_flow_rad": 0,
            "published_range": 0,
            "host_timestamp_repairs": 0,
        }

        self.flow_pub = self.create_publisher(
            OpticalFlow,
            self.get_parameter("flow_output_topic").value,
            qos_profile_sensor_data,
        )
        self.flow_rad_pub = self.create_publisher(
            OpticalFlowRad,
            self.get_parameter("flow_rad_output_topic").value,
            qos_profile_sensor_data,
        )
        self.range_pub = self.create_publisher(
            Range,
            self.get_parameter("range_output_topic").value,
            qos_profile_sensor_data,
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/fcu/mavlink_flow_route_diagnostics", 10
        )
        self.create_subscription(
            Mavlink, self.input_topic, self._mavlink, qos_profile_sensor_data
        )
        self.param_client = self.create_client(
            GetParameters, self.get_parameter("parameter_service").value
        )
        self.create_timer(1.0, self._request_parameter)
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            "Companion flow receiver reading ArduPilot-routed MAVLink from "
            f"{self.input_topic}; sensor sysid={self.sensor_system_id}"
        )

    def _request_parameter(self):
        if self.parameter_request_pending:
            return
        missing = [name for name in PARAMETERS_TO_VERIFY if name not in self.parameters]
        if not missing or not self.param_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = missing
        future = self.param_client.call_async(request)
        self.parameter_request_pending = True
        future.add_done_callback(
            lambda completed, names=tuple(missing): self._parameter_result(
                names, completed
            )
        )

    def _parameter_result(self, names, future):
        self.parameter_request_pending = False
        try:
            response = future.result()
            if response is None or len(response.values) != len(names):
                self.last_error = "parameter read returned an incomplete response"
                return
            for name, value in zip(names, response.values):
                if value.type == ParameterType.PARAMETER_INTEGER:
                    self.parameters[name] = float(value.integer_value)
                elif value.type == ParameterType.PARAMETER_DOUBLE:
                    self.parameters[name] = float(value.double_value)
                elif value.type == ParameterType.PARAMETER_BOOL:
                    self.parameters[name] = float(value.bool_value)
            self.last_error = ""
        except Exception as exc:
            self.last_error = f"parameter read: {exc}"

    def _mavlink(self, message):
        self.counts["received_total"] += 1
        self.last_raw_arrival = time.monotonic()
        if message.framing_status != Mavlink.FRAMING_OK:
            self.counts["bad_framing"] += 1
            return
        if message.msgid not in (OPTICAL_FLOW_MESSAGE_ID, DISTANCE_SENSOR_MESSAGE_ID):
            return
        if int(message.sysid) != self.sensor_system_id:
            if message.msgid == OPTICAL_FLOW_MESSAGE_ID:
                self.counts["wrong_source_flow"] += 1
            return
        if message.magic != Mavlink.MAVLINK_V10:
            self.counts["non_mavlink1"] += 1
            return
        try:
            payload = mavros_payload_bytes(message.payload64, message.len)
            if message.msgid == OPTICAL_FLOW_MESSAGE_ID:
                self._optical_flow(decode_optical_flow_payload(payload))
            else:
                self._distance_sensor(decode_distance_sensor_payload(payload))
        except (ValueError, TypeError) as exc:
            self.counts["decode_errors"] += 1
            self.last_error = str(exc)

    def _mapped_stamp_ns(self, source_time_us):
        source_time_us = int(source_time_us)
        if source_time_us <= 0:
            self.counts["missing_timestamp"] += 1
            return None
        if self.last_source_time_us is not None:
            if source_time_us <= self.last_source_time_us:
                self.counts["timestamp_regressions"] += 1
                return None
            self.last_source_period_s = (
                source_time_us - self.last_source_time_us
            ) * 1.0e-6
            self.source_periods_s.append(self.last_source_period_s)
            self.source_periods_s = self.source_periods_s[-300:]
        self.last_source_time_us = source_time_us
        mapped_ns = self.get_clock().now().nanoseconds
        if self.last_output_stamp_ns is not None and mapped_ns <= self.last_output_stamp_ns:
            mapped_ns = self.last_output_stamp_ns + 1000
            self.counts["host_timestamp_repairs"] += 1
        self.last_output_stamp_ns = mapped_ns
        return mapped_ns

    @staticmethod
    def _stamp_from_ns(nanoseconds):
        return rclpy.time.Time(nanoseconds=int(nanoseconds)).to_msg()

    def _optical_flow(self, decoded):
        self.counts["optical_flow"] += 1
        stamp_ns = self._mapped_stamp_ns(decoded["time_usec"])
        if stamp_ns is None:
            return
        arrival = time.monotonic()
        self.last_flow_arrival = arrival
        self.arrivals.append(arrival)
        self.arrivals = self.arrivals[-300:]

        flow_x, flow_y = sensor_frd_to_ros_flu(
            decoded["flow_x"], decoded["flow_y"]
        )
        velocity_x, velocity_y = sensor_frd_to_ros_flu(
            decoded["flow_comp_m_x"], decoded["flow_comp_m_y"]
        )
        output = OpticalFlow()
        output.header.stamp = self._stamp_from_ns(stamp_ns)
        output.header.frame_id = "flow_sensor_frd_via_fcu"
        output.flow.x = flow_x
        output.flow.y = flow_y
        output.flow_comp_m.x = velocity_x
        output.flow_comp_m.y = velocity_y
        # MTF01P mavlink_apm uses the MAVLink1 base payload, so rate extensions are absent.
        output.flow_rate.x = 0.0
        output.flow_rate.y = 0.0
        output.quality = max(0, min(255, int(decoded["quality"])))
        output.ground_distance = float(decoded["ground_distance"])
        self.flow_pub.publish(output)
        self.counts["published_flow"] += 1

        if self.last_source_period_s is not None and 0.005 <= self.last_source_period_s <= 0.5:
            focal = focal_length_px()
            integrated_x, integrated_y = pixels_to_integrated_radians(
                decoded["flow_x"], decoded["flow_y"], focal, focal
            )
            radial = OpticalFlowRad()
            radial.header = output.header
            radial.header.frame_id = "flow_sensor_frd_via_fcu"
            radial.integration_time_us = max(
                1, int(round(self.last_source_period_s * 1.0e6))
            )
            radial.integrated_x = float(integrated_x)
            radial.integrated_y = float(integrated_y)
            radial.integrated_xgyro = float("nan")
            radial.integrated_ygyro = float("nan")
            radial.integrated_zgyro = float("nan")
            radial.temperature = 0
            radial.quality = output.quality
            radial.time_delta_distance_us = 0
            radial.distance = output.ground_distance
            self.flow_rad_pub.publish(radial)
            self.counts["published_flow_rad"] += 1

    def _distance_sensor(self, decoded):
        self.counts["distance_sensor"] += 1
        output = Range()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "flow_range_down_via_fcu"
        output.radiation_type = Range.INFRARED
        output.field_of_view = math.radians(1.5)
        output.min_range = max(
            self.minimum_range, decoded["min_distance_cm"] * 0.01
        )
        output.max_range = min(
            self.maximum_range, decoded["max_distance_cm"] * 0.01
        )
        output.range = decoded["current_distance_cm"] * 0.01
        self.range_pub.publish(output)
        self.counts["published_range"] += 1

    @staticmethod
    def _value(key, value):
        return KeyValue(key=key, value=str(value))

    def _rate_hz(self):
        if len(self.arrivals) < 2 or self.arrivals[-1] <= self.arrivals[0]:
            return 0.0
        return (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])

    def _report(self):
        timestamped = self.counts["optical_flow"] - self.counts["missing_timestamp"]
        timestamp_ratio = (
            timestamped / self.counts["optical_flow"]
            if self.counts["optical_flow"]
            else 0.0
        )
        return {
            "route": "MTF01P -> SERIAL1 -> ArduPilot -> SERIAL0 -> MAVROS raw source",
            "input_wire_protocol": "MAVLink1",
            "mavros_raw_topic": self.input_topic,
            "sensor_system_id": self.sensor_system_id,
            "message_ids": {
                "optical_flow": OPTICAL_FLOW_MESSAGE_ID,
                "distance_sensor": DISTANCE_SENSOR_MESSAGE_ID,
            },
            "mav_channel_mapping": {
                "MAV1_OPTIONS": "first MAVLink port, SERIAL0/MAVROS companion link",
                "MAV2_OPTIONS": "second MAVLink port, SERIAL1/MTF01P",
                "no_forward_bit_value": 2,
            },
            "verified_parameters": dict(self.parameters),
            "parameters_complete": all(
                name in self.parameters for name in PARAMETERS_TO_VERIFY
            ),
            "measured_routed_flow_rate_hz": self._rate_hz(),
            "source_period_median_s": (
                statistics.median(self.source_periods_s)
                if self.source_periods_s
                else None
            ),
            "source_timestamp_present_ratio": timestamp_ratio,
            "ros_header_stamp_mode": "monotonic_repaired_FCU_route_receive_time",
            "counts": dict(self.counts),
            "last_error": self.last_error,
        }

    def _diagnostics(self):
        report = self._report()
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "fcu/mavlink_flow_route"
        status.hardware_id = "ardupilot_sitl"
        fresh = (
            self.last_flow_arrival is not None
            and time.monotonic() - self.last_flow_arrival < 1.0
        )
        parameter_values_ok = all(
            self.parameters.get(name) == expected
            for name, expected in (
                ("SERIAL0_PROTOCOL", 2.0),
                ("SERIAL1_PROTOCOL", 1.0),
                ("MAV1_OPTIONS", 0.0),
                ("MAV2_OPTIONS", 0.0),
                ("FLOW_TYPE", 5.0),
                ("RNGFND1_TYPE", 10.0),
            )
        )
        healthy = (
            fresh
            and report["parameters_complete"]
            and parameter_values_ok
            and self.counts["bad_framing"] == 0
            and self.counts["non_mavlink1"] == 0
            and self.counts["decode_errors"] == 0
            and self.counts["timestamp_regressions"] == 0
            and report["source_timestamp_present_ratio"] >= 0.99
        )
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "route_verified" if healthy else "route_incomplete"
        status.values = [
            self._value("input_message", "MAVLink1 OPTICAL_FLOW(100)"),
            self._value("range_message", "MAVLink1 DISTANCE_SENSOR(132)"),
            self._value("sensor_system_id", self.sensor_system_id),
            self._value("routed_rate_hz", f"{report['measured_routed_flow_rate_hz']:.3f}"),
            self._value("source_period_median_s", report["source_period_median_s"]),
            self._value("source_timestamp_present_ratio", f"{report['source_timestamp_present_ratio']:.6f}"),
            self._value("source_timestamp_regressions", self.counts["timestamp_regressions"]),
            self._value("parameters_complete", report["parameters_complete"]),
            self._value("verified_parameters", json.dumps(self.parameters, sort_keys=True)),
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
    node = FcuMavlinkFlowReceiver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._diagnostics()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
