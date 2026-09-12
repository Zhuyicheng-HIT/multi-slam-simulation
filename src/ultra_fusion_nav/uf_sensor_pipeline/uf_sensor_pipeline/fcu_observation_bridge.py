import copy
import math
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import GPSRAW, OpticalFlow, OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, Range

from .fcu_observation import (
    integrate_flu_gyro_as_sensor_frd,
    legacy_pixel_flow_to_sensor_frd,
    legacy_flow_rate_to_sensor_frd,
    stamp_seconds,
    valid_interval,
)


class FcuObservationBridge(Node):
    """Normalize only observations returned by the flight controller through MAVROS."""

    def __init__(self):
        super().__init__("fcu_observation_bridge")
        defaults = {
            "flow_input_topic": "/mavros/optical_flow/raw/optical_flow",
            "flow_rad_input_topic": "",
            "flow_output_topic": "/fcu/optical_flow/rad",
            "imu_input_topic": "/livox/imu",
            "range_input_topic": "/mavros/rangefinder/rangefinder",
            "gnss_fix_input_topic": "/mavros/global_position/raw/fix",
            "gnss_fix_output_topic": "/fcu/gnss/fix",
            "gnss_raw_input_topic": "/mavros/gpsstatus/gps1/raw",
            "gnss_raw_output_topic": "/fcu/gnss/raw",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("minimum_flow_interval_s", 0.005)
        self.declare_parameter("maximum_flow_interval_s", 0.5)
        self.declare_parameter("maximum_imu_gap_s", 0.12)
        self.declare_parameter("maximum_flow_wait_s", 0.3)
        self.declare_parameter("range_timeout_s", 0.5)
        self.declare_parameter("stale_after_s", 1.0)
        self.declare_parameter("flow_focal_length_x_px", 130.254)
        self.declare_parameter("flow_focal_length_y_px", 130.254)

        self.minimum_flow_interval = float(self.get_parameter("minimum_flow_interval_s").value)
        self.maximum_flow_interval = float(self.get_parameter("maximum_flow_interval_s").value)
        self.maximum_imu_gap = float(self.get_parameter("maximum_imu_gap_s").value)
        self.maximum_flow_wait = float(self.get_parameter("maximum_flow_wait_s").value)
        self.range_timeout = float(self.get_parameter("range_timeout_s").value)
        self.stale_after = float(self.get_parameter("stale_after_s").value)
        self.flow_focal_length_x = float(
            self.get_parameter("flow_focal_length_x_px").value
        )
        self.flow_focal_length_y = float(
            self.get_parameter("flow_focal_length_y_px").value
        )
        self.imu_samples = deque(maxlen=2000)
        self.pending_flows = deque(maxlen=100)
        self.previous_flow_stamp_s = None
        self.latest_range = None
        self.latest_range_stamp_s = None
        self.counts = {"flow": 0, "flow_rejected": 0, "flow_without_gyro": 0,
                       "flow_arrival_aligned_gyro": 0,
                       "flow_rad_encoding": 0,
                       "flow_pixel_encoding": 0, "flow_rate_encoding": 0,
                       "flow_range_fallback": 0, "range": 0,
                       "gnss_fix": 0, "gnss_raw": 0}
        self.last_arrival = {}

        self.flow_pub = self.create_publisher(
            OpticalFlowRad, self.get_parameter("flow_output_topic").value,
            qos_profile_sensor_data,
        )
        self.gnss_fix_pub = self.create_publisher(
            NavSatFix, self.get_parameter("gnss_fix_output_topic").value,
            qos_profile_sensor_data,
        )
        self.gnss_raw_pub = self.create_publisher(
            GPSRAW, self.get_parameter("gnss_raw_output_topic").value,
            qos_profile_sensor_data,
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/fcu_observation/diagnostics", 10
        )
        flow_rad_input_topic = str(self.get_parameter("flow_rad_input_topic").value)
        if flow_rad_input_topic:
            self.create_subscription(
                OpticalFlowRad,
                flow_rad_input_topic,
                self._flow_rad,
                qos_profile_sensor_data,
            )
        else:
            self.create_subscription(
                OpticalFlow, self.get_parameter("flow_input_topic").value,
                self._flow, qos_profile_sensor_data,
            )
        self.create_subscription(
            Imu, self.get_parameter("imu_input_topic").value,
            self._imu, qos_profile_sensor_data,
        )
        self.create_subscription(
            Range, self.get_parameter("range_input_topic").value,
            self._range, qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix, self.get_parameter("gnss_fix_input_topic").value,
            self._gnss_fix, qos_profile_sensor_data,
        )
        self.create_subscription(
            GPSRAW, self.get_parameter("gnss_raw_input_topic").value,
            self._gnss_raw, qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            "FCU observation bridge active: MAVROS flow/GNSS with MID360 gyro compensation"
        )

    def _touch(self, name):
        self.last_arrival[name] = self._now_s()

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _imu(self, msg):
        timestamp_s = stamp_seconds(msg.header.stamp)
        if timestamp_s <= 0.0:
            return
        sample = (
            timestamp_s,
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        self.imu_samples.append(sample)
        cutoff_s = timestamp_s - 2.0
        while self.imu_samples and self.imu_samples[0][0] < cutoff_s:
            self.imu_samples.popleft()
        self._flush_pending_flows()

    def _range(self, msg):
        if math.isfinite(float(msg.range)) and msg.min_range <= msg.range <= msg.max_range:
            self.latest_range = float(msg.range)
            self.latest_range_stamp_s = stamp_seconds(msg.header.stamp)
            if self.latest_range_stamp_s <= 0.0:
                self.latest_range_stamp_s = self._now_s()
            self.counts["range"] += 1
            self._touch("range")

    def _flow(self, msg):
        end_s = stamp_seconds(msg.header.stamp)
        interval_s = valid_interval(
            self.previous_flow_stamp_s, end_s,
            self.minimum_flow_interval, self.maximum_flow_interval,
        )
        self.previous_flow_stamp_s = end_s
        if interval_s is None:
            self.counts["flow_rejected"] += 1
            return
        start_s = end_s - interval_s
        queued_ros_s = self._now_s()
        self.pending_flows.append((start_s, end_s, interval_s, queued_ros_s, msg))
        self._flush_pending_flows()

    def _flow_rad(self, msg):
        end_s = stamp_seconds(msg.header.stamp)
        interval_s = float(msg.integration_time_us) * 1.0e-6
        if (
            end_s <= 0.0
            or interval_s < self.minimum_flow_interval
            or interval_s > self.maximum_flow_interval
        ):
            self.counts["flow_rejected"] += 1
            return
        queued_ros_s = self._now_s()
        self.pending_flows.append(
            (end_s - interval_s, end_s, interval_s, queued_ros_s, msg)
        )
        self._flush_pending_flows()

    def _flush_pending_flows(self):
        now = self._now_s()
        while self.pending_flows:
            start_s, end_s, interval_s, queued_at, msg = self.pending_flows[0]
            if queued_at > now:
                self.pending_flows.clear()
                return
            source_covered = bool(
                self.imu_samples
                and self.imu_samples[0][0] <= start_s
                and self.imu_samples[-1][0] >= end_s
            )
            expired = now - queued_at >= self.maximum_flow_wait
            if not source_covered and not expired:
                break
            self.pending_flows.popleft()
            self._publish_flow(msg, start_s, end_s, interval_s)

    def _publish_flow(self, msg, start_s, end_s, interval_s):
        if isinstance(msg, OpticalFlowRad):
            integrated_x = float(msg.integrated_x)
            integrated_y = float(msg.integrated_y)
            quality = int(msg.quality)
            embedded_distance = float(msg.distance)
            self.counts["flow_rad_encoding"] += 1
        else:
            has_rate_extension = (
                math.isfinite(float(msg.flow_rate.x))
                and math.isfinite(float(msg.flow_rate.y))
                and (
                    abs(float(msg.flow_rate.x)) > 1.0e-9
                    or abs(float(msg.flow_rate.y)) > 1.0e-9
                )
            )
            if has_rate_extension:
                integrated_x, integrated_y = legacy_flow_rate_to_sensor_frd(
                    msg.flow_rate.x, msg.flow_rate.y, interval_s
                )
                self.counts["flow_rate_encoding"] += 1
            else:
                integrated_x, integrated_y = legacy_pixel_flow_to_sensor_frd(
                    msg.flow.x,
                    msg.flow.y,
                    self.flow_focal_length_x,
                    self.flow_focal_length_y,
                )
                self.counts["flow_pixel_encoding"] += 1
            quality = int(msg.quality)
            embedded_distance = float(msg.ground_distance)
        gyro = integrate_flu_gyro_as_sensor_frd(
            list(self.imu_samples),
            start_s,
            end_s,
            self.maximum_imu_gap,
        )
        if gyro is None:
            gyro = (float("nan"), float("nan"), float("nan"))
            self.counts["flow_without_gyro"] += 1

        output = OpticalFlowRad()
        output.header = copy.deepcopy(msg.header)
        output.header.frame_id = "flow_sensor_frd_fcu"
        output.integration_time_us = max(1, int(round(interval_s * 1.0e6)))
        output.integrated_x = float(integrated_x)
        output.integrated_y = float(integrated_y)
        output.integrated_xgyro = float(gyro[0])
        output.integrated_ygyro = float(gyro[1])
        output.integrated_zgyro = float(gyro[2])
        output.temperature = 0
        output.quality = quality
        output.time_delta_distance_us = 0
        range_fresh = (
            self.latest_range is not None and self.latest_range_stamp_s is not None
            and abs(end_s - self.latest_range_stamp_s) <= self.range_timeout
        )
        output.distance = (
            float(self.latest_range) if range_fresh else embedded_distance
        )
        if not range_fresh:
            self.counts["flow_range_fallback"] += 1
        self.flow_pub.publish(output)
        self.counts["flow"] += 1
        self._touch("flow")

    def _gnss_fix(self, msg):
        self.gnss_fix_pub.publish(msg)
        self.counts["gnss_fix"] += 1
        self._touch("gnss_fix")

    def _gnss_raw(self, msg):
        self.gnss_raw_pub.publish(msg)
        self.counts["gnss_raw"] += 1
        self._touch("gnss_raw")

    @staticmethod
    def _value(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _diagnostics(self):
        now = self._now_s()
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        for name, source in (
            ("flow", "FCU/MAVLink OPTICAL_FLOW"),
            ("range", "FCU/MAVLink RANGEFINDER"),
            ("gnss_fix", "FCU/MAVLink GPS_RAW_INT NavSatFix"),
            ("gnss_raw", "FCU/MAVLink GPS_RAW_INT"),
        ):
            status = DiagnosticStatus()
            status.name = f"fcu_observation/{name}"
            status.hardware_id = "flight_controller"
            last = self.last_arrival.get(name)
            age_s = math.inf if last is None or last > now else now - last
            status.level = DiagnosticStatus.OK if age_s <= self.stale_after else DiagnosticStatus.ERROR
            status.message = "ok" if status.level == DiagnosticStatus.OK else "stale_or_missing"
            status.values = [
                self._value("source", source),
                self._value("samples", self.counts[name]),
                self._value("age_s", f"{age_s:.3f}" if math.isfinite(age_s) else "missing"),
            ]
            if name == "flow":
                status.values.extend([
                    self._value("rejected_intervals", self.counts["flow_rejected"]),
                    self._value("missing_gyro_integrals", self.counts["flow_without_gyro"]),
                    self._value(
                        "arrival_aligned_gyro_integrals",
                        self.counts["flow_arrival_aligned_gyro"],
                    ),
                    self._value("gyro_integration_clock", "message_stamp"),
                    self._value("range_fallback_samples", self.counts["flow_range_fallback"]),
                    self._value("mavlink1_pixel_samples", self.counts["flow_pixel_encoding"]),
                    self._value("mavlink2_rate_samples", self.counts["flow_rate_encoding"]),
                    self._value("routed_rad_samples", self.counts["flow_rad_encoding"]),
                    self._value("fused_local_position_used", "false"),
                ])
            output.status.append(status)
        self.diagnostic_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FcuObservationBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
