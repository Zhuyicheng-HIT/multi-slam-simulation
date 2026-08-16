"""Publish paired GNSS fixes and receiver metadata at the algorithm rate."""

from collections import deque
import copy
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import GPSRAW
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


def _stamp_ns(message):
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class LatestGnssPairBuffer:
    """Keep the newest complete fix/raw pair without changing either stamp."""

    def __init__(self, tolerance_s, capacity=32):
        self.tolerance_ns = max(0, int(round(float(tolerance_s) * 1.0e9)))
        self.fixes = deque(maxlen=max(2, int(capacity)))
        self.raw_samples = deque(maxlen=max(2, int(capacity)))
        self.last_fix_stamp_ns = None

    def add_fix(self, message):
        stamp_ns = _stamp_ns(message)
        if stamp_ns > 0:
            self.fixes.append((stamp_ns, copy.deepcopy(message)))

    def add_raw(self, message):
        stamp_ns = _stamp_ns(message)
        if stamp_ns > 0:
            self.raw_samples.append((stamp_ns, copy.deepcopy(message)))

    def take_latest(self):
        if not self.fixes:
            return None
        fix_stamp_ns, fix = self.fixes[-1]
        if fix_stamp_ns == self.last_fix_stamp_ns:
            return None
        raw_match = None
        if self.raw_samples:
            raw_stamp_ns, raw = min(
                self.raw_samples,
                key=lambda item: abs(item[0] - fix_stamp_ns),
            )
            if abs(raw_stamp_ns - fix_stamp_ns) <= self.tolerance_ns:
                raw_match = raw
        self.last_fix_stamp_ns = fix_stamp_ns
        while self.fixes and self.fixes[0][0] <= fix_stamp_ns:
            self.fixes.popleft()
        raw_cutoff_ns = fix_stamp_ns - self.tolerance_ns
        while self.raw_samples and self.raw_samples[0][0] < raw_cutoff_ns:
            self.raw_samples.popleft()
        return fix, raw_match


class GnssMetadataRelay(Node):
    def __init__(self):
        super().__init__("gnss_metadata_relay")
        self.declare_parameter("input_topic", "/mavros/gpsstatus/gps1/raw")
        self.declare_parameter("output_topic", "/sensors/gnss/raw")
        self.declare_parameter(
            "fix_input_topic", "/sensors/gnss/fix_unthrottled"
        )
        self.declare_parameter("fix_output_topic", "/sensors/gnss/fix")
        self.declare_parameter("output_rate_hz", 2.5)
        self.declare_parameter("association_tolerance_s", 0.06)
        output_rate_hz = float(self.get_parameter("output_rate_hz").value)
        if not math.isfinite(output_rate_hz) or not 2.0 <= output_rate_hz <= 3.0:
            raise ValueError("output_rate_hz must be within the realistic 2-3 Hz range")
        self.buffer = LatestGnssPairBuffer(
            float(self.get_parameter("association_tolerance_s").value)
        )
        self.published_fixes = 0
        self.published_raw_pairs = 0
        self.unpaired_fixes = 0
        self.last_fix_had_raw = False
        self.raw_publisher = self.create_publisher(
            GPSRAW, str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.fix_publisher = self.create_publisher(
            NavSatFix, str(self.get_parameter("fix_output_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GPSRAW, str(self.get_parameter("input_topic").value),
            self.buffer.add_raw, qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix, str(self.get_parameter("fix_input_topic").value),
            self.buffer.add_fix, qos_profile_sensor_data,
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/sensors/gnss/rate_diagnostics", 10
        )
        self.create_timer(1.0 / output_rate_hz, self._publish_latest_pair)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            "algorithm GNSS relay: paired latest-sample output at "
            f"{output_rate_hz:.3f} Hz"
        )

    def _publish_latest_pair(self):
        pair = self.buffer.take_latest()
        if pair is None:
            return
        fix, raw = pair
        self.fix_publisher.publish(fix)
        self.published_fixes += 1
        self.last_fix_had_raw = raw is not None
        if raw is None:
            self.unpaired_fixes += 1
        else:
            self.raw_publisher.publish(raw)
            self.published_raw_pairs += 1

    @staticmethod
    def _diagnostic_value(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "sensor_pipeline/gnss_algorithm_rate"
        status.hardware_id = "companion_gnss_input"
        status.level = (
            DiagnosticStatus.OK
            if self.published_fixes == 0 or self.last_fix_had_raw
            else DiagnosticStatus.WARN
        )
        status.message = (
            "waiting_for_fix"
            if self.published_fixes == 0
            else "paired_fix_and_raw"
            if self.last_fix_had_raw
            else "fix_published_without_fresh_raw_metadata"
        )
        status.values = [
            self._diagnostic_value("fixes_published", self.published_fixes),
            self._diagnostic_value("raw_pairs_published", self.published_raw_pairs),
            self._diagnostic_value("unpaired_fixes", self.unpaired_fixes),
            self._diagnostic_value(
                "raw_metadata_required_for_fix", "false"
            ),
        ]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status.append(status)
        self.diagnostic_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GnssMetadataRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
