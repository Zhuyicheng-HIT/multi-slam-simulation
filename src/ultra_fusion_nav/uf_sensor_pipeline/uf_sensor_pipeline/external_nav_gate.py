import copy
import math
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from uf_interfaces.msg import SchedulerState


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def scheduler_state_allowed(health_state, allowed_states):
    """Return whether a scheduler state permits an ExternalNav hand-off."""
    normalized = str(health_state).strip().upper()
    return normalized in {
        str(state).strip().upper()
        for state in allowed_states
        if str(state).strip()
    }


class ExternalNavGate(Node):
    """Validate fusion odometry before exposing it to the MAVROS ODOMETRY plugin."""

    def __init__(self):
        super().__init__("external_nav_gate")
        self.declare_parameter("input_topic", "/fusion/gps_flow/odom")
        self.declare_parameter("output_topic", "/mavros/odometry/out")
        self.declare_parameter("expected_map_frame", "map")
        self.declare_parameter("expected_body_frame", "base_link")
        self.declare_parameter("maximum_input_age_s", 0.25)
        self.declare_parameter("minimum_rate_hz", 4.0)
        self.declare_parameter("enabled", True)
        # Disabled by default for the legacy GPS/flow path. The unified backend
        # explicitly enables this, so an unhealthy scheduler cannot be sent to
        # the FCU just because the odometry packet itself is well formed.
        self.declare_parameter("require_scheduler_health", False)
        self.declare_parameter("scheduler_topic", "/reliability/scheduler_state")
        self.declare_parameter("scheduler_timeout_s", 0.5)
        self.declare_parameter("allowed_scheduler_states", ["NORMAL", "RECOVERED"])
        self.expected_map_frame = str(self.get_parameter("expected_map_frame").value)
        self.expected_body_frame = str(self.get_parameter("expected_body_frame").value)
        self.maximum_input_age = float(self.get_parameter("maximum_input_age_s").value)
        self.minimum_rate = float(self.get_parameter("minimum_rate_hz").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.require_scheduler_health = bool(
            self.get_parameter("require_scheduler_health").value
        )
        self.scheduler_timeout_s = max(
            0.01, float(self.get_parameter("scheduler_timeout_s").value)
        )
        self.allowed_scheduler_states = tuple(
            str(state).strip().upper()
            for state in self.get_parameter("allowed_scheduler_states").value
            if str(state).strip()
        )
        self.arrivals = deque(maxlen=500)
        self.callback_ms = deque(maxlen=500)
        self.accepted = 0
        self.rejected = 0
        self.last_arrival = None
        self.last_reason = "waiting_for_fusion"
        self.last_scheduler_arrival = None
        self.last_scheduler_state = "WAITING"
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/external_nav/diagnostics", 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("input_topic").value), self._odom, 20)
        if self.require_scheduler_health:
            self.create_subscription(
                SchedulerState,
                str(self.get_parameter("scheduler_topic").value),
                self._scheduler_state,
                20,
            )
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            f"ExternalNav gate {'enabled' if self.enabled else 'disabled'}: "
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}; "
            f"scheduler_health={'required' if self.require_scheduler_health else 'not_required'}")

    def _scheduler_state(self, msg):
        self.last_scheduler_arrival = time.monotonic()
        self.last_scheduler_state = str(msg.health_state).strip().upper() or "UNKNOWN"

    def _scheduler_reason(self):
        if not self.require_scheduler_health:
            return "ok"
        if self.last_scheduler_arrival is None:
            return "missing_scheduler_state"
        if time.monotonic() - self.last_scheduler_arrival > self.scheduler_timeout_s:
            return "stale_scheduler_state"
        if not scheduler_state_allowed(
            self.last_scheduler_state, self.allowed_scheduler_states
        ):
            return f"scheduler_{self.last_scheduler_state.lower()}"
        return "ok"

    def _validate(self, msg):
        if not self.enabled:
            return "disabled"
        scheduler_reason = self._scheduler_reason()
        if scheduler_reason != "ok":
            return scheduler_reason
        if msg.header.frame_id != self.expected_map_frame:
            return "unexpected_map_frame"
        if msg.child_frame_id != self.expected_body_frame:
            return "unexpected_body_frame"
        age_s = self.get_clock().now().nanoseconds * 1.0e-9 - stamp_seconds(msg.header.stamp)
        if not math.isfinite(age_s) or age_s < -0.05 or age_s > self.maximum_input_age:
            return "stale_timestamp"
        pose = msg.pose.pose
        twist = msg.twist.twist
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return "nonfinite_state"
        quaternion_norm = math.sqrt(
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w)
        if quaternion_norm < 0.95 or quaternion_norm > 1.05:
            return "invalid_quaternion"
        pose_diagonal = [msg.pose.covariance[index] for index in (0, 7, 14, 21, 28, 35)]
        twist_diagonal = [msg.twist.covariance[index] for index in (0, 7, 14, 21, 28, 35)]
        if not all(math.isfinite(value) and value >= 0.0 for value in pose_diagonal + twist_diagonal):
            return "invalid_covariance"
        return "ok"

    def _odom(self, msg):
        started = time.perf_counter_ns()
        now = time.monotonic()
        self.last_arrival = now
        self.arrivals.append(now)
        reason = self._validate(msg)
        self.last_reason = reason
        if reason == "ok":
            output = copy.deepcopy(msg)
            quaternion = output.pose.pose.orientation
            norm = math.sqrt(
                quaternion.x * quaternion.x + quaternion.y * quaternion.y
                + quaternion.z * quaternion.z + quaternion.w * quaternion.w)
            quaternion.x /= norm
            quaternion.y /= norm
            quaternion.z /= norm
            quaternion.w /= norm
            self.publisher.publish(output)
            self.accepted += 1
        else:
            self.rejected += 1
        self.callback_ms.append((time.perf_counter_ns() - started) * 1.0e-6)

    def _rate(self):
        now = time.monotonic()
        recent = [value for value in self.arrivals if now - value <= 5.0]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / max(1.0e-6, recent[-1] - recent[0])

    @staticmethod
    def _value(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _diagnostics(self):
        rate = self._rate()
        age_s = math.inf if self.last_arrival is None else time.monotonic() - self.last_arrival
        scheduler_age_s = (
            math.inf
            if self.last_scheduler_arrival is None
            else time.monotonic() - self.last_scheduler_arrival
        )
        healthy = self.last_reason == "ok" and age_s <= self.maximum_input_age * 2.0
        if len(self.arrivals) >= 5:
            healthy = healthy and rate >= self.minimum_rate
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "external_nav/gate"
        status.hardware_id = "mavros_odometry_out"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "forwarding" if healthy else self.last_reason
        status.values = [
            self._value("accepted", self.accepted),
            self._value("rejected", self.rejected),
            self._value("input_rate_hz", f"{rate:.3f}"),
            self._value("input_age_s", f"{age_s:.3f}"),
            self._value("minimum_rate_hz", self.minimum_rate),
            self._value("require_scheduler_health", self.require_scheduler_health),
            self._value("scheduler_state", self.last_scheduler_state),
            self._value("scheduler_age_s", f"{scheduler_age_s:.3f}"),
            self._value("allowed_scheduler_states", ",".join(self.allowed_scheduler_states)),
            self._value(
                "timing_callback_mean_ms",
                f"{sum(self.callback_ms) / len(self.callback_ms):.4f}" if self.callback_ms else "0.0"),
            self._value(
                "timing_callback_max_ms",
                f"{max(self.callback_ms):.4f}" if self.callback_ms else "0.0"),
            self._value("mavros_quality_reset_supported", "false"),
        ]
        output.status.append(status)
        self.diagnostic_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ExternalNavGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
