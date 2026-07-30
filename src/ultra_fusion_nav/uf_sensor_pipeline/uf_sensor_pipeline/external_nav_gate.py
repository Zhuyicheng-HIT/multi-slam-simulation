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
    normalized = str(health_state).strip().upper()
    return normalized in {
        str(state).strip().upper()
        for state in allowed_states
        if str(state).strip()
    }


def capability_support_allowed(support, required, minimum_support):
    return all(float(support.get(name, 0.0)) >= minimum_support for name in required)


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("zero quaternion")
    return tuple(value / norm for value in quaternion)


def _rotate_vector(quaternion, vector):
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # Equivalent to q * [v, 0] * conjugate(q), with fewer operations.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def propagate_odometry(
    message,
    target_stamp_s,
    position_process_noise_mps=0.50,
    angular_process_noise_radps=0.20,
    covariance_scale=1.0,
):
    """Propagate body-frame twist to a nearby publication timestamp."""
    output = copy.deepcopy(message)
    source_stamp_s = stamp_seconds(message.header.stamp)
    dt = max(0.0, float(target_stamp_s) - source_stamp_s)
    orientation = _normalize_quaternion((
        message.pose.pose.orientation.x,
        message.pose.pose.orientation.y,
        message.pose.pose.orientation.z,
        message.pose.pose.orientation.w,
    ))
    velocity_world = _rotate_vector(orientation, (
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
        message.twist.twist.linear.z,
    ))
    output.pose.pose.position.x += velocity_world[0] * dt
    output.pose.pose.position.y += velocity_world[1] * dt
    output.pose.pose.position.z += velocity_world[2] * dt

    angular = (
        message.twist.twist.angular.x,
        message.twist.twist.angular.y,
        message.twist.twist.angular.z,
    )
    angular_speed = math.sqrt(sum(value * value for value in angular))
    if angular_speed > 1.0e-12 and dt > 0.0:
        half_angle = 0.5 * angular_speed * dt
        scale = math.sin(half_angle) / angular_speed
        delta = (
            angular[0] * scale,
            angular[1] * scale,
            angular[2] * scale,
            math.cos(half_angle),
        )
        orientation = _normalize_quaternion(
            _quaternion_multiply(orientation, delta)
        )
    quaternion = output.pose.pose.orientation
    quaternion.x, quaternion.y, quaternion.z, quaternion.w = orientation

    covariance_scale = max(1.0, float(covariance_scale))
    for index in (0, 7, 14, 21, 28, 35):
        output.pose.covariance[index] *= covariance_scale
        output.twist.covariance[index] *= covariance_scale
    position_variance = (float(position_process_noise_mps) * dt) ** 2
    angular_variance = (float(angular_process_noise_radps) * dt) ** 2
    for index in (0, 7, 14):
        output.pose.covariance[index] += position_variance
    for index in (21, 28, 35):
        output.pose.covariance[index] += angular_variance

    seconds = math.floor(target_stamp_s)
    nanoseconds = int(round((target_stamp_s - seconds) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    output.header.stamp.sec = int(seconds)
    output.header.stamp.nanosec = nanoseconds
    return output


class ExternalNavGate(Node):
    """Publish bounded, capability-gated fusion odometry at a stable FCU rate."""

    def __init__(self):
        super().__init__("external_nav_gate")
        self.declare_parameter("input_topic", "/fusion/gps_flow/odom")
        self.declare_parameter("output_topic", "/mavros/odometry/out")
        self.declare_parameter("expected_map_frame", "map")
        self.declare_parameter("expected_body_frame", "base_link")
        self.declare_parameter("maximum_input_age_s", 0.25)
        self.declare_parameter("maximum_propagation_age_s", 0.35)
        self.declare_parameter("minimum_rate_hz", 4.0)
        self.declare_parameter("output_rate_hz", 20.0)
        self.declare_parameter("position_process_noise_mps", 0.50)
        self.declare_parameter("angular_process_noise_radps", 0.20)
        self.declare_parameter("maximum_covariance_scale", 5.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("require_scheduler_health", False)
        self.declare_parameter("scheduler_topic", "/reliability/scheduler_state")
        self.declare_parameter("scheduler_timeout_s", 0.5)
        self.declare_parameter("allowed_scheduler_states", ["NORMAL", "RECOVERED"])
        self.declare_parameter("require_capability_support", False)
        self.declare_parameter("required_capabilities", [""])
        self.declare_parameter("minimum_capability_support", 0.15)

        self.expected_map_frame = str(self.get_parameter("expected_map_frame").value)
        self.expected_body_frame = str(self.get_parameter("expected_body_frame").value)
        self.maximum_input_age = float(self.get_parameter("maximum_input_age_s").value)
        self.maximum_propagation_age = float(
            self.get_parameter("maximum_propagation_age_s").value
        )
        self.minimum_rate = float(self.get_parameter("minimum_rate_hz").value)
        self.output_rate = max(1.0, float(self.get_parameter("output_rate_hz").value))
        self.position_process_noise = float(
            self.get_parameter("position_process_noise_mps").value
        )
        self.angular_process_noise = float(
            self.get_parameter("angular_process_noise_radps").value
        )
        self.maximum_covariance_scale = max(
            1.0, float(self.get_parameter("maximum_covariance_scale").value)
        )
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
        self.require_capability_support = bool(
            self.get_parameter("require_capability_support").value
        )
        self.required_capabilities = tuple(
            str(name).strip()
            for name in self.get_parameter("required_capabilities").value
            if str(name).strip()
        )
        self.minimum_capability_support = float(
            self.get_parameter("minimum_capability_support").value
        )

        self.arrivals = deque(maxlen=500)
        self.publications = deque(maxlen=1000)
        self.callback_ms = deque(maxlen=500)
        self.accepted_inputs = 0
        self.rejected_inputs = 0
        self.published = 0
        self.latest_source = None
        self.last_arrival = None
        self.last_publish_arrival = None
        self.last_reason = "waiting_for_fusion"
        self.last_scheduler_arrival = None
        self.last_scheduler_state = "WAITING"
        self.last_estimator_support = 0.0
        self.capability_support = {}

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/external_nav/diagnostics", 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("input_topic").value), self._odom, 20
        )
        if self.require_scheduler_health or self.require_capability_support:
            self.create_subscription(
                SchedulerState,
                str(self.get_parameter("scheduler_topic").value),
                self._scheduler_state,
                20,
            )
        self.create_timer(1.0 / self.output_rate, self._publish_latest)
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            f"ExternalNav gate {'enabled' if self.enabled else 'disabled'}: "
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} at {self.output_rate:.1f} Hz"
        )

    def _scheduler_state(self, msg):
        self.last_scheduler_arrival = time.monotonic()
        self.last_scheduler_state = str(msg.health_state).strip().upper() or "UNKNOWN"
        self.capability_support = {
            name: float(value)
            for name, value in zip(msg.capability_names, msg.capability_support)
        }
        self.last_estimator_support = float(msg.estimator_support)

    def _scheduler_reason(self):
        if not (self.require_scheduler_health or self.require_capability_support):
            return "ok"
        if self.last_scheduler_arrival is None:
            return "missing_scheduler_state"
        if time.monotonic() - self.last_scheduler_arrival > self.scheduler_timeout_s:
            return "stale_scheduler_state"
        if self.require_scheduler_health and not scheduler_state_allowed(
            self.last_scheduler_state, self.allowed_scheduler_states
        ):
            return f"scheduler_{self.last_scheduler_state.lower()}"
        if self.require_capability_support and not capability_support_allowed(
            self.capability_support,
            self.required_capabilities,
            self.minimum_capability_support,
        ):
            return "insufficient_capability_support"
        return "ok"

    def _validate_input(self, msg):
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
            + pose.orientation.w * pose.orientation.w
        )
        if quaternion_norm < 0.95 or quaternion_norm > 1.05:
            return "invalid_quaternion"
        pose_diagonal = [msg.pose.covariance[index] for index in (0, 7, 14, 21, 28, 35)]
        twist_diagonal = [msg.twist.covariance[index] for index in (0, 7, 14, 21, 28, 35)]
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in pose_diagonal + twist_diagonal
        ):
            return "invalid_covariance"
        return "ok"

    def _odom(self, msg):
        started = time.perf_counter_ns()
        now = time.monotonic()
        self.last_arrival = now
        self.arrivals.append(now)
        reason = self._validate_input(msg)
        if reason == "ok":
            self.latest_source = copy.deepcopy(msg)
            quaternion = self.latest_source.pose.pose.orientation
            normalized = _normalize_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            )
            quaternion.x, quaternion.y, quaternion.z, quaternion.w = normalized
            self.accepted_inputs += 1
        else:
            self.rejected_inputs += 1
            self.last_reason = reason
        self.callback_ms.append((time.perf_counter_ns() - started) * 1.0e-6)

    def _publish_latest(self):
        if not self.enabled:
            self.last_reason = "disabled"
            return
        scheduler_reason = self._scheduler_reason()
        if scheduler_reason != "ok":
            self.last_reason = scheduler_reason
            return
        if self.latest_source is None:
            self.last_reason = "waiting_for_fusion"
            return
        now_ros_s = self.get_clock().now().nanoseconds * 1.0e-9
        source_age_s = now_ros_s - stamp_seconds(self.latest_source.header.stamp)
        arrival_age_s = time.monotonic() - self.last_arrival
        if (
            not math.isfinite(source_age_s)
            or source_age_s < -0.05
            or source_age_s > self.maximum_propagation_age
            or arrival_age_s > self.maximum_propagation_age
        ):
            self.last_reason = "fusion_source_stale"
            return
        covariance_scale = min(
            self.maximum_covariance_scale,
            1.0 / max(self.minimum_capability_support, self.last_estimator_support),
        ) if self.require_capability_support else 1.0
        output = propagate_odometry(
            self.latest_source,
            now_ros_s,
            self.position_process_noise,
            self.angular_process_noise,
            covariance_scale,
        )
        self.publisher.publish(output)
        published_at = time.monotonic()
        self.publications.append(published_at)
        self.last_publish_arrival = published_at
        self.published += 1
        self.last_reason = "ok"

    @staticmethod
    def _rate(values):
        now = time.monotonic()
        recent = [value for value in values if now - value <= 5.0]
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
        input_rate = self._rate(self.arrivals)
        output_rate = self._rate(self.publications)
        input_age_s = math.inf if self.last_arrival is None else time.monotonic() - self.last_arrival
        output_age_s = (
            math.inf
            if self.last_publish_arrival is None
            else time.monotonic() - self.last_publish_arrival
        )
        scheduler_age_s = (
            math.inf
            if self.last_scheduler_arrival is None
            else time.monotonic() - self.last_scheduler_arrival
        )
        healthy = self.last_reason == "ok" and output_age_s <= 2.0 / self.output_rate
        if len(self.arrivals) >= 5:
            healthy = healthy and input_rate >= self.minimum_rate
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "external_nav/gate"
        status.hardware_id = "mavros_odometry_out"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "publishing" if healthy else self.last_reason
        status.values = [
            self._value("accepted_inputs", self.accepted_inputs),
            self._value("rejected_inputs", self.rejected_inputs),
            self._value("published", self.published),
            self._value("input_rate_hz", f"{input_rate:.3f}"),
            self._value("output_rate_hz", f"{output_rate:.3f}"),
            self._value("input_age_s", f"{input_age_s:.3f}"),
            self._value("output_age_s", f"{output_age_s:.3f}"),
            self._value("scheduler_state", self.last_scheduler_state),
            self._value("scheduler_age_s", f"{scheduler_age_s:.3f}"),
            self._value("estimator_support", f"{self.last_estimator_support:.3f}"),
            self._value("required_capabilities", ",".join(self.required_capabilities)),
            self._value(
                "capability_support",
                ",".join(
                    f"{name}:{value:.3f}"
                    for name, value in sorted(self.capability_support.items())
                ),
            ),
            self._value(
                "timing_callback_mean_ms",
                f"{sum(self.callback_ms) / len(self.callback_ms):.4f}"
                if self.callback_ms else "0.0",
            ),
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
