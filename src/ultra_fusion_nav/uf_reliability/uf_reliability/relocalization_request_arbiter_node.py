"""Unique production publisher for the aggregate relocalization request."""

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool
from uf_interfaces.msg import RelocalizationRequestIntent

from .relocalization_request_arbiter import RelocalizationRequestArbiterCore


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class RelocalizationRequestArbiter(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "relocalization_request_arbiter",
            parameter_overrides=parameter_overrides or [],
        )
        self.declare_parameter(
            "allowed_sources", [
                "reliability_scheduler", "localization_safety", "manual_control"
            ]
        )
        self.declare_parameter(
            "intent_topic", "/relocalization/request_intent"
        )
        self.declare_parameter("request_topic", "/relocalization/request")
        self.declare_parameter(
            "diagnostics_topic", "/relocalization/request_arbiter_diagnostics"
        )
        self.declare_parameter("minimum_lease_s", 0.20)
        self.declare_parameter("maximum_lease_s", 5.0)
        self.declare_parameter("maximum_stamp_age_s", 2.0)
        self.declare_parameter("maximum_future_skew_s", 0.50)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.core = RelocalizationRequestArbiterCore(
            allowed_sources=tuple(self.get_parameter("allowed_sources").value),
            minimum_lease_s=float(self.get_parameter("minimum_lease_s").value),
            maximum_lease_s=float(self.get_parameter("maximum_lease_s").value),
            maximum_stamp_age_s=float(
                self.get_parameter("maximum_stamp_age_s").value
            ),
            maximum_future_skew_s=float(
                self.get_parameter("maximum_future_skew_s").value
            ),
        )
        final_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.request_pub = self.create_publisher(
            Bool, str(self.get_parameter("request_topic").value), final_qos
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.create_subscription(
            RelocalizationRequestIntent,
            str(self.get_parameter("intent_topic").value),
            self._intent,
            50,
        )
        self._last_ros_s = None
        self._publish_request(False)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

    def _ros_now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _observe_clock(self, now_s):
        if self._last_ros_s is not None and now_s < self._last_ros_s:
            decision = self.core.reset("ros_clock_regression")
            if decision.output_changed:
                self._publish_request(decision.output_active)
        self._last_ros_s = now_s

    def _intent(self, msg):
        ros_now_s = self._ros_now_s()
        self._observe_clock(ros_now_s)
        decision = self.core.update(
            source_id=msg.source_id,
            instance_id=msg.source_instance_id,
            sequence=msg.sequence,
            episode_id=msg.episode_id,
            active=msg.active,
            lease_duration_s=msg.lease_duration_s,
            source_stamp_s=stamp_seconds(msg.header.stamp),
            steady_now_s=time.monotonic(),
            ros_now_s=ros_now_s,
            reason=msg.reason,
        )
        if decision.output_changed:
            self._publish_request(decision.output_active)
        if not decision.accepted:
            self.get_logger().warning(
                f"Rejected relocalization intent from {msg.source_id!r}: "
                f"{decision.reason}"
            )
        self._publish_diagnostics(decision)

    def _tick(self):
        now_s = self._ros_now_s()
        self._observe_clock(now_s)
        decision = self.core.tick(time.monotonic())
        if decision.output_changed:
            self._publish_request(decision.output_active)
        if decision.expired_sources:
            self.get_logger().error(
                "Expired relocalization request lease(s): "
                + ",".join(decision.expired_sources)
            )
        self._publish_diagnostics(decision)

    def _publish_request(self, active):
        message = Bool()
        message.data = bool(active)
        self.request_pub.publish(message)

    @staticmethod
    def _key(key, value):
        item = KeyValue()
        item.key = str(key)
        item.value = str(value)
        return item

    def _publish_diagnostics(self, decision):
        status = DiagnosticStatus()
        status.name = "relocalization/request_arbiter"
        status.hardware_id = "companion_computer"
        status.level = (
            DiagnosticStatus.WARN
            if decision.expired_sources or not decision.accepted
            else DiagnosticStatus.OK
        )
        status.message = (
            "REQUESTED" if decision.output_active else "IDLE"
        )
        status.values = [
            self._key("active_sources", ",".join(decision.active_sources)),
            self._key("last_decision", decision.reason),
            self._key("accepted_intents", self.core.accepted_intents),
            self._key("rejected_intents", self.core.rejected_intents),
            self._key("duplicate_intents", self.core.duplicate_intents),
            self._key("expired_leases", self.core.expired_leases),
            self._key("source_restarts", self.core.source_restarts),
            self._key("output_transitions", self.core.output_transitions),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationRequestArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
