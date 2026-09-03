"""Bounded, operator-facing relocalization intent producer.

This node never publishes the aggregate request or vehicle commands.  The
request arbiter remains the sole owner of /relocalization/request.
"""

import math
import uuid

import rclpy
from rclpy.node import Node
from uf_interfaces.msg import RelocalizationRequestIntent
from uf_interfaces.srv import ManualRelocalization


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class ManualRelocalizationControl(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("manual_relocalization_control", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("source_id", "manual_control")
        self.declare_parameter("intent_topic", "/relocalization/request_intent")
        self.declare_parameter("default_lease_s", 1.0)
        self.declare_parameter("minimum_lease_s", 0.20)
        self.declare_parameter("maximum_lease_s", 5.0)
        self.declare_parameter("keepalive_period_s", 0.25)
        self.source_id = str(self.get_parameter("source_id").value)
        self.intent_topic = str(self.get_parameter("intent_topic").value)
        self.minimum_lease_s = float(self.get_parameter("minimum_lease_s").value)
        self.maximum_lease_s = float(self.get_parameter("maximum_lease_s").value)
        self.default_lease_s = float(self.get_parameter("default_lease_s").value)
        period = float(self.get_parameter("keepalive_period_s").value)
        if not self.source_id or self.source_id != "manual_control":
            raise ValueError("source_id must be the registered manual_control owner")
        if not (0.0 < self.minimum_lease_s <= self.default_lease_s <= self.maximum_lease_s):
            raise ValueError("invalid manual relocalization lease configuration")
        if not (0.0 < period < self.default_lease_s):
            raise ValueError("keepalive_period_s must be shorter than default lease")
        self._instance = f"manual-{uuid.uuid4().hex}"
        self._sequence = 0
        self._episode = 0
        self._lease_s = self.default_lease_s
        self._active = False
        self._last_stamp_s = -1.0
        self._pub = self.create_publisher(RelocalizationRequestIntent, self.intent_topic, 10)
        self._service = self.create_service(ManualRelocalization, "/relocalization/manual_control", self._handle)
        self._timer = self.create_timer(period, self._keepalive)

    def _now(self):
        return self.get_clock().now()

    def _publish(self, active, episode, lease_s, reason, stamp):
        stamp_s = stamp.nanoseconds * 1.0e-9
        if stamp_s < self._last_stamp_s:
            return False, "timestamp_regression"
        self._sequence += 1
        msg = RelocalizationRequestIntent()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "manual_control"
        msg.source_id = self.source_id
        msg.source_instance_id = self._instance
        msg.sequence = self._sequence
        msg.episode_id = int(episode)
        msg.active = bool(active)
        msg.lease_duration_s = float(lease_s if active else 0.0)
        msg.reason = str(reason)
        self._pub.publish(msg)
        self._last_stamp_s = stamp_s
        return True, "accepted"

    def _handle(self, request, response):
        now = self._now()
        now_s = now.nanoseconds * 1.0e-9
        source = str(request.source).strip()
        supplied_s = _stamp_seconds(request.timestamp)
        stamp = now if supplied_s == 0.0 else rclpy.time.Time.from_msg(request.timestamp)
        stamp_s = stamp.nanoseconds * 1.0e-9
        if source != self.source_id:
            response.reason = "source_mismatch"
            return response
        if request.command not in (
            ManualRelocalization.Request.START,
            ManualRelocalization.Request.CANCEL,
        ):
            response.reason = "invalid_command"
            return response
        if not math.isfinite(stamp_s) or stamp_s < 0.0 or stamp_s > now_s + 0.5 or now_s - stamp_s > 2.0:
            response.reason = "stale_or_future_timestamp"
            return response
        if request.command == ManualRelocalization.Request.START:
            lease_s = float(request.lease_duration_s or self.default_lease_s)
            if not math.isfinite(lease_s) or not self.minimum_lease_s <= lease_s <= self.maximum_lease_s:
                response.reason = "invalid_lease"
                return response
            if self._active:
                response.reason = "already_active"
                response.request_sequence = self._sequence
                return response
            self._episode = int(request.episode_id)
            self._lease_s = lease_s
            self._active = True
            ok, reason = self._publish(True, self._episode, lease_s, "manual_start", stamp)
        else:
            if not self._active:
                response.reason = "already_cancelled"
                response.request_sequence = self._sequence
                return response
            self._active = False
            ok, reason = self._publish(False, self._episode, 0.0, "manual_cancel", stamp)
        response.accepted = ok
        response.reason = reason
        response.request_sequence = self._sequence
        return response

    def _keepalive(self):
        if not self._active:
            return
        stamp = self._now()
        ok, reason = self._publish(True, self._episode, self._lease_s, "manual_keepalive", stamp)
        if not ok:
            self._active = False
            self.get_logger().error("manual relocalization closed: %s", reason)


def main(args=None):
    rclpy.init(args=args)
    node = ManualRelocalizationControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
