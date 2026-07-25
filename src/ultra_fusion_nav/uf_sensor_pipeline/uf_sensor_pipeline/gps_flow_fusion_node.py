import copy
import math
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from uf_interfaces.msg import ReliabilityScore, SchedulerState

from .gps_flow_fusion import (
    GpsFlowComplementaryFilter,
    LocalEnuProjector,
    compensated_flow_velocity_frd,
    velocity_enu_to_flu,
    yaw_from_quaternion,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class TimingWindow:
    def __init__(self, size=500):
        self.values_ms = deque(maxlen=size)

    def add_ns(self, elapsed_ns):
        self.values_ms.append(float(elapsed_ns) * 1.0e-6)

    def mean(self):
        return sum(self.values_ms) / len(self.values_ms) if self.values_ms else 0.0

    def maximum(self):
        return max(self.values_ms) if self.values_ms else 0.0


class GpsFlowFusionNode(Node):
    """Fuse raw GNSS position and optical-flow velocity without FCU local position."""

    def __init__(self):
        super().__init__("gps_flow_fusion")
        defaults = {
            "gnss_topic": "/sensors/gnss/fix",
            "flow_topic": "/sensors/optical_flow/rad",
            "imu_topic": "/mavros/imu/data",
            "gnss_reliability_topic": "/reliability/gnss_score",
            "flow_reliability_topic": "/reliability/optical_flow_score",
            "scheduler_topic": "/reliability/scheduler_state",
            "output_topic": "/fusion/gps_flow/odom",
            "diagnostic_topic": "/fusion/gps_flow/diagnostics",
            "map_frame": "map",
            "body_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("gps_position_gain", 0.35)
        self.declare_parameter("flow_velocity_gain", 0.65)
        self.declare_parameter("gps_jump_gate_m", 20.0)
        self.declare_parameter("default_gnss_variance_m2", 4.0)
        self.declare_parameter("gnss_weight_scale_m", 5.0)
        self.declare_parameter("minimum_flow_quality", 20)
        self.declare_parameter("minimum_flow_distance_m", 0.08)
        self.declare_parameter("maximum_flow_distance_m", 12.0)
        self.declare_parameter("maximum_flow_speed_mps", 8.0)
        self.declare_parameter("gnss_timeout_s", 5.0)
        self.declare_parameter("flow_timeout_s", 1.0)
        self.declare_parameter("imu_timeout_s", 0.5)
        self.declare_parameter("reliability_timeout_s", 1.0)
        self.declare_parameter("scheduler_timeout_s", 1.0)
        self.declare_parameter("use_scheduler", True)
        self.declare_parameter("require_flow_observation", True)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.default_gnss_variance = float(
            self.get_parameter("default_gnss_variance_m2").value)
        self.gnss_weight_scale = float(self.get_parameter("gnss_weight_scale_m").value)
        self.minimum_flow_quality = int(self.get_parameter("minimum_flow_quality").value)
        self.minimum_flow_distance = float(
            self.get_parameter("minimum_flow_distance_m").value)
        self.maximum_flow_distance = float(
            self.get_parameter("maximum_flow_distance_m").value)
        self.maximum_flow_speed = float(self.get_parameter("maximum_flow_speed_mps").value)
        self.gnss_timeout = float(self.get_parameter("gnss_timeout_s").value)
        self.flow_timeout = float(self.get_parameter("flow_timeout_s").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout_s").value)
        self.reliability_timeout = float(
            self.get_parameter("reliability_timeout_s").value)
        self.scheduler_timeout = float(
            self.get_parameter("scheduler_timeout_s").value)
        self.use_scheduler = bool(self.get_parameter("use_scheduler").value)
        self.require_flow_observation = bool(
            self.get_parameter("require_flow_observation").value)

        self.filter = GpsFlowComplementaryFilter(
            gps_position_gain=self.get_parameter("gps_position_gain").value,
            flow_velocity_gain=self.get_parameter("flow_velocity_gain").value,
            gps_jump_gate_m=self.get_parameter("gps_jump_gate_m").value,
        )
        self.projector = None
        self.orientation = None
        self.angular_velocity = (0.0, 0.0, 0.0)
        self.yaw = None
        self.last_arrival = {}
        self.reliability = {}
        self.scheduler = {}
        self.scheduler_arrival = None
        self.scheduler_health = "UNAVAILABLE"
        self.counts = {
            "gnss": 0,
            "gnss_rejected": 0,
            "flow": 0,
            "flow_rejected": 0,
            "published": 0,
        }
        self.last_innovation_m = 0.0
        self.last_health = "INITIALIZING"
        self.timings = {
            "gnss": TimingWindow(),
            "flow": TimingWindow(),
            "publish": TimingWindow(),
        }

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostic_topic").value), 10)
        self.create_subscription(
            NavSatFix, str(self.get_parameter("gnss_topic").value),
            self._gnss, qos_profile_sensor_data)
        self.create_subscription(
            OpticalFlowRad, str(self.get_parameter("flow_topic").value),
            self._flow, qos_profile_sensor_data)
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value),
            self._imu, qos_profile_sensor_data)
        self.create_subscription(
            ReliabilityScore, str(self.get_parameter("gnss_reliability_topic").value),
            self._reliability, 20)
        self.create_subscription(
            ReliabilityScore, str(self.get_parameter("flow_reliability_topic").value),
            self._reliability, 20)
        self.create_subscription(
            SchedulerState, str(self.get_parameter("scheduler_topic").value),
            self._scheduler, 20)
        rate_hz = max(4.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._publish)
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            "GPS/flow fusion active: raw GNSS position + direct optical-flow velocity; "
            "FCU local position is not subscribed")

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _message_time(self, header):
        value = stamp_seconds(header.stamp)
        return value if value > 0.0 else self._now_s()

    def _touch(self, name):
        self.last_arrival[name] = time.monotonic()

    def _age(self, name):
        value = self.last_arrival.get(name)
        return math.inf if value is None else time.monotonic() - value

    def _reliability(self, msg):
        if msg.modality not in ("gnss", "optical_flow"):
            return
        self.reliability[msg.modality] = (
            float(msg.reliability_weight) if msg.valid else 0.0,
            time.monotonic(),
        )

    def _effective_weight(self, modality, native_weight):
        if (
            self.use_scheduler
            and self.scheduler_arrival is not None
            and time.monotonic() - self.scheduler_arrival <= self.scheduler_timeout
        ):
            decision = self.scheduler.get(modality)
            if decision is not None:
                if not decision[1]:
                    return 0.0
                return max(0.0, min(1.0, native_weight * decision[0]))
        score = self.reliability.get(modality)
        if score is None or time.monotonic() - score[1] > self.reliability_timeout:
            return max(0.0, min(1.0, native_weight))
        return max(0.0, min(1.0, native_weight * score[0]))

    def _scheduler(self, msg):
        lengths = (
            len(msg.modality_names),
            len(msg.reliability_weights),
            len(msg.factor_enabled),
            len(msg.covariance_inflation),
        )
        if min(lengths) != max(lengths):
            self.get_logger().warning("Rejected malformed scheduler state arrays")
            return
        self.scheduler = {
            name: (
                max(0.0, min(1.0, float(weight))),
                bool(enabled),
                max(1.0, float(inflation)),
            )
            for name, weight, enabled, inflation in zip(
                msg.modality_names,
                msg.reliability_weights,
                msg.factor_enabled,
                msg.covariance_inflation,
            )
        }
        self.scheduler_health = str(msg.health_state)
        self.scheduler_arrival = time.monotonic()

    def _scheduler_inflation(self, modality):
        if (
            not self.use_scheduler
            or self.scheduler_arrival is None
            or time.monotonic() - self.scheduler_arrival > self.scheduler_timeout
        ):
            return 1.0
        decision = self.scheduler.get(modality)
        return 1.0 if decision is None else decision[2]

    def _imu(self, msg):
        orientation = msg.orientation
        yaw = yaw_from_quaternion(
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w))
        if yaw is None:
            return
        self.orientation = copy.deepcopy(orientation)
        self.yaw = yaw
        self.angular_velocity = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        self._touch("imu")

    def _gnss_variance(self, msg):
        diagonal = [
            float(msg.position_covariance[0]),
            float(msg.position_covariance[4]),
        ]
        valid = [value for value in diagonal if math.isfinite(value) and value > 0.0]
        return sum(valid) / len(valid) if valid else self.default_gnss_variance

    def _gnss(self, msg):
        started = time.perf_counter_ns()
        try:
            if msg.status.status < NavSatStatus.STATUS_FIX:
                self.counts["gnss_rejected"] += 1
                return
            values = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
            if not all(math.isfinite(value) for value in values):
                self.counts["gnss_rejected"] += 1
                return
            if self.projector is None:
                self.projector = LocalEnuProjector(*values)
            position = self.projector.project(*values)
            variance = self._gnss_variance(msg)
            native_weight = 1.0 / (1.0 + math.sqrt(variance) / max(0.1, self.gnss_weight_scale))
            weight = self._effective_weight("gnss", native_weight)
            if weight <= 0.0:
                self.counts["gnss_rejected"] += 1
                return
            result = self.filter.update_gnss(
                position, variance, self._message_time(msg.header), weight)
            self.last_innovation_m = result.innovation_m
            if result.accepted:
                self.counts["gnss"] += 1
                self._touch("gnss")
            else:
                self.counts["gnss_rejected"] += 1
        finally:
            self.timings["gnss"].add_ns(time.perf_counter_ns() - started)

    def _flow(self, msg):
        started = time.perf_counter_ns()
        try:
            self._touch("flow_observation")
            integration_s = float(msg.integration_time_us) * 1.0e-6
            distance = float(msg.distance)
            velocity = compensated_flow_velocity_frd(
                msg.integrated_x, msg.integrated_y,
                msg.integrated_xgyro, msg.integrated_ygyro,
                integration_s, distance,
            )
            if velocity is None or self.yaw is None:
                self.counts["flow_rejected"] += 1
                return
            speed = math.hypot(*velocity)
            distance_valid = self.minimum_flow_distance <= distance <= self.maximum_flow_distance
            if not distance_valid or speed > self.maximum_flow_speed:
                self.counts["flow_rejected"] += 1
                return
            native_weight = max(0.0, min(1.0, float(msg.quality) / 255.0))
            if int(msg.quality) < self.minimum_flow_quality:
                native_weight = 0.0
            weight = self._effective_weight("optical_flow", native_weight)
            if weight <= 0.0:
                self.counts["flow_rejected"] += 1
                return
            result = self.filter.update_flow(
                velocity[0], velocity[1], self.yaw,
                self._message_time(msg.header), weight)
            if result.accepted:
                self.counts["flow"] += 1
                if weight > 0.0:
                    self._touch("flow_valid")
            else:
                self.counts["flow_rejected"] += 1
        finally:
            self.timings["flow"].add_ns(time.perf_counter_ns() - started)

    def _healthy(self):
        if not self.filter.initialized or self.orientation is None:
            return False, "INITIALIZING"
        if self._age("imu") > self.imu_timeout:
            return False, "IMU_STALE"
        if self._age("gnss") > self.gnss_timeout:
            return False, "GNSS_OUTAGE_LIMIT"
        if self.require_flow_observation and self._age("flow_observation") > self.flow_timeout:
            return False, "FLOW_STALE"
        if self._age("flow_valid") > self.flow_timeout:
            return True, "GPS_ONLY_DEGRADED"
        return True, "GPS_FLOW_FUSED"

    def _publish(self):
        started = time.perf_counter_ns()
        try:
            healthy, health = self._healthy()
            self.last_health = health
            if not healthy:
                return
            now = self.get_clock().now()
            self.filter.predict_to(now.nanoseconds * 1.0e-9)
            gnss_age = self._age("gnss")
            flow_age = self._age("flow_valid")
            inflation = 1.0 + min(20.0, gnss_age * 2.0)
            if not math.isfinite(flow_age) or flow_age > self.flow_timeout:
                inflation *= 4.0
            position_variance = (
                max(0.04, self.filter.last_gnss_variance_m2)
                * inflation
                * self._scheduler_inflation("gnss")
            )
            flow_weight = max(0.05, self.filter.last_flow_weight)
            velocity_variance = (
                (0.05 / flow_weight) ** 2
                * self._scheduler_inflation("optical_flow")
            )

            output = Odometry()
            output.header.stamp = now.to_msg()
            output.header.frame_id = self.map_frame
            output.child_frame_id = self.body_frame
            output.pose.pose.position.x = float(self.filter.position[0])
            output.pose.pose.position.y = float(self.filter.position[1])
            output.pose.pose.position.z = float(self.filter.position[2])
            output.pose.pose.orientation = copy.deepcopy(self.orientation)
            output.pose.covariance[0] = position_variance
            output.pose.covariance[7] = position_variance
            output.pose.covariance[14] = position_variance * 2.0
            output.pose.covariance[21] = 0.04
            output.pose.covariance[28] = 0.04
            output.pose.covariance[35] = 0.09

            forward, left = velocity_enu_to_flu(
                self.filter.velocity_enu[0], self.filter.velocity_enu[1], self.yaw)
            output.twist.twist.linear.x = float(forward)
            output.twist.twist.linear.y = float(left)
            output.twist.twist.linear.z = 0.0
            output.twist.twist.angular.x = self.angular_velocity[0]
            output.twist.twist.angular.y = self.angular_velocity[1]
            output.twist.twist.angular.z = self.angular_velocity[2]
            output.twist.covariance[0] = velocity_variance
            output.twist.covariance[7] = velocity_variance
            output.twist.covariance[14] = 4.0
            output.twist.covariance[21] = 0.01
            output.twist.covariance[28] = 0.01
            output.twist.covariance[35] = 0.01
            self.odom_pub.publish(output)
            self.counts["published"] += 1
        finally:
            self.timings["publish"].add_ns(time.perf_counter_ns() - started)

    @staticmethod
    def _value(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _diagnostics(self):
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "fusion/gps_flow"
        status.hardware_id = "companion_computer"
        healthy, health = self._healthy()
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        if health == "GPS_ONLY_DEGRADED":
            status.level = DiagnosticStatus.WARN
        status.message = health
        status.values = [
            self._value("gnss_samples", self.counts["gnss"]),
            self._value("gnss_rejected", self.counts["gnss_rejected"]),
            self._value("flow_samples", self.counts["flow"]),
            self._value("flow_rejected", self.counts["flow_rejected"]),
            self._value("published", self.counts["published"]),
            self._value("gnss_age_s", f"{self._age('gnss'):.3f}"),
            self._value("flow_age_s", f"{self._age('flow_valid'):.3f}"),
            self._value("scheduler_health", self.scheduler_health),
            self._value(
                "scheduler_age_s",
                "inf" if self.scheduler_arrival is None else
                f"{time.monotonic() - self.scheduler_arrival:.3f}"),
            self._value("scheduler_used", self.use_scheduler),
            self._value("last_gnss_innovation_m", f"{self.last_innovation_m:.3f}"),
            self._value("timing_gnss_mean_ms", f"{self.timings['gnss'].mean():.4f}"),
            self._value("timing_gnss_max_ms", f"{self.timings['gnss'].maximum():.4f}"),
            self._value("timing_flow_mean_ms", f"{self.timings['flow'].mean():.4f}"),
            self._value("timing_flow_max_ms", f"{self.timings['flow'].maximum():.4f}"),
            self._value("timing_publish_mean_ms", f"{self.timings['publish'].mean():.4f}"),
            self._value("timing_publish_max_ms", f"{self.timings['publish'].maximum():.4f}"),
            self._value("uses_fcu_local_position", "false"),
        ]
        output.status.append(status)
        self.diagnostic_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GpsFlowFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
