import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from uf_interfaces.msg import ReliabilityScore, RelocalizationResult, SchedulerState

from .scheduler_core import (
    CAPABILITIES,
    MODALITIES,
    ReliabilitySchedulerCore,
    SchedulerConfig,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class ReliabilityScheduler(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "reliability_scheduler",
            parameter_overrides=parameter_overrides or [],
        )
        self.declare_parameter("active_modalities", list(MODALITIES))
        self.declare_parameter("required_modalities", [""])
        self.declare_parameter("minimum_usable_modalities", 1)
        self.declare_parameter("score_timeout_s", 1.0)
        self.declare_parameter("degraded_threshold", 0.35)
        self.declare_parameter("risk_threshold", 0.60)
        self.declare_parameter("failsafe_threshold", 0.85)
        self.declare_parameter("factor_disable_threshold", 0.80)
        self.declare_parameter("factor_enable_threshold", 0.55)
        self.declare_parameter("minimum_weight", 0.05)
        self.declare_parameter("maximum_covariance_inflation", 20.0)
        self.declare_parameter("imu_soft_max_degradation", 0.80)
        self.declare_parameter("transition_dwell_s", 0.5)
        self.declare_parameter("recovery_dwell_s", 1.5)
        self.declare_parameter("recovered_hold_s", 1.0)
        self.declare_parameter("capability_observable_threshold", 0.15)
        self.declare_parameter("publish_rate_hz", 10.0)
        active = tuple(self.get_parameter("active_modalities").value)
        required = tuple(
            name
            for name in self.get_parameter("required_modalities").value
            if name
        )
        self.core = ReliabilitySchedulerCore(SchedulerConfig(
            active_modalities=active,
            required_modalities=required,
            minimum_usable_modalities=max(
                1, int(self.get_parameter("minimum_usable_modalities").value)
            ),
            stale_after_s=float(self.get_parameter("score_timeout_s").value),
            degraded_threshold=float(self.get_parameter("degraded_threshold").value),
            risk_threshold=float(self.get_parameter("risk_threshold").value),
            failsafe_threshold=float(self.get_parameter("failsafe_threshold").value),
            factor_disable_threshold=float(self.get_parameter("factor_disable_threshold").value),
            factor_enable_threshold=float(self.get_parameter("factor_enable_threshold").value),
            minimum_weight=float(self.get_parameter("minimum_weight").value),
            maximum_covariance_inflation=float(self.get_parameter("maximum_covariance_inflation").value),
            imu_soft_max_degradation=float(
                self.get_parameter("imu_soft_max_degradation").value
            ),
            transition_dwell_s=float(self.get_parameter("transition_dwell_s").value),
            recovery_dwell_s=float(self.get_parameter("recovery_dwell_s").value),
            recovered_hold_s=float(self.get_parameter("recovered_hold_s").value),
            capability_observable_threshold=float(
                self.get_parameter("capability_observable_threshold").value
            ),
        ))
        self.scores = {}
        self._last_clock_s = None
        self.relocalization_requested = False
        self.relocalization_failed = False
        self.state_pub = self.create_publisher(
            SchedulerState, "/reliability/scheduler_state", 20)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/reliability/scheduler_diagnostics", 10)
        for modality in MODALITIES:
            self.create_subscription(
                ReliabilityScore,
                f"/reliability/{modality}_score",
                lambda msg, name=modality: self._score(name, msg),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Bool, "/relocalization/request", self._relocalization, 10)
        self.create_subscription(
            RelocalizationResult, "/relocalization/result",
            self._relocalization_result, 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "ReliabilityScheduler active: "
            f"active_modalities={','.join(self.core.active_modalities)} "
            f"required_modalities={','.join(self.core.required_modalities)} "
            f"minimum_usable_modalities="
            f"{self.core.config.minimum_usable_modalities}")

    def _score(self, modality, msg):
        now_s = self._now_s()
        self._observe_ros_clock(now_s)
        source_s = stamp_seconds(msg.header.stamp)
        if source_s <= 0.0:
            return
        previous = self.scores.get(modality)
        if previous is not None and source_s <= previous["arrival_s"]:
            return
        evidence = {
            name: float(value)
            for name, value in zip(msg.evidence_names, msg.evidence_values)
        }
        self.scores[modality] = {
            "degradation_score": float(msg.degradation_score),
            "valid": bool(msg.valid),
            "hard_gate_allowed": bool(
                evidence.get("hard_gate_allowed", 1.0) >= 0.5
            ),
            "observation_count": int(msg.observation_count),
            "minimum_observation_count": int(msg.minimum_observation_count),
            "reasons": tuple(msg.reasons),
            "arrival_s": source_s,
        }

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _observe_ros_clock(self, now_s):
        if self._last_clock_s is not None and now_s < self._last_clock_s:
            self.scores.clear()
        self._last_clock_s = now_s

    def _relocalization(self, msg):
        self.relocalization_requested = bool(msg.data)
        if msg.data:
            self.relocalization_failed = False

    def _relocalization_result(self, msg):
        if int(msg.state) == int(RelocalizationResult.SUCCESS) and msg.accepted:
            self.relocalization_requested = False
            self.relocalization_failed = False
        elif int(msg.state) == int(RelocalizationResult.FAILED):
            self.relocalization_requested = False
            self.relocalization_failed = True

    @staticmethod
    def _key(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _publish(self):
        now_s = self._now_s()
        self._observe_ros_clock(now_s)
        result = self.core.update(
            self.scores, now_s, self.relocalization_requested,
            self.relocalization_failed)
        stamp = self.get_clock().now().to_msg()
        msg = SchedulerState()
        msg.header.stamp = stamp
        msg.header.frame_id = "reliability_scheduler"
        msg.health_state = result.health_state
        for name in MODALITIES:
            msg.modality_names.append(name)
            msg.degradation_scores.append(float(result.degradation_scores[name]))
            msg.reliability_weights.append(float(result.reliability_weights[name]))
            msg.covariance_inflation.append(float(result.covariance_inflation[name]))
            msg.factor_enabled.append(bool(result.factor_enabled[name]))
            msg.reasons.append(",".join(result.reasons[name]))
        for name in CAPABILITIES:
            msg.capability_names.append(name)
            msg.capability_support.append(float(result.capability_support[name]))
            msg.capability_observable.append(
                bool(result.capability_observable[name])
            )
        msg.estimator_support = float(result.estimator_support)
        msg.relocalization_requested = result.relocalization_requested
        self.state_pub.publish(msg)

        diagnostic = DiagnosticStatus()
        diagnostic.name = "reliability/scheduler"
        diagnostic.hardware_id = "companion_computer"
        diagnostic.level = (
            DiagnosticStatus.OK
            if result.health_state in ("NORMAL", "RECOVERED")
            else DiagnosticStatus.WARN
        )
        if result.health_state in ("RISK", "RELOCALIZING", "FAILSAFE"):
            diagnostic.level = DiagnosticStatus.ERROR
        diagnostic.message = result.health_state
        diagnostic.values = [self._key("health_state", result.health_state)]
        diagnostic.values.extend(
            self._key(f"{name}_weight", f"{result.reliability_weights[name]:.3f}")
            for name in MODALITIES
        )
        diagnostic.values.extend(
            self._key(f"{name}_enabled", result.factor_enabled[name])
            for name in MODALITIES
        )
        diagnostic.values.extend(
            self._key(f"capability_{name}", f"{result.capability_support[name]:.3f}")
            for name in CAPABILITIES
        )
        diagnostic.values.append(
            self._key("estimator_support", f"{result.estimator_support:.3f}")
        )
        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status.append(diagnostic)
        self.diagnostic_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ReliabilityScheduler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
