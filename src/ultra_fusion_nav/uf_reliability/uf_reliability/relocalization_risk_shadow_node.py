"""ROS diagnostics wrapper for the shadow-only relocalization risk core."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from uf_interfaces.msg import (
    FusionEpoch,
    ObstacleSafetyState,
    RelocalizationResult,
    SchedulerState,
)

from .relocalization_risk_shadow import (
    RelocalizationRiskConfig,
    RelocalizationRiskCore,
    RelocalizationRiskSample,
    RiskLevel,
)


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class RelocalizationRiskShadowNode(Node):
    """Observe topics and publish diagnostics without controlling intent."""

    def __init__(self):
        super().__init__("relocalization_risk_shadow")
        self.declare_parameter("watch_threshold", 0.25)
        self.declare_parameter("degraded_threshold", 0.48)
        self.declare_parameter("relocalize_threshold", 0.72)
        self.declare_parameter("watch_dwell_s", 0.30)
        self.declare_parameter("degraded_dwell_s", 1.00)
        self.declare_parameter("relocalize_dwell_s", 1.50)
        self.declare_parameter("recovery_dwell_s", 2.00)
        self.declare_parameter("request_cooldown_s", 15.0)
        self.declare_parameter("directional_shadow_enabled", False)
        self.declare_parameter("maximum_pose_step_m", 0.35)
        self.declare_parameter("covariance_growth_reference_m2ps", 0.05)
        self.core = RelocalizationRiskCore(RelocalizationRiskConfig(
            watch_threshold=float(self.get_parameter("watch_threshold").value),
            degraded_threshold=float(
                self.get_parameter("degraded_threshold").value
            ),
            relocalize_threshold=float(
                self.get_parameter("relocalize_threshold").value
            ),
            watch_dwell_s=float(self.get_parameter("watch_dwell_s").value),
            degraded_dwell_s=float(
                self.get_parameter("degraded_dwell_s").value
            ),
            relocalize_dwell_s=float(
                self.get_parameter("relocalize_dwell_s").value
            ),
            recovery_dwell_s=float(
                self.get_parameter("recovery_dwell_s").value
            ),
            request_cooldown_s=float(
                self.get_parameter("request_cooldown_s").value
            ),
        ))
        self.maximum_pose_step_m = max(
            1.0e-3, float(self.get_parameter("maximum_pose_step_m").value)
        )
        self.covariance_growth_reference = max(
            1.0e-6,
            float(
                self.get_parameter(
                    "covariance_growth_reference_m2ps"
                ).value
            ),
        )
        self.directional_enabled = bool(
            self.get_parameter("directional_shadow_enabled").value
        )
        self.latest_odom = None
        self.latest_covariance = None
        self.pose_jump_risk = 0.0
        self.covariance_growth_risk = 0.0
        self.request_active = False
        self.latest_result = "NONE"
        self.result_transaction = 0
        self.result_candidate = 0
        self.matching_epoch_applied = False
        self.obstacle_state = "CLEAR"
        self.directional_weakness = 0.0

        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/relocalization/shadow_risk", 10
        )
        self.create_subscription(
            SchedulerState,
            "/reliability/scheduler_state",
            self._scheduler,
            20,
        )
        self.create_subscription(
            Odometry, "/fusion/unified/odom", self._odom, 20
        )
        self.create_subscription(
            Bool, "/relocalization/request", self._request, 10
        )
        self.create_subscription(
            RelocalizationResult,
            "/relocalization/result",
            self._result,
            10,
        )
        self.create_subscription(
            FusionEpoch,
            "/fusion/unified/epoch",
            self._epoch,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            ObstacleSafetyState,
            "/safety/raw_obstacle_state",
            self._obstacle,
            10,
        )
        if self.directional_enabled:
            self.create_subscription(
                Float32,
                "/reliability/lidar_directional_weakness_shadow",
                self._directional,
                10,
            )

    def _odom(self, message):
        stamp_s = _stamp_seconds(message.header.stamp)
        position = message.pose.pose.position
        covariance = sum(
            max(0.0, float(message.pose.covariance[index]))
            for index in (0, 7, 14)
        )
        if self.latest_odom is not None:
            previous_stamp, previous_position = self.latest_odom
            dt_s = stamp_s - previous_stamp
            if dt_s > 0.0:
                step = math.sqrt(
                    (float(position.x) - previous_position[0]) ** 2
                    + (float(position.y) - previous_position[1]) ** 2
                    + (float(position.z) - previous_position[2]) ** 2
                )
                self.pose_jump_risk = min(1.0, step / self.maximum_pose_step_m)
                if self.latest_covariance is not None:
                    growth = max(
                        0.0, covariance - self.latest_covariance
                    ) / dt_s
                    self.covariance_growth_risk = min(
                        1.0, growth / self.covariance_growth_reference
                    )
        self.latest_odom = (
            stamp_s,
            (float(position.x), float(position.y), float(position.z)),
        )
        self.latest_covariance = covariance

    def _request(self, message):
        self.request_active = bool(message.data)

    def _result(self, message):
        if int(message.state) == int(RelocalizationResult.SUCCESS):
            self.latest_result = "SUCCESS"
        elif int(message.state) == int(RelocalizationResult.FAILED):
            self.latest_result = "FAILED"
        else:
            self.latest_result = "NONE"
        self.result_transaction = int(message.transaction_id)
        self.result_candidate = int(message.candidate_id)
        self.matching_epoch_applied = False

    def _epoch(self, message):
        self.matching_epoch_applied = bool(
            message.applied
            and int(message.transaction_id) == self.result_transaction
            and int(message.candidate_id) == self.result_candidate
            and self.result_transaction > 0
        )

    def _obstacle(self, message):
        states = {
            int(ObstacleSafetyState.CLEAR): "CLEAR",
            int(ObstacleSafetyState.CAUTION): "CAUTION",
            int(ObstacleSafetyState.BRAKE): "BRAKE",
            int(ObstacleSafetyState.HOVER_REQUIRED): "HOVER_REQUIRED",
        }
        self.obstacle_state = states.get(int(message.state), "UNHEALTHY")
        if not bool(message.raw_sensor_healthy):
            self.obstacle_state = "UNHEALTHY"

    def _directional(self, message):
        self.directional_weakness = max(0.0, min(1.0, float(message.data)))

    @staticmethod
    def _key(key, value):
        item = KeyValue()
        item.key = str(key)
        item.value = str(value)
        return item

    def _scheduler(self, message):
        degradations = {
            name: float(value)
            for name, value in zip(
                message.modality_names, message.degradation_scores
            )
        }
        enabled = {
            name: bool(value)
            for name, value in zip(
                message.modality_names, message.factor_enabled
            )
        }
        capabilities = {
            name: float(value)
            for name, value in zip(
                message.capability_names, message.capability_support
            )
        }
        sample = RelocalizationRiskSample(
            stamp_s=_stamp_seconds(message.header.stamp),
            scheduler_health=str(message.health_state),
            source_degradation=degradations,
            factor_enabled=enabled,
            capability_support=capabilities,
            estimator_support=float(message.estimator_support),
            covariance_growth_risk=self.covariance_growth_risk,
            pose_jump_risk=self.pose_jump_risk,
            directional_weakness_shadow=self.directional_weakness,
            directional_shadow_valid=self.directional_enabled,
            request_sources=("shared_request_topic",)
            if self.request_active else (),
            relocalization_result=self.latest_result,
            epoch_applied=self.matching_epoch_applied,
            obstacle_state=self.obstacle_state,
        )
        decision = self.core.update(sample)
        status = DiagnosticStatus()
        status.name = "relocalization/risk_shadow"
        status.hardware_id = "companion_computer"
        status.level = (
            DiagnosticStatus.OK
            if decision.level == RiskLevel.NORMAL else DiagnosticStatus.WARN
        )
        if decision.level >= RiskLevel.RELOCALIZE:
            status.level = DiagnosticStatus.ERROR
        status.message = decision.level.name
        status.values = [
            self._key("shadow_only", True),
            self._key("level", int(decision.level)),
            self._key("level_name", decision.level.name),
            self._key("target_level", decision.target_level.name),
            self._key("risk_score", f"{decision.score:.6f}"),
            self._key("drift_risk", f"{decision.drift_risk:.6f}"),
            self._key("would_request", decision.would_request),
            self._key("production_eligible", decision.production_eligible),
            self._key("request_suppressed", decision.request_suppressed),
            self._key(
                "duplicate_source_count", decision.duplicate_source_count
            ),
            self._key("reasons", ",".join(decision.reasons)),
        ]
        array = DiagnosticArray()
        array.header = message.header
        array.status.append(status)
        self.diagnostic_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationRiskShadowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
