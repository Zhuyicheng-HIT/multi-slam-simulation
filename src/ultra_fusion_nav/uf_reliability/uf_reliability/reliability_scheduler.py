import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool
from uf_interfaces.msg import (
    FusionEpoch,
    ReliabilityScore,
    RelocalizationResult,
    SchedulerState,
    RelocalizationRequestIntent,
)

from .scheduler_core import (
    CAPABILITIES,
    MODALITIES,
    ReliabilitySchedulerCore,
    SchedulerConfig,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def automatic_relocalization_trigger(
    lidar_degradation,
    lidar_enabled,
    now_s,
    candidate_since_s,
    last_request_s,
    hold_s,
    cooldown_s,
    degradation_threshold,
    request_active=False,
    horizontal_position_supported=False,
    evidence_count=1,
    minimum_evidence_count=1,
):
    """Debounce relocalization after LiDAR and position-support loss."""
    now_s = float(now_s)
    degraded = float(lidar_degradation) >= float(degradation_threshold)
    candidate = (
        (degraded or not bool(lidar_enabled))
        and not bool(horizontal_position_supported)
    )
    if not candidate:
        return False, None
    if candidate_since_s is None or now_s < float(candidate_since_s):
        candidate_since_s = now_s
    if int(evidence_count) < max(1, int(minimum_evidence_count)):
        return False, candidate_since_s
    if request_active:
        return False, candidate_since_s
    if last_request_s is not None and now_s - float(last_request_s) < float(cooldown_s):
        return False, candidate_since_s
    if now_s - float(candidate_since_s) < float(hold_s):
        return False, candidate_since_s
    return True, candidate_since_s


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
        self.declare_parameter("lidar_score_timeout_s", 1.6)
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
        self.declare_parameter("automatic_relocalization_enabled", True)
        self.declare_parameter("automatic_relocalization_degradation_threshold", 0.85)
        self.declare_parameter("automatic_relocalization_hold_s", 1.0)
        self.declare_parameter("automatic_relocalization_cooldown_s", 15.0)
        self.declare_parameter("automatic_relocalization_startup_grace_s", 10.0)
        self.declare_parameter(
            "automatic_relocalization_minimum_lidar_observations", 3
        )
        self.declare_parameter(
            "automatic_relocalization_position_support_threshold", 0.15
        )
        self.declare_parameter("relocalization_commit_timeout_s", 2.0)
        self.declare_parameter("relocalization_ready_topic", "/relocalization/ready")
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
            modality_stale_after_s=((
                "lidar",
                float(self.get_parameter("lidar_score_timeout_s").value),
            ),),
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
        self.automatic_relocalization_enabled = bool(
            self.get_parameter("automatic_relocalization_enabled").value
        )
        self.automatic_relocalization_degradation_threshold = float(
            self.get_parameter(
                "automatic_relocalization_degradation_threshold"
            ).value
        )
        self.automatic_relocalization_hold_s = float(
            self.get_parameter("automatic_relocalization_hold_s").value
        )
        self.automatic_relocalization_cooldown_s = float(
            self.get_parameter("automatic_relocalization_cooldown_s").value
        )
        self.automatic_relocalization_startup_grace_s = float(
            self.get_parameter("automatic_relocalization_startup_grace_s").value
        )
        self.automatic_relocalization_minimum_lidar_observations = max(
            1,
            int(self.get_parameter(
                "automatic_relocalization_minimum_lidar_observations"
            ).value),
        )
        self.automatic_relocalization_position_support_threshold = max(
            0.0,
            float(self.get_parameter(
                "automatic_relocalization_position_support_threshold"
            ).value),
        )
        self.relocalization_candidate_since_s = None
        self.relocalization_lidar_observations = 0
        self.last_relocalization_lidar_score_s = None
        self.relocalization_horizontal_position_supported = False
        self.last_relocalization_request_s = None
        self.first_lidar_score_s = None
        self.automatic_relocalization_requests = 0
        self.relocalization_commit_timeout_s = max(
            0.2, float(self.get_parameter("relocalization_commit_timeout_s").value)
        )
        self.relocalization_candidate_accepted = False
        self.relocalization_commit_deadline_s = None
        self.relocalization_epoch_at_request = 0
        self.current_fusion_epoch = 0
        self.current_fusion_session = 0
        self.active_relocalization_transaction_id = 0
        self.active_relocalization_candidate_id = 0
        self.pending_fusion_epochs = {}
        self.relocalization_commit_timeouts = 0
        self.relocalization_commits = 0
        self.relocalization_ready = False
        self.relocalization_failures = 0
        self.last_relocalization_failure_reason = "none"
        self.state_pub = self.create_publisher(
            SchedulerState, "/reliability/scheduler_state", 20)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/reliability/scheduler_diagnostics", 10)
        self.relocalization_request_pub = self.create_publisher(
            RelocalizationRequestIntent, "/relocalization/request_intent", 10)
        self.relocalization_intent_sequence = 0
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
        self.fusion_epoch_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            FusionEpoch,
            "/fusion/unified/epoch",
            self._fusion_epoch,
            self.fusion_epoch_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("relocalization_ready_topic").value),
            self._relocalization_ready,
            self.fusion_epoch_qos,
        )
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
        if modality == "lidar" and self.first_lidar_score_s is None:
            self.first_lidar_score_s = source_s

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _observe_ros_clock(self, now_s):
        if self._last_clock_s is not None and now_s < self._last_clock_s:
            self.scores.clear()
            self.relocalization_candidate_since_s = None
            self.relocalization_lidar_observations = 0
            self.last_relocalization_lidar_score_s = None
            self.relocalization_horizontal_position_supported = False
            self.last_relocalization_request_s = None
            self.first_lidar_score_s = None
            self.relocalization_candidate_accepted = False
            self.relocalization_commit_deadline_s = None
            self.active_relocalization_transaction_id = 0
            self.active_relocalization_candidate_id = 0
            self.pending_fusion_epochs.clear()
        self._last_clock_s = now_s

    def _relocalization(self, msg):
        requested = bool(msg.data)
        if requested and not self.relocalization_requested:
            self.relocalization_failed = False
            self.last_relocalization_request_s = self._now_s()
            self.relocalization_epoch_at_request = self.current_fusion_epoch
            self.relocalization_candidate_accepted = False
            self.relocalization_commit_deadline_s = None
            self.active_relocalization_transaction_id = 0
            self.active_relocalization_candidate_id = 0
        elif not requested:
            self.relocalization_candidate_accepted = False
            self.relocalization_commit_deadline_s = None
        self.relocalization_requested = requested

    def _relocalization_ready(self, msg):
        self.relocalization_ready = bool(msg.data)
        if not self.relocalization_ready:
            self.relocalization_candidate_since_s = None
            self.relocalization_lidar_observations = 0
            self.last_relocalization_lidar_score_s = None

    def _relocalization_result(self, msg):
        if int(msg.state) == int(RelocalizationResult.SUCCESS) and msg.accepted:
            if int(msg.transaction_id) <= 0:
                self.relocalization_requested = False
                self.relocalization_failed = True
                self._release_relocalization_request()
                return
            # Registration acceptance is only a proposal. Remain in
            # RELOCALIZING until the unified backend commits a new epoch.
            self.relocalization_requested = True
            self.relocalization_failed = False
            self.relocalization_candidate_accepted = True
            self.active_relocalization_transaction_id = int(msg.transaction_id)
            self.active_relocalization_candidate_id = int(msg.candidate_id)
            self.relocalization_commit_deadline_s = (
                self._now_s() + self.relocalization_commit_timeout_s
            )
            self._complete_relocalization_if_committed()
        elif int(msg.state) == int(RelocalizationResult.FAILED):
            self.relocalization_requested = False
            self.relocalization_failed = True
            self.relocalization_failures += 1
            self.last_relocalization_failure_reason = str(msg.reason)
            self.relocalization_candidate_accepted = False
            self.relocalization_commit_deadline_s = None
            self.active_relocalization_transaction_id = 0
            self.active_relocalization_candidate_id = 0
            self._release_relocalization_request()

    def _release_relocalization_request(self):
        release = RelocalizationRequestIntent()
        release.header.stamp = self.get_clock().now().to_msg()
        release.source_id = "reliability_scheduler"
        release.source_instance_id = self.get_name()
        self.relocalization_intent_sequence += 1
        release.sequence = self.relocalization_intent_sequence
        release.episode_id = int(self.current_fusion_epoch)
        release.active = False
        release.lease_duration_s = 0.5
        release.reason = "scheduler_release"
        self.relocalization_request_pub.publish(release)

    def _fusion_epoch(self, msg):
        session_id = int(msg.session_id)
        counter = int(msg.reset_counter)
        if session_id <= 0:
            return
        if (
            self.current_fusion_session > 0
            and session_id < self.current_fusion_session
        ):
            return
        if session_id != self.current_fusion_session:
            self.current_fusion_session = session_id
            self.current_fusion_epoch = counter
            self.pending_fusion_epochs.clear()
        elif counter < self.current_fusion_epoch:
            return
        if not bool(msg.applied):
            return
        self.current_fusion_epoch = max(self.current_fusion_epoch, counter)
        transaction_id = int(msg.transaction_id)
        if transaction_id <= 0:
            return
        self.pending_fusion_epochs[transaction_id] = (
            session_id, counter, int(msg.candidate_id)
        )
        while len(self.pending_fusion_epochs) > 16:
            self.pending_fusion_epochs.pop(next(iter(self.pending_fusion_epochs)))
        self._complete_relocalization_if_committed()

    def _complete_relocalization_if_committed(self):
        transaction_id = self.active_relocalization_transaction_id
        committed = self.pending_fusion_epochs.get(transaction_id)
        if transaction_id <= 0 or committed is None:
            return False
        session_id, counter, candidate_id = committed
        if (
            session_id != self.current_fusion_session
            or candidate_id != self.active_relocalization_candidate_id
        ):
            return False
        self.current_fusion_epoch = max(self.current_fusion_epoch, counter)
        self.pending_fusion_epochs.pop(transaction_id, None)
        self.relocalization_requested = False
        self.relocalization_failed = False
        self.relocalization_candidate_accepted = False
        self.relocalization_commit_deadline_s = None
        self.relocalization_commits += 1
        self._release_relocalization_request()
        return True

    def _expire_relocalization_commit(self, now_s):
        if (
            self.relocalization_candidate_accepted
            and self.relocalization_commit_deadline_s is not None
            and now_s >= self.relocalization_commit_deadline_s
        ):
            self.relocalization_requested = False
            self.relocalization_failed = True
            self.relocalization_candidate_accepted = False
            self.relocalization_commit_deadline_s = None
            self.relocalization_commit_timeouts += 1
            self._release_relocalization_request()

    def _maybe_request_relocalization(self, result, now_s):
        if not self.automatic_relocalization_enabled or "lidar" not in self.scores:
            return
        if not self.relocalization_ready:
            self.relocalization_candidate_since_s = None
            return
        if (
            self.first_lidar_score_s is None
            or now_s - self.first_lidar_score_s
            < self.automatic_relocalization_startup_grace_s
        ):
            return
        lidar_score_s = float(self.scores["lidar"]["arrival_s"])
        lidar_candidate = (
            result.degradation_scores["lidar"]
            >= self.automatic_relocalization_degradation_threshold
            or not result.factor_enabled["lidar"]
        )
        horizontal_position_support = float(
            result.capability_support["horizontal_position"]
        )
        horizontal_position_supported = (
            horizontal_position_support
            >= self.automatic_relocalization_position_support_threshold
        )
        self.relocalization_horizontal_position_supported = (
            horizontal_position_supported
        )
        if lidar_score_s != self.last_relocalization_lidar_score_s:
            self.last_relocalization_lidar_score_s = lidar_score_s
            if lidar_candidate and not horizontal_position_supported:
                self.relocalization_lidar_observations += 1
            else:
                self.relocalization_lidar_observations = 0
        trigger, candidate_since = automatic_relocalization_trigger(
            result.degradation_scores["lidar"],
            result.factor_enabled["lidar"],
            now_s,
            self.relocalization_candidate_since_s,
            self.last_relocalization_request_s,
            self.automatic_relocalization_hold_s,
            self.automatic_relocalization_cooldown_s,
            self.automatic_relocalization_degradation_threshold,
            self.relocalization_requested,
            horizontal_position_supported,
            self.relocalization_lidar_observations,
            self.automatic_relocalization_minimum_lidar_observations,
        )
        self.relocalization_candidate_since_s = candidate_since
        if not trigger:
            return
        request = RelocalizationRequestIntent()
        request.header.stamp = self.get_clock().now().to_msg()
        request.source_id = "reliability_scheduler"
        request.source_instance_id = self.get_name()
        self.relocalization_intent_sequence += 1
        request.sequence = self.relocalization_intent_sequence
        request.episode_id = int(self.current_fusion_epoch)
        request.active = True
        request.lease_duration_s = max(0.5, self.automatic_relocalization_cooldown_s)
        request.reason = "persistent_lidar_loss"
        self.relocalization_request_pub.publish(request)
        self.relocalization_requested = True
        self.relocalization_failed = False
        self.relocalization_epoch_at_request = self.current_fusion_epoch
        self.relocalization_candidate_accepted = False
        self.active_relocalization_transaction_id = 0
        self.active_relocalization_candidate_id = 0
        self.relocalization_commit_deadline_s = None
        self.last_relocalization_request_s = now_s
        self.automatic_relocalization_requests += 1
        self.get_logger().warning(
            "automatic relocalization requested after persistent LiDAR loss "
            "without independent horizontal-position support"
        )

    @staticmethod
    def _key(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _publish(self):
        now_s = self._now_s()
        self._observe_ros_clock(now_s)
        self._expire_relocalization_commit(now_s)
        result = self.core.update(
            self.scores, now_s, self.relocalization_requested,
            self.relocalization_failed)
        self._maybe_request_relocalization(result, now_s)
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
        diagnostic.values.append(
            self._key(
                "automatic_relocalization_requests",
                self.automatic_relocalization_requests,
            )
        )
        diagnostic.values.extend((
            self._key(
                "automatic_relocalization_lidar_observations",
                self.relocalization_lidar_observations,
            ),
            self._key(
                "automatic_relocalization_horizontal_position_supported",
                self.relocalization_horizontal_position_supported,
            ),
            self._key("relocalization_ready", self.relocalization_ready),
            self._key("relocalization_failures", self.relocalization_failures),
            self._key(
                "last_relocalization_failure_reason",
                self.last_relocalization_failure_reason,
            ),
            self._key("fusion_epoch", self.current_fusion_epoch),
            self._key("fusion_session", self.current_fusion_session),
            self._key(
                "relocalization_transaction_id",
                self.active_relocalization_transaction_id,
            ),
            self._key(
                "relocalization_candidate_accepted",
                self.relocalization_candidate_accepted,
            ),
            self._key("relocalization_commits", self.relocalization_commits),
            self._key(
                "relocalization_commit_timeouts",
                self.relocalization_commit_timeouts,
            ),
        ))
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
