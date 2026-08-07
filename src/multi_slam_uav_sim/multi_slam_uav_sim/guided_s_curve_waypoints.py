"""Long 3D S-curve GUIDED flight with a mostly locked nose direction."""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from mavros_msgs.srv import CommandTOL
from nav_msgs.msg import Odometry
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String
from uf_interfaces.msg import SchedulerState

from .guided_rectangle_waypoints import GuidedRectangleWaypoints
from .localization_safety import (
    LocalizationSafetyStateMachine,
    RELOCALIZING_HOLD,
    TRACKING,
    diagnostic_level_value,
    mission_hold_required,
    scheduler_localization_loss,
)
from .relocalization_checkpoints import MissionCheckpoint, encode_checkpoint
from .s_curve_path import (
    backend_error_to_fcu_setpoint,
    generate_calibration_figure_eight,
    generate_s_curve,
    normalize_angle,
    polyline_length,
    resample_polyline,
)


class GuidedSCurveWaypoints(GuidedRectangleWaypoints):
    def __init__(self):
        super().__init__(node_name="guided_s_curve_waypoints")
        self.declare_parameter("longitudinal_span", 12.0)
        self.declare_parameter("lateral_amplitude", 4.5)
        self.declare_parameter("vertical_amplitude", 1.0)
        self.declare_parameter("vertical_cycles", 2)
        self.declare_parameter("pass_count", 3)
        self.declare_parameter("path_samples", 241)
        self.declare_parameter("minimum_clearance_alt", 3.5)
        self.declare_parameter("locked_yaw_offset_deg", 0.0)
        self.declare_parameter("calibration_yaw_sweep_deg", 12.0)
        self.declare_parameter("calibration_motion_enabled", True)
        self.declare_parameter("calibration_motion_radius_m", 1.0)
        self.declare_parameter("calibration_motion_speed_mps", 0.60)
        self.declare_parameter("calibration_motion_samples", 161)
        self.declare_parameter("return_home_before_land", True)
        self.declare_parameter("waypoint_spacing_m", 3.0)
        self.declare_parameter("waypoint_hold_s", 1.0)
        self.declare_parameter("waypoint_position_tolerance_m", 0.60)
        self.declare_parameter("waypoint_settle_s", 0.50)
        self.declare_parameter("waypoint_status_period_s", 8.0)
        self.declare_parameter("localization_safety_enabled", True)
        self.declare_parameter(
            "scheduler_topic", "/reliability/scheduler_state")
        self.declare_parameter(
            "relocalization_request_topic", "/relocalization/request")
        self.declare_parameter(
            "relocalization_ready_topic", "/relocalization/ready")
        self.declare_parameter(
            "unified_odom_topic", "/fusion/unified/odom")
        self.declare_parameter("unified_map_frame", "camera_init")
        self.declare_parameter("unified_body_frame", "body")
        self.declare_parameter("route_feedback_source", "unified_backend")
        self.declare_parameter("max_route_command_offset_m", 1.50)
        self.declare_parameter("max_route_vertical_offset_m", 0.75)
        self.declare_parameter(
            "external_nav_diagnostics_topic", "/external_nav/diagnostics")
        self.declare_parameter("scheduler_timeout_s", 1.0)
        self.declare_parameter("unified_odom_timeout_s", 0.60)
        self.declare_parameter("external_nav_gate_timeout_s", 1.50)
        self.declare_parameter("relocalization_retry_cooldown_s", 5.0)
        self.declare_parameter("localization_min_support", 0.15)
        self.declare_parameter("localization_loss_dwell_s", 0.30)
        self.declare_parameter("localization_hold_s", 1.0)
        self.declare_parameter("localization_recovery_dwell_s", 0.75)

        self.longitudinal_span = float(
            self.get_parameter("longitudinal_span").value)
        self.lateral_amplitude = float(
            self.get_parameter("lateral_amplitude").value)
        self.vertical_amplitude = float(
            self.get_parameter("vertical_amplitude").value)
        self.vertical_cycles = int(self.get_parameter("vertical_cycles").value)
        self.pass_count = int(self.get_parameter("pass_count").value)
        self.path_samples = int(self.get_parameter("path_samples").value)
        self.minimum_clearance_alt = float(
            self.get_parameter("minimum_clearance_alt").value)
        self.locked_yaw_offset = math.radians(float(
            self.get_parameter("locked_yaw_offset_deg").value))
        self.calibration_yaw_sweep = math.radians(max(
            0.0, float(self.get_parameter("calibration_yaw_sweep_deg").value)))
        self.calibration_motion_enabled = bool(
            self.get_parameter("calibration_motion_enabled").value)
        self.calibration_motion_radius_m = max(
            0.2,
            float(self.get_parameter("calibration_motion_radius_m").value),
        )
        self.calibration_motion_speed_mps = max(
            0.2,
            float(self.get_parameter("calibration_motion_speed_mps").value),
        )
        self.calibration_motion_samples = max(
            33,
            int(self.get_parameter("calibration_motion_samples").value),
        )
        self.return_home_before_land = bool(
            self.get_parameter("return_home_before_land").value)
        self.waypoint_spacing_m = max(
            0.5, float(self.get_parameter("waypoint_spacing_m").value))
        self.waypoint_hold_s = max(
            0.0, float(self.get_parameter("waypoint_hold_s").value))
        self.waypoint_position_tolerance_m = max(
            0.05, float(
                self.get_parameter("waypoint_position_tolerance_m").value))
        self.waypoint_settle_s = max(
            0.0, float(self.get_parameter("waypoint_settle_s").value))
        self.waypoint_status_period_s = max(
            1.0, float(self.get_parameter("waypoint_status_period_s").value))
        self.localization_safety_enabled = bool(
            self.get_parameter("localization_safety_enabled").value)
        self.scheduler_timeout_s = max(
            0.1, float(self.get_parameter("scheduler_timeout_s").value))
        self.unified_odom_timeout_s = max(
            0.1, float(self.get_parameter("unified_odom_timeout_s").value))
        self.external_nav_gate_timeout_s = max(
            0.5,
            float(self.get_parameter("external_nav_gate_timeout_s").value),
        )
        self.relocalization_retry_cooldown_s = max(
            0.0,
            float(
                self.get_parameter("relocalization_retry_cooldown_s").value
            ),
        )
        self.localization_min_support = max(
            0.0, float(self.get_parameter("localization_min_support").value))
        self.unified_map_frame = str(
            self.get_parameter("unified_map_frame").value)
        self.unified_body_frame = str(
            self.get_parameter("unified_body_frame").value)
        self.route_feedback_source = str(
            self.get_parameter("route_feedback_source").value).strip().lower()
        if self.route_feedback_source != "unified_backend":
            raise ValueError(
                "long S evaluation requires route_feedback_source=unified_backend")
        self.max_route_command_offset = max(
            0.20,
            float(self.get_parameter("max_route_command_offset_m").value),
        )
        self.max_route_vertical_offset = max(
            0.10,
            float(self.get_parameter("max_route_vertical_offset_m").value),
        )
        self.latest_scheduler = None
        self.latest_scheduler_arrival = None
        self.latest_unified_odom = None
        self.latest_external_nav_gate_stamp_s = None
        self.latest_external_nav_gate_healthy = False
        self.latest_external_nav_gate_reason = "missing"
        self.relocalization_ready = False
        self.relocalization_request_active = False
        self.last_relocalization_release_s = None
        self.relocalization_deferred_logged = False
        self.last_safety_state = TRACKING
        self.route_control_active = False
        self.route_origin_backend = None
        self.route_origin_backend_yaw = None
        self.backend_to_fcu_yaw = None
        self.last_route_fcu_setpoint = None
        self.route_hold_fcu_setpoint = None
        self.route_checkpoint_index = 0
        self.mission_checkpoint_pub = self.create_publisher(
            String, "/mission/checkpoint", 10)
        self.localization_safety = LocalizationSafetyStateMachine(
            loss_dwell_s=float(
                self.get_parameter("localization_loss_dwell_s").value),
            minimum_hold_s=float(
                self.get_parameter("localization_hold_s").value),
            recovery_dwell_s=float(
                self.get_parameter("localization_recovery_dwell_s").value),
        )
        relocalization_request_topic = str(
            self.get_parameter("relocalization_request_topic").value
        )
        self.relocalization_request_pub = self.create_publisher(
            Bool,
            relocalization_request_topic,
            10,
        )
        self.create_subscription(
            Bool,
            relocalization_request_topic,
            self._relocalization_request_state_cb,
            10,
        )
        self.create_subscription(
            SchedulerState,
            str(self.get_parameter("scheduler_topic").value),
            self._scheduler_cb,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("unified_odom_topic").value),
            self._unified_odom_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray,
            str(
                self.get_parameter("external_nav_diagnostics_topic").value
            ),
            self._external_nav_diagnostics_cb,
            10,
        )
        readiness_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("relocalization_ready_topic").value),
            self._relocalization_ready_cb,
            readiness_qos,
        )

        if self.pass_count < 1:
            raise ValueError("pass_count must be at least one")
        if self.takeoff_alt - self.vertical_amplitude < self.minimum_clearance_alt:
            raise ValueError(
                "takeoff_alt - vertical_amplitude is below minimum_clearance_alt")

    def _scheduler_cb(self, msg):
        self.latest_scheduler = msg
        self.latest_scheduler_arrival = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )

    def _unified_odom_cb(self, msg):
        self.latest_unified_odom = msg

    def _external_nav_diagnostics_cb(self, msg):
        for status in msg.status:
            if status.name != "external_nav/gate":
                continue
            stamp = msg.header.stamp
            self.latest_external_nav_gate_stamp_s = (
                float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            )
            self.latest_external_nav_gate_healthy = (
                diagnostic_level_value(status.level)
                == diagnostic_level_value(DiagnosticStatus.OK)
                and str(status.message) == "publishing"
            )
            self.latest_external_nav_gate_reason = str(status.message)

    def _relocalization_request_state_cb(self, msg):
        active = bool(msg.data)
        if self.relocalization_request_active and not active:
            self.last_relocalization_release_s = self._now_s()
        self.relocalization_request_active = active

    def _relocalization_ready_cb(self, msg):
        self.relocalization_ready = bool(msg.data)
        if self.relocalization_ready:
            self.relocalization_deferred_logged = False

    def _unified_odom_health(self, now):
        if self.latest_unified_odom is None:
            return False, False
        stamp = self.latest_unified_odom.header.stamp
        source_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        fresh = (
            source_s > 0.0
            and 0.0 <= now - source_s <= self.unified_odom_timeout_s
        )
        pose = self.latest_unified_odom.pose.pose
        values = (
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z), float(pose.orientation.x),
            float(pose.orientation.y), float(pose.orientation.z),
            float(pose.orientation.w),
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        frame_valid = (
            self.latest_unified_odom.header.frame_id == self.unified_map_frame
            and self.latest_unified_odom.child_frame_id
            == self.unified_body_frame
        )
        finite = (
            all(math.isfinite(value) for value in values)
            and quaternion_norm > 1.0e-6
            and frame_valid
        )
        return fresh, finite

    @staticmethod
    def _pose_yaw(pose):
        orientation = pose.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _backend_state(self):
        now = self._now_s()
        fresh, valid = self._unified_odom_health(now)
        if not fresh or not valid:
            frame = "missing"
            child = "missing"
            if self.latest_unified_odom is not None:
                frame = self.latest_unified_odom.header.frame_id
                child = self.latest_unified_odom.child_frame_id
            raise RuntimeError(
                "unified backend route feedback is unavailable: "
                f"fresh={fresh}, valid={valid}, frame={frame}, child={child}")
        pose = self.latest_unified_odom.pose.pose
        return (
            (float(pose.position.x), float(pose.position.y),
             float(pose.position.z)),
            self._pose_yaw(pose),
        )

    def wait_unified_route_ready(self):
        deadline = self._now_s() + self.preflight_wait_s
        stable_since = None
        while rclpy.ok() and self._now_s() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self._now_s()
            fresh, valid = self._unified_odom_health(now)
            if not fresh or not valid or self.pose is None:
                stable_since = None
                self._log_status(
                    "waiting for strict unified-backend route feedback")
                continue
            stable_since = stable_since or now
            if now - stable_since >= self.navigation_stable_s:
                return
        raise RuntimeError(
            "unified backend route feedback did not become fresh and valid; "
            "the long S mission will not fall back to FCU or Gazebo position")

    def activate_unified_route_control(self):
        self.wait_unified_route_ready()
        backend_position, backend_yaw = self._backend_state()
        if self.pose is None:
            raise RuntimeError("FCU local pose is required to express APM setpoints")
        fcu_yaw = self._pose_yaw(self.pose.pose)
        self.route_origin_backend = backend_position
        self.route_origin_backend_yaw = backend_yaw
        self.backend_to_fcu_yaw = normalize_angle(fcu_yaw - backend_yaw)
        self.route_control_active = True
        self.get_logger().warning(
            "Strict route control enabled: target/error/convergence use "
            f"{self.unified_map_frame}->{self.unified_body_frame}; MAVROS local "
            "pose is only the APM setpoint coordinate adapter. No Gazebo truth "
            f"or FCU navigation fallback is accepted. yaw_alignment="
            f"{math.degrees(self.backend_to_fcu_yaw):.2f}deg")

    def _scheduler_is_fresh(self, now):
        return (
            self.latest_scheduler is not None
            and self.latest_scheduler_arrival is not None
            and 0.0 <= now - self.latest_scheduler_arrival
            <= self.scheduler_timeout_s
        )

    def _external_nav_gate_health(self, now):
        stamp_s = self.latest_external_nav_gate_stamp_s
        fresh = (
            stamp_s is not None
            and stamp_s > 0.0
            and 0.0 <= now - stamp_s <= self.external_nav_gate_timeout_s
        )
        return (
            fresh,
            self.latest_external_nav_gate_healthy,
            self.latest_external_nav_gate_reason,
        )

    def _obvious_localization_loss(self, now):
        fresh = self._scheduler_is_fresh(now)
        estimator_fresh, estimator_finite = self._unified_odom_health(now)
        gate_fresh, gate_healthy, gate_reason = (
            self._external_nav_gate_health(now)
        )
        if not fresh:
            return scheduler_localization_loss(
                "UNAVAILABLE", 0.0, (), (), self.localization_min_support,
                fresh=False, estimator_fresh=estimator_fresh,
                estimator_finite=estimator_finite,
                external_nav_gate_fresh=gate_fresh,
                external_nav_gate_healthy=gate_healthy,
                external_nav_gate_reason=gate_reason)
        msg = self.latest_scheduler
        return scheduler_localization_loss(
            msg.health_state,
            msg.estimator_support,
            msg.capability_names,
            msg.capability_observable,
            self.localization_min_support,
            fresh=True,
            estimator_fresh=estimator_fresh,
            estimator_finite=estimator_finite,
            external_nav_gate_fresh=gate_fresh,
            external_nav_gate_healthy=gate_healthy,
            external_nav_gate_reason=gate_reason,
        )

    def _publish_relocalization_request(self, active):
        previous = self.relocalization_request_active
        message = Bool()
        message.data = bool(active)
        self.relocalization_request_pub.publish(message)
        self.relocalization_request_active = bool(active)
        if previous and not self.relocalization_request_active:
            self.last_relocalization_release_s = self._now_s()

    def _update_relocalization_request(self, decision):
        if decision.clear_relocalization_request:
            if self.relocalization_request_active:
                self._publish_relocalization_request(False)
            return
        should_request = decision.request_relocalization or (
            decision.hold and decision.state == RELOCALIZING_HOLD
        )
        if not should_request or self.relocalization_request_active:
            return
        now = self._now_s()
        if (
            self.last_relocalization_release_s is not None
            and now >= self.last_relocalization_release_s
            and now - self.last_relocalization_release_s
            < self.relocalization_retry_cooldown_s
        ):
            return
        if not self.relocalization_ready:
            if not self.relocalization_deferred_logged:
                self.get_logger().warning(
                    "Localization hold active, but relocalization database is "
                    "not ready; deferring the recovery request.")
                self.relocalization_deferred_logged = True
            return
        self._publish_relocalization_request(True)

    def _current_hold_target(self):
        if self.route_control_active:
            if self.latest_unified_odom is not None:
                pose = self.latest_unified_odom.pose.pose
                values = (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                    self._pose_yaw(pose),
                )
                if all(math.isfinite(value) for value in values):
                    return values
            if self.route_origin_backend is not None:
                return (
                    *self.route_origin_backend,
                    float(self.route_origin_backend_yaw),
                )
            return None
        if self.pose is None:
            return self.last_commanded_setpoint
        position = self.pose.pose.position
        yaw = self._pose_yaw(self.pose.pose)
        return (
            float(position.x), float(position.y), float(position.z), float(yaw)
        )

    def publish_setpoint(self, x, y, z, yaw=0.0):
        if not self.route_control_active:
            return super().publish_setpoint(x, y, z, yaw)
        if self.route_hold_fcu_setpoint is not None:
            return super().publish_setpoint(*self.route_hold_fcu_setpoint)
        if self.pose is None or self.backend_to_fcu_yaw is None:
            raise RuntimeError(
                "strict unified route control lost its FCU setpoint adapter")
        backend_position, _ = self._backend_state()
        fcu_pose = self.pose.pose.position
        fcu_position = (
            float(fcu_pose.x), float(fcu_pose.y), float(fcu_pose.z))
        command = backend_error_to_fcu_setpoint(
            backend_position,
            (float(x), float(y), float(z)),
            fcu_position,
            self.backend_to_fcu_yaw,
            self.max_route_command_offset,
            self.max_route_vertical_offset,
        )
        command_yaw = normalize_angle(float(yaw) + self.backend_to_fcu_yaw)
        self.last_route_fcu_setpoint = (*command, command_yaw)
        return super().publish_setpoint(*self.last_route_fcu_setpoint)

    def _freeze_route_setpoint(self):
        if not self.route_control_active or self.route_hold_fcu_setpoint is not None:
            return
        if self.pose is not None:
            position = self.pose.pose.position
            self.route_hold_fcu_setpoint = (
                float(position.x),
                float(position.y),
                float(position.z),
                self._pose_yaw(self.pose.pose),
            )
        elif self.last_route_fcu_setpoint is not None:
            self.route_hold_fcu_setpoint = self.last_route_fcu_setpoint
        else:
            raise RuntimeError(
                "localization loss occurred before an FCU safety-hold setpoint existed")
        self.get_logger().warning(
            "Freezing the current FCU-local hold setpoint for localization safety; "
            "this hold is not estimator feedback and cannot advance the route.")

    def _release_route_setpoint(self):
        if self.route_hold_fcu_setpoint is None:
            return
        self.route_hold_fcu_setpoint = None
        self.get_logger().info(
            "Unified localization recovered; releasing the frozen FCU hold setpoint.")

    def wait_localization_safety_ready(self):
        if not self.localization_safety_enabled:
            self.get_logger().warning(
                "Localization safety supervision is disabled for this mission.")
            return
        deadline = self._now_s() + self.preflight_wait_s
        healthy_since = None
        while rclpy.ok() and self._now_s() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self._now_s()
            lost, reason = self._obvious_localization_loss(now)
            if lost:
                healthy_since = None
                self._log_status(f"localization safety wait ({reason})")
                continue
            healthy_since = healthy_since or now
            if now - healthy_since >= self.navigation_stable_s:
                self.get_logger().info(
                    "Localization safety input is fresh and observable; "
                    "mission supervision armed.")
                return
        raise RuntimeError(
            "Localization safety readiness timed out: require a fresh scheduler "
            "state with propagation, horizontal motion, and yaw observability")

    def mission_safety_checkpoint(self, label):
        if not self.localization_safety_enabled:
            return
        now = self._now_s()
        lost, reason = self._obvious_localization_loss(now)
        decision = self.localization_safety.update(lost, now)
        if decision.state != self.last_safety_state:
            self.get_logger().warning(
                f"Localization safety {self.last_safety_state} -> "
                f"{decision.state}: {reason}; mission={label}")
            self.last_safety_state = decision.state
        self._update_relocalization_request(decision)
        # A stale backend cannot be used to transform the next route target.
        # Freeze immediately during the confirmation dwell, then keep the same
        # FCU-local setpoint through HOLDING/RELOCALIZING/RECOVERY_PENDING.
        effective_hold = mission_hold_required(
            decision.hold, lost, self.relocalization_request_active)
        if not effective_hold:
            return

        self._freeze_route_setpoint()

        hold_target = self._current_hold_target()
        if hold_target is None:
            hold_target = (
                self.home_x, self.home_y, self.takeoff_alt, self.home_yaw)
        period = 1.0 / self.rate_hz
        next_publish_ros_s = self._now_s()
        last_observed_ros_s = next_publish_ros_s
        while rclpy.ok() and mission_hold_required(
            decision.hold, lost, self.relocalization_request_active
        ):
            self.ensure_guided(f"localization safety hold during {label}")
            self.publish_setpoint(*hold_target)
            rclpy.spin_once(self, timeout_sec=0.0)
            next_publish_ros_s = max(
                next_publish_ros_s + period, self._now_s()
            )
            last_observed_ros_s = self._wait_until_sim_time(
                next_publish_ros_s, last_observed_ros_s
            )
            now = self._now_s()
            lost, reason = self._obvious_localization_loss(now)
            decision = self.localization_safety.update(lost, now)
            if decision.state != self.last_safety_state:
                self.get_logger().warning(
                    f"Localization safety {self.last_safety_state} -> "
                    f"{decision.state}: {reason}; holding the last safe setpoint")
                self.last_safety_state = decision.state
            self._update_relocalization_request(decision)
        self._release_route_setpoint()
        self.get_logger().info(
            f"Localization recovered; resuming {label} after conservative hold.")

    def _position_error(self, point):
        if self.route_control_active:
            position, _ = self._backend_state()
            return math.dist(position, point)
        if self.pose is None:
            return math.inf
        position = self.pose.pose.position
        return math.dist(
            (float(position.x), float(position.y), float(position.z)), point)

    def _publish_route_checkpoint(self, label, distance_m, point):
        self.route_checkpoint_index += 1
        checkpoint = MissionCheckpoint(
            index=self.route_checkpoint_index,
            label=str(label),
            distance_m=float(distance_m),
            position=tuple(float(value) for value in point),
        )
        message = String()
        message.data = encode_checkpoint(checkpoint)
        self.mission_checkpoint_pub.publish(message)
        self.get_logger().info(
            f"Mission checkpoint {checkpoint.index}: {checkpoint.label}, "
            f"distance={checkpoint.distance_m:.1f}m")

    def settle_waypoint(self, point, yaw, label, hold_s=None):
        hold_s = self.waypoint_hold_s if hold_s is None else max(0.0, hold_s)
        started = self._now_s()
        next_warning = started + self.waypoint_status_period_s
        within_since = None
        next_publish_ros_s = started
        last_observed_ros_s = started
        while rclpy.ok():
            self.ensure_guided(label)
            self.mission_safety_checkpoint(label)
            self.publish_setpoint(*point, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            now = self._now_s()
            if now < started:
                raise RuntimeError("ROS clock moved backwards while settling waypoint")
            error = self._position_error(point)
            if error <= self.waypoint_position_tolerance_m:
                within_since = within_since or now
            else:
                within_since = None
            held_long_enough = now - started >= hold_s
            settled = (
                within_since is not None
                and now - within_since >= self.waypoint_settle_s
            )
            if held_long_enough and settled:
                return
            if now >= next_warning:
                self.get_logger().warning(
                    f"{label}: still holding for convergence; "
                    f"position_error={error:.2f}m")
                next_warning = now + self.waypoint_status_period_s
            next_publish_ros_s = max(
                next_publish_ros_s + 1.0 / self.rate_hz, self._now_s()
            )
            last_observed_ros_s = self._wait_until_sim_time(
                next_publish_ros_s, last_observed_ros_s
            )

    def _absolute_path(self):
        if not self.route_control_active or self.route_origin_backend is None:
            raise RuntimeError(
                "S-curve path cannot be anchored before unified route control")
        origin_x, origin_y, origin_z = self.route_origin_backend
        relative = generate_s_curve(
            self.longitudinal_span,
            self.lateral_amplitude,
            origin_z,
            self.vertical_amplitude,
            self.path_samples,
            self.vertical_cycles,
        )
        return [
            (origin_x + x, origin_y + y, z)
            for x, y, z in relative
        ]

    def _route_anchor_index(self, path):
        if not path or self.route_origin_backend is None:
            raise RuntimeError("cannot anchor an empty S-curve route")
        index = min(
            range(len(path)),
            key=lambda candidate: math.dist(
                path[candidate], self.route_origin_backend),
        )
        distance = math.dist(path[index], self.route_origin_backend)
        if distance > self.waypoint_position_tolerance_m:
            raise RuntimeError(
                "unified-backend route origin is not on the planned S curve: "
                f"nearest_distance={distance:.2f}m")
        return index

    def fly_path(
        self,
        path,
        yaw,
        label,
        *,
        speed_mps=None,
        checkpoint_spacing_m=None,
        yaw_sweep_rad=0.0,
        publish_relocalization_checkpoints=False,
    ):
        speed_mps = self.speed_mps if speed_mps is None else max(
            0.1, float(speed_mps)
        )
        checkpoint_spacing_m = (
            self.waypoint_spacing_m
            if checkpoint_spacing_m is None
            else float(checkpoint_spacing_m)
        )
        spacing = speed_mps / self.rate_hz
        points = resample_polyline(path, spacing)
        length = polyline_length(points)
        self.get_logger().info(
            f"{label}: points={len(points)}, distance={length:.2f}m, "
            f"duration={length / speed_mps:.1f}s, "
            f"feedback={'unified_backend' if self.route_control_active else 'fcu_calibration'}, "
            f"center_yaw={math.degrees(yaw):.1f}deg, "
            f"yaw_sweep={math.degrees(yaw_sweep_rad):.1f}deg")
        travelled = 0.0
        next_waypoint_distance = checkpoint_spacing_m
        last_progress_ros_s = self._now_s()
        next_publish_ros_s = last_progress_ros_s
        last_observed_ros_s = last_progress_ros_s
        last_index = -1
        while rclpy.ok() and travelled < length:
            now_ros_s = self._now_s()
            if now_ros_s < last_progress_ros_s:
                raise RuntimeError("ROS clock moved backwards during S-curve")
            travelled = min(
                length,
                travelled + speed_mps * (now_ros_s - last_progress_ros_s),
            )
            last_progress_ros_s = now_ros_s
            index = min(len(points) - 1, int(travelled / max(spacing, 1.0e-6)))
            if travelled >= length:
                index = len(points) - 1
            point = points[index]
            progress = travelled / max(length, 1.0e-9)
            commanded_yaw = yaw + yaw_sweep_rad * math.sin(
                2.0 * math.pi * progress
            )
            self.ensure_guided(label)
            self.mission_safety_checkpoint(label)
            last_progress_ros_s = self._now_s()
            self.publish_setpoint(*point, commanded_yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            is_endpoint = index == len(points) - 1
            if (
                index != last_index
                and not is_endpoint
                and travelled + 1.0e-6 >= next_waypoint_distance
            ):
                if publish_relocalization_checkpoints:
                    self._publish_route_checkpoint(label, travelled, point)
                self.settle_waypoint(
                    point, commanded_yaw,
                    f"{label} checkpoint {travelled:.1f}m")
                next_waypoint_distance += checkpoint_spacing_m
                last_progress_ros_s = self._now_s()
            last_index = index
            next_publish_ros_s = max(
                next_publish_ros_s + 1.0 / self.rate_hz, self._now_s()
            )
            last_observed_ros_s = self._wait_until_sim_time(
                next_publish_ros_s, last_observed_ros_s
            )
        self.publish_setpoint(*points[-1], yaw)

    def calibration_warmup(self, position, locked_yaw):
        if (
            not self.calibration_motion_enabled
            and self.calibration_yaw_sweep <= math.radians(0.5)
        ):
            return
        self._publish_mission_phase("calibration_excitation")
        if self.calibration_motion_enabled:
            path = generate_calibration_figure_eight(
                position,
                self.calibration_motion_radius_m,
                self.calibration_motion_samples,
            )
            self.get_logger().info(
                "Continuous multi-axis calibration excitation begins; "
                "the mission yaw is locked after it."
            )
            self.fly_path(
                path,
                locked_yaw,
                "calibration figure-eight",
                speed_mps=self.calibration_motion_speed_mps,
                checkpoint_spacing_m=math.inf,
                yaw_sweep_rad=self.calibration_yaw_sweep,
            )
            self.hold_setpoint(
                *position,
                seconds=1.0,
                yaw=locked_yaw,
                label="calibration settle",
                require_guided=True,
            )
            return
        self.get_logger().info(
            "Short calibration warmup begins; the mission yaw is locked after it.")
        self.rotate_in_place(
            position, locked_yaw, locked_yaw + self.calibration_yaw_sweep,
            "calibration yaw positive")
        self.rotate_in_place(
            position, locked_yaw + self.calibration_yaw_sweep,
            locked_yaw - self.calibration_yaw_sweep,
            "calibration yaw negative")
        self.rotate_in_place(
            position, locked_yaw - self.calibration_yaw_sweep, locked_yaw,
            "calibration yaw settle")

    def run(self):
        self._publish_mission_phase("preflight")
        self.wait_ready()
        navigation_source = self.wait_navigation_ready()
        # Refuse to arm if the authoritative estimator is absent. Takeoff and
        # calibration still use FCU setpoints, but the evaluation route never
        # starts from FCU or Gazebo position feedback.
        self.wait_unified_route_ready()
        self.wait_localization_safety_ready()
        start = (self.home_x, self.home_y, self.takeoff_alt)
        self.get_logger().info(
            f"Preflight accepted using {navigation_source}; entering S-curve mission.")
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()
        self.ensure_guided("post-takeoff")
        fcu_locked_yaw = self.home_yaw + self.locked_yaw_offset
        self._publish_mission_phase("post_takeoff_hold")
        self.hold_setpoint(
            *start, seconds=self.post_takeoff_hold_time_s, yaw=fcu_locked_yaw,
            label="post-takeoff hold", require_guided=True)
        self.calibration_warmup(start, fcu_locked_yaw)

        self.activate_unified_route_control()
        locked_yaw = (
            self.route_origin_backend_yaw + self.locked_yaw_offset)

        base_path = self._absolute_path()
        anchor_index = self._route_anchor_index(base_path)
        entry_path = list(reversed(base_path[:anchor_index + 1]))
        route_length = polyline_length(base_path)
        return_path = (
            list(reversed(base_path[anchor_index:]))
            if self.pass_count % 2 == 1
            else base_path[:anchor_index + 1]
        )
        total_path_distance = (
            polyline_length(entry_path)
            + route_length * self.pass_count
            + (
                polyline_length(return_path)
                if self.return_home_before_land else 0.0
            )
        )
        altitude_center = self.route_origin_backend[2]
        self.get_logger().info(
            f"S-curve plan: passes={self.pass_count}, "
            f"planned_path_distance={total_path_distance:.2f}m, "
            f"altitude_range={altitude_center - self.vertical_amplitude:.2f}.."
            f"{altitude_center + self.vertical_amplitude:.2f}m")
        current = self.route_origin_backend
        self._publish_mission_phase("route_active")
        if math.dist(current, entry_path[0]) > 0.05:
            self.fly_segment(
                current, entry_path[0], locked_yaw,
                "align with S-route center anchor")
        self.fly_path(entry_path, locked_yaw, "S-route protected entry")
        current = entry_path[-1]
        self.settle_waypoint(current, locked_yaw, "S-route entry endpoint")

        for pass_index in range(self.pass_count):
            path = base_path if pass_index % 2 == 0 else list(reversed(base_path))
            first = path[0]
            if math.dist(current, first) > self.waypoint_position_tolerance_m:
                raise RuntimeError(
                    "S-route traversal is discontinuous; refusing a direct "
                    f"transit through urban geometry before pass {pass_index + 1}")
            self.settle_waypoint(
                first, locked_yaw,
                f"S pass {pass_index + 1}/{self.pass_count} start")
            self.fly_path(
                path, locked_yaw,
                f"S pass {pass_index + 1}/{self.pass_count}",
                publish_relocalization_checkpoints=True)
            current = path[-1]
            self.hold_setpoint(
                *current, seconds=self.hold_time, yaw=locked_yaw,
                label=f"S pass {pass_index + 1} endpoint hold",
                require_guided=True)

        if self.return_home_before_land:
            home_hover = self.route_origin_backend
            if math.dist(current, return_path[0]) > self.waypoint_position_tolerance_m:
                raise RuntimeError(
                    "S-route return is discontinuous; refusing a direct "
                    "return through urban geometry")
            self.fly_path(return_path, locked_yaw, "S-route protected return")
            current = return_path[-1]
            if math.dist(current, home_hover) > 0.05:
                self.fly_segment(
                    current, home_hover, locked_yaw,
                    "final route-anchor alignment")
            current = home_hover
            self.settle_waypoint(
                current, locked_yaw, "return-home convergence",
                hold_s=self.hold_time)

        if self.final_hold_time_s > 0.0:
            self._publish_mission_phase("final_loop_hold")
            self.hold_setpoint(
                *current, seconds=self.final_hold_time_s, yaw=locked_yaw,
                label="final loop-closure hold", require_guided=True)

        if self.land_at_end:
            self._publish_mission_phase("landing")
            if self.land_cli.wait_for_service(timeout_sec=5.0):
                land_req = CommandTOL.Request()
                land_req.min_pitch = 0.0
                land_req.yaw = 0.0
                land_req.latitude = 0.0
                land_req.longitude = 0.0
                land_req.altitude = 0.0
                self.call(self.land_cli, land_req, "land")
        else:
            self._publish_mission_phase("complete_hold")
            self.get_logger().info(
                "S-curve mission complete. Holding final setpoint; Ctrl+C to stop.")
            while rclpy.ok():
                self.hold_setpoint(
                    *current, seconds=1.0, yaw=locked_yaw,
                    label="final hold", require_guided=True)


def main(args=None):
    rclpy.init(args=args)
    node = GuidedSCurveWaypoints()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
