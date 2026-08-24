"""Large 3D figure-eight GUIDED flight with a mostly locked nose direction."""

from __future__ import annotations

import math
import time
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from mavros_msgs.srv import CommandTOL
from nav_msgs.msg import Odometry
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from uf_interfaces.msg import RelocalizationRequestIntent, SchedulerState

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
from .relocalization_motion import (
    MotionStatus,
    body_offset_to_local,
    decode_motion_command,
    encode_motion_status,
    motion_observations,
)
from .s_curve_path import (
    clamp_route_altitude_setpoint,
    feedback_error_to_fcu_setpoint,
    generate_calibration_figure_eight,
    generate_large_figure_eight,
    normalize_angle,
    normalize_route_feedback_source,
    polyline_length,
    resample_polyline,
)


class GuidedSCurveWaypoints(GuidedRectangleWaypoints):
    def __init__(
        self,
        node_name="guided_s_curve_waypoints",
        enforce_figure8_constraints=True,
    ):
        super().__init__(node_name=node_name)
        self.declare_parameter("longitudinal_span", 9.0)
        self.declare_parameter("lateral_amplitude", 1.5)
        self.declare_parameter("vertical_amplitude", 4.5)
        self.declare_parameter("vertical_cycles", 2)
        self.declare_parameter("pass_count", 1)
        self.declare_parameter("path_samples", 241)
        self.declare_parameter("figure_eight_rotation_deg", 158.0)
        self.declare_parameter("figure_eight_altitude_power", 4)
        self.declare_parameter("minimum_clearance_alt", 3.5)
        self.declare_parameter("locked_yaw_offset_deg", 0.0)
        self.declare_parameter("calibration_yaw_sweep_deg", 12.0)
        self.declare_parameter("calibration_yaw_cycles", 3.0)
        self.declare_parameter("calibration_motion_enabled", True)
        self.declare_parameter("calibration_motion_radius_m", 1.0)
        self.declare_parameter("calibration_motion_speed_mps", 0.60)
        self.declare_parameter("calibration_motion_samples", 161)
        self.declare_parameter("calibration_only", False)
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
            "relocalization_request_intent_topic",
            "/relocalization/request_intent",
        )
        self.declare_parameter("relocalization_request_lease_s", 1.0)
        self.declare_parameter("relocalization_request_heartbeat_s", 0.25)
        self.declare_parameter(
            "relocalization_ready_topic", "/relocalization/ready")
        self.declare_parameter(
            "unified_odom_topic", "/fusion/unified/odom")
        self.declare_parameter("unified_map_frame", "camera_init")
        self.declare_parameter("unified_body_frame", "body")
        self.declare_parameter("route_feedback_source", "unified_backend")
        self.declare_parameter(
            "gazebo_truth_odom_topic", "/sim/mid360/ground_truth_odom")
        self.declare_parameter("gazebo_truth_map_frame", "camera_init")
        self.declare_parameter("gazebo_truth_body_frame", "mid360_link")
        self.declare_parameter("gazebo_truth_timeout_s", 0.30)
        self.declare_parameter("max_route_command_offset_m", 1.50)
        self.declare_parameter("max_route_vertical_offset_m", 0.75)
        self.declare_parameter("route_altitude_margin_m", 0.50)
        self.declare_parameter(
            "external_nav_diagnostics_topic", "/external_nav/diagnostics")
        self.declare_parameter("scheduler_timeout_s", 1.0)
        self.declare_parameter("unified_odom_timeout_s", 0.60)
        self.declare_parameter("external_nav_gate_timeout_s", 1.50)
        self.declare_parameter("relocalization_retry_cooldown_s", 5.0)
        self.declare_parameter("relocalization_motion_enabled", False)
        self.declare_parameter(
            "relocalization_motion_command_topic",
            "/relocalization/motion_command",
        )
        self.declare_parameter(
            "relocalization_motion_status_topic",
            "/relocalization/motion_status",
        )
        self.declare_parameter("relocalization_motion_radius_m", 0.6)
        self.declare_parameter("relocalization_motion_speed_mps", 0.25)
        self.declare_parameter("relocalization_motion_yaw_rate_deg_s", 12.0)
        self.declare_parameter("relocalization_motion_yaw_step_deg", 45.0)
        # The backend stationary initializer consumes a 1.5 s IMU window.
        # Leave one second of margin so the window cannot include the motion.
        self.declare_parameter("relocalization_motion_settle_s", 2.5)
        self.declare_parameter("relocalization_motion_settle_timeout_s", 6.0)
        self.declare_parameter("relocalization_motion_position_tolerance_m", 0.35)
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
        self.figure_eight_rotation_deg = float(
            self.get_parameter("figure_eight_rotation_deg").value)
        self.figure_eight_altitude_power = int(
            self.get_parameter("figure_eight_altitude_power").value)
        self.minimum_clearance_alt = float(
            self.get_parameter("minimum_clearance_alt").value)
        self.locked_yaw_offset = math.radians(float(
            self.get_parameter("locked_yaw_offset_deg").value))
        self.calibration_yaw_sweep = math.radians(max(
            0.0, float(self.get_parameter("calibration_yaw_sweep_deg").value)))
        self.calibration_yaw_cycles = max(
            1.0, float(self.get_parameter("calibration_yaw_cycles").value)
        )
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
        self.calibration_only = bool(
            self.get_parameter("calibration_only").value)
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
        self.relocalization_motion_enabled = bool(
            self.get_parameter("relocalization_motion_enabled").value
        )
        self.relocalization_motion_radius_m = float(
            self.get_parameter("relocalization_motion_radius_m").value
        )
        self.relocalization_motion_speed_mps = float(
            self.get_parameter("relocalization_motion_speed_mps").value
        )
        self.relocalization_motion_yaw_rate_radps = math.radians(float(
            self.get_parameter("relocalization_motion_yaw_rate_deg_s").value
        ))
        self.relocalization_motion_yaw_step_deg = float(
            self.get_parameter("relocalization_motion_yaw_step_deg").value
        )
        self.relocalization_motion_settle_s = float(
            self.get_parameter("relocalization_motion_settle_s").value
        )
        self.relocalization_motion_settle_timeout_s = float(
            self.get_parameter("relocalization_motion_settle_timeout_s").value
        )
        self.relocalization_motion_position_tolerance_m = float(
            self.get_parameter(
                "relocalization_motion_position_tolerance_m"
            ).value
        )
        if not 0.1 <= self.relocalization_motion_radius_m <= 1.0:
            raise ValueError("relocalization motion radius must be in [0.1, 1.0] m")
        if not 0.1 <= self.relocalization_motion_speed_mps <= 0.5:
            raise ValueError("relocalization motion speed must be in [0.1, 0.5] m/s")
        if not math.radians(5.0) <= self.relocalization_motion_yaw_rate_radps <= math.radians(30.0):
            raise ValueError("relocalization yaw rate must be in [5, 30] deg/s")
        # This validates the step size even before a command arrives.
        motion_observations(
            "yaw_scan",
            self.relocalization_motion_radius_m,
            self.relocalization_motion_yaw_step_deg,
        )
        if not 0.5 <= self.relocalization_motion_settle_s <= 5.0:
            raise ValueError("relocalization motion settle must be in [0.5, 5.0] s")
        if (
            self.relocalization_motion_settle_timeout_s
            < self.relocalization_motion_settle_s
        ):
            raise ValueError("relocalization motion settle timeout is too short")
        if not 0.1 <= self.relocalization_motion_position_tolerance_m <= 0.75:
            raise ValueError(
                "relocalization motion position tolerance must be in [0.1, 0.75] m"
            )
        self.localization_min_support = max(
            0.0, float(self.get_parameter("localization_min_support").value))
        self.unified_map_frame = str(
            self.get_parameter("unified_map_frame").value)
        self.unified_body_frame = str(
            self.get_parameter("unified_body_frame").value)
        self.route_feedback_source = normalize_route_feedback_source(
            self.get_parameter("route_feedback_source").value)
        if (
            self.route_feedback_source == "gazebo_truth"
            and self.localization_safety_enabled
        ):
            raise ValueError(
                "gazebo_truth route control requires "
                "localization_safety_enabled=false so SLAM cannot hold or "
                "advance the diagnostic mission"
            )
        self.gazebo_truth_map_frame = str(
            self.get_parameter("gazebo_truth_map_frame").value)
        self.gazebo_truth_body_frame = str(
            self.get_parameter("gazebo_truth_body_frame").value)
        self.gazebo_truth_timeout_s = max(
            0.05,
            float(self.get_parameter("gazebo_truth_timeout_s").value),
        )
        self.max_route_command_offset = max(
            0.20,
            float(self.get_parameter("max_route_command_offset_m").value),
        )
        self.max_route_vertical_offset = max(
            0.10,
            float(self.get_parameter("max_route_vertical_offset_m").value),
        )
        self.route_altitude_margin_m = max(
            0.0,
            float(self.get_parameter("route_altitude_margin_m").value),
        )
        self.latest_scheduler = None
        self.latest_scheduler_arrival = None
        self.latest_unified_odom = None
        self.latest_gazebo_truth_odom = None
        self.latest_external_nav_gate_stamp_s = None
        self.latest_external_nav_gate_healthy = False
        self.latest_external_nav_gate_reason = "missing"
        self.relocalization_ready = False
        self.relocalization_request_active = False
        self.localization_safety_request_active = False
        self.last_relocalization_release_s = None
        self.relocalization_request_lease_s = max(
            0.20,
            float(self.get_parameter("relocalization_request_lease_s").value),
        )
        self.relocalization_request_heartbeat_s = max(
            0.05,
            min(
                self.relocalization_request_lease_s * 0.5,
                float(
                    self.get_parameter(
                        "relocalization_request_heartbeat_s"
                    ).value
                ),
            ),
        )
        self._request_instance_id = uuid.uuid4().hex
        self._request_sequence = 0
        self._request_episode_id = 0
        self._request_last_publish_monotonic_s = None
        self.relocalization_deferred_logged = False
        self.last_safety_state = TRACKING
        self.route_control_active = False
        self.route_origin_feedback = None
        self.route_origin_feedback_yaw = None
        self.route_origin_fcu_z = None
        self.feedback_to_fcu_yaw = None
        self.route_altitude_guard_logged = False
        self.last_route_fcu_setpoint = None
        self.route_hold_fcu_setpoint = None
        self.route_checkpoint_index = 0
        self.relocalization_motion_pending = None
        self.relocalization_motion_sequence_id = None
        self.relocalization_motion_profile = None
        self.relocalization_motion_anchor = None
        self.relocalization_motion_last_completed_step = -1
        self.mission_checkpoint_pub = self.create_publisher(
            String, "/mission/checkpoint", 10)
        self.relocalization_motion_status_pub = self.create_publisher(
            String,
            str(self.get_parameter(
                "relocalization_motion_status_topic"
            ).value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter(
                "relocalization_motion_command_topic"
            ).value),
            self._relocalization_motion_command_cb,
            10,
        )
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
        self.relocalization_request_intent_pub = self.create_publisher(
            RelocalizationRequestIntent,
            str(
                self.get_parameter(
                    "relocalization_request_intent_topic"
                ).value
            ),
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
        latest_odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("unified_odom_topic").value),
            self._unified_odom_cb,
            latest_odom_qos,
        )
        if self.route_feedback_source == "gazebo_truth":
            self.create_subscription(
                Odometry,
                str(self.get_parameter("gazebo_truth_odom_topic").value),
                self._gazebo_truth_odom_cb,
                latest_odom_qos,
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

        if enforce_figure8_constraints:
            if self.pass_count != 1:
                raise ValueError(
                    "the large figure-eight route must run exactly once"
                )
            if self.takeoff_alt < self.minimum_clearance_alt:
                raise ValueError(
                    "takeoff_alt is below minimum_clearance_alt"
                )

    def _scheduler_cb(self, msg):
        self.latest_scheduler = msg
        self.latest_scheduler_arrival = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )

    def _unified_odom_cb(self, msg):
        self.latest_unified_odom = msg

    def _gazebo_truth_odom_cb(self, msg):
        self.latest_gazebo_truth_odom = msg

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
        self.relocalization_request_active = active

    def _publish_relocalization_motion_status(
        self, command, state, reason, *, distance_m=0.0, duration_s=0.0
    ):
        status = MotionStatus(
            command.sequence_id,
            command.profile,
            command.step_index,
            command.step_count,
            state,
            reason,
            distance_m,
            duration_s,
        )
        message = String()
        message.data = encode_motion_status(status)
        self.relocalization_motion_status_pub.publish(message)
        self.get_logger().info(f"RELOCALIZATION_MOTION_STATUS {message.data}")

    def _relocalization_motion_command_cb(self, msg):
        try:
            command = decode_motion_command(msg.data)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        if not self.relocalization_motion_enabled:
            self._publish_relocalization_motion_status(
                command, "failed", "motion_disabled"
            )
            return
        if self.relocalization_motion_pending is not None:
            self._publish_relocalization_motion_status(
                command, "failed", "another_motion_command_is_pending"
            )
            return
        new_sequence = command.sequence_id != self.relocalization_motion_sequence_id
        if new_sequence and command.step_index != 0:
            self._publish_relocalization_motion_status(
                command, "failed", "new_sequence_must_start_at_step_zero"
            )
            return
        if not new_sequence and (
            command.profile != self.relocalization_motion_profile
            or command.step_index != self.relocalization_motion_last_completed_step + 1
        ):
            self._publish_relocalization_motion_status(
                command, "failed", "motion_step_is_not_the_next_sequence_step"
            )
            return
        self.relocalization_motion_pending = command

    def _relocalization_ready_cb(self, msg):
        self.relocalization_ready = bool(msg.data)
        if self.relocalization_ready:
            self.relocalization_deferred_logged = False

    @staticmethod
    def _odom_health(message, now, timeout_s, map_frame, body_frame):
        if message is None:
            return False, False
        stamp = message.header.stamp
        source_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        fresh = (
            source_s > 0.0
            and 0.0 <= now - source_s <= float(timeout_s)
        )
        pose = message.pose.pose
        values = (
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z), float(pose.orientation.x),
            float(pose.orientation.y), float(pose.orientation.z),
            float(pose.orientation.w),
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        frame_valid = (
            message.header.frame_id == map_frame
            and message.child_frame_id == body_frame
        )
        finite = (
            all(math.isfinite(value) for value in values)
            and quaternion_norm > 1.0e-6
            and frame_valid
        )
        return fresh, finite

    def _unified_odom_health(self, now):
        return self._odom_health(
            self.latest_unified_odom,
            now,
            self.unified_odom_timeout_s,
            self.unified_map_frame,
            self.unified_body_frame,
        )

    def _gazebo_truth_odom_health(self, now):
        return self._odom_health(
            self.latest_gazebo_truth_odom,
            now,
            self.gazebo_truth_timeout_s,
            self.gazebo_truth_map_frame,
            self.gazebo_truth_body_frame,
        )

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
                stamp = self.latest_unified_odom.header.stamp
                source_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            else:
                source_s = -1.0
            raise RuntimeError(
                "unified backend route feedback is unavailable: "
                f"fresh={fresh}, valid={valid}, frame={frame}, child={child}, "
                f"now_s={now:.3f}, source_s={source_s:.3f}, "
                f"source_age_s={now - source_s:.3f}")
        pose = self.latest_unified_odom.pose.pose
        return (
            (float(pose.position.x), float(pose.position.y),
             float(pose.position.z)),
            self._pose_yaw(pose),
        )

    def _gazebo_truth_state(self):
        now = self._now_s()
        fresh, valid = self._gazebo_truth_odom_health(now)
        if not fresh or not valid:
            frame = "missing"
            child = "missing"
            source_s = -1.0
            if self.latest_gazebo_truth_odom is not None:
                frame = self.latest_gazebo_truth_odom.header.frame_id
                child = self.latest_gazebo_truth_odom.child_frame_id
                stamp = self.latest_gazebo_truth_odom.header.stamp
                source_s = (
                    float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
                )
            raise RuntimeError(
                "Gazebo-truth route feedback is unavailable: "
                f"fresh={fresh}, valid={valid}, frame={frame}, child={child}, "
                f"now_s={now:.3f}, source_s={source_s:.3f}, "
                f"source_age_s={now - source_s:.3f}"
            )
        pose = self.latest_gazebo_truth_odom.pose.pose
        return (
            (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ),
            self._pose_yaw(pose),
        )

    def _route_feedback_state(self):
        if self.route_feedback_source == "gazebo_truth":
            return self._gazebo_truth_state()
        if self.route_feedback_source == "fcu_local":
            if self.pose is None:
                raise RuntimeError("FCU-local route feedback is unavailable")
            pose = self.pose.pose
            return (
                (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ),
                self._pose_yaw(pose),
            )
        return self._backend_state()

    def wait_unified_route_ready(self):
        deadline = self._now_s() + self.preflight_wait_s
        stable_since = None
        while rclpy.ok() and self._now_s() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self._now_s()
            fresh, valid = self._unified_odom_health(now)
            if not fresh or not valid or self.pose is None:
                stable_since = None
                purpose = (
                    "observer output"
                    if self.route_feedback_source == "gazebo_truth"
                    else "strict route feedback"
                )
                self._log_status(f"waiting for unified-backend {purpose}")
                continue
            stable_since = stable_since or now
            if now - stable_since >= self.navigation_stable_s:
                return
        if self.route_feedback_source == "gazebo_truth":
            raise RuntimeError(
                "unified backend observer output did not become fresh and "
                "valid; aborting because there would be no SLAM trajectory "
                "to diagnose"
            )
        raise RuntimeError(
            "unified backend route feedback did not become fresh and valid; "
            "the figure-eight mission will not fall back to FCU or Gazebo position")

    def wait_selected_route_feedback_ready(self):
        if self.route_feedback_source == "unified_backend":
            self.wait_unified_route_ready()
            return
        if self.route_feedback_source == "fcu_local":
            if self.pose is None:
                raise RuntimeError("FCU-local route feedback is unavailable")
            return
        deadline = self._now_s() + self.preflight_wait_s
        stable_since = None
        while rclpy.ok() and self._now_s() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self._now_s()
            fresh, valid = self._gazebo_truth_odom_health(now)
            if not fresh or not valid or self.pose is None:
                stable_since = None
                self._log_status(
                    "waiting for isolated Gazebo-truth route feedback")
                continue
            stable_since = stable_since or now
            if now - stable_since >= self.navigation_stable_s:
                return
        raise RuntimeError(
            "Gazebo-truth route feedback did not become fresh and valid; "
            "the observer mission will not fall back to FCU or SLAM position"
        )

    def activate_route_control(self):
        self.wait_selected_route_feedback_ready()
        feedback_position, feedback_yaw = self._route_feedback_state()
        if self.pose is None:
            raise RuntimeError("FCU local pose is required to express APM setpoints")
        fcu_yaw = self._pose_yaw(self.pose.pose)
        self.route_origin_feedback = feedback_position
        self.route_origin_feedback_yaw = feedback_yaw
        self.route_origin_fcu_z = float(self.pose.pose.position.z)
        self.feedback_to_fcu_yaw = normalize_angle(fcu_yaw - feedback_yaw)
        self.route_control_active = True
        if self.route_feedback_source == "fcu_local":
            self.get_logger().warning(
                "ESTIMATOR-ONLY ROUTE ISOLATION ENABLED: target/error/"
                "convergence use MAVROS FCU-local position; unified SLAM is "
                "observer-only and cannot hold, advance, or correct the route."
            )
            return
        if self.route_feedback_source == "gazebo_truth":
            self.get_logger().warning(
                "DIAGNOSTIC CONTROL ISOLATION ENABLED: target/error/convergence "
                "use Gazebo world truth; unified SLAM remains observer-only and "
                "cannot hold, advance, or correct the route. MAVROS local pose "
                "is only the APM setpoint coordinate adapter. yaw_alignment="
                f"{math.degrees(self.feedback_to_fcu_yaw):.2f}deg"
            )
            return
        self.get_logger().warning(
            "Strict route control enabled: target/error/convergence use "
            f"{self.unified_map_frame}->{self.unified_body_frame}; MAVROS local "
            "pose is only the APM setpoint coordinate adapter. No Gazebo truth "
            f"or FCU navigation fallback is accepted. yaw_alignment="
            f"{math.degrees(self.feedback_to_fcu_yaw):.2f}deg")

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

    def _publish_relocalization_request(self, active, reason="localization_safety"):
        active = bool(active)
        now_monotonic_s = time.monotonic()
        if active == self.localization_safety_request_active:
            if not active:
                return False
            if (
                self._request_last_publish_monotonic_s is not None
                and now_monotonic_s - self._request_last_publish_monotonic_s
                < self.relocalization_request_heartbeat_s
            ):
                return False
        if active and not self.localization_safety_request_active:
            self._request_episode_id += 1
        self._request_sequence += 1
        message = RelocalizationRequestIntent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_name()
        message.source_id = "localization_safety"
        message.source_instance_id = self._request_instance_id
        message.sequence = self._request_sequence
        message.episode_id = self._request_episode_id
        message.active = active
        message.lease_duration_s = float(self.relocalization_request_lease_s)
        message.reason = str(reason)
        self.relocalization_request_intent_pub.publish(message)
        previous = self.localization_safety_request_active
        self.localization_safety_request_active = active
        self._request_last_publish_monotonic_s = now_monotonic_s
        if previous and not active:
            self.last_relocalization_release_s = self._now_s()
        return True

    def _update_relocalization_request(self, decision):
        if decision.clear_relocalization_request:
            if self.localization_safety_request_active:
                self._publish_relocalization_request(
                    False, "localization_recovered"
                )
            return
        should_request = decision.request_relocalization or (
            decision.hold and decision.state == RELOCALIZING_HOLD
        )
        if not should_request:
            return
        if self.localization_safety_request_active:
            self._publish_relocalization_request(
                True, "persistent_localization_safety_hold"
            )
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
        self._publish_relocalization_request(
            True, "persistent_localization_safety_hold"
        )

    def _current_hold_target(self):
        if self.route_control_active:
            try:
                position, yaw = self._route_feedback_state()
                values = (*position, yaw)
                if all(math.isfinite(value) for value in values):
                    return values
            except RuntimeError:
                pass
            if self.route_origin_feedback is not None:
                return (
                    *self.route_origin_feedback,
                    float(self.route_origin_feedback_yaw),
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
        if self.pose is None or self.feedback_to_fcu_yaw is None:
            raise RuntimeError(
                "selected route control lost its FCU setpoint adapter")
        feedback_position, _ = self._route_feedback_state()
        fcu_pose = self.pose.pose.position
        fcu_position = (
            float(fcu_pose.x), float(fcu_pose.y), float(fcu_pose.z))
        command = feedback_error_to_fcu_setpoint(
            feedback_position,
            (float(x), float(y), float(z)),
            fcu_position,
            self.feedback_to_fcu_yaw,
            self.max_route_command_offset,
            self.max_route_vertical_offset,
        )
        guarded_z = clamp_route_altitude_setpoint(
            command[2],
            self.route_origin_fcu_z,
            self.vertical_amplitude,
            self.route_altitude_margin_m,
        )
        if abs(guarded_z - command[2]) > 1.0e-6:
            if not self.route_altitude_guard_logged:
                self.get_logger().error(
                    "Route altitude safety envelope engaged; refusing to "
                    f"command z={command[2]:.2f}m outside "
                    f"[{self.route_origin_fcu_z - self.route_altitude_margin_m:.2f}, "
                    f"{self.route_origin_fcu_z + self.vertical_amplitude + self.route_altitude_margin_m:.2f}]m. "
                    "This is a control safety stop, not estimator feedback."
                )
                self.route_altitude_guard_logged = True
            command = (command[0], command[1], guarded_z)
        command_yaw = normalize_angle(float(yaw) + self.feedback_to_fcu_yaw)
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

    def _active_motion_health_check(self):
        if self.relocalization_request_active:
            raise RuntimeError("relocalization_request_became_active_during_motion")
        lost, reason = self._obvious_localization_loss(self._now_s())
        if lost:
            raise RuntimeError(f"localization_became_unsafe:{reason}")

    def _publish_active_motion_setpoint(self, target):
        # Active-motion experiments use a bounded FCU-local offset from the
        # frozen anchor. They must not advance the unified-map route target.
        GuidedRectangleWaypoints.publish_setpoint(self, *target)

    def _settle_active_motion_target(self, target, started_s):
        minimum_end_s = self._now_s() + self.relocalization_motion_settle_s
        deadline_s = self._now_s() + self.relocalization_motion_settle_timeout_s
        next_publish_s = self._now_s()
        last_observed_s = next_publish_s
        while rclpy.ok() and self._now_s() < deadline_s:
            self.ensure_guided("active relocalization motion settle")
            self._active_motion_health_check()
            self._publish_active_motion_setpoint(target)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.pose is not None:
                position = self.pose.pose.position
                error = math.dist(
                    (
                        float(position.x),
                        float(position.y),
                        float(position.z),
                    ),
                    target[:3],
                )
                if (
                    self._now_s() >= minimum_end_s
                    and error <= self.relocalization_motion_position_tolerance_m
                ):
                    return self._now_s() - started_s
            next_publish_s = max(
                next_publish_s + 1.0 / self.rate_hz, self._now_s()
            )
            last_observed_s = self._wait_until_sim_time(
                min(next_publish_s, deadline_s), last_observed_s
            )
        raise RuntimeError("active_motion_failed_to_settle")

    def _wait_for_active_motion_search(self, target):
        deadline_s = self._now_s() + self.relocalization_motion_settle_timeout_s
        next_publish_s = self._now_s()
        last_observed_s = next_publish_s
        while rclpy.ok() and self._now_s() < deadline_s:
            if self.relocalization_request_active:
                return
            lost, reason = self._obvious_localization_loss(self._now_s())
            if lost:
                raise RuntimeError(
                    f"localization_became_unsafe_before_search:{reason}"
                )
            self.ensure_guided("active relocalization observation hold")
            self._publish_active_motion_setpoint(target)
            rclpy.spin_once(self, timeout_sec=0.0)
            next_publish_s = max(
                next_publish_s + 1.0 / self.rate_hz, self._now_s()
            )
            last_observed_s = self._wait_until_sim_time(
                min(next_publish_s, deadline_s), last_observed_s
            )
        raise RuntimeError("motion_search_request_handshake_timeout")

    def _execute_relocalization_motion(self):
        command = self.relocalization_motion_pending
        if command is None:
            return
        new_sequence = command.sequence_id != self.relocalization_motion_sequence_id
        try:
            if self.pose is None:
                raise RuntimeError("fcu_local_pose_is_unavailable")
            self.ensure_guided("active relocalization motion")
            self._active_motion_health_check()
            if new_sequence:
                position = self.pose.pose.position
                self.relocalization_motion_sequence_id = command.sequence_id
                self.relocalization_motion_profile = command.profile
                self.relocalization_motion_anchor = (
                    float(position.x),
                    float(position.y),
                    float(position.z),
                    self._pose_yaw(self.pose.pose),
                )
                self.relocalization_motion_last_completed_step = -1
            if self.relocalization_motion_anchor is None:
                raise RuntimeError("motion_anchor_is_unavailable")

            observations = motion_observations(
                command.profile,
                self.relocalization_motion_radius_m,
                self.relocalization_motion_yaw_step_deg,
            )
            if len(observations) != command.step_count:
                raise RuntimeError("motion_profile_step_count_mismatch")
            observation = observations[command.step_index]
            anchor_x, anchor_y, anchor_z, anchor_yaw = (
                self.relocalization_motion_anchor
            )
            target_x, target_y = body_offset_to_local(
                anchor_x,
                anchor_y,
                anchor_yaw,
                observation.forward_m,
                observation.left_m,
            )
            target_yaw = normalize_angle(
                anchor_yaw + observation.yaw_offset_rad
            )
            target = (target_x, target_y, anchor_z, target_yaw)
            current_position = self.pose.pose.position
            start = (
                float(current_position.x),
                float(current_position.y),
                float(current_position.z),
                self._pose_yaw(self.pose.pose),
            )
            distance = math.dist(start[:3], target[:3])
            yaw_delta = normalize_angle(target[3] - start[3])
            duration = max(
                distance / self.relocalization_motion_speed_mps,
                abs(yaw_delta) / self.relocalization_motion_yaw_rate_radps,
                0.1,
            )
            self._publish_relocalization_motion_status(
                command, "started", "executing", distance_m=distance
            )
            started_s = self._now_s()
            next_publish_s = started_s
            last_observed_s = started_s
            while rclpy.ok():
                now_s = self._now_s()
                if now_s < started_s:
                    raise RuntimeError("ros_clock_moved_backwards_during_motion")
                self.ensure_guided("active relocalization motion")
                self._active_motion_health_check()
                progress = min(1.0, (now_s - started_s) / duration)
                target_tick = (
                    start[0] + (target[0] - start[0]) * progress,
                    start[1] + (target[1] - start[1]) * progress,
                    start[2] + (target[2] - start[2]) * progress,
                    normalize_angle(start[3] + yaw_delta * progress),
                )
                self._publish_active_motion_setpoint(target_tick)
                rclpy.spin_once(self, timeout_sec=0.0)
                if progress >= 1.0:
                    break
                next_publish_s = max(
                    next_publish_s + 1.0 / self.rate_hz, self._now_s()
                )
                last_observed_s = self._wait_until_sim_time(
                    next_publish_s, last_observed_s
                )
            elapsed = self._settle_active_motion_target(target, started_s)
            self.relocalization_motion_last_completed_step = command.step_index
            self._publish_relocalization_motion_status(
                command,
                "settled",
                "ok",
                distance_m=distance,
                duration_s=elapsed,
            )
            self._wait_for_active_motion_search(target)
        except Exception as error:
            self._publish_relocalization_motion_status(
                command, "failed", str(error)
            )
            if new_sequence:
                self.relocalization_motion_sequence_id = None
                self.relocalization_motion_profile = None
                self.relocalization_motion_anchor = None
                self.relocalization_motion_last_completed_step = -1
        finally:
            self.relocalization_motion_pending = None

    def mission_safety_checkpoint(self, label):
        if not self.localization_safety_enabled:
            return
        if (
            self.relocalization_motion_pending is not None
            and not self.relocalization_request_active
        ):
            self._execute_relocalization_motion()
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
            position, _ = self._route_feedback_state()
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
        if not self.route_control_active or self.route_origin_feedback is None:
            raise RuntimeError(
                "figure-eight path cannot be anchored before route control")
        origin_x, origin_y, origin_z = self.route_origin_feedback
        relative = generate_large_figure_eight(
            self.longitudinal_span,
            self.lateral_amplitude,
            origin_z,
            self.vertical_amplitude,
            self.path_samples,
            self.figure_eight_rotation_deg,
            self.figure_eight_altitude_power,
        )
        return [
            (origin_x + x, origin_y + y, z)
            for x, y, z in relative
        ]

    def _route_anchor_index(self, path):
        if not path or self.route_origin_feedback is None:
            raise RuntimeError("cannot anchor an empty figure-eight route")
        index = min(
            range(len(path)),
            key=lambda candidate: math.dist(
                path[candidate], self.route_origin_feedback),
        )
        distance = math.dist(path[index], self.route_origin_feedback)
        if distance > self.waypoint_position_tolerance_m:
            raise RuntimeError(
                "selected route origin is not on the planned figure-eight: "
                f"nearest_distance={distance:.2f}m")
        return index

    @staticmethod
    def _path_heading(path, index):
        if len(path) < 2:
            raise ValueError("path heading requires at least two points")
        index = max(0, min(len(path) - 1, int(index)))
        lower = max(0, index - 1)
        upper = min(len(path) - 1, index + 1)
        while lower > 0 and math.dist(path[lower], path[index]) <= 1.0e-9:
            lower -= 1
        while (
            upper < len(path) - 1
            and math.dist(path[upper], path[index]) <= 1.0e-9
        ):
            upper += 1
        dx = path[upper][0] - path[lower][0]
        dy = path[upper][1] - path[lower][1]
        if math.hypot(dx, dy) <= 1.0e-9:
            raise ValueError("path heading is undefined for coincident points")
        return math.atan2(dy, dx)

    def fly_path(
        self,
        path,
        yaw,
        label,
        *,
        speed_mps=None,
        checkpoint_spacing_m=None,
        yaw_sweep_rad=0.0,
        yaw_cycles=1.0,
        follow_heading_fraction=0.0,
        heading_yaw_offset=0.0,
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
        yaw_cycles = max(1.0, float(yaw_cycles))
        follow_heading_fraction = max(
            0.0, min(1.0, float(follow_heading_fraction)))
        heading_yaw_offset = float(heading_yaw_offset)
        spacing = speed_mps / self.rate_hz
        points = resample_polyline(path, spacing)
        length = polyline_length(points)
        heading_split_index = min(
            len(points) - 1,
            int(round(follow_heading_fraction * (len(points) - 1))),
        )
        post_follow_yaw = normalize_angle(
            self._path_heading(points, heading_split_index)
            + heading_yaw_offset
        )
        self.get_logger().info(
            f"{label}: points={len(points)}, distance={length:.2f}m, "
            f"duration={length / speed_mps:.1f}s, "
            f"feedback={self.route_feedback_source if self.route_control_active else 'fcu_calibration'}, "
            f"center_yaw={math.degrees(yaw):.1f}deg, "
            f"yaw_sweep={math.degrees(yaw_sweep_rad):.1f}deg, "
            f"yaw_cycles={yaw_cycles:.1f}, "
            f"heading_follow_fraction={follow_heading_fraction:.2f}, "
            f"post_follow_yaw={math.degrees(post_follow_yaw):.1f}deg")
        travelled = 0.0
        next_waypoint_distance = checkpoint_spacing_m
        last_progress_ros_s = self._now_s()
        next_publish_ros_s = last_progress_ros_s
        last_observed_ros_s = last_progress_ros_s
        last_index = -1
        while rclpy.ok() and travelled < length:
            now_ros_s = self._now_s()
            if now_ros_s < last_progress_ros_s:
                raise RuntimeError("ROS clock moved backwards during route flight")
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
            if follow_heading_fraction > 0.0 and (
                progress <= follow_heading_fraction + 1.0e-9
            ):
                commanded_yaw = normalize_angle(
                    self._path_heading(points, index) + heading_yaw_offset)
            elif follow_heading_fraction > 0.0:
                commanded_yaw = post_follow_yaw
            else:
                commanded_yaw = yaw + yaw_sweep_rad * math.sin(
                    2.0 * math.pi * yaw_cycles * progress
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
        final_yaw = post_follow_yaw if follow_heading_fraction > 0.0 else yaw
        self.publish_setpoint(*points[-1], final_yaw)

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
                yaw_cycles=self.calibration_yaw_cycles,
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

    def finish_mission(self, current, yaw, completion_label):
        if self.final_hold_time_s > 0.0:
            self._publish_mission_phase(f"final_{completion_label}_hold")
            self.hold_setpoint(
                *current,
                seconds=self.final_hold_time_s,
                yaw=yaw,
                label=f"final {completion_label} hold",
                require_guided=True,
            )
        if self.land_at_end:
            self._publish_mission_phase("landing")
            if not self.land_cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("LAND service is unavailable")
            land_req = CommandTOL.Request()
            land_req.min_pitch = 0.0
            land_req.yaw = 0.0
            land_req.latitude = 0.0
            land_req.longitude = 0.0
            land_req.altitude = 0.0
            response = self.call(self.land_cli, land_req, "land")
            if not bool(getattr(response, "success", False)):
                raise RuntimeError("LAND command was rejected by the FCU")
            if not self.state.armed:
                self.get_logger().info(
                    "LAND completed and FCU disarm confirmed.")
                self._publish_mission_phase("landed")
                return
            started_ros_s = self._now_s()
            wall_deadline = (
                time.monotonic()
                + max(120.0, self.land_disarm_timeout_s * 10.0)
            )
            while rclpy.ok() and time.monotonic() < wall_deadline:
                if not self.state.armed:
                    self.get_logger().info(
                        "LAND completed and FCU disarm confirmed.")
                    self._publish_mission_phase("landed")
                    return
                if self._now_s() - started_ros_s >= self.land_disarm_timeout_s:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
                self._log_status("landing descent")
            raise RuntimeError(
                "LAND was accepted but FCU did not disarm within "
                f"{self.land_disarm_timeout_s:.1f}s simulation time "
                "or the wall-clock watchdog"
            )
        self._publish_mission_phase("complete_hold")
        self.get_logger().info(
            f"{completion_label.title()} mission complete. Holding final "
            "setpoint; Ctrl+C to stop.")
        while rclpy.ok():
            self.hold_setpoint(
                *current,
                seconds=1.0,
                yaw=yaw,
                label="final hold",
                require_guided=True,
            )

    def run(self):
        self._publish_mission_phase("preflight")
        self.wait_ready()
        navigation_source = self.wait_navigation_ready()
        # In the normal route, the unified backend is the authoritative
        # feedback source and must be fresh before arming.  The explicit
        # gazebo_truth mode is an observer-only diagnostic: truth controls the
        # route while unified odometry is recorded independently, so requiring
        # it here would prevent the diagnostic flight from starting whenever
        # the backend is temporarily stale.
        if self.route_feedback_source in {"unified_backend", "fcu_local"}:
            self.wait_unified_route_ready()
        self.wait_localization_safety_ready()
        start = (self.home_x, self.home_y, self.takeoff_alt)
        self.get_logger().info(
            f"Preflight accepted using {navigation_source}; entering large "
            "figure-eight mission.")
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()
        self.ensure_guided("post-takeoff")
        fcu_locked_yaw = self.home_yaw + self.locked_yaw_offset
        self._publish_mission_phase("post_takeoff_hold")
        self.hold_setpoint(
            *start, seconds=self.post_takeoff_hold_time_s, yaw=fcu_locked_yaw,
            label="post-takeoff hold", require_guided=True)
        if self.route_feedback_source == "gazebo_truth":
            self.activate_route_control()
            calibration_position = self.route_origin_feedback
            calibration_yaw = normalize_angle(
                self.route_origin_feedback_yaw + self.locked_yaw_offset)
        else:
            calibration_position = start
            calibration_yaw = fcu_locked_yaw
        self.calibration_warmup(calibration_position, calibration_yaw)

        if not self.route_control_active:
            self.activate_route_control()
        if self.calibration_only:
            self._publish_mission_phase("calibration_complete")
            current = self.route_origin_feedback
            yaw = self.route_origin_feedback_yaw
            self.get_logger().info(
                "Calibration-only mode completed the excitation trajectory; "
                "holding the same position through the selected route "
                "feedback before landing.")
            self.hold_setpoint(
                *current,
                seconds=2.0,
                yaw=yaw,
                label="post-calibration unified hold",
                require_guided=True,
            )
            self.finish_mission(current, yaw, "calibration")
            return

        base_path = self._absolute_path()
        anchor_index = self._route_anchor_index(base_path)
        if anchor_index not in (0, len(base_path) // 2, len(base_path) - 1):
            raise RuntimeError(
                "large figure-eight does not contain the unified route origin at "
                "a planned center crossing")
        if math.dist(base_path[0], self.route_origin_feedback) > 0.05:
            raise RuntimeError(
                "large figure-eight must start at the unified route origin")
        route_length = polyline_length(base_path)
        route_initial_yaw = normalize_angle(
            self._path_heading(base_path, 0) + self.locked_yaw_offset)
        route_midpoint_yaw = normalize_angle(
            self._path_heading(base_path, len(base_path) // 2)
            + self.locked_yaw_offset)
        low_altitude_ratio = sum(
            point[2] <= 8.0 for point in base_path
        ) / len(base_path)
        altitude_min = min(point[2] for point in base_path)
        altitude_max = max(point[2] for point in base_path)
        if low_altitude_ratio < 0.50:
            raise RuntimeError(
                "large figure-eight violates the low-altitude route contract: "
                f"ratio_at_or_below_8m={low_altitude_ratio:.3f}")
        self.get_logger().info(
            "Large figure-eight plan: one closed traversal, "
            f"planned_path_distance={route_length:.2f}m, "
            f"altitude_range={altitude_min:.2f}..{altitude_max:.2f}m, "
            f"ratio_at_or_below_8m={low_altitude_ratio:.1%}, "
            f"axis={self.figure_eight_rotation_deg:.1f}deg, "
            "yaw_mode=first_lobe_heading_follow/second_lobe_locked, "
            f"second_lobe_yaw={math.degrees(route_midpoint_yaw):.1f}deg"
        )
        self._publish_mission_phase("route_active")
        current = base_path[0]
        self.settle_waypoint(
            current, route_initial_yaw, "large figure-eight start")
        self.hold_setpoint(
            *current,
            seconds=1.0,
            yaw=route_initial_yaw,
            label="align nose with first-lobe heading",
            require_guided=True,
        )
        self.fly_path(
            base_path,
            route_initial_yaw,
            "large figure-eight single traversal",
            follow_heading_fraction=0.5,
            heading_yaw_offset=self.locked_yaw_offset,
            publish_relocalization_checkpoints=True,
        )
        self.get_logger().info(
            "Large figure-eight route completed: "
            f"distance={route_length:.2f}m"
        )
        current = base_path[-1]
        if math.dist(current, self.route_origin_feedback) > 0.05:
            raise RuntimeError(
                "large figure-eight did not close at the unified route origin")
        self.settle_waypoint(
            current,
            route_midpoint_yaw,
            "closed-loop return convergence",
            hold_s=self.hold_time,
        )

        self.finish_mission(
            current, route_midpoint_yaw, "loop")


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
