"""Long 3D S-curve GUIDED flight with a mostly locked nose direction."""

from __future__ import annotations

import math

import rclpy
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import Bool
from uf_interfaces.msg import SchedulerState

from .guided_rectangle_waypoints import GuidedRectangleWaypoints
from .localization_safety import (
    LocalizationSafetyStateMachine,
    TRACKING,
    scheduler_localization_loss,
)
from .s_curve_path import generate_s_curve, polyline_length, resample_polyline


class GuidedSCurveWaypoints(GuidedRectangleWaypoints):
    def __init__(self):
        super().__init__(node_name="guided_s_curve_waypoints")
        self.declare_parameter("longitudinal_span", 12.0)
        self.declare_parameter("lateral_amplitude", 4.5)
        self.declare_parameter("vertical_amplitude", 1.0)
        self.declare_parameter("vertical_cycles", 1)
        self.declare_parameter("pass_count", 3)
        self.declare_parameter("path_samples", 241)
        self.declare_parameter("minimum_clearance_alt", 3.5)
        self.declare_parameter("locked_yaw_offset_deg", 0.0)
        self.declare_parameter("calibration_yaw_sweep_deg", 12.0)
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
        self.declare_parameter("scheduler_timeout_s", 1.0)
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
        self.localization_min_support = max(
            0.0, float(self.get_parameter("localization_min_support").value))
        self.latest_scheduler = None
        self.latest_scheduler_arrival = None
        self.last_safety_state = TRACKING
        self.localization_safety = LocalizationSafetyStateMachine(
            loss_dwell_s=float(
                self.get_parameter("localization_loss_dwell_s").value),
            minimum_hold_s=float(
                self.get_parameter("localization_hold_s").value),
            recovery_dwell_s=float(
                self.get_parameter("localization_recovery_dwell_s").value),
        )
        self.relocalization_request_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("relocalization_request_topic").value),
            10,
        )
        self.create_subscription(
            SchedulerState,
            str(self.get_parameter("scheduler_topic").value),
            self._scheduler_cb,
            20,
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

    def _scheduler_is_fresh(self, now):
        return (
            self.latest_scheduler is not None
            and self.latest_scheduler_arrival is not None
            and 0.0 <= now - self.latest_scheduler_arrival
            <= self.scheduler_timeout_s
        )

    def _obvious_localization_loss(self, now):
        fresh = self._scheduler_is_fresh(now)
        if not fresh:
            return scheduler_localization_loss(
                "UNAVAILABLE", 0.0, (), (), self.localization_min_support,
                fresh=False)
        msg = self.latest_scheduler
        return scheduler_localization_loss(
            msg.health_state,
            msg.estimator_support,
            msg.capability_names,
            msg.capability_observable,
            self.localization_min_support,
            fresh=True,
        )

    def _publish_relocalization_request(self, active):
        message = Bool()
        message.data = bool(active)
        self.relocalization_request_pub.publish(message)

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
        if decision.request_relocalization:
            self._publish_relocalization_request(True)
        if not decision.hold:
            return

        hold_target = self.last_commanded_setpoint
        if hold_target is None:
            hold_target = (
                self.home_x, self.home_y, self.takeoff_alt, self.home_yaw)
        period = 1.0 / self.rate_hz
        next_publish_ros_s = self._now_s()
        last_observed_ros_s = next_publish_ros_s
        while rclpy.ok() and decision.hold:
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
            if decision.request_relocalization:
                self._publish_relocalization_request(True)
            if decision.clear_relocalization_request:
                self._publish_relocalization_request(False)
        self.get_logger().info(
            f"Localization recovered; resuming {label} after conservative hold.")

    def _position_error(self, point):
        if self.pose is None:
            return math.inf
        position = self.pose.pose.position
        return math.dist(
            (float(position.x), float(position.y), float(position.z)), point)

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
        relative = generate_s_curve(
            self.longitudinal_span,
            self.lateral_amplitude,
            self.takeoff_alt,
            self.vertical_amplitude,
            self.path_samples,
            self.vertical_cycles,
        )
        return [
            (self.home_x + x, self.home_y + y, z)
            for x, y, z in relative
        ]

    def fly_path(self, path, yaw, label):
        spacing = self.speed_mps / self.rate_hz
        points = resample_polyline(path, spacing)
        length = polyline_length(points)
        self.get_logger().info(
            f"{label}: points={len(points)}, distance={length:.2f}m, "
            f"duration={length / self.speed_mps:.1f}s, "
            f"locked_yaw={math.degrees(yaw):.1f}deg")
        travelled = 0.0
        next_waypoint_distance = self.waypoint_spacing_m
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
                travelled + self.speed_mps * (now_ros_s - last_progress_ros_s),
            )
            last_progress_ros_s = now_ros_s
            index = min(len(points) - 1, int(travelled / max(spacing, 1.0e-6)))
            if travelled >= length:
                index = len(points) - 1
            point = points[index]
            self.ensure_guided(label)
            self.mission_safety_checkpoint(label)
            last_progress_ros_s = self._now_s()
            self.publish_setpoint(*point, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            is_endpoint = index == len(points) - 1
            if (
                index != last_index
                and not is_endpoint
                and travelled + 1.0e-6 >= next_waypoint_distance
            ):
                self.settle_waypoint(
                    point, yaw,
                    f"{label} checkpoint {travelled:.1f}m")
                next_waypoint_distance += self.waypoint_spacing_m
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
        if self.calibration_yaw_sweep <= math.radians(0.5):
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
        self.wait_ready()
        navigation_source = self.wait_navigation_ready()
        self.wait_localization_safety_ready()
        start = (self.home_x, self.home_y, self.takeoff_alt)
        self.get_logger().info(
            f"Preflight accepted using {navigation_source}; entering S-curve mission.")
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()
        self.ensure_guided("post-takeoff")
        locked_yaw = self.home_yaw + self.locked_yaw_offset
        self.hold_setpoint(
            *start, seconds=3.0, yaw=locked_yaw,
            label="post-takeoff hold", require_guided=True)
        self.calibration_warmup(start, locked_yaw)

        base_path = self._absolute_path()
        total_path_distance = polyline_length(base_path) * self.pass_count
        self.get_logger().info(
            f"S-curve plan: passes={self.pass_count}, "
            f"planned_path_distance={total_path_distance:.2f}m, "
            f"altitude_range={self.takeoff_alt - self.vertical_amplitude:.2f}.."
            f"{self.takeoff_alt + self.vertical_amplitude:.2f}m")
        current = start
        for pass_index in range(self.pass_count):
            path = base_path if pass_index % 2 == 0 else list(reversed(base_path))
            first = path[0]
            if math.dist(current, first) > 0.05:
                self.fly_segment(
                    current, first, locked_yaw,
                    f"transit to S pass {pass_index + 1}/{self.pass_count}")
            self.settle_waypoint(
                first, locked_yaw,
                f"S pass {pass_index + 1}/{self.pass_count} start")
            self.fly_path(
                path, locked_yaw,
                f"S pass {pass_index + 1}/{self.pass_count}")
            current = path[-1]
            self.hold_setpoint(
                *current, seconds=self.hold_time, yaw=locked_yaw,
                label=f"S pass {pass_index + 1} endpoint hold",
                require_guided=True)

        if self.return_home_before_land:
            home_hover = (self.home_x, self.home_y, self.takeoff_alt)
            self.fly_segment(current, home_hover, locked_yaw, "return home")
            current = home_hover
            self.settle_waypoint(
                current, locked_yaw, "return-home convergence",
                hold_s=self.hold_time)

        if self.land_at_end:
            if self.land_cli.wait_for_service(timeout_sec=5.0):
                land_req = CommandTOL.Request()
                land_req.min_pitch = 0.0
                land_req.yaw = 0.0
                land_req.latitude = 0.0
                land_req.longitude = 0.0
                land_req.altitude = 0.0
                self.call(self.land_cli, land_req, "land")
        else:
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
