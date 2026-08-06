#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import String

from multi_slam_uav_sim.guided_rectangle_waypoints import GuidedRectangleWaypoints


PROFILES = (
    "horizontal",
    "vertical",
    "yaw",
    "combined",
    "small_rectangle",
    "loop_return",
    "long_loop_return",
)


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class D435iSpeedEnvelopeMotion(GuidedRectangleWaypoints):
    """Opt-in, bounded GPS/GUIDED motion for D435i speed-envelope tests."""

    def __init__(self):
        super().__init__()
        self.declare_parameter("speed_test_profile", "horizontal")
        self.declare_parameter("stage_topic", "/d435i_visual_slam/stage")
        self.declare_parameter(
            "commanded_motion_topic", "/d435i_visual_slam/commanded_motion")
        self.declare_parameter("pre_observation_s", 5.0)
        self.declare_parameter("post_observation_s", 5.0)
        self.declare_parameter("flight_altitude_m", 0.5)
        self.declare_parameter("horizontal_speed_mps", 0.10)
        self.declare_parameter("vertical_speed_mps", 0.10)
        # yaw_rate_deg_s is declared by GuidedRectangleWaypoints.  Reuse that
        # inherited parameter so ROS 2 does not reject this derived node for a
        # duplicate declaration.  Matrix runners provide the profile-specific
        # value explicitly.
        self.declare_parameter("straight_distance_m", 3.0)
        self.declare_parameter("vertical_excursion_m", 0.70)
        self.declare_parameter("rectangle_x_m", 0.75)
        self.declare_parameter("rectangle_y_m", 0.50)
        self.declare_parameter("motion_hold_s", 3.0)
        self.declare_parameter("long_route_design_speed_mps", 0.75)
        self.declare_parameter("long_route_acceleration_s", 2.0)
        self.declare_parameter("long_route_min_steady_s", 3.0)
        self.declare_parameter("long_route_safety_margin_m", 0.75)
        self.declare_parameter("long_route_distance_m", 0.0)
        self.declare_parameter("landing_wait_s", 30.0)

        self.profile = str(self.get_parameter("speed_test_profile").value)
        if self.profile not in PROFILES:
            raise ValueError(f"speed_test_profile must be one of {', '.join(PROFILES)}")
        self.pre_observation_s = min(30.0, max(
            2.0, float(self.get_parameter("pre_observation_s").value)))
        self.post_observation_s = min(30.0, max(
            2.0, float(self.get_parameter("post_observation_s").value)))
        self.flight_altitude_m = min(1.0, max(
            0.4, float(self.get_parameter("flight_altitude_m").value)))
        self.horizontal_speed_mps = min(0.75, max(
            0.05, float(self.get_parameter("horizontal_speed_mps").value)))
        self.vertical_speed_mps = min(0.50, max(
            0.05, float(self.get_parameter("vertical_speed_mps").value)))
        self.yaw_rate_rad_s = math.radians(min(60.0, max(
            3.0, float(self.get_parameter("yaw_rate_deg_s").value))))
        self.straight_distance_m = min(4.0, max(
            1.5, float(self.get_parameter("straight_distance_m").value)))
        self.vertical_excursion_m = min(0.75, max(
            0.40, float(self.get_parameter("vertical_excursion_m").value)))
        self.rectangle_x_m = min(1.5, max(
            0.50, float(self.get_parameter("rectangle_x_m").value)))
        self.rectangle_y_m = min(1.0, max(
            0.40, float(self.get_parameter("rectangle_y_m").value)))
        self.motion_hold_s = min(10.0, max(
            2.0, float(self.get_parameter("motion_hold_s").value)))
        self.long_route_design_speed_mps = min(0.75, max(
            0.20, float(self.get_parameter(
                "long_route_design_speed_mps").value)))
        self.long_route_acceleration_s = min(4.0, max(
            1.0, float(self.get_parameter(
                "long_route_acceleration_s").value)))
        self.long_route_min_steady_s = min(10.0, max(
            3.0, float(self.get_parameter(
                "long_route_min_steady_s").value)))
        self.long_route_safety_margin_m = min(1.0, max(
            0.25, float(self.get_parameter(
                "long_route_safety_margin_m").value)))
        configured_long_distance = float(
            self.get_parameter("long_route_distance_m").value)
        automatic_long_distance = (
            self.long_route_design_speed_mps
            * (self.long_route_acceleration_s
               + self.long_route_min_steady_s)
            + self.long_route_safety_margin_m)
        self.long_route_distance_m = min(4.50, max(
            3.0, configured_long_distance if configured_long_distance > 0.0
            else automatic_long_distance))
        self.landing_wait_s = min(60.0, max(
            10.0, float(self.get_parameter("landing_wait_s").value)))

        self.stage_pub = self.create_publisher(
            String, str(self.get_parameter("stage_topic").value), 20)
        self.command_pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("commanded_motion_topic").value), 20)
        self.last_commanded_position = None
        self.last_commanded_yaw = 0.0
        self.get_logger().info(
            "D435i speed envelope motion: "
            f"profile={self.profile} horizontal={self.horizontal_speed_mps:.3f}m/s "
            f"vertical={self.vertical_speed_mps:.3f}m/s "
            f"yaw={math.degrees(self.yaw_rate_rad_s):.1f}deg/s "
            f"long_route={self.long_route_distance_m:.2f}m")

    def stage(self, name):
        message = String()
        message.data = f"speed_{self.profile}:{name}"
        self.stage_pub.publish(message)
        rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f"SPEED_ENVELOPE_STAGE {message.data}")

    def publish_command(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.linear.z = float(vz)
        message.twist.angular.z = float(yaw_rate)
        self.command_pub.publish(message)

    @staticmethod
    def _check_position(position):
        x, y, z = position
        if not (-1.0 <= x <= 5.0 and -3.0 <= y <= 3.0 and 0.10 <= z <= 1.35):
            raise RuntimeError(
                f"Refusing speed-envelope setpoint outside verified safe box: {position}")

    def hold(self, position, seconds, yaw, label):
        self._check_position(position)
        self.stage(label)
        self.last_commanded_position = position
        self.last_commanded_yaw = yaw
        period = 1.0 / self.rate_hz
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            self.ensure_guided(label)
            self.publish_setpoint(*position, yaw)
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            time.sleep(period)

    def move(self, start, goal, yaw, label, speed=None, hold=True):
        self._check_position(start)
        self._check_position(goal)
        self.stage(label)
        selected_speed = speed or self.horizontal_speed_mps
        delta = tuple(goal[index] - start[index] for index in range(3))
        distance = math.sqrt(sum(value * value for value in delta))
        duration = max(distance / selected_speed, 1.0)
        steps = max(1, int(math.ceil(duration * self.rate_hz)))
        velocity = tuple(value / duration for value in delta)
        self.get_logger().info(
            f"{label}: {start} -> {goal}, command={selected_speed:.3f}m/s "
            f"duration={duration:.2f}s")
        for index in range(steps + 1):
            self.ensure_guided(label)
            fraction = index / float(steps)
            position = tuple(
                start[axis] + delta[axis] * fraction for axis in range(3))
            self.publish_setpoint(*position, yaw)
            self.publish_command(*velocity)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            time.sleep(1.0 / self.rate_hz)
        self.last_commanded_position = goal
        self.last_commanded_yaw = yaw
        if hold:
            self.hold(goal, self.motion_hold_s, yaw, f"{label}_deceleration_hold")

    def rotate(self, position, start_yaw, goal_yaw, label, hold=True):
        self._check_position(position)
        self.stage(label)
        delta = wrap_angle(goal_yaw - start_yaw)
        duration = max(abs(delta) / self.yaw_rate_rad_s, 1.0)
        steps = max(1, int(math.ceil(duration * self.rate_hz)))
        yaw_rate = delta / duration
        self.get_logger().info(
            f"{label}: yaw {math.degrees(start_yaw):.1f} -> "
            f"{math.degrees(goal_yaw):.1f}deg, "
            f"command={math.degrees(abs(yaw_rate)):.1f}deg/s")
        for index in range(steps + 1):
            self.ensure_guided(label)
            yaw = start_yaw + delta * index / float(steps)
            self.publish_setpoint(*position, yaw)
            self.publish_command(yaw_rate=yaw_rate)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            time.sleep(1.0 / self.rate_hz)
        self.last_commanded_position = position
        self.last_commanded_yaw = goal_yaw
        if hold:
            self.hold(
                position, self.motion_hold_s, goal_yaw,
                f"{label}_deceleration_hold")

    def move_and_rotate(self, start, goal, start_yaw, goal_yaw, label):
        self._check_position(start)
        self._check_position(goal)
        self.stage(label)
        delta_position = tuple(goal[index] - start[index] for index in range(3))
        distance = math.sqrt(sum(value * value for value in delta_position))
        delta_yaw = wrap_angle(goal_yaw - start_yaw)
        duration = max(
            distance / self.horizontal_speed_mps,
            abs(delta_yaw) / self.yaw_rate_rad_s,
            1.0,
        )
        steps = max(1, int(math.ceil(duration * self.rate_hz)))
        velocity = tuple(value / duration for value in delta_position)
        yaw_rate = delta_yaw / duration
        self.get_logger().info(
            f"{label}: commanded horizontal={math.hypot(*velocity[:2]):.3f}m/s "
            f"yaw={math.degrees(abs(yaw_rate)):.1f}deg/s duration={duration:.2f}s")
        for index in range(steps + 1):
            self.ensure_guided(label)
            fraction = index / float(steps)
            position = tuple(
                start[axis] + delta_position[axis] * fraction
                for axis in range(3))
            yaw = start_yaw + delta_yaw * fraction
            self.publish_setpoint(*position, yaw)
            self.publish_command(*velocity, yaw_rate=yaw_rate)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            time.sleep(1.0 / self.rate_hz)
        self.last_commanded_position = goal
        self.last_commanded_yaw = goal_yaw
        self.hold(
            goal, self.motion_hold_s, goal_yaw,
            f"{label}_deceleration_hold")

    def profiled_move(self, start, goal, yaw, label):
        """Publish a bounded trapezoidal translation with explicit phases."""
        self._check_position(start)
        self._check_position(goal)
        delta = tuple(goal[index] - start[index] for index in range(3))
        distance = math.sqrt(sum(value * value for value in delta))
        direction = tuple(value / distance for value in delta)
        speed = self.horizontal_speed_mps
        acceleration_s = self.long_route_acceleration_s
        steady_s = distance / speed - acceleration_s
        if steady_s < self.long_route_min_steady_s:
            raise RuntimeError(
                f"Long route cannot provide {self.long_route_min_steady_s:.1f}s "
                f"steady motion: distance={distance:.2f}m speed={speed:.2f}m/s")
        acceleration = speed / acceleration_s
        acceleration_distance = 0.5 * speed * acceleration_s
        steady_distance = speed * steady_s
        duration = 2.0 * acceleration_s + steady_s
        steps = max(1, int(math.ceil(duration * self.rate_hz)))
        current_phase = None
        self.get_logger().info(
            f"{label}: distance={distance:.2f}m target={speed:.3f}m/s "
            f"accel={acceleration_s:.2f}s steady={steady_s:.2f}s "
            f"decel={acceleration_s:.2f}s")
        for index in range(steps + 1):
            elapsed = min(duration, index / self.rate_hz)
            if elapsed < acceleration_s:
                phase = "acceleration"
                phase_time = elapsed
                travelled = 0.5 * acceleration * phase_time ** 2
                current_speed = acceleration * phase_time
            elif elapsed < acceleration_s + steady_s:
                phase = "steady"
                phase_time = elapsed - acceleration_s
                travelled = acceleration_distance + speed * phase_time
                current_speed = speed
            else:
                phase = "deceleration"
                phase_time = min(
                    acceleration_s,
                    elapsed - acceleration_s - steady_s)
                travelled = (
                    acceleration_distance + steady_distance
                    + speed * phase_time
                    - 0.5 * acceleration * phase_time ** 2)
                current_speed = max(0.0, speed - acceleration * phase_time)
            if phase != current_phase:
                self.stage(f"{label}_{phase}")
                current_phase = phase
            fraction = min(1.0, travelled / distance)
            position = tuple(
                start[axis] + delta[axis] * fraction for axis in range(3))
            velocity = tuple(component * current_speed for component in direction)
            self.ensure_guided(f"{label}_{phase}")
            self.publish_setpoint(*position, yaw)
            self.publish_command(*velocity)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(f"{label}_{phase}")
            time.sleep(1.0 / self.rate_hz)
        self.last_commanded_position = goal
        self.last_commanded_yaw = yaw
        self.hold(goal, self.motion_hold_s, yaw, f"{label}_hover")

    def land(self):
        self.stage("land")
        self.publish_command()
        if not self.state.armed:
            return
        if not self.land_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MAVROS land service is not available")
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = self.last_commanded_yaw
        response = self.call(self.land_cli, request, "speed envelope land")
        if not bool(getattr(response, "success", False)):
            raise RuntimeError(f"Landing command rejected: {response}")
        deadline = time.monotonic() + self.landing_wait_s
        while rclpy.ok() and self.state.armed and time.monotonic() < deadline:
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.1)
            self._log_status("landing")
        if self.state.armed:
            raise RuntimeError("Vehicle did not disarm before landing timeout")

    def prepare_flight(self):
        self.stage("pre_ground_observation")
        self.spin_without_setpoints(
            self.pre_observation_s, label="speed pre-ground observation")
        self.stage("guided_arm_takeoff")
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()
        initial = (self.home_x, self.home_y, self.takeoff_alt)
        self._check_position(initial)
        self.hold(initial, 2.0, 0.0, "initial_takeoff_hold")
        if self.profile == "vertical":
            return initial
        start = (
            self.home_x,
            self.home_y,
            max(self.takeoff_alt, self.home_z + self.flight_altitude_m),
        )
        if start[2] > initial[2] + 0.01:
            self.move(
                initial, start, 0.0, "baseline_climb",
                speed=0.10, hold=False)
        self.hold(start, self.motion_hold_s, 0.0, "pre_motion_hover")
        return start

    def finish_flight(self, start):
        current = self.last_commanded_position or start
        if math.dist(current, start) > 0.05:
            self.move(
                current, start, self.last_commanded_yaw,
                "safe_return", speed=min(self.horizontal_speed_mps, 0.20))
        if abs(wrap_angle(self.last_commanded_yaw)) > math.radians(2.0):
            original_rate = self.yaw_rate_rad_s
            self.yaw_rate_rad_s = min(original_rate, math.radians(15.0))
            self.rotate(
                start, self.last_commanded_yaw, 0.0, "restore_start_yaw")
            self.yaw_rate_rad_s = original_rate
        self.hold(start, self.post_observation_s, 0.0, "post_motion_observation")
        self.land()
        self.stage("complete")
        self.spin_without_setpoints(0.5, label="speed envelope complete")

    def run(self):
        self.wait_ready()
        if self.wait_navigation_ready() != "gps":
            raise RuntimeError("Speed envelope requires the existing GPS/GUIDED source")
        start = self.prepare_flight()
        try:
            self.run_profile(start)
            self.finish_flight(start)
        except Exception:
            if self.state.armed:
                try:
                    self.land()
                except Exception as land_error:
                    self.get_logger().error(f"Safety landing failed: {land_error}")
            raise

    def run_profile(self, start):
        x, y, z = start
        if self.profile == "horizontal":
            goal = (x + self.straight_distance_m, y, z)
            self.move(start, goal, 0.0, "horizontal_out")
            self.move(goal, start, 0.0, "horizontal_return")
            return
        if self.profile == "yaw":
            self.rotate(start, 0.0, math.pi / 2.0, "yaw_out")
            self.rotate(start, math.pi / 2.0, 0.0, "yaw_return")
            return
        if self.profile == "vertical":
            top = (x, y, min(1.20, z + self.vertical_excursion_m))
            self.move(
                start, top, 0.0, "vertical_up",
                speed=self.vertical_speed_mps)
            self.move(
                top, start, 0.0, "vertical_down",
                speed=self.vertical_speed_mps)
            return
        if self.profile == "combined":
            duration = (math.pi / 2.0) / self.yaw_rate_rad_s
            path_length = min(2.5, max(
                0.8, self.horizontal_speed_mps * duration))
            component = path_length / math.sqrt(2.0)
            corner = (x + component, y + component, z)
            self.move_and_rotate(
                start, corner, 0.0, math.pi / 2.0, "combined_out")
            self.move_and_rotate(
                corner, start, math.pi / 2.0, 0.0, "combined_return")
            return
        if self.profile == "long_loop_return":
            goal = (x + self.long_route_distance_m, y, z)
            self.profiled_move(start, goal, 0.0, "long_out")
            self.profiled_move(goal, start, 0.0, "long_return")
            self.hold(
                start, self.motion_hold_s * 3.0, 0.0,
                "long_restore_start_view")
            return

        p1 = (x + self.rectangle_x_m, y, z)
        p2 = (x + self.rectangle_x_m, y + self.rectangle_y_m, z)
        p3 = (x, y + self.rectangle_y_m, z)
        if self.profile == "small_rectangle":
            for beginning, goal, label in (
                    (start, p1, "rectangle_leg_1"),
                    (p1, p2, "rectangle_leg_2"),
                    (p2, p3, "rectangle_leg_3"),
                    (p3, start, "rectangle_leg_4")):
                self.move(beginning, goal, 0.0, label)
            return

        self.move(start, p1, 0.0, "loop_leg_1")
        self.rotate(p1, 0.0, math.pi / 2.0, "loop_view_change_1")
        self.move(p1, p2, math.pi / 2.0, "loop_leg_2")
        self.rotate(p2, math.pi / 2.0, math.pi, "loop_view_change_2")
        self.move(p2, p3, math.pi, "loop_leg_3")
        self.rotate(p3, math.pi, -math.pi / 2.0, "loop_view_change_3")
        self.move(p3, start, -math.pi / 2.0, "loop_return_to_start")
        self.rotate(start, -math.pi / 2.0, 0.0, "loop_restore_start_view")
        self.hold(
            start, self.motion_hold_s * 2.0, 0.0, "loop_revisit_hold")


def main(args=None):
    rclpy.init(args=args)
    node = D435iSpeedEnvelopeMotion()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().error(str(error))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
