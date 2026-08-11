#!/usr/bin/env python3
import math
import time

import rclpy
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import String

from multi_slam_uav_sim.guided_rectangle_waypoints import GuidedRectangleWaypoints


PROFILES = (
    "stationary",
    "hover",
    "yaw_30",
    "yaw_90",
    "straight",
    "l_shape",
    "single_corner",
    "small_rectangle",
    "loop_return",
)


class ProgressiveVisualMotion(GuidedRectangleWaypoints):
    """Low-speed GPS/GUIDED motion profiles for read-only visual evaluation."""

    def __init__(self):
        super().__init__()
        self.declare_parameter("profile", "stationary")
        self.declare_parameter("stage_topic", "/d435i_visual_slam/stage")
        self.declare_parameter("pre_observation_s", 5.0)
        self.declare_parameter("post_observation_s", 5.0)
        self.declare_parameter("stationary_s", 30.0)
        self.declare_parameter("hover_s", 30.0)
        self.declare_parameter("flight_altitude_m", 0.5)
        self.declare_parameter("distance_m", 1.0)
        self.declare_parameter("rectangle_x_m", 0.75)
        self.declare_parameter("rectangle_y_m", 0.50)
        self.declare_parameter("motion_speed_mps", 0.10)
        self.declare_parameter("yaw_speed_deg_s", 8.0)
        self.declare_parameter("motion_hold_s", 3.0)
        self.declare_parameter("landing_wait_s", 30.0)

        self.profile = str(self.get_parameter("profile").value)
        if self.profile not in PROFILES:
            raise ValueError(
                f"profile must be one of {', '.join(PROFILES)}")
        self.pre_observation_s = max(
            2.0, float(self.get_parameter("pre_observation_s").value))
        self.post_observation_s = max(
            2.0, float(self.get_parameter("post_observation_s").value))
        self.stationary_s = min(
            60.0, max(10.0, float(self.get_parameter("stationary_s").value)))
        self.hover_s = min(
            60.0, max(5.0, float(self.get_parameter("hover_s").value)))
        self.flight_altitude_m = min(
            1.0, max(0.5, float(
                self.get_parameter("flight_altitude_m").value)))
        self.distance_m = min(
            1.0, max(0.25, float(self.get_parameter("distance_m").value)))
        self.rectangle_x_m = min(
            1.0, max(0.25, float(
                self.get_parameter("rectangle_x_m").value)))
        self.rectangle_y_m = min(
            0.75, max(0.25, float(
                self.get_parameter("rectangle_y_m").value)))
        self.motion_speed_mps = min(
            0.20, max(0.05, float(
                self.get_parameter("motion_speed_mps").value)))
        self.yaw_speed_rad_s = math.radians(min(
            15.0, max(3.0, float(
                self.get_parameter("yaw_speed_deg_s").value))))
        self.motion_hold_s = min(
            10.0, max(2.0, float(
                self.get_parameter("motion_hold_s").value)))
        self.landing_wait_s = min(
            60.0, max(10.0, float(
                self.get_parameter("landing_wait_s").value)))
        self.stage_pub = self.create_publisher(
            String, str(self.get_parameter("stage_topic").value), 20)
        self.last_commanded_position = None
        self.last_commanded_yaw = 0.0

    def stage(self, name):
        message = String()
        message.data = f"{self.profile}:{name}"
        self.stage_pub.publish(message)
        rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f"ROBUSTNESS_STAGE {message.data}")

    def hold(self, position, seconds, yaw, label):
        self.stage(label)
        self.last_commanded_position = position
        self.last_commanded_yaw = yaw
        self.hold_setpoint(
            *position, seconds=seconds, yaw=yaw, label=label,
            require_guided=True)

    def move(self, start, goal, yaw, label):
        self.stage(label)
        distance = math.sqrt(sum(
            (goal[index] - start[index]) ** 2 for index in range(3)))
        duration = max(distance / self.motion_speed_mps, 1.0)
        steps = max(1, int(duration * self.rate_hz))
        self.get_logger().info(
            f"{label}: {start} -> {goal}, speed={self.motion_speed_mps:.2f}m/s")
        for index in range(steps + 1):
            self.ensure_guided(label)
            fraction = index / float(steps)
            position = tuple(
                start[axis] + (goal[axis] - start[axis]) * fraction
                for axis in range(3))
            self.publish_setpoint(*position, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            time.sleep(1.0 / self.rate_hz)
        self.last_commanded_position = goal
        self.last_commanded_yaw = yaw
        self.hold_setpoint(
            *goal, seconds=self.motion_hold_s, yaw=yaw,
            label=f"{label}_hold", require_guided=True)

    def rotate(self, position, start_yaw, goal_yaw, label):
        self.stage(label)
        delta = wrap_angle(goal_yaw - start_yaw)
        duration = max(abs(delta) / self.yaw_speed_rad_s, 1.0)
        steps = max(1, int(duration * self.rate_hz))
        self.get_logger().info(
            f"{label}: yaw {math.degrees(start_yaw):.1f} -> "
            f"{math.degrees(goal_yaw):.1f} deg")
        for index in range(steps + 1):
            self.ensure_guided(label)
            yaw = start_yaw + delta * index / float(steps)
            self.publish_setpoint(*position, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            time.sleep(1.0 / self.rate_hz)
        self.last_commanded_position = position
        self.last_commanded_yaw = goal_yaw
        self.hold_setpoint(
            *position, seconds=self.motion_hold_s, yaw=goal_yaw,
            label=f"{label}_hold", require_guided=True)

    def land(self):
        self.stage("land")
        if not self.state.armed:
            self.get_logger().info("Vehicle is already disarmed.")
            return
        if not self.land_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MAVROS land service is not available")
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = self.last_commanded_yaw
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0
        response = self.call(self.land_cli, request, "progressive visual land")
        if not bool(getattr(response, "success", False)):
            raise RuntimeError(f"Landing command rejected: {response}")
        deadline = time.monotonic() + self.landing_wait_s
        while rclpy.ok() and self.state.armed and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._log_status("landing")
        if self.state.armed:
            raise RuntimeError(
                "LAND was accepted but the vehicle did not disarm before timeout")

    def prepare_flight(self):
        self.stage("pre_ground_observation")
        self.spin_without_setpoints(
            self.pre_observation_s, label="pre-ground observation")
        self.stage("guided_arm_takeoff")
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()
        start = (
            self.home_x,
            self.home_y,
            max(self.takeoff_alt, self.home_z + self.flight_altitude_m),
        )
        initial = (self.home_x, self.home_y, self.takeoff_alt)
        self.ensure_guided("post-takeoff")
        self.hold_setpoint(
            *initial, seconds=2.0, yaw=0.0,
            label="initial takeoff hold", require_guided=True)
        if start[2] > initial[2] + 0.01:
            self.move(initial, start, 0.0, "slow_climb")
        self.hold(start, self.motion_hold_s, 0.0, "pre_motion_hover")
        return start

    def finish_flight(self, start):
        current = self.last_commanded_position or start
        if math.dist(current, start) > 0.05:
            self.move(current, start, self.last_commanded_yaw, "safe_return")
        if abs(wrap_angle(self.last_commanded_yaw)) > math.radians(2.0):
            self.rotate(start, self.last_commanded_yaw, 0.0, "restore_start_yaw")
        self.hold(start, self.post_observation_s, 0.0, "post_motion_observation")
        self.land()
        self.stage("complete")
        self.spin_without_setpoints(0.5, label="complete")

    def run(self):
        self.wait_ready()
        navigation_source = self.wait_navigation_ready()
        if navigation_source != "gps":
            raise RuntimeError(
                "Progressive profiles require GPS/GUIDED; RTAB remains evaluation-only")
        if self.profile == "stationary":
            self.stage("pre_observation")
            self.spin_without_setpoints(
                self.pre_observation_s, label="stationary pre-observation")
            self.stage("stationary")
            self.spin_without_setpoints(
                self.stationary_s, label="stationary measurement")
            self.stage("post_observation")
            self.spin_without_setpoints(
                self.post_observation_s, label="stationary post-observation")
            self.stage("complete")
            return

        start = self.prepare_flight()
        try:
            self._run_flight_profile(start)
            self.finish_flight(start)
        except Exception:
            if self.state.armed:
                try:
                    self.land()
                except Exception as land_error:
                    self.get_logger().error(f"Safety landing also failed: {land_error}")
            raise

    def _run_flight_profile(self, start):
        x, y, z = start
        if self.profile == "hover":
            self.hold(start, self.hover_s, 0.0, "hover_measurement")
            return
        if self.profile in ("yaw_30", "yaw_90"):
            degrees = 30.0 if self.profile == "yaw_30" else 90.0
            target = math.radians(degrees)
            self.rotate(start, 0.0, target, "yaw_out")
            self.rotate(start, target, 0.0, "yaw_return")
            return
        if self.profile == "straight":
            forward = (x + self.distance_m, y, z)
            self.move(start, forward, 0.0, "straight_out")
            self.move(forward, start, 0.0, "straight_return")
            return

        p1 = (x + self.rectangle_x_m, y, z)
        p2 = (x + self.rectangle_x_m, y + self.rectangle_y_m, z)
        if self.profile == "l_shape":
            self.move(start, p1, 0.0, "l_first_leg")
            self.move(p1, p2, 0.0, "l_second_leg")
            self.move(p2, p1, 0.0, "l_second_return")
            self.move(p1, start, 0.0, "l_first_return")
            return
        if self.profile == "single_corner":
            self.move(start, p1, 0.0, "corner_approach")
            self.rotate(p1, 0.0, math.pi / 2.0, "single_corner_turn")
            self.move(p1, p2, math.pi / 2.0, "corner_exit")
            self.move(p2, start, math.pi / 2.0, "corner_direct_return")
            return

        p3 = (x, y + self.rectangle_y_m, z)
        if self.profile == "small_rectangle":
            for beginning, goal, label in (
                    (start, p1, "rectangle_leg_1"),
                    (p1, p2, "rectangle_leg_2"),
                    (p2, p3, "rectangle_leg_3"),
                    (p3, start, "rectangle_leg_4")):
                self.move(beginning, goal, 0.0, label)
            return

        # loop_return deliberately revisits the start with its original view.
        self.move(start, p1, 0.0, "loop_leg_1")
        self.rotate(p1, 0.0, math.pi / 2.0, "loop_view_change_1")
        self.move(p1, p2, math.pi / 2.0, "loop_leg_2")
        self.rotate(p2, math.pi / 2.0, math.pi, "loop_view_change_2")
        self.move(p2, p3, math.pi, "loop_leg_3")
        self.rotate(p3, math.pi, -math.pi / 2.0, "loop_view_change_3")
        self.move(p3, start, -math.pi / 2.0, "loop_return_to_start")
        self.rotate(start, -math.pi / 2.0, 0.0, "loop_restore_start_view")
        self.hold(start, self.motion_hold_s * 2.0, 0.0, "loop_revisit_hold")


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def main(args=None):
    rclpy.init(args=args)
    node = ProgressiveVisualMotion()
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
