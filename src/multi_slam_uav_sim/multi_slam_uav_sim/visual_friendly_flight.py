#!/usr/bin/env python3
import math
import time

import rclpy
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import String

from multi_slam_uav_sim.guided_rectangle_waypoints import GuidedRectangleWaypoints


class VisualFriendlyFlight(GuidedRectangleWaypoints):
    """A short GPS/GUIDED motion sequence designed for visual evaluation."""

    def __init__(self):
        super().__init__()
        self.declare_parameter("ground_static_s", 10.0)
        self.declare_parameter("climb_height_m", 0.5)
        self.declare_parameter("vertical_speed_mps", 0.10)
        self.declare_parameter("horizontal_distance_m", 0.5)
        self.declare_parameter("horizontal_speed_mps", 0.10)
        self.declare_parameter("visual_hold_s", 5.0)
        self.declare_parameter("landing_wait_s", 20.0)
        self.declare_parameter("fixed_yaw_rad", 0.0)
        self.declare_parameter("stage_topic", "/d435i_visual_slam/stage")

        self.ground_static_s = max(
            10.0, float(self.get_parameter("ground_static_s").value))
        self.climb_height_m = max(
            0.25, float(self.get_parameter("climb_height_m").value))
        self.vertical_speed_mps = min(0.25, max(
            0.05, float(self.get_parameter("vertical_speed_mps").value)))
        self.horizontal_distance_m = min(1.0, max(
            0.1, float(self.get_parameter("horizontal_distance_m").value)))
        self.horizontal_speed_mps = min(0.25, max(
            0.05, float(self.get_parameter("horizontal_speed_mps").value)))
        self.visual_hold_s = max(
            3.0, float(self.get_parameter("visual_hold_s").value))
        self.landing_wait_s = min(
            60.0, max(5.0, float(
                self.get_parameter("landing_wait_s").value)))
        self.fixed_yaw = float(self.get_parameter("fixed_yaw_rad").value)
        self.stage_pub = self.create_publisher(
            String, str(self.get_parameter("stage_topic").value), 10)

    def stage(self, name):
        message = String()
        message.data = name
        self.stage_pub.publish(message)
        rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f"VISUAL_FLIGHT_STAGE {name}")

    def fly_limited_segment(self, start, goal, speed_mps, label):
        sx, sy, sz = start
        gx, gy, gz = goal
        distance = math.sqrt(
            (gx - sx) ** 2 + (gy - sy) ** 2 + (gz - sz) ** 2)
        duration = max(distance / max(speed_mps, 0.01), 1.0)
        steps = max(1, int(duration * self.rate_hz))
        self.stage(label)
        self.get_logger().info(
            f"{label}: ({sx:.2f},{sy:.2f},{sz:.2f}) -> "
            f"({gx:.2f},{gy:.2f},{gz:.2f}) at {speed_mps:.2f}m/s")
        for index in range(steps + 1):
            self.ensure_guided(label)
            fraction = index / float(steps)
            self.publish_setpoint(
                sx + (gx - sx) * fraction,
                sy + (gy - sy) * fraction,
                sz + (gz - sz) * fraction,
                self.fixed_yaw,
            )
            rclpy.spin_once(self, timeout_sec=0.0)
            self._log_status(label)
            time.sleep(1.0 / self.rate_hz)

    def run(self):
        self.wait_ready()
        navigation_source = self.wait_navigation_ready()
        if navigation_source != "gps":
            raise RuntimeError(
                "Visual-friendly baseline flight requires GPS navigation; "
                "RTAB-Map remains evaluation-only.")

        self.stage("A_ground_static")
        self.spin_without_setpoints(
            self.ground_static_s, label="A ground static")

        self.stage("B_guided_arm_initial_takeoff")
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()

        initial = (self.home_x, self.home_y, self.takeoff_alt)
        final_height = self.home_z + self.climb_height_m
        final_height = max(final_height, self.takeoff_alt)
        self.ensure_guided("initial takeoff")
        self.hold_setpoint(
            *initial, seconds=2.0, yaw=self.fixed_yaw,
            label="initial takeoff hold", require_guided=True)

        climb_goal = (self.home_x, self.home_y, final_height)
        self.fly_limited_segment(
            initial, climb_goal, self.vertical_speed_mps, "B_slow_climb")

        self.stage("C_hover_after_climb")
        self.hold_setpoint(
            *climb_goal, seconds=self.visual_hold_s, yaw=self.fixed_yaw,
            label="C hover", require_guided=True)

        forward_goal = (
            self.home_x + self.horizontal_distance_m,
            self.home_y,
            final_height,
        )
        self.fly_limited_segment(
            climb_goal, forward_goal, self.horizontal_speed_mps,
            "D_slow_forward")

        self.stage("E_forward_hover")
        self.hold_setpoint(
            *forward_goal, seconds=self.visual_hold_s, yaw=self.fixed_yaw,
            label="E hover", require_guided=True)

        self.fly_limited_segment(
            forward_goal, climb_goal, self.horizontal_speed_mps,
            "F_slow_return")
        self.stage("F_return_hover")
        self.hold_setpoint(
            *climb_goal, seconds=2.0, yaw=self.fixed_yaw,
            label="F return hover", require_guided=True)

        self.stage("G_land")
        if not self.land_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MAVROS land service is not available")
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = self.fixed_yaw
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0
        response = self.call(self.land_cli, request, "visual-friendly land")
        if not bool(getattr(response, "success", False)):
            raise RuntimeError(f"Landing command rejected: {response}")
        landing_deadline = time.monotonic() + self.landing_wait_s
        while (rclpy.ok() and self.state.armed and
               time.monotonic() < landing_deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
            self._log_status("G landing")
        if self.state.armed:
            self.get_logger().warning(
                "LAND was accepted but the vehicle did not disarm before the "
                f"{self.landing_wait_s:.1f}s observation timeout")
        else:
            self.get_logger().info("Landing complete; vehicle is disarmed.")
        self.stage("complete")
        self.spin_without_setpoints(0.25, label="complete")


def main(args=None):
    rclpy.init(args=args)
    node = VisualFriendlyFlight()
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


if __name__ == "__main__":
    main()
