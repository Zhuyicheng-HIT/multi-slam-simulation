"""Controlled FCU-frame vertical profile for estimator diagnostics only."""

from __future__ import annotations

import math
import time

import rclpy
from mavros_msgs.srv import CommandTOL

from .guided_rectangle_waypoints import GuidedRectangleWaypoints


class GuidedVerticalProfile(GuidedRectangleWaypoints):
    """Fly an audited clear corridor without using estimator feedback.

    This node deliberately uses FCU-local setpoints to provide a repeatable
    vertical excitation. It is not a navigation-closure acceptance route; the
    unified estimator is evaluated independently against Gazebo truth.
    """

    def __init__(self):
        super().__init__(node_name="guided_vertical_profile")
        self.declare_parameter("staging_x_m", -4.3)
        self.declare_parameter("staging_y_m", 0.9)
        self.declare_parameter("peak_altitude_m", 9.5)
        self.staging_x_m = float(self.get_parameter("staging_x_m").value)
        self.staging_y_m = float(self.get_parameter("staging_y_m").value)
        self.peak_altitude_m = float(
            self.get_parameter("peak_altitude_m").value
        )
        values = (
            self.staging_x_m,
            self.staging_y_m,
            self.peak_altitude_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("vertical profile parameters must be finite")
        if self.peak_altitude_m <= self.takeoff_alt:
            raise ValueError("peak_altitude_m must exceed takeoff_alt")

    def _land_and_wait(self):
        self._publish_mission_phase("landing")
        if not self.land_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("LAND service is unavailable")
        request = CommandTOL.Request()
        response = self.call(self.land_cli, request, "land")
        if not bool(getattr(response, "success", False)):
            raise RuntimeError("LAND command was rejected by the FCU")
        deadline = time.monotonic() + self.land_disarm_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._log_status("landing descent")
            if not self.state.armed:
                self.get_logger().info("LAND completed and FCU disarm confirmed.")
                return
        raise RuntimeError(
            "LAND was accepted but FCU did not disarm within "
            f"{self.land_disarm_timeout_s:.1f}s"
        )

    def run(self):
        self._publish_mission_phase("preflight")
        self.wait_ready()
        navigation_source = self.wait_navigation_ready()
        self.get_logger().warning(
            "Vertical estimator diagnostic uses FCU-local setpoints only; "
            "it does not count as unified-navigation closure."
        )
        self.get_logger().info(
            f"Preflight accepted using {navigation_source}; starting vertical "
            "profile."
        )
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()
        self.ensure_guided("post-takeoff")

        home = (self.home_x, self.home_y, self.takeoff_alt)
        staging = (
            self.home_x + self.staging_x_m,
            self.home_y + self.staging_y_m,
            self.takeoff_alt,
        )
        peak = (staging[0], staging[1], self.peak_altitude_m)
        self._publish_mission_phase("post_takeoff_hold")
        self.hold_setpoint(
            *home,
            seconds=self.post_takeoff_hold_time_s,
            yaw=self.home_yaw,
            label="vertical diagnostic post-takeoff hold",
            require_guided=True,
        )
        self._publish_mission_phase("diagnostic_vertical_profile")
        self.fly_segment(home, staging, self.home_yaw, "clear-corridor entry")
        self.fly_segment(staging, peak, self.home_yaw, "vertical ascent")
        self.fly_segment(peak, staging, self.home_yaw, "vertical descent")
        self.fly_segment(staging, home, self.home_yaw, "clear-corridor return")
        self._land_and_wait()


def main(args=None):
    rclpy.init(args=args)
    node = GuidedVerticalProfile()
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
