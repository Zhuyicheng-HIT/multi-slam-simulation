
#!/usr/bin/env python3
"""Bounded GPS/GUIDED positioning for cross-session relocalization."""

import json
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from multi_slam_uav_sim.d435i_speed_envelope_motion import (
    D435iSpeedEnvelopeMotion,
    wrap_angle,
)


class D435iRelocalizationMotion(D435iSpeedEnvelopeMotion):
    """Move before RTAB-Map starts, then expose a bounded observation window."""

    def __init__(self):
        super().__init__()
        self.declare_parameter("condition", "start_same")
        self.declare_parameter("target_x_m", 0.0)
        self.declare_parameter("target_y_m", 0.0)
        self.declare_parameter("target_z_m", 0.5)
        self.declare_parameter("target_yaw_rad", 0.0)
        self.declare_parameter("observation_sweep_deg", 12.0)
        self.declare_parameter("observation_hold_s", 8.0)
        self.declare_parameter("control_timeout_s", 120.0)
        self.declare_parameter(
            "relocalization_stage_topic", "/d435i_cross_session/stage")
        self.declare_parameter(
            "relocalization_ready_topic", "/d435i_cross_session/ready")
        self.declare_parameter(
            "relocalization_control_topic", "/d435i_cross_session/control")

        self.condition = str(self.get_parameter("condition").value)
        self.target_offset = (
            float(self.get_parameter("target_x_m").value),
            float(self.get_parameter("target_y_m").value),
            min(0.90, max(0.40, float(
                self.get_parameter("target_z_m").value))),
        )
        self.target_yaw = wrap_angle(float(
            self.get_parameter("target_yaw_rad").value))
        self.sweep_rad = math.radians(min(25.0, max(
            5.0, float(self.get_parameter("observation_sweep_deg").value))))
        self.observation_hold_s = min(30.0, max(
            5.0, float(self.get_parameter("observation_hold_s").value)))
        self.control_timeout_s = min(240.0, max(
            30.0, float(self.get_parameter("control_timeout_s").value)))
        self.control_command = ""

        durable = QoSProfile(depth=10)
        durable.reliability = ReliabilityPolicy.RELIABLE
        durable.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.relocal_stage_pub = self.create_publisher(
            String, str(self.get_parameter(
                "relocalization_stage_topic").value), durable)
        self.ready_pub = self.create_publisher(
            String, str(self.get_parameter(
                "relocalization_ready_topic").value), durable)
        self.create_subscription(
            String, str(self.get_parameter(
                "relocalization_control_topic").value),
            self._control_cb, 10)
        self.get_logger().info(
            "Cross-session positioning: "
            f"condition={self.condition} offset={self.target_offset} "
            f"yaw={math.degrees(self.target_yaw):.1f}deg")

    def _control_cb(self, message):
        self.control_command = str(message.data).strip().lower()

    def relocal_stage(self, name, **details):
        payload = {
            "condition": self.condition,
            "stage": name,
            "wall_monotonic_ns": time.monotonic_ns(),
            **details,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.relocal_stage_pub.publish(message)
        rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f"RELOCALIZATION_STAGE {message.data}")

    def wait_for_observe(self, target):
        self.relocal_stage("position_ready", target=list(target),
                           target_yaw_rad=self.target_yaw)
        ready = String()
        ready.data = json.dumps({
            "condition": self.condition,
            "target": list(target),
            "target_yaw_rad": self.target_yaw,
        }, sort_keys=True)
        deadline = time.monotonic() + self.control_timeout_s
        next_ready = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_ready:
                self.ready_pub.publish(ready)
                next_ready = now + 1.0
            self.ensure_guided("waiting for relocalization observer")
            self.publish_setpoint(*target, self.target_yaw)
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.control_command == "observe":
                return
            if self.control_command == "abort":
                raise RuntimeError("Observation was aborted by the runner")
            time.sleep(max(0.0, 1.0 / self.rate_hz - 0.05))
        raise RuntimeError("Timed out waiting for the observe control command")

    def observe(self, target):
        self.relocal_stage("observation_started")
        self.hold(target, 2.0, self.target_yaw, "relocalization_settle")
        left = wrap_angle(self.target_yaw + self.sweep_rad)
        right = wrap_angle(self.target_yaw - self.sweep_rad)
        self.rotate(target, self.target_yaw, left,
                    "relocalization_sweep_left")
        self.rotate(target, left, right, "relocalization_sweep_right")
        self.rotate(target, right, self.target_yaw,
                    "relocalization_restore_view")
        self.hold(target, self.observation_hold_s, self.target_yaw,
                  "relocalization_stable_observation")
        self.relocal_stage("observation_complete")

    def wait_for_return(self, target):
        """Hold the evaluated pose while the runner freezes RTAB evidence."""
        self.control_command = ""
        self.relocal_stage("awaiting_evidence_freeze")
        deadline = time.monotonic() + self.control_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.ensure_guided("waiting for evidence freeze")
            self.publish_setpoint(*target, self.target_yaw)
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.control_command == "return":
                self.relocal_stage("return_authorized")
                return
            if self.control_command == "abort":
                raise RuntimeError("Return was aborted by the runner")
            time.sleep(max(0.0, 1.0 / self.rate_hz - 0.05))
        raise RuntimeError("Timed out waiting for evidence-freeze completion")

    def run(self):
        self.wait_ready()
        if self.wait_navigation_ready() != "gps":
            raise RuntimeError(
                "Cross-session positioning requires the existing GPS/GUIDED source")
        self.relocal_stage("pre_takeoff")
        self.set_guided_arm_takeoff()
        self.wait_for_takeoff_climb()
        takeoff = (self.home_x, self.home_y, self.takeoff_alt)
        self._check_position(takeoff)
        self.hold(takeoff, 2.0, 0.0, "relocalization_takeoff_hold")
        target = (
            self.home_x + self.target_offset[0],
            self.home_y + self.target_offset[1],
            self.home_z + self.target_offset[2],
        )
        self._check_position(target)
        try:
            if math.dist(takeoff, target) > 0.02:
                self.move(takeoff, target, 0.0, "position_target",
                          speed=min(self.horizontal_speed_mps, 0.20))
            if abs(wrap_angle(self.target_yaw)) > math.radians(1.0):
                self.rotate(target, 0.0, self.target_yaw,
                            "orient_target")
            else:
                self.hold(target, 2.0, self.target_yaw,
                          "position_target_hold")
            self.wait_for_observe(target)
            self.observe(target)
            self.wait_for_return(target)
            if abs(wrap_angle(self.target_yaw)) > math.radians(1.0):
                self.rotate(target, self.target_yaw, 0.0,
                            "restore_return_yaw")
            if math.dist(target, takeoff) > 0.02:
                self.move(target, takeoff, 0.0, "safe_return",
                          speed=min(self.horizontal_speed_mps, 0.20))
            self.hold(takeoff, 2.0, 0.0, "pre_land_hold")
            self.last_commanded_position = takeoff
            self.last_commanded_yaw = 0.0
            self.land()
            self.relocal_stage("complete")
        except Exception:
            self.relocal_stage("failed")
            if self.state.armed:
                try:
                    self.last_commanded_yaw = 0.0
                    self.land()
                except Exception as land_error:
                    self.get_logger().error(
                        f"Safety landing failed: {land_error}")
            raise


def main(args=None):
    rclpy.init(args=args)
    node = D435iRelocalizationMotion()
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
