#!/usr/bin/env python3
"""Exercise the real ROS message chain without controlling an FCU."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from uf_interfaces.msg import FlightCommandDecision, ObstacleSafetyState


class Smoke(Node):
    def __init__(self):
        super().__init__("safety_slice_ros_smoke")
        self.raw_pub = self.create_publisher(
            CustomMsg, "/livox/lidar", qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/fusion/unified/odom", qos_profile_sensor_data
        )
        self.motion_pub = self.create_publisher(
            Odometry, "/mavros/local_position/odom", qos_profile_sensor_data
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", qos_profile_sensor_data
        )
        self.mission_pub = self.create_publisher(
            PoseStamped, "/autonomy/intent/mission/pose", 10
        )
        self.planner_pub = self.create_publisher(
            PoseStamped, "/autonomy/intent/planner/pose", 10
        )
        self.reloc_pub = self.create_publisher(
            PoseStamped, "/autonomy/intent/relocalization/pose", 10
        )
        self.fcu_pub = self.create_publisher(State, "/mavros/state", 10)
        self.hold_pub = self.create_publisher(Bool, "/safety/localization_hold", 10)
        self.create_subscription(
            ObstacleSafetyState, "/safety/raw_obstacle_state", self._state, 10
        )
        self.create_subscription(
            FlightCommandDecision, "/autonomy/command_decision", self._decision, 10
        )
        self.states = []
        self.decisions = []

    def _state(self, message):
        self.states.append(message)

    def _decision(self, message):
        self.decisions.append(message)

    def now(self):
        return self.get_clock().now().to_msg()

    def publish_common(self, speed=1.0):
        stamp = self.now()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        odom = Odometry()
        odom.header = pose.header
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = speed
        self.odom_pub.publish(odom)
        self.motion_pub.publish(odom)
        mission = PoseStamped()
        mission.header = pose.header
        mission.pose.position.x = 1.0
        mission.pose.position.z = 1.0
        mission.pose.orientation.w = 1.0
        self.mission_pub.publish(mission)
        fcu = State()
        fcu.connected = True
        fcu.mode = "GUIDED"
        self.fcu_pub.publish(fcu)

    def raw(self, obstacle_x=None):
        message = CustomMsg()
        message.header.stamp = self.now()
        message.header.frame_id = "livox_frame"
        for index in range(40):
            point = CustomPoint()
            point.offset_time = index * 1000
            point.x = float(obstacle_x if obstacle_x is not None else 5.0)
            point.y = float((index - 20) * 0.005 if obstacle_x is not None else 2.0)
            point.z = 0.0
            point.reflectivity = 100
            point.line = index % 4
            message.points.append(point)
        message.point_num = len(message.points)
        self.raw_pub.publish(message)

    def pump(self, seconds, obstacle_x=None, publish_raw=True, speed=1.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish_common(speed)
            if publish_raw:
                self.raw(obstacle_x)
            for _ in range(6):
                rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def last_state(self):
        assert self.states, "no obstacle state received"
        return self.states[-1]

    def last_decision(self):
        assert self.decisions, "no command decision received"
        return self.decisions[-1]


def main():
    rclpy.init()
    node = Smoke()
    results = {}
    try:
        node.pump(0.5)
        results["clear"] = node.last_state().state == ObstacleSafetyState.CLEAR
        results["normal_navigation_owner"] = node.last_decision().owner in (
            "mission",
            "local_planner",
        )

        node.pump(0.35, obstacle_x=0.75)
        results["wall_brake"] = node.last_state().state == ObstacleSafetyState.BRAKE
        results["obstacle_owner"] = node.last_decision().owner == "obstacle_safety"

        node.pump(0.35, publish_raw=False)
        results["dropout_hover"] = node.last_state().state == ObstacleSafetyState.HOVER_REQUIRED

        hold = Bool()
        hold.data = True
        node.hold_pub.publish(hold)
        node.pump(0.35)
        results["localization_hold"] = node.last_decision().owner == "localization_safety"
        hold.data = False
        node.hold_pub.publish(hold)

        planner = PoseStamped()
        planner.header.stamp = node.now()
        planner.pose.position.x = math.nan
        planner.pose.orientation.w = 1.0
        node.planner_pub.publish(planner)
        node.pump(0.25)
        # The production local planner now continuously refreshes a verified
        # NAVIGATING intent.  A rogue nonfinite publisher therefore either
        # loses to that newer verified intent or causes the arbiter to close;
        # it must never become the selected automatic owner.
        results["planner_nonfinite_not_forwarded"] = (
            node.last_decision().owner == "local_planner"
            or (
                node.last_decision().fail_closed
                and node.last_decision().reason == "planner_intent_stale_or_invalid"
            )
        )

        # Restart the smoke node state for the independent relocalization conflict.
        node.destroy_node()
        node = Smoke()
        node.pump(0.3)
        reloc = PoseStamped()
        reloc.header.stamp = node.now()
        reloc.pose.position.x = 0.25
        reloc.pose.position.z = 1.0
        reloc.pose.orientation.w = 1.0
        node.reloc_pub.publish(reloc)
        node.pump(0.3, obstacle_x=0.75)
        results["relocalization_blocked_by_obstacle"] = (
            node.last_decision().owner == "obstacle_safety"
        )

        publishers = node.get_publishers_info_by_topic("/mavros/setpoint_position/local")
        owners = sorted({publisher.node_name for publisher in publishers})
        results["single_setpoint_owner"] = owners == ["flight_command_arbiter"]
        results["setpoint_publishers"] = owners
        print(json.dumps(results, sort_keys=True))
        if not all(value for key, value in results.items() if key != "setpoint_publishers"):
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
