#!/usr/bin/env python3
"""Closed-loop kinematic ROS smoke for Raw-MID360 local avoidance."""

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
from uf_interfaces.msg import FlightCommandDecision, LocalAvoidanceStatus


class AvoidanceSmoke(Node):
    def __init__(self):
        super().__init__("local_avoidance_ros_smoke")
        self.raw_pub = self.create_publisher(CustomMsg, "/livox/lidar", qos_profile_sensor_data)
        self.unified_pub = self.create_publisher(
            Odometry, "/fusion/unified/odom", qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/mavros/local_position/odom", qos_profile_sensor_data
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", qos_profile_sensor_data
        )
        self.mission_pub = self.create_publisher(
            PoseStamped, "/autonomy/intent/mission/pose", 10
        )
        self.fcu_pub = self.create_publisher(State, "/mavros/state", 10)
        self.create_subscription(
            PoseStamped, "/mavros/setpoint_position/local", self._setpoint, 10
        )
        self.create_subscription(
            LocalAvoidanceStatus, "/safety/local_avoidance_status", self._status, 10
        )
        self.create_subscription(
            FlightCommandDecision, "/autonomy/command_decision", self._decision, 10
        )
        self.position = [0.0, 0.0, 1.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.command = None
        self.states = []
        self.owners = []
        self.maximum_replans = 0
        self.reasons = []

    def _setpoint(self, message):
        self.command = [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ]

    def _status(self, message):
        if not self.states or self.states[-1] != int(message.state):
            self.states.append(int(message.state))
        self.maximum_replans = max(self.maximum_replans, int(message.replan_count))
        if not self.reasons or self.reasons[-1] != message.reason:
            self.reasons.append(message.reason)

    def _decision(self, message):
        self.owners.append(message.owner)

    def publish_inputs(self, wall_points, goal_x=2.0):
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = (
            self.velocity
        )
        self.odom_pub.publish(odom)
        self.unified_pub.publish(odom)

        mission = PoseStamped()
        mission.header = pose.header
        mission.pose.position.x = float(goal_x)
        mission.pose.position.y = 0.0
        mission.pose.position.z = 1.0
        mission.pose.orientation.w = 1.0
        self.mission_pub.publish(mission)

        fcu = State()
        fcu.connected = True
        fcu.mode = "GUIDED"
        self.fcu_pub.publish(fcu)

        raw = CustomMsg()
        raw.header.stamp = stamp
        raw.header.frame_id = "livox_frame"
        points = list(wall_points)
        while len(points) < 40:
            points.append((5.0, 2.0 + 0.02 * len(points)))
        for index, (world_x, world_y) in enumerate(points):
            point = CustomPoint()
            point.offset_time = index * 1000
            point.x = float(world_x - self.position[0])
            point.y = float(world_y - self.position[1])
            point.z = 0.0
            point.reflectivity = 100
            point.line = index % 4
            raw.points.append(point)
        raw.point_num = len(raw.points)
        self.raw_pub.publish(raw)

    def advance(self, dt):
        self.velocity = [0.0, 0.0, 0.0]
        if self.command is None:
            return
        delta = [self.command[i] - self.position[i] for i in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        if distance < 1.0e-6:
            return
        speed = 0.7
        step = min(distance, speed * dt)
        self.velocity = [value / distance * speed for value in delta]
        for index in range(3):
            self.position[index] += delta[index] / distance * step


def main():
    rclpy.init()
    node = AvoidanceSmoke()
    wall = [(1.0, -0.45 + 0.05 * index) for index in range(19)]
    collision = False
    started = time.monotonic()
    cycle = 0
    try:
        while time.monotonic() - started < 12.0:
            # Establish a healthy NAVIGATING state before introducing the
            # deterministic obstacle.  This exercises the complete blocked ->
            # brake -> replan -> verify -> resume contract, rather than the
            # separate startup fail-closed recovery path.
            # Hold the initial goal at the current pose for three deterministic
            # input cycles.  Then atomically reveal the wall and original goal,
            # avoiding any scheduler-dependent pre-obstacle motion.
            initialized = cycle >= 3
            node.publish_inputs(wall if initialized else [], 2.0 if initialized else 0.0)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.01)
            node.advance(0.08)
            for world_x, world_y in wall:
                if math.hypot(node.position[0] - world_x, node.position[1] - world_y) < 0.64:
                    collision = True
            if math.dist(node.position, [2.0, 0.0, 1.0]) < 0.25:
                break
            time.sleep(0.03)
            cycle += 1

        publishers = node.get_publishers_info_by_topic("/mavros/setpoint_position/local")
        publisher_names = sorted({item.node_name for item in publishers})
        required_sequence = [
            LocalAvoidanceStatus.PATH_BLOCKED,
            LocalAvoidanceStatus.BRAKE_HOLD,
            LocalAvoidanceStatus.REPLAN,
            LocalAvoidanceStatus.TRAJECTORY_VERIFY,
            LocalAvoidanceStatus.RESUME,
        ]
        cursor = 0
        for state in node.states:
            if cursor < len(required_sequence) and state == required_sequence[cursor]:
                cursor += 1
        result = {
            "collision": collision,
            "goal_reached": math.dist(node.position, [2.0, 0.0, 1.0]) < 0.25,
            "planner_owned": "local_planner" in node.owners,
            "replan_count": node.maximum_replans,
            "final_position": node.position,
            "reasons": node.reasons[-12:],
            "single_setpoint_owner": publisher_names == ["flight_command_arbiter"],
            "setpoint_publishers": publisher_names,
            "state_sequence_complete": cursor == len(required_sequence),
            "states": node.states,
            "wall_time_s": time.monotonic() - started,
        }
        print(json.dumps(result, sort_keys=True))
        if (
            result["collision"]
            or not result["goal_reached"]
            or not result["planner_owned"]
            or not result["single_setpoint_owner"]
            or not result["state_sequence_complete"]
        ):
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
