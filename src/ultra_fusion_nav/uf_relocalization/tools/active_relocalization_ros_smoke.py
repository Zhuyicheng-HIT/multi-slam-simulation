#!/usr/bin/env python3
"""Closed-loop ROS smoke for the production active-relocalization flight contract."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from uf_interfaces.msg import (
    ActiveRelocalizationStatus,
    FlightCommandDecision,
    FusionEpoch,
    RelocalizationResult,
    SchedulerState,
)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


class RelocalizationSmoke(Node):
    def __init__(self, mode):
        super().__init__("active_relocalization_ros_smoke")
        self.mode = mode
        self.raw_pub = self.create_publisher(CustomMsg, "/livox/lidar", qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/mavros/local_position/odom", qos_profile_sensor_data
        )
        self.unified_pub = self.create_publisher(
            Odometry, "/fusion/unified/odom", qos_profile_sensor_data
        )
        self.mission_pub = self.create_publisher(
            PoseStamped, "/autonomy/intent/mission/pose", 10
        )
        self.request_pub = self.create_publisher(Bool, "/relocalization/request", 10)
        self.result_pub = self.create_publisher(
            RelocalizationResult, "/relocalization/result", 10
        )
        epoch_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.epoch_pub = self.create_publisher(FusionEpoch, "/fusion/unified/epoch", epoch_qos)
        self.scheduler_pub = self.create_publisher(
            SchedulerState, "/reliability/scheduler_state", 10
        )
        self.fcu_pub = self.create_publisher(State, "/mavros/state", 10)
        self.create_subscription(
            PoseStamped, "/mavros/setpoint_position/local", self._setpoint, 10
        )
        self.create_subscription(
            FlightCommandDecision, "/autonomy/command_decision", self._decision, 10
        )
        self.create_subscription(
            ActiveRelocalizationStatus,
            "/safety/active_relocalization_status",
            self._status,
            10,
        )
        self.position = [0.0, 0.0, 1.0]
        self.yaw = 0.0
        self.command = None
        self.command_yaw = 0.0
        self.request_active = False
        self.recovered = False
        self.obstacle = False
        self.status = None
        self.states = []
        self.actions = []
        self.owners = []
        self.authorized_count = 0
        self.maximum_safe_motion_steps = 0
        self.obstacle_veto_count = 0
        self.maximum_hold_displacement = 0.0
        self.anchor = list(self.position)
        self.result_sent = False
        self.wrong_epoch_sent = False
        self.matching_epoch_sent = False
        self.recovery_seen_before_epoch = False
        self.active_started = None
        self.resume_time = None

    def _setpoint(self, message):
        self.command = [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ]
        q = message.pose.orientation
        self.command_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _decision(self, message):
        self.owners.append(message.owner)
        if message.owner == "obstacle_safety":
            self.obstacle_veto_count += 1

    def _status(self, message):
        self.status = message
        state = int(message.state)
        if not self.states or self.states[-1] != state:
            self.states.append(state)
        if not self.actions or self.actions[-1] != message.action:
            self.actions.append(message.action)
        if message.motion_authorized:
            self.authorized_count += 1
        self.maximum_safe_motion_steps = max(
            self.maximum_safe_motion_steps, int(message.safe_motion_steps_completed)
        )
        if (
            state == ActiveRelocalizationStatus.ACTIVE_RELOCALIZATION
            and self.active_started is None
        ):
            self.active_started = time.monotonic()
        if state == ActiveRelocalizationStatus.RESUME and self.resume_time is None:
            self.resume_time = time.monotonic()

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
        pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)
        self.unified_pub.publish(odom)

        mission = PoseStamped()
        mission.header = pose.header
        mission.pose.position.x = 2.0
        mission.pose.position.z = 1.0
        mission.pose.orientation.w = 1.0
        self.mission_pub.publish(mission)

        request = Bool()
        request.data = self.request_active
        self.request_pub.publish(request)

        scheduler = SchedulerState()
        scheduler.header = pose.header
        scheduler.health_state = "RECOVERED" if self.recovered else "DEGRADED"
        scheduler.capability_names = [
            "propagation",
            "horizontal_motion",
            "vertical_position",
            "yaw_tracking",
        ]
        scheduler.capability_support = [1.0, 1.0 if self.recovered else 0.0, 1.0, 1.0]
        scheduler.capability_observable = [True, self.recovered, True, True]
        scheduler.estimator_support = 0.9 if self.recovered else 0.2
        scheduler.relocalization_requested = self.request_active
        self.scheduler_pub.publish(scheduler)

        fcu = State()
        fcu.connected = True
        fcu.mode = "GUIDED"
        self.fcu_pub.publish(fcu)
        self.publish_raw(stamp)

    def publish_raw(self, stamp):
        raw = CustomMsg()
        raw.header.stamp = stamp
        raw.header.frame_id = "livox_frame"
        for index in range(40):
            point = CustomPoint()
            point.offset_time = index * 1000
            point.x = 0.62 if self.obstacle else 5.0
            point.y = (index - 20) * 0.008 if self.obstacle else 2.0 + index * 0.01
            point.z = 0.0
            point.reflectivity = 100
            point.line = index % 4
            raw.points.append(point)
        raw.point_num = len(raw.points)
        self.raw_pub.publish(raw)

    def send_result(self, accepted=True):
        result = RelocalizationResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.request_active = True
        result.accepted = accepted
        result.state = RelocalizationResult.SUCCESS if accepted else RelocalizationResult.FAILED
        result.state_name = "SUCCESS" if accepted else "FAILED"
        result.transaction_id = 42
        result.candidate_id = 7
        result.reason = "deterministic_smoke"
        self.result_pub.publish(result)

    def send_epoch(self, transaction_id, candidate_id):
        epoch = FusionEpoch()
        epoch.header.stamp = self.get_clock().now().to_msg()
        epoch.applied = True
        epoch.session_id = 1
        epoch.transaction_id = transaction_id
        epoch.candidate_id = candidate_id
        epoch.reason = "deterministic_smoke"
        self.epoch_pub.publish(epoch)

    def advance(self, dt):
        if self.command is None:
            return
        delta = [self.command[index] - self.position[index] for index in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        if distance > 1.0e-6:
            step = min(distance, 0.65 * dt)
            for index in range(3):
                self.position[index] += delta[index] / distance * step
        yaw_delta = wrap(self.command_yaw - self.yaw)
        self.yaw = wrap(self.yaw + math.copysign(min(abs(yaw_delta), 1.8 * dt), yaw_delta))
        if self.status and self.status.state in (
            ActiveRelocalizationStatus.HOLD,
            ActiveRelocalizationStatus.RECOVERY_VALIDATION,
            ActiveRelocalizationStatus.FAILSAFE,
        ):
            self.maximum_hold_displacement = max(
                self.maximum_hold_displacement, math.dist(self.position, self.anchor)
            )


def ordered_subsequence(sequence, expected):
    cursor = 0
    for value in sequence:
        if cursor < len(expected) and value == expected[cursor]:
            cursor += 1
    return cursor == len(expected)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("success", "safe_motion", "obstacle", "failure"),
        default="success",
    )
    args = parser.parse_args()
    rclpy.init()
    node = RelocalizationSmoke(args.mode)
    started = time.monotonic()
    request_time = None
    obstacle_cleared = False
    try:
        while time.monotonic() - started < 14.0:
            elapsed = time.monotonic() - started
            if elapsed > 0.8 and request_time is None:
                node.request_active = True
                request_time = time.monotonic()
                node.anchor = list(node.position)

            if (
                node.status
                and node.status.state
                == ActiveRelocalizationStatus.ACTIVE_RELOCALIZATION
            ):
                active_elapsed = time.monotonic() - node.active_started
                if args.mode == "obstacle" and active_elapsed > 0.25 and not obstacle_cleared:
                    node.obstacle = True
                    if active_elapsed > 1.25:
                        node.obstacle = False
                        obstacle_cleared = True
                if args.mode == "failure" and active_elapsed > 0.45 and not node.result_sent:
                    node.send_result(False)
                    node.result_sent = True
                safe_motion_ready = (
                    args.mode == "safe_motion" and node.maximum_safe_motion_steps >= 1
                )
                timed_success_ready = args.mode in ("success", "obstacle") and active_elapsed > (
                    1.65 if args.mode == "obstacle" else 0.8
                )
                if (safe_motion_ready or timed_success_ready) and not node.result_sent:
                    node.send_result(True)
                    node.result_sent = True

            if node.status and node.status.state == ActiveRelocalizationStatus.RECOVERY_VALIDATION:
                if not node.matching_epoch_sent:
                    node.recovery_seen_before_epoch = not node.status.epoch_committed
                if not node.wrong_epoch_sent:
                    node.send_epoch(42, 8)
                    node.wrong_epoch_sent = True
                elif time.monotonic() - started > 3.2 and not node.matching_epoch_sent:
                    node.send_epoch(42, 7)
                    node.matching_epoch_sent = True
                    node.recovered = True
                    node.request_active = False

            node.publish_inputs()
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.008)
            node.advance(0.07)
            if (
                args.mode == "failure"
                and node.status
                and node.status.state == ActiveRelocalizationStatus.FAILSAFE
            ):
                if time.monotonic() - node.active_started > 1.1:
                    break
            if (
                args.mode != "failure"
                and node.status
                and node.status.state
                == ActiveRelocalizationStatus.NORMAL_NAVIGATION
                and node.resume_time
            ):
                if math.dist(node.position, [2.0, 0.0, 1.0]) < 0.28:
                    break
            time.sleep(0.025)

        publishers = node.get_publishers_info_by_topic("/mavros/setpoint_position/local")
        setpoint_publishers = sorted({item.node_name for item in publishers})
        state_flow = ordered_subsequence(
            node.states,
            [
                ActiveRelocalizationStatus.NORMAL_NAVIGATION,
                ActiveRelocalizationStatus.HOLD,
                ActiveRelocalizationStatus.ACTIVE_RELOCALIZATION,
            ]
            + ([] if args.mode == "failure" else [
                ActiveRelocalizationStatus.RECOVERY_VALIDATION,
                ActiveRelocalizationStatus.RESUME,
                ActiveRelocalizationStatus.NORMAL_NAVIGATION,
            ]),
        )
        result = {
            "mode": args.mode,
            "state_flow": state_flow,
            "states": node.states,
            "actions": node.actions,
            "owners": sorted(set(node.owners)),
            "active_authorized_samples": node.authorized_count,
            "safe_motion_steps": node.maximum_safe_motion_steps,
            "obstacle_veto_samples": node.obstacle_veto_count,
            "wrong_epoch_did_not_release": node.recovery_seen_before_epoch,
            "matching_epoch_sent": node.matching_epoch_sent,
            "mission_resumed": "mission" in node.owners or "local_planner" in node.owners,
            "goal_reached": math.dist(node.position, [2.0, 0.0, 1.0]) < 0.28,
            "maximum_hold_displacement_m": node.maximum_hold_displacement,
            "single_setpoint_owner": setpoint_publishers == ["flight_command_arbiter"],
            "setpoint_publishers": setpoint_publishers,
            "wall_time_s": time.monotonic() - started,
            "recovery_time_s": (
                node.resume_time - request_time
                if node.resume_time is not None and request_time is not None
                else None
            ),
        }
        if args.mode == "failure":
            passed = (
                result["state_flow"]
                and node.status is not None
                and node.status.state == ActiveRelocalizationStatus.FAILSAFE
                and node.owners[-1] == "active_relocalization"
                and result["single_setpoint_owner"]
            )
        else:
            passed = (
                result["state_flow"]
                and result["active_authorized_samples"] > 0
                and result["wrong_epoch_did_not_release"]
                and result["matching_epoch_sent"]
                and result["mission_resumed"]
                and result["goal_reached"]
                and result["single_setpoint_owner"]
                and (args.mode != "obstacle" or result["obstacle_veto_samples"] > 0)
                and (args.mode != "safe_motion" or result["safe_motion_steps"] > 0)
            )
        result["passed"] = passed
        print(json.dumps(result, sort_keys=True))
        if not passed:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
