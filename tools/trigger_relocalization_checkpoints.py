#!/usr/bin/env python3
"""Trigger and verify relocalization at selected mission checkpoints."""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String
from uf_interfaces.msg import FusionEpoch, RelocalizationResult

from multi_slam_uav_sim.relocalization_checkpoints import (
    decode_checkpoint,
    parse_checkpoint_indices,
)
from multi_slam_uav_sim.relocalization_motion import (
    MotionCommand,
    decode_motion_status,
    encode_motion_command,
    motion_observations,
    normalize_motion_profile,
)


def stamp_seconds(message):
    return (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) * 1.0e-9
    )


class CheckpointRelocalizationTrigger(Node):
    def __init__(
        self,
        request_topic,
        checkpoint_topic,
        phase_topic,
        motion_command_topic,
        motion_status_topic,
    ):
        super().__init__("checkpoint_relocalization_validation_trigger")
        self.publisher = self.create_publisher(Bool, request_topic, 10)
        self.checkpoints = {}
        self.result = None
        self.epoch = None
        self.phase = None
        self.motion_status = None
        self.motion_publisher = self.create_publisher(
            String, motion_command_topic, 10)
        self.create_subscription(String, checkpoint_topic, self._checkpoint, 20)
        self.create_subscription(
            RelocalizationResult, "/relocalization/result", self._result, 10)
        self.create_subscription(FusionEpoch, "/fusion/unified/epoch", self._epoch, 10)
        self.create_subscription(String, phase_topic, self._phase, 10)
        self.create_subscription(
            String, motion_status_topic, self._motion_status, 10)

    def _checkpoint(self, message):
        try:
            checkpoint = decode_checkpoint(message.data)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.checkpoints[checkpoint.index] = checkpoint

    def _result(self, message):
        if message.state in (
            RelocalizationResult.SUCCESS,
            RelocalizationResult.FAILED,
        ):
            self.result = message

    def _epoch(self, message):
        if message.applied:
            self.epoch = message

    def _phase(self, message):
        self.phase = str(message.data)

    def _motion_status(self, message):
        try:
            self.motion_status = decode_motion_status(message.data)
        except ValueError as error:
            self.get_logger().error(str(error))

    def publish_request(self, active=True):
        message = Bool()
        message.data = bool(active)
        self.publisher.publish(message)

    def publish_motion(self, sequence_id, profile, step_index, step_count):
        message = String()
        message.data = encode_motion_command(MotionCommand(
            sequence_id, profile, step_index, step_count
        ))
        self.motion_status = None
        self.motion_publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--indices", required=True)
    parser.add_argument("--checkpoint-topic", default="/mission/checkpoint")
    parser.add_argument("--phase-topic", default="/mission/phase")
    parser.add_argument("--topic", default="/relocalization/request")
    parser.add_argument("--transaction-timeout", type=float, default=20.0)
    parser.add_argument("--motion-profile", default="hold")
    parser.add_argument(
        "--motion-command-topic", default="/relocalization/motion_command")
    parser.add_argument(
        "--motion-status-topic", default="/relocalization/motion_status")
    parser.add_argument("--motion-step-timeout", type=float, default=120.0)
    parser.add_argument("--wall-timeout", type=float, default=600.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    indices = parse_checkpoint_indices(args.indices)
    motion_profile = normalize_motion_profile(args.motion_profile)
    motion_step_count = len(motion_observations(motion_profile))
    if args.motion_step_timeout <= 0.0:
        parser.error("--motion-step-timeout must be positive")

    rclpy.init()
    node = CheckpointRelocalizationTrigger(
        args.topic,
        args.checkpoint_topic,
        args.phase_topic,
        args.motion_command_topic,
        args.motion_status_topic,
    )
    started_wall_s = time.monotonic()
    target_offset = 0
    pending = None
    transactions = []
    motion_attempts = []
    reason = "wall_timeout"

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now_wall_s = time.monotonic()
            now_ros_s = node.get_clock().now().nanoseconds * 1.0e-9

            if pending is None and target_offset < len(indices):
                target_index = indices[target_offset]
                checkpoint = node.checkpoints.get(target_index)
                if checkpoint is not None:
                    node.result = None
                    node.epoch = None
                    sequence_id = time.monotonic_ns()
                    pending = {
                        "checkpoint": checkpoint,
                        "sequence_id": sequence_id,
                        "motion_profile": motion_profile,
                        "motion_step_index": 0,
                        "motion_step_count": motion_step_count,
                        "motion_distance_m": 0.0,
                        "motion_duration_s": 0.0,
                        "state": "search" if motion_profile == "hold" else "motion",
                        "sequence_trigger_ros_s": now_ros_s,
                        "trigger_wall_s": now_wall_s,
                        "state_wall_s": now_wall_s,
                        "search_trigger_ros_s": now_ros_s,
                        "search_trigger_wall_s": now_wall_s,
                    }
                    if motion_profile == "hold":
                        node.publish_request()
                    else:
                        node.publish_motion(
                            sequence_id, motion_profile, 0, motion_step_count)

            if pending is not None:
                if pending["state"] == "motion":
                    status = node.motion_status
                    matching_status = (
                        status is not None
                        and status.sequence_id == pending["sequence_id"]
                        and status.profile == pending["motion_profile"]
                        and status.step_index == pending["motion_step_index"]
                    )
                    if matching_status and status.state == "failed":
                        reason = f"relocalization_motion_failed:{status.reason}"
                        break
                    if matching_status and status.state == "settled":
                        pending["motion_distance_m"] += status.distance_m
                        pending["motion_duration_s"] += status.duration_s
                        pending["current_motion_status"] = status
                        pending["state"] = "search"
                        pending["search_trigger_ros_s"] = now_ros_s
                        pending["search_trigger_wall_s"] = now_wall_s
                        node.result = None
                        node.epoch = None
                        node.publish_request()
                    elif (
                        now_wall_s - pending["state_wall_s"]
                        >= args.motion_step_timeout
                    ):
                        reason = "relocalization_motion_step_timeout"
                        break
                else:
                    result = node.result
                    epoch = node.epoch
                    matching_epoch = (
                        result is not None
                        and epoch is not None
                        and result.accepted
                        and result.transaction_id == epoch.transaction_id
                        and result.candidate_id == epoch.candidate_id
                        and stamp_seconds(result)
                        >= pending["search_trigger_ros_s"]
                    )
                    if matching_epoch:
                        checkpoint = pending["checkpoint"]
                        motion_attempts.append({
                            "accepted": True,
                            "checkpoint_index": checkpoint.index,
                            "motion_profile": pending["motion_profile"],
                            "motion_step_index": pending["motion_step_index"],
                            "search_wall_s": (
                                now_wall_s - pending["search_trigger_wall_s"]
                            ),
                            "result_reason": str(result.reason),
                        })
                        transactions.append({
                            "checkpoint_index": checkpoint.index,
                            "checkpoint_label": checkpoint.label,
                            "checkpoint_distance_m": checkpoint.distance_m,
                            "trigger_ros_s": pending["sequence_trigger_ros_s"],
                            "transaction_id": int(result.transaction_id),
                            "candidate_id": int(result.candidate_id),
                            "reset_counter": int(epoch.reset_counter),
                            "recovery_wall_s": (
                                now_wall_s - pending["trigger_wall_s"]
                            ),
                            "final_search_wall_s": (
                                now_wall_s - pending["search_trigger_wall_s"]
                            ),
                            "motion_profile": pending["motion_profile"],
                            "motion_steps_executed": (
                                pending["motion_step_index"]
                                + (0 if motion_profile == "hold" else 1)
                            ),
                            "motion_distance_m": pending["motion_distance_m"],
                            "motion_duration_s": pending["motion_duration_s"],
                        })
                        node.publish_request(False)
                        target_offset += 1
                        pending = None
                        if target_offset == len(indices):
                            reason = "all_checkpoint_epochs_committed"
                            break
                    elif (
                        result is not None
                        and result.state == RelocalizationResult.FAILED
                        and stamp_seconds(result)
                        >= pending["search_trigger_ros_s"]
                    ):
                        checkpoint = pending["checkpoint"]
                        motion_attempts.append({
                            "accepted": False,
                            "checkpoint_index": checkpoint.index,
                            "motion_profile": pending["motion_profile"],
                            "motion_step_index": pending["motion_step_index"],
                            "search_wall_s": (
                                now_wall_s - pending["search_trigger_wall_s"]
                            ),
                            "result_reason": str(result.reason),
                        })
                        node.publish_request(False)
                        next_step = pending["motion_step_index"] + 1
                        if (
                            motion_profile == "hold"
                            or next_step >= pending["motion_step_count"]
                        ):
                            reason = (
                                "relocalization_observations_exhausted:"
                                + str(result.reason)
                            )
                            break
                        pending["motion_step_index"] = next_step
                        pending["state"] = "motion"
                        pending["state_wall_s"] = now_wall_s
                        node.result = None
                        node.epoch = None
                        node.publish_motion(
                            pending["sequence_id"],
                            motion_profile,
                            next_step,
                            pending["motion_step_count"],
                        )
                    elif (
                        now_ros_s - pending["search_trigger_ros_s"]
                        >= args.transaction_timeout
                    ):
                        reason = "fusion_epoch_timeout"
                        break

            if node.phase in ("landing", "complete_hold") and target_offset < len(indices):
                reason = "mission_finished_before_requested_checkpoints"
                break
            if now_wall_s - started_wall_s >= args.wall_timeout:
                break
    finally:
        node.publish_request(False)
        rclpy.spin_once(node, timeout_sec=0.1)
        success = target_offset == len(indices)
        report = {
            "success": success,
            "reason": reason,
            "requested_checkpoint_indices": list(indices),
            "completed_transactions": transactions,
            "motion_attempts": motion_attempts,
            "motion_profile": motion_profile,
            "observed_checkpoint_indices": sorted(node.checkpoints),
            "observed_phase": node.phase,
            "wall_elapsed_s": time.monotonic() - started_wall_s,
        }
        with open(args.output, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(report, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
