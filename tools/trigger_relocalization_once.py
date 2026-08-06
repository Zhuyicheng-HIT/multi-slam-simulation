#!/usr/bin/env python3
"""Trigger one relocalization transaction on the ROS simulation clock."""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String
from uf_interfaces.msg import FusionEpoch, RelocalizationResult


def stamp_seconds(message):
    return (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) * 1.0e-9
    )


class RelocalizationTrigger(Node):
    def __init__(self, topic, phase_topic):
        super().__init__("relocalization_validation_trigger")
        self.publisher = self.create_publisher(Bool, topic, 10)
        self.result = None
        self.epoch = None
        self.phase = None
        self.create_subscription(
            RelocalizationResult,
            "/relocalization/result",
            self._result,
            10,
        )
        self.create_subscription(
            FusionEpoch,
            "/fusion/unified/epoch",
            self._epoch,
            10,
        )
        self.create_subscription(String, phase_topic, self._phase, 10)

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

    def publish_request(self):
        message = Bool()
        message.data = True
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    trigger = parser.add_mutually_exclusive_group(required=True)
    trigger.add_argument("--after", type=float)
    trigger.add_argument("--wait-for-phase")
    parser.add_argument("--phase-topic", default="/mission/phase")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--wall-timeout", type=float, default=300.0)
    parser.add_argument("--topic", default="/relocalization/request")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    node = RelocalizationTrigger(args.topic, args.phase_topic)
    first_ros_s = None
    trigger_ros_s = None
    publish_count = 0
    started_wall_s = time.monotonic()
    success = False
    reason = "wall_timeout"

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now_wall_s = time.monotonic()
            now_ros_s = node.get_clock().now().nanoseconds * 1.0e-9
            if now_ros_s > 0.0 and first_ros_s is None:
                first_ros_s = now_ros_s

            elapsed_trigger = (
                args.after is not None
                and first_ros_s is not None
                and now_ros_s - first_ros_s >= args.after
            )
            phase_trigger = (
                args.wait_for_phase is not None
                and node.phase == args.wait_for_phase
            )
            if trigger_ros_s is None and (elapsed_trigger or phase_trigger):
                # Ignore terminal messages from an earlier automatic request;
                # this report must prove the transaction initiated here.
                node.result = None
                node.epoch = None
                node.publish_request()
                trigger_ros_s = now_ros_s
                publish_count = 1

            if node.epoch is not None and trigger_ros_s is not None:
                matching_result = (
                    node.result is not None
                    and node.result.accepted
                    and node.result.transaction_id == node.epoch.transaction_id
                    and node.result.candidate_id == node.epoch.candidate_id
                )
                if matching_result:
                    success = True
                    reason = "fusion_epoch_committed"
                    break

            if (
                node.result is not None
                and node.result.state == RelocalizationResult.FAILED
                and trigger_ros_s is not None
                and stamp_seconds(node.result) >= trigger_ros_s
            ):
                reason = f"relocalization_failed:{node.result.reason}"
                break

            if (
                trigger_ros_s is not None
                and now_ros_s - trigger_ros_s >= args.timeout
            ):
                reason = "fusion_epoch_timeout"
                break
            if now_wall_s - started_wall_s >= args.wall_timeout:
                break
    finally:
        result = node.result
        epoch = node.epoch
        report = {
            "success": success,
            "reason": reason,
            "trigger_after_sim_s": args.after,
            "trigger_phase": args.wait_for_phase,
            "observed_phase": node.phase,
            "trigger_ros_s": trigger_ros_s,
            "publish_count": publish_count,
            "result_state": None if result is None else result.state_name,
            "result_accepted": None if result is None else bool(result.accepted),
            "result_candidate_id": None if result is None else int(result.candidate_id),
            "result_transaction_id": (
                None if result is None else int(result.transaction_id)
            ),
            "result_reason": None if result is None else str(result.reason),
            "epoch_reset_counter": None if epoch is None else int(epoch.reset_counter),
            "epoch_session_id": None if epoch is None else int(epoch.session_id),
            "epoch_transaction_id": (
                None if epoch is None else int(epoch.transaction_id)
            ),
            "epoch_candidate_id": None if epoch is None else int(epoch.candidate_id),
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
