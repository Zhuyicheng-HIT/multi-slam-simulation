#!/usr/bin/env python3
"""Mask selected modalities in recorded scheduler state for replay A/B tests."""

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from uf_interfaces.msg import SchedulerState

SCHEDULER_OUTPUT_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)


def mask_scheduler_state(message, disabled_modalities):
    output = SchedulerState()
    output.header = message.header
    output.health_state = message.health_state
    output.modality_names = list(message.modality_names)
    output.degradation_scores = list(message.degradation_scores)
    output.reliability_weights = list(message.reliability_weights)
    output.covariance_inflation = list(message.covariance_inflation)
    output.factor_enabled = list(message.factor_enabled)
    output.reasons = list(message.reasons)
    output.capability_names = list(message.capability_names)
    output.capability_support = list(message.capability_support)
    output.capability_observable = list(message.capability_observable)
    output.estimator_support = message.estimator_support
    output.relocalization_requested = message.relocalization_requested
    disabled = set(disabled_modalities)
    for index, name in enumerate(output.modality_names):
        if name not in disabled:
            continue
        output.degradation_scores[index] = 1.0
        output.reliability_weights[index] = 0.0
        output.covariance_inflation[index] = 1.0e6
        output.factor_enabled[index] = False
        output.reasons[index] = ",".join(
            item for item in (output.reasons[index], "replay_ablation_disabled") if item
        )
    return output


class SchedulerMask(Node):
    def __init__(self, input_topic, output_topic, disabled_modalities):
        super().__init__("replay_scheduler_mask")
        self.disabled_modalities = frozenset(disabled_modalities)
        self.publisher = self.create_publisher(SchedulerState, output_topic, SCHEDULER_OUTPUT_QOS)
        self.subscription = self.create_subscription(
            SchedulerState, input_topic, self._message, qos_profile_sensor_data
        )

    def _message(self, message):
        self.publisher.publish(mask_scheduler_state(message, self.disabled_modalities))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="/replay/reliability/scheduler_state_input")
    parser.add_argument("--output", default="/reliability/scheduler_state")
    parser.add_argument("--disable", action="append", choices=("lidar", "gnss"), default=[])
    args = parser.parse_args()
    rclpy.init()
    node = SchedulerMask(args.input, args.output, args.disable)
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
