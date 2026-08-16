#!/usr/bin/env python3
"""Read one ROS 2 parameter without relying on the ROS graph daemon."""

import argparse
import sys
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node


def parameter_value_text(value):
    value_type = int(value.type)
    fields = {
        ParameterType.PARAMETER_BOOL: lambda: str(value.bool_value).lower(),
        ParameterType.PARAMETER_INTEGER: lambda: str(value.integer_value),
        ParameterType.PARAMETER_DOUBLE: lambda: repr(value.double_value),
        ParameterType.PARAMETER_STRING: lambda: value.string_value,
    }
    formatter = fields.get(value_type)
    if formatter is None:
        raise ValueError(f"unsupported parameter type: {value_type}")
    return formatter()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", required=True)
    parser.add_argument("--parameter", required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    rclpy.init(args=None)
    node = Node("wait_for_ros_parameter")
    service_name = f"{args.node.rstrip('/')}/get_parameters"
    client = node.create_client(GetParameters, service_name)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=0.2):
                rclpy.spin_once(node, timeout_sec=0.1)
                continue
            request = GetParameters.Request()
            request.names = [args.parameter]
            future = client.call_async(request)
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=min(2.0, remaining)
            )
            if not future.done() or future.result() is None:
                continue
            values = tuple(future.result().values)
            if not values or int(values[0].type) == ParameterType.PARAMETER_NOT_SET:
                continue
            print(parameter_value_text(values[0]))
            return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(
        f"timed out reading {args.node}:{args.parameter}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
