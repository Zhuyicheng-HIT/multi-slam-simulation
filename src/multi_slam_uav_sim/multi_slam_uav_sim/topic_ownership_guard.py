import argparse
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser(
        description="Monitor ROS topic publisher ownership with one persistent DDS node."
    )
    parser.add_argument("--topic", action="append", required=True)
    parser.add_argument("--expected-publishers", type=int, default=1)
    parser.add_argument("--startup-grace-s", type=float, default=10.0)
    parser.add_argument("--check-period-s", type=float, default=1.0)
    parser.add_argument("--missing-limit", type=int, default=5)
    parser.add_argument("--duplicate-limit", type=int, default=2)
    parser.add_argument("--terminate-pgid", type=int, default=0)
    return parser.parse_args()


class TopicOwnershipGuard(Node):
    def __init__(self, args):
        super().__init__("topic_ownership_guard")
        self.topics = tuple(dict.fromkeys(args.topic))
        self.expected = int(args.expected_publishers)
        self.startup_grace_s = float(args.startup_grace_s)
        self.check_period_s = float(args.check_period_s)
        self.missing_limit = int(args.missing_limit)
        self.duplicate_limit = int(args.duplicate_limit)
        self.terminate_pgid = int(args.terminate_pgid)
        if not self.topics:
            raise ValueError("at least one topic is required")
        if self.expected < 1:
            raise ValueError("expected publishers must be positive")
        if self.startup_grace_s < 0.0 or self.check_period_s <= 0.0:
            raise ValueError("invalid timing configuration")
        if self.missing_limit < 1 or self.duplicate_limit < 1:
            raise ValueError("failure limits must be positive")

    def counts(self):
        return {topic: int(self.count_publishers(topic)) for topic in self.topics}

    def terminate_owner(self):
        if self.terminate_pgid <= 0:
            return
        try:
            if os.getpgid(self.terminate_pgid) != self.terminate_pgid:
                self.get_logger().error(
                    f"refusing to terminate non-session pid {self.terminate_pgid}"
                )
                return
            os.killpg(self.terminate_pgid, signal.SIGTERM)
        except ProcessLookupError:
            return
        except OSError as error:
            self.get_logger().error(
                f"failed to terminate process group {self.terminate_pgid}: {error}"
            )


def main():
    args = parse_args()
    rclpy.init(args=None)
    node = TopicOwnershipGuard(args)
    started = time.monotonic()
    missing_streak = 0
    duplicate_streak = 0
    previous_counts = None
    status = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=node.check_period_s)
            counts = node.counts()
            if counts != previous_counts:
                node.get_logger().info(
                    "publisher ownership: "
                    + " ".join(f"{topic}={count}" for topic, count in counts.items())
                )
                previous_counts = counts
            if time.monotonic() - started < node.startup_grace_s:
                continue
            missing = any(count < node.expected for count in counts.values())
            duplicate = any(count > node.expected for count in counts.values())
            missing_streak = missing_streak + 1 if missing else 0
            duplicate_streak = duplicate_streak + 1 if duplicate else 0
            if duplicate_streak >= node.duplicate_limit:
                node.get_logger().error(
                    f"duplicate publisher ownership persisted: {counts}"
                )
                status = 3
                node.terminate_owner()
                break
            if missing_streak >= node.missing_limit:
                node.get_logger().error(
                    f"publisher ownership loss persisted: {counts}"
                )
                status = 4
                node.terminate_owner()
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
