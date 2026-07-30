#!/usr/bin/env python3
import threading

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GazeboGroundTruthBridge(Node):
    """Publish one Gazebo model pose as diagnostic-only ROS odometry."""

    def __init__(self):
        super().__init__("gazebo_ground_truth_bridge")
        self.declare_parameter("world_name", "simple_apm_d435i_only")
        self.declare_parameter("model_name", "apm_iris")
        self.declare_parameter(
            "output_topic", "/d435i_visual_slam/ground_truth")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("max_publish_hz", 60.0)

        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.minimum_period_s = 1.0 / max(
            float(self.get_parameter("max_publish_hz").value), 1.0)

        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.lock = threading.Lock()
        self.latest_pose = None
        self.last_publish_ns = 0
        self.gz_node = GzNode()
        for suffix in ("dynamic_pose/info", "pose/info"):
            self.gz_node.subscribe(
                Pose_V, f"/world/{self.world_name}/{suffix}", self._pose_cb)
        self.create_timer(self.minimum_period_s, self._publish_latest)
        self.get_logger().info(
            f"Gazebo ground truth: world={self.world_name} "
            f"model={self.model_name} -> {output_topic} (evaluation only)")

    def _pose_cb(self, message):
        for pose in message.pose:
            if pose.name == self.model_name or pose.name.endswith(
                    f"::{self.model_name}"):
                with self.lock:
                    self.latest_pose = pose
                return

    def _publish_latest(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if now_ns - self.last_publish_ns < int(self.minimum_period_s * 1.0e9):
            return
        with self.lock:
            pose = self.latest_pose
        if pose is None:
            return

        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.world_frame
        message.child_frame_id = self.body_frame
        message.pose.pose.position.x = float(pose.position.x)
        message.pose.pose.position.y = float(pose.position.y)
        message.pose.pose.position.z = float(pose.position.z)
        message.pose.pose.orientation.x = float(pose.orientation.x)
        message.pose.pose.orientation.y = float(pose.orientation.y)
        message.pose.pose.orientation.z = float(pose.orientation.z)
        message.pose.pose.orientation.w = float(pose.orientation.w)
        self.publisher.publish(message)
        self.last_publish_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGroundTruthBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
