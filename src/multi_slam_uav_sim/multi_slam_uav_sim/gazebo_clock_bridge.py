"""Bridge Gazebo Sim world time to the ROS 2 simulation clock."""

import os

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as RosClock

from gz.msgs10.clock_pb2 import Clock as GazeboClock
from gz.transport13 import Node as GzNode


class GazeboClockBridge(Node):
    def __init__(self):
        super().__init__("gazebo_clock_bridge")
        self.declare_parameter("world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("gz_topic", "")
        self.declare_parameter("ros_topic", "/clock")
        world_name = str(self.get_parameter("world_name").value)
        gz_topic = str(self.get_parameter("gz_topic").value).strip()
        self.gz_topic = gz_topic or f"/world/{world_name}/clock"
        self.ros_topic = str(self.get_parameter("ros_topic").value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(RosClock, self.ros_topic, qos)
        self.gz_node = GzNode()
        # gz.transport13.subscribe returns None on success, so subscription
        # setup is intentionally not used as a boolean status check here.
        self.gz_node.subscribe(GazeboClock, self.gz_topic, self._clock_cb)
        self.last_sim_ns = None
        self.get_logger().info(
            f"Gazebo clock bridge active: {self.gz_topic} -> {self.ros_topic}"
        )

    def _clock_cb(self, message):
        try:
            sec = int(message.sim.sec)
            nanosec = int(message.sim.nsec)
        except (AttributeError, TypeError, ValueError):
            return
        if sec < 0 or nanosec < 0:
            return
        stamp_ns = sec * 1_000_000_000 + nanosec
        if self.last_sim_ns is not None and stamp_ns < self.last_sim_ns:
            self.get_logger().warning(
                f"Gazebo clock rewind: {self.last_sim_ns} -> {stamp_ns}"
            )
        self.last_sim_ns = stamp_ns
        output = RosClock()
        output.clock.sec = sec
        output.clock.nanosec = nanosec
        self.publisher.publish(output)

    def destroy_node(self):
        try:
            self.gz_node.unsubscribe(self.gz_topic)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboClockBridge()
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
