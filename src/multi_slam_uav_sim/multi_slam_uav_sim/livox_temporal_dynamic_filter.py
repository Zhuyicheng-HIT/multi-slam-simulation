"""Remove transient LiDAR voxels before FAST-LIO using local odometry only."""

from collections import deque
import copy
import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError as exc:  # pragma: no cover - requires the external overlay
    CustomMsg = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


def _rotate(q, vector):
    x, y, z, w = q
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


class LivoxTemporalDynamicFilter(Node):
    def __init__(self):
        if CustomMsg is None:
            raise RuntimeError(
                "livox_ros_driver2 messages are unavailable: " + str(IMPORT_ERROR)
            )
        super().__init__("livox_temporal_dynamic_filter")
        self.declare_parameter("input_topic", "/livox/lidar_raw")
        self.declare_parameter("output_topic", "/livox/lidar")
        self.declare_parameter("odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("voxel_size_m", 0.50)
        self.declare_parameter("history_frames", 5)
        self.declare_parameter("minimum_support", 2)
        self.declare_parameter("lidar_to_body_rotation", [
            0.9659258263, 0.0, 0.2588190451,
            0.0, 1.0, 0.0,
            -0.2588190451, 0.0, 0.9659258263,
        ])
        self.declare_parameter("lidar_to_body_translation", [0.05, 0.0, 0.10])

        self.voxel_size = float(self.get_parameter("voxel_size_m").value)
        self.history_frames = max(1, int(self.get_parameter("history_frames").value))
        self.minimum_support = max(1, int(self.get_parameter("minimum_support").value))
        self.rotation = tuple(float(v) for v in self.get_parameter(
            "lidar_to_body_rotation").value)
        self.translation = tuple(float(v) for v in self.get_parameter(
            "lidar_to_body_translation").value)
        if len(self.rotation) != 9 or len(self.translation) != 3:
            raise ValueError("LiDAR extrinsic arrays have invalid lengths")
        if not math.isfinite(self.voxel_size) or self.voxel_size <= 0.0:
            raise ValueError("voxel_size_m must be positive")

        reliable = QoSProfile(depth=2)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(
            CustomMsg, str(self.get_parameter("output_topic").value), reliable)
        self.create_subscription(
            CustomMsg, str(self.get_parameter("input_topic").value),
            self._cloud, reliable)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._odom, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.odom = None
        self.history = deque(maxlen=self.history_frames)
        self.frames = 0
        self.removed = 0
        self.passed = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"temporal filter active: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}, "
            f"odom={self.get_parameter('odom_topic').value}, "
            f"voxel={self.voxel_size:.2f} history={self.history_frames} "
            f"support={self.minimum_support}"
        )

    def _odom(self, message):
        pose = message.pose.pose
        q = (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w)
        norm = math.sqrt(sum(value * value for value in q))
        if norm <= 1.0e-9 or not all(math.isfinite(value) for value in q):
            return
        with self.lock:
            self.odom = (
                (pose.position.x, pose.position.y, pose.position.z),
                tuple(value / norm for value in q),
            )

    def _key(self, point, odom):
        px = self.rotation[0] * point.x + self.rotation[1] * point.y + self.rotation[2] * point.z + self.translation[0]
        py = self.rotation[3] * point.x + self.rotation[4] * point.y + self.rotation[5] * point.z + self.translation[1]
        pz = self.rotation[6] * point.x + self.rotation[7] * point.y + self.rotation[8] * point.z + self.translation[2]
        rotated = _rotate(odom[1], (px, py, pz))
        return tuple(math.floor((rotated[index] + odom[0][index]) / self.voxel_size) for index in range(3))

    def _cloud(self, message):
        with self.lock:
            odom = self.odom
        points = list(message.points)
        self.frames += 1
        if odom is None:
            self.publisher.publish(message)
            self.passed += len(points)
            return
        current = {self._key(point, odom) for point in points}
        if len(self.history) < self.history_frames:
            self.history.append(current)
            self.publisher.publish(message)
            self.passed += len(points)
            return
        kept = []
        for point in points:
            key = self._key(point, odom)
            support = sum(key in frame for frame in self.history)
            if support >= self.minimum_support:
                kept.append(point)
            else:
                self.removed += 1
        output = copy.deepcopy(message)
        output.points = kept
        output.point_num = len(kept)
        self.history.append(current)
        self.publisher.publish(output)
        self.passed += len(kept)

    def _report(self):
        self.get_logger().info(
            f"temporal_filter frames={self.frames} passed={self.passed} "
            f"removed={self.removed} ratio={self.removed / max(1, self.passed + self.removed):.5f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LivoxTemporalDynamicFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
