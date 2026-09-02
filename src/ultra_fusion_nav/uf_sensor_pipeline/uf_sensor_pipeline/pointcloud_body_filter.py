import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

from .pointcloud_utils import filter_cloud


class PointCloudBodyFilter(Node):
    def __init__(self):
        super().__init__("pointcloud_body_filter")
        self.declare_parameter("input_topic", "/sim/mid360/points_raw")
        self.declare_parameter("output_topic", "/sensors/lidar/points_body_filtered")
        self.declare_parameter("body_min_x_m", -0.45)
        self.declare_parameter("body_max_x_m", 0.45)
        self.declare_parameter("body_min_y_m", -0.45)
        self.declare_parameter("body_max_y_m", 0.45)
        self.declare_parameter("body_min_z_m", -0.35)
        self.declare_parameter("body_max_z_m", 0.15)
        self.declare_parameter("min_range_m", 0.10)
        self.declare_parameter("max_range_m", 40.0)
        self.declare_parameter(
            "lidar_to_body_rotation", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter("lidar_to_body_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("reliable_input", False)

        self.bounds = tuple(float(self.get_parameter(name).value) for name in (
            "body_min_x_m", "body_max_x_m", "body_min_y_m", "body_max_y_m",
            "body_min_z_m", "body_max_z_m",
        ))
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.lidar_to_body_rotation = tuple(
            float(value) for value in self.get_parameter("lidar_to_body_rotation").value
        )
        self.lidar_to_body_translation = tuple(
            float(value) for value in self.get_parameter("lidar_to_body_translation").value
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.ratio_pub = self.create_publisher(
            Float32, "/sensors/lidar/body_removed_ratio", qos_profile_sensor_data
        )
        input_qos = qos_profile_sensor_data
        if bool(self.get_parameter("reliable_input").value):
            input_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=2,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self._callback,
            input_qos,
        )
        self.frames = 0
        self.removed_body = 0
        self.input_points = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"Body filter active with body-frame bounds {self.bounds} and LiDAR extrinsic"
        )

    def _callback(self, msg):
        try:
            output, removed_body, _, total = filter_cloud(
                msg,
                self.bounds,
                self.min_range,
                self.max_range,
                self.lidar_to_body_rotation,
                self.lidar_to_body_translation,
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.frames += 1
        self.removed_body += removed_body
        self.input_points += total
        ratio = Float32()
        ratio.data = float(removed_body / max(1, total))
        self.ratio_pub.publish(ratio)
        self.cloud_pub.publish(output)

    def _report(self):
        ratio = self.removed_body / max(1, self.input_points)
        self.get_logger().info(
            f"body_filter frames={self.frames} input_points={self.input_points} "
            f"removed_body={self.removed_body} ratio={ratio:.5f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudBodyFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
