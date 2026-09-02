"""Convert the Livox hardware CustomMsg boundary to PointCloud2."""

import struct

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class LivoxCustomToPointCloud(Node):
    def __init__(self):
        super().__init__("livox_custom_to_pointcloud")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/sensors/lidar/points_raw")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, qos
        )
        self.subscription = self.create_subscription(
            CustomMsg, self.get_parameter("input_topic").value, self._convert, qos
        )
        self.get_logger().info(
            f"Livox CustomMsg adapter: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def _convert(self, message):
        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = len(message.points)
        output.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
        ]
        output.is_bigendian = False
        output.point_step = 24
        output.row_step = output.point_step * output.width
        output.is_dense = True
        data = bytearray(output.row_step)
        for index, point in enumerate(message.points):
            struct.pack_into(
                "<fffffH2x", data, index * output.point_step,
                point.x, point.y, point.z, float(point.reflectivity),
                float(point.offset_time) * 1.0e-9, int(point.line),
            )
        output.data = bytes(data)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
