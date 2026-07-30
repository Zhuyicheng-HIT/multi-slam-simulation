"""Normalize simulated MAVROS GNSS metadata onto the hardware-neutral topic."""

import rclpy
from mavros_msgs.msg import GPSRAW
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GnssMetadataRelay(Node):
    def __init__(self):
        super().__init__("gnss_metadata_relay")
        self.declare_parameter("input_topic", "/mavros/gpsstatus/gps1/raw")
        self.declare_parameter("output_topic", "/sensors/gnss/raw")
        self.publisher = self.create_publisher(
            GPSRAW, str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GPSRAW, str(self.get_parameter("input_topic").value),
            self.publisher.publish, qos_profile_sensor_data,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GnssMetadataRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
