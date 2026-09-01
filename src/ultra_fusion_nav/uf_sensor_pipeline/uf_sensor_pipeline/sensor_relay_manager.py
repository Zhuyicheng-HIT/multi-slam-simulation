"""Production pass-through relays for active sensor modalities.

Fault injection remains in the per-modality injector nodes.  This manager is
deliberately limited to copy/route (and the existing IMU unit normalization)
so production consolidation cannot change estimator semantics.
"""

import copy
from collections import Counter

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from mavros_msgs.msg import OpticalFlowRad

from .fault_models import standardize_imu_acceleration


MESSAGE_TYPES = {
    "lidar": PointCloud2,
    "imu": Imu,
    "gnss": NavSatFix,
    "optical_flow": OpticalFlowRad,
    "depth": Image,
    "color": Image,
}


class SensorRelayManager(Node):
    def __init__(self):
        super().__init__("sensor_relay_manager")
        self.declare_parameter("active_modalities", ["lidar", "imu", "gnss", "optical_flow"])
        self.declare_parameter("lidar_input_topic", "/sensors/lidar/points_body_filtered")
        self.declare_parameter("lidar_output_topic", "/sensors/lidar/points")
        self.declare_parameter("imu_input_topic", "/livox/imu")
        self.declare_parameter("imu_output_topic", "/sensors/imu")
        self.declare_parameter("gnss_input_topic", "/mavros/global_position/raw/fix")
        self.declare_parameter("gnss_output_topic", "/sensors/gnss/fix")
        self.declare_parameter("optical_flow_input_topic", "/sim/optical_flow/rad")
        self.declare_parameter("optical_flow_output_topic", "/sensors/optical_flow/rad")
        self.declare_parameter("depth_input_topic", "/front/d435i/aligned_depth_to_color/image_raw")
        self.declare_parameter("depth_output_topic", "/sensors/rgbd/depth")
        self.declare_parameter("color_input_topic", "/front/d435i/color/image_raw")
        self.declare_parameter("color_output_topic", "/sensors/rgbd/color")
        self.declare_parameter("imu_acceleration_scale", 1.0)
        self.declare_parameter("restamp_gnss", True)
        self.relay_count = 0
        self.relay_counts = Counter()
        self.relay_publishers = {}
        self.relay_groups = {}
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/sensors/relay/diagnostics", 10
        )
        self.diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        active = {str(name) for name in self.get_parameter("active_modalities").value}
        for modality, message_type in MESSAGE_TYPES.items():
            if modality not in active:
                continue
            input_topic = str(self.get_parameter(f"{modality}_input_topic").value)
            output_topic = str(self.get_parameter(f"{modality}_output_topic").value)
            self.relay_publishers[modality] = self.create_publisher(
                message_type, output_topic, qos_profile_sensor_data
            )
            # Keep ordering within one stream while allowing independent sensor
            # callbacks to run concurrently in the MultiThreadedExecutor.
            self.relay_groups[modality] = MutuallyExclusiveCallbackGroup()
            self.create_subscription(
                message_type,
                input_topic,
                lambda msg, name=modality: self._relay(name, msg),
                qos_profile_sensor_data,
                callback_group=self.relay_groups[modality],
            )
            self.get_logger().info(f"relay {modality}: {input_topic} -> {output_topic}")

    def _relay(self, modality, message):
        output = copy.deepcopy(message)
        if modality == "imu":
            output = standardize_imu_acceleration(
                output, float(self.get_parameter("imu_acceleration_scale").value)
            )
        elif modality == "gnss" and bool(self.get_parameter("restamp_gnss").value):
            now = self.get_clock().now().to_msg()
            output.header.stamp = now
        self.relay_publishers[modality].publish(output)
        self.relay_count += 1
        self.relay_counts[modality] += 1

    def _publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "uf_sensor_pipeline/sensor_relay_manager"
        status.hardware_id = "sensor-normalization"
        status.level = DiagnosticStatus.OK
        status.message = "sensor relays active"
        status.values = [
            KeyValue(key="total_relayed", value=str(self.relay_count)),
            *[
                KeyValue(key=f"{name}_relayed", value=str(self.relay_counts[name]))
                for name in sorted(self.relay_publishers)
            ],
            KeyValue(key="executor_threads", value="2"),
            KeyValue(key="queue_policy", value="DDS sensor_data keep_last"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SensorRelayManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
