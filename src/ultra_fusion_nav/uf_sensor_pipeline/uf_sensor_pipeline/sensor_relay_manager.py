"""Production pass-through relays for active sensor modalities.

Fault injection remains in the per-modality injector nodes.  This manager is
deliberately limited to copy/route (and the existing IMU unit normalization)
so production consolidation cannot change estimator semantics.
"""

import copy

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from mavros_msgs.msg import OpticalFlowRad

from .fault_models import standardize_imu_to_body
from .geometry_contract import imu_parameters, load_geometry_contract


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
        self.declare_parameter("geometry_contract_file", "")
        contract_file = str(self.get_parameter("geometry_contract_file").value)
        contract_parameters = (
            dict(imu_parameters(load_geometry_contract(contract_file)))
            if contract_file else {}
        )
        self.declare_parameter("active_modalities", ["lidar", "imu", "gnss", "optical_flow"])
        self.declare_parameter("lidar_input_topic", "/sensors/lidar/points_body_filtered")
        self.declare_parameter("lidar_output_topic", "/sensors/lidar/points")
        self.declare_parameter(
            "imu_input_topic", contract_parameters.get("imu_input_topic", "/livox/imu")
        )
        self.declare_parameter(
            "imu_output_topic", contract_parameters.get("imu_output_topic", "/sensors/imu")
        )
        self.declare_parameter("gnss_input_topic", "/mavros/global_position/raw/fix")
        self.declare_parameter("gnss_output_topic", "/sensors/gnss/fix")
        self.declare_parameter("optical_flow_input_topic", "/sim/optical_flow/rad")
        self.declare_parameter("optical_flow_output_topic", "/sensors/optical_flow/rad")
        self.declare_parameter("depth_input_topic", "/front/d435i/aligned_depth_to_color/image_raw")
        self.declare_parameter("depth_output_topic", "/sensors/rgbd/depth")
        self.declare_parameter("color_input_topic", "/front/d435i/color/image_raw")
        self.declare_parameter("color_output_topic", "/sensors/rgbd/color")
        self.declare_parameter(
            "imu_acceleration_scale",
            contract_parameters.get("imu_acceleration_scale", 1.0),
        )
        self.declare_parameter(
            "mid360_to_body_rotation",
            contract_parameters.get(
                "mid360_to_body_rotation",
                [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            ),
        )
        self.declare_parameter(
            "imu_output_frame_id",
            contract_parameters.get("imu_output_frame_id", ""),
        )
        self.declare_parameter("restamp_gnss", True)
        self.mid360_to_body_rotation = tuple(
            float(value)
            for value in self.get_parameter("mid360_to_body_rotation").value
        )
        self.imu_output_frame_id = str(
            self.get_parameter("imu_output_frame_id").value
        )
        self.relay_count = 0
        self.relay_publishers = {}
        active = {str(name) for name in self.get_parameter("active_modalities").value}
        for modality, message_type in MESSAGE_TYPES.items():
            if modality not in active:
                continue
            input_topic = str(self.get_parameter(f"{modality}_input_topic").value)
            output_topic = str(self.get_parameter(f"{modality}_output_topic").value)
            self.relay_publishers[modality] = self.create_publisher(
                message_type, output_topic, qos_profile_sensor_data
            )
            self.create_subscription(
                message_type,
                input_topic,
                lambda msg, name=modality: self._relay(name, msg),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"relay {modality}: {input_topic} -> {output_topic}")

    def _relay(self, modality, message):
        if modality == "imu":
            output = standardize_imu_to_body(
                message,
                float(self.get_parameter("imu_acceleration_scale").value),
                self.mid360_to_body_rotation,
                self.imu_output_frame_id or message.header.frame_id,
            )
        else:
            output = copy.deepcopy(message)
        if modality == "gnss" and bool(self.get_parameter("restamp_gnss").value):
            now = self.get_clock().now().to_msg()
            output.header.stamp = now
        self.relay_publishers[modality].publish(output)
        self.relay_count += 1


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
