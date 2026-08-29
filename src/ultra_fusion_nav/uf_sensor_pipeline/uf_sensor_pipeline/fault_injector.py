import copy

import numpy as np
import rclpy
from mavros_msgs.msg import OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from uf_interfaces.msg import FaultState

from .fault_models import (
    add_depth_holes,
    add_gnss_jump,
    add_moving_lidar_cluster,
    drop_pointcloud,
    ensure_monotonic_stamp,
    flatten_image,
    shift_stamp,
    standardize_imu_to_body,
)
from .geometry_contract import imu_parameters, load_geometry_contract


MESSAGE_TYPES = {
    "lidar": PointCloud2,
    "imu": Imu,
    "gnss": NavSatFix,
    "optical_flow": OpticalFlowRad,
    "depth": Image,
    "color": Image,
}


class FaultInjector(Node):
    def __init__(self):
        super().__init__("fault_injector")
        self.declare_parameter("modality", "lidar")
        self.declare_parameter("input_topic", "/sensors/input")
        self.declare_parameter("output_topic", "/sensors/output")
        self.declare_parameter("fault_type", "none")
        self.declare_parameter("fault_start_s", 0.0)
        self.declare_parameter("fault_duration_s", 0.0)
        self.declare_parameter("magnitude", 0.0)
        self.declare_parameter("secondary_magnitude", 0.0)
        self.declare_parameter("seed", 7)
        # Simulation-only clock alignment. Real hardware and rosbag replay
        # keep source timestamps unless this option is explicitly enabled.
        self.declare_parameter("restamp_output", False)
        self.declare_parameter("repair_nonmonotonic_timestamps", True)
        self.declare_parameter("imu_acceleration_scale", 1.0)
        self.declare_parameter(
            "mid360_to_body_rotation",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("imu_output_frame_id", "")
        self.declare_parameter("geometry_contract_file", "")

        self.modality = str(self.get_parameter("modality").value)
        self.restamp_output = bool(self.get_parameter("restamp_output").value)
        self.repair_nonmonotonic_timestamps = bool(
            self.get_parameter("repair_nonmonotonic_timestamps").value
        )
        contract_file = str(self.get_parameter("geometry_contract_file").value)
        contract_parameters = (
            dict(imu_parameters(load_geometry_contract(contract_file)))
            if self.modality == "imu" and contract_file else {}
        )
        self.imu_acceleration_scale = float(
            contract_parameters.get(
                "imu_acceleration_scale",
                self.get_parameter("imu_acceleration_scale").value,
            )
        )
        self.mid360_to_body_rotation = tuple(
            float(value) for value in contract_parameters.get(
                "mid360_to_body_rotation",
                self.get_parameter("mid360_to_body_rotation").value,
            )
        )
        self.imu_output_frame_id = str(
            contract_parameters.get(
                "imu_output_frame_id", self.get_parameter("imu_output_frame_id").value
            )
        )
        if self.modality not in MESSAGE_TYPES:
            raise ValueError(f"Unsupported modality: {self.modality}")
        message_type = MESSAGE_TYPES[self.modality]
        self.input_topic = str(contract_parameters.get(
            "imu_input_topic", self.get_parameter("input_topic").value
        ))
        self.output_topic = str(contract_parameters.get(
            "imu_output_topic", self.get_parameter("output_topic").value
        ))
        self.publisher = self.create_publisher(
            message_type, self.output_topic, qos_profile_sensor_data
        )
        self.state_pub = self.create_publisher(FaultState, "/fault/state", 20)
        self.create_subscription(
            message_type,
            self.input_topic,
            self._callback,
            qos_profile_sensor_data,
        )
        self.started_ns = None
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self.affected_messages = 0
        self.last_output_stamp_ns = 0
        self.timestamp_repairs = 0
        self.active_fault_type = "none"
        self.active_fault_start_stamp_s = None
        self.get_logger().info(
            f"fault injector modality={self.modality} "
            f"input={self.input_topic} "
            f"output={self.output_topic} "
            f"restamp_output={self.restamp_output}"
        )

    def _settings(self):
        return (
            str(self.get_parameter("fault_type").value),
            float(self.get_parameter("magnitude").value),
            float(self.get_parameter("secondary_magnitude").value),
        )

    def _active(self, fault_type, source_stamp_ns):
        if fault_type == "none":
            return False
        if source_stamp_ns <= 0:
            return False
        if self.started_ns is None or source_stamp_ns < self.started_ns:
            self.started_ns = source_stamp_ns
        elapsed = (source_stamp_ns - self.started_ns) * 1.0e-9
        start = float(self.get_parameter("fault_start_s").value)
        duration = float(self.get_parameter("fault_duration_s").value)
        return elapsed >= start and (duration <= 0.0 or elapsed < start + duration)

    def _state(self, msg, fault_type, magnitude, active, timestamp_repaired=False):
        state = FaultState()
        state.header = copy.deepcopy(msg.header)
        state.modality = self.modality
        state.fault_type = fault_type
        state.active = active
        state.magnitude = magnitude
        state.affected_messages = self.affected_messages
        state.timestamp_repaired = timestamp_repaired
        state.timestamp_repairs = self.timestamp_repairs
        self.state_pub.publish(state)

    def _apply(self, msg, fault_type, magnitude, secondary, fault_elapsed_s=0.0):
        output = copy.deepcopy(msg)
        if fault_type == "time_offset":
            shift_stamp(output.header.stamp, magnitude)
        elif self.modality == "lidar" and fault_type == "point_dropout":
            output = drop_pointcloud(msg, magnitude, self.rng)
        elif self.modality == "lidar" and fault_type == "dynamic_cluster":
            output = add_moving_lidar_cluster(
                msg, magnitude, fault_elapsed_s, secondary if secondary else 0.6
            )
        elif self.modality == "imu" and fault_type == "bias":
            output.angular_velocity.z += magnitude
            output.linear_acceleration.x += secondary
        elif self.modality == "imu" and fault_type == "saturation":
            limit = abs(magnitude)
            output.angular_velocity.x = max(-limit, min(limit, output.angular_velocity.x))
            output.angular_velocity.y = max(-limit, min(limit, output.angular_velocity.y))
            output.angular_velocity.z = max(-limit, min(limit, output.angular_velocity.z))
        elif self.modality == "gnss" and fault_type == "jump":
            output = add_gnss_jump(msg, magnitude, secondary)
        elif self.modality == "gnss" and fault_type == "covariance_scale":
            output.position_covariance = [value * magnitude for value in output.position_covariance]
        elif self.modality == "optical_flow" and fault_type == "low_quality":
            output.quality = max(0, min(255, int(magnitude)))
        elif self.modality == "optical_flow" and fault_type == "scale":
            output.integrated_x *= magnitude
            output.integrated_y *= magnitude
        elif self.modality == "depth" and fault_type == "holes":
            output = add_depth_holes(msg, magnitude, self.rng)
        elif self.modality == "color" and fault_type == "low_texture":
            output = flatten_image(msg, magnitude if magnitude else 127)
        return output

    def _callback(self, msg):
        fault_type, magnitude, secondary = self._settings()
        source_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        active = self._active(fault_type, source_stamp_ns)
        stamp_s = source_stamp_ns * 1.0e-9
        if active and fault_type != self.active_fault_type:
            self.active_fault_type = fault_type
            self.active_fault_start_stamp_s = stamp_s
        elif not active:
            self.active_fault_type = "none"
            self.active_fault_start_stamp_s = None
        fault_elapsed_s = (
            0.0 if self.active_fault_start_stamp_s is None
            else max(0.0, stamp_s - self.active_fault_start_stamp_s)
        )
        if active:
            self.affected_messages += 1
        if active and fault_type == "outage":
            self._state(msg, fault_type, magnitude, active)
            return
        normalized = (
            standardize_imu_to_body(
                msg,
                self.imu_acceleration_scale,
                self.mid360_to_body_rotation,
                self.imu_output_frame_id or msg.header.frame_id,
            )
            if self.modality == "imu" else msg
        )
        output = (
            self._apply(normalized, fault_type, magnitude, secondary, fault_elapsed_s)
            if active else msg
        )
        if self.modality == "imu" and not active:
            output = normalized
        if self.restamp_output:
            output = copy.deepcopy(output)
            output.header.stamp = self.get_clock().now().to_msg()
        self.last_output_stamp_ns, repaired = ensure_monotonic_stamp(
            output.header.stamp,
            self.last_output_stamp_ns,
            repair=self.repair_nonmonotonic_timestamps,
        )
        if repaired:
            self.timestamp_repairs += 1
            if self.timestamp_repairs <= 3:
                self.get_logger().warning(
                    f"repaired non-monotonic {self.modality} timestamp"
                )
        self._state(output, fault_type, magnitude, active, repaired)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
