#!/usr/bin/env python3
"""In-process ROS serialization/executor smoke for sensor_relay_manager."""
import time
import struct
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from mavros_msgs.msg import OpticalFlowRad
from uf_sensor_pipeline.pointcloud_body_filter import PointCloudBodyFilter
from uf_sensor_pipeline.sensor_relay_manager import SensorRelayManager


class Sink(Node):
    def __init__(self):
        super().__init__("sensor_pipeline_e2e_sink")
        self.lidar = 0
        self.imu = 0
        self.flow = 0
        self.create_subscription(PointCloud2, "/sensors/lidar/points", lambda _: setattr(self, "lidar", self.lidar + 1), qos_profile_sensor_data)
        self.create_subscription(Imu, "/sensors/imu", lambda _: setattr(self, "imu", self.imu + 1), qos_profile_sensor_data)
        self.create_subscription(OpticalFlowRad, "/sensors/optical_flow/rad", lambda _: setattr(self, "flow", self.flow + 1), qos_profile_sensor_data)


def cloud(points=2000):
    message = PointCloud2()
    message.height = 1; message.width = points; message.point_step = 16
    message.row_step = points * 16
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.data = b"".join(struct.pack("<ffff", 2.0, 0.0, 0.0, 10.0) for _ in range(points))
    return message


def main():
    rclpy.init()
    manager = SensorRelayManager()
    body_filter = PointCloudBodyFilter()
    sink = Sink()
    pub_lidar = sink.create_publisher(PointCloud2, "/sim/mid360/points_raw", qos_profile_sensor_data)
    pub_imu = sink.create_publisher(Imu, "/livox/imu", qos_profile_sensor_data)
    pub_flow = sink.create_publisher(OpticalFlowRad, "/sim/optical_flow/rad", qos_profile_sensor_data)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(manager); executor.add_node(body_filter); executor.add_node(sink)
    lidar_message = cloud()
    started = time.monotonic()
    next_lidar = next_imu = next_flow = started
    while time.monotonic() - started < 3.0:
        now = time.monotonic()
        if now >= next_lidar: pub_lidar.publish(lidar_message); next_lidar += 0.1
        if now >= next_imu: pub_imu.publish(Imu()); next_imu += 0.005
        if now >= next_flow: pub_flow.publish(OpticalFlowRad()); next_flow += 0.01
        executor.spin_once(timeout_sec=0.001)
    print(f"serialized_executor_smoke lidar={sink.lidar} imu={sink.imu} flow={sink.flow} executor=MultiThreadedExecutor threads=4")
    if sink.lidar < 27 or sink.imu < 540 or sink.flow < 270:
        raise SystemExit(1)
    executor.remove_node(manager); executor.remove_node(body_filter); executor.remove_node(sink)
    executor.shutdown()
    manager.destroy_node(); body_filter.destroy_node(); sink.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
