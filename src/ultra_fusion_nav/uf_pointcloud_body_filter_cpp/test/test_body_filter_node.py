import os
import signal
import struct
import subprocess
import tempfile
import time

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32


def make_cloud():
    message = PointCloud2()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = "mid360_link"
    message.height = 1
    message.width = 2
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 16
    message.row_step = 32
    message.data = b"".join(
        struct.pack("<ffff", *point)
        for point in ((0.2, 0.1, 0.0, 10.0), (1.0, 0.0, 0.0, 20.0))
    )
    message.is_dense = True
    return message


def test_cpp_node_preserves_ros_contract_and_sensor_data_qos():
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    previous_rmw = os.environ.get("RMW_IMPLEMENTATION")
    domain_id = 150 + os.getpid() % 20
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    namespace = f"/test/body_filter/run_{os.getpid()}"
    input_topic = f"{namespace}/input"
    output_topic = f"{namespace}/output"
    ratio_topic = f"{namespace}/ratio"
    log = tempfile.TemporaryFile(mode="w+")
    process = subprocess.Popen(
        [
            "ros2", "run", "uf_pointcloud_body_filter_cpp", "pointcloud_body_filter_cpp",
            "--ros-args",
            "-p", f"input_topic:={input_topic}",
            "-p", f"output_topic:={output_topic}",
            "-r", f"/sensors/lidar/body_removed_ratio:={ratio_topic}",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init(domain_id=domain_id)
    node = rclpy.create_node("body_filter_cpp_contract_test")
    outputs = []
    ratios = []
    output_sub = node.create_subscription(
        PointCloud2, output_topic, outputs.append, qos_profile_sensor_data
    )
    ratio_sub = node.create_subscription(
        Float32, ratio_topic, ratios.append, qos_profile_sensor_data
    )
    publisher = node.create_publisher(PointCloud2, input_topic, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                log.flush()
                log.seek(0)
                pytest.fail(f"C++ body-filter node exited during startup:\n{log.read()}")
            if publisher.get_subscription_count() > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() == 1

        source = make_cloud()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (not outputs or not ratios):
            publisher.publish(source)
            rclpy.spin_once(node, timeout_sec=0.05)

        assert outputs
        assert ratios
        output = outputs[-1]
        assert output.header == source.header
        assert output.fields == source.fields
        assert output.height == 1
        assert output.width == 1
        assert output.row_step == source.point_step
        assert bytes(output.data) == bytes(source.data[source.point_step:])
        assert output.is_dense is False
        assert ratios[-1].data == pytest.approx(0.5)

        publishers = node.get_publishers_info_by_topic(output_topic)
        assert len(publishers) == 1
        assert publishers[0].node_name == "pointcloud_body_filter"
        assert publishers[0].qos_profile.reliability == qos_profile_sensor_data.reliability
        assert publishers[0].qos_profile.durability == qos_profile_sensor_data.durability
        assert publishers[0].qos_profile.depth == qos_profile_sensor_data.depth
    finally:
        del output_sub, ratio_sub, publisher
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
        log.close()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        if previous_rmw is None:
            os.environ.pop("RMW_IMPLEMENTATION", None)
        else:
            os.environ["RMW_IMPLEMENTATION"] = previous_rmw
