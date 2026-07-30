import struct

import rclpy
from sensor_msgs.msg import PointCloud2, PointField

from multi_slam_uav_sim.livox_mid360_bridge import LivoxMid360Bridge


class CapturePublisher:
    def __init__(self):
        self.message = None

    def publish(self, message):
        self.message = message


def make_cloud(point_count=8):
    cloud = PointCloud2()
    cloud.header.stamp.sec = 100
    cloud.header.frame_id = "mid360_link"
    cloud.height = 1
    cloud.width = point_count
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * point_count
    cloud.data = b"".join(
        struct.pack("<ffff", float(i + 1), 0.0, 0.0, 100.0)
        for i in range(point_count)
    )
    return cloud


def test_simulated_custom_msg_matches_mid360s_contract():
    rclpy.init()
    node = LivoxMid360Bridge()
    capture = CapturePublisher()
    node.lidar_pub = capture
    try:
        node._cloud_cb(make_cloud())
        message = capture.message
        assert message is not None
        assert message.header.frame_id == "mid360_link"
        assert message.timebase == 100_000_000_000
        assert message.point_num == 8
        assert [point.line for point in message.points] == [0, 1, 2, 3, 0, 1, 2, 3]
        assert {point.tag for point in message.points} == {0}
        offsets = [point.offset_time for point in message.points]
        assert offsets == sorted(offsets)
        assert offsets[0] == 0
        assert offsets[-1] == 100_000_000
    finally:
        node.destroy_node()
        rclpy.shutdown()
