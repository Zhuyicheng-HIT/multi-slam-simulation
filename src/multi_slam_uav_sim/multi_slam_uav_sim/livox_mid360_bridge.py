import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
except Exception as exc:  # pragma: no cover - depends on the external LiDAR workspace overlay
    CustomMsg = None
    CustomPoint = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


def _field_map(msg):
    return {field.name: field for field in msg.fields}


def _read_float(data, offset, datatype):
    raw = data[offset:]
    if datatype == PointField.FLOAT32:
        return struct.unpack_from("<f", raw, 0)[0]
    if datatype == PointField.FLOAT64:
        return float(struct.unpack_from("<d", raw, 0)[0])
    if datatype == PointField.INT8:
        return float(struct.unpack_from("<b", raw, 0)[0])
    if datatype == PointField.UINT8:
        return float(struct.unpack_from("<B", raw, 0)[0])
    if datatype == PointField.INT16:
        return float(struct.unpack_from("<h", raw, 0)[0])
    if datatype == PointField.UINT16:
        return float(struct.unpack_from("<H", raw, 0)[0])
    if datatype == PointField.INT32:
        return float(struct.unpack_from("<i", raw, 0)[0])
    if datatype == PointField.UINT32:
        return float(struct.unpack_from("<I", raw, 0)[0])
    return 0.0


class LivoxMid360Bridge(Node):
    def __init__(self):
        if CustomMsg is None or CustomPoint is None:
            raise RuntimeError(
                "livox_ros_driver2 messages are unavailable. Source the MID360/FAST-LIO workspace "
                f"before running this node. Import error: {IMPORT_ERROR}"
            )
        super().__init__("livox_mid360_bridge")
        self.declare_parameter("input_cloud_topic", "/sim/mid360/points_raw")
        self.declare_parameter("input_imu_topic", "/uav/imu")
        self.declare_parameter("livox_lidar_topic", "/livox/lidar")
        self.declare_parameter("livox_imu_topic", "/livox/imu")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("scan_lines", 4)
        self.declare_parameter("frame_rate_hz", 10.0)
        self.declare_parameter("vertical_min_deg", -7.0)
        self.declare_parameter("vertical_max_deg", 52.0)
        self.declare_parameter("max_points", 65000)
        self.declare_parameter("point_stride", 1)

        self.frame_id = self.get_parameter("frame_id").value
        self.scan_lines = max(1, int(self.get_parameter("scan_lines").value))
        frame_rate_hz = max(0.1, float(self.get_parameter("frame_rate_hz").value))
        self.scan_period_us = int(round(1.0e6 / frame_rate_hz))
        self.vertical_min = math.radians(float(self.get_parameter("vertical_min_deg").value))
        self.vertical_max = math.radians(float(self.get_parameter("vertical_max_deg").value))
        self.max_points = max(1, int(self.get_parameter("max_points").value))
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        in_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        fastlio_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.lidar_pub = self.create_publisher(
            CustomMsg, self.get_parameter("livox_lidar_topic").value, fastlio_qos
        )
        self.imu_pub = self.create_publisher(
            Imu, self.get_parameter("livox_imu_topic").value, fastlio_qos
        )
        self.create_subscription(
            PointCloud2, self.get_parameter("input_cloud_topic").value, self._cloud_cb, in_qos
        )
        self.create_subscription(
            Imu, self.get_parameter("input_imu_topic").value, self._imu_cb, qos_profile_sensor_data
        )

        self.cloud_count = 0
        self.point_count_last = 0
        self.status_timer = self.create_timer(2.0, self._status)
        self.get_logger().info(
            "MID360 Livox bridge active: raw PointCloud2 -> /livox/lidar CustomMsg, /uav/imu -> /livox/imu"
        )

    def _stamp_to_ns(self, stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _line_from_pitch(self, x, y, z):
        pitch = math.atan2(z, math.hypot(x, y))
        span = max(1.0e-6, self.vertical_max - self.vertical_min)
        ratio = (pitch - self.vertical_min) / span
        return max(0, min(self.scan_lines - 1, int(ratio * self.scan_lines)))

    def _cloud_cb(self, msg):
        fields = _field_map(msg)
        if not {"x", "y", "z"}.issubset(fields):
            self.get_logger().warning("Input PointCloud2 has no x/y/z fields; skipping.")
            return

        intensity_field = fields.get("intensity")
        tag_field = fields.get("tag")
        line_field = fields.get("line")
        declared = int(msg.width) * int(msg.height)
        available = len(msg.data) // int(msg.point_step) if msg.point_step else 0
        count = min(declared, available)
        if count <= 0:
            return

        out = CustomMsg()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.timebase = self._stamp_to_ns(msg.header.stamp)
        out.lidar_id = 1
        out.rsvd = [0, 0, 0]

        points = []
        usable = min(count, self.max_points * self.point_stride)
        for src_idx in range(0, usable, self.point_stride):
            base = src_idx * int(msg.point_step)
            raw = msg.data[base:base + int(msg.point_step)]
            x = _read_float(raw, fields["x"].offset, fields["x"].datatype)
            y = _read_float(raw, fields["y"].offset, fields["y"].datatype)
            z = _read_float(raw, fields["z"].offset, fields["z"].datatype)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            p = CustomPoint()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            if intensity_field is not None:
                intensity = _read_float(raw, intensity_field.offset, intensity_field.datatype)
            else:
                intensity = 120.0
            p.reflectivity = max(0, min(255, int(round(intensity))))
            if tag_field is not None:
                p.tag = max(0, min(255, int(_read_float(raw, tag_field.offset, tag_field.datatype))))
            else:
                p.tag = 0x10
            if line_field is not None:
                p.line = max(0, min(self.scan_lines - 1, int(_read_float(raw, line_field.offset, line_field.datatype))))
            else:
                p.line = self._line_from_pitch(x, y, z)
            p.offset_time = int(round((len(points) / max(1, self.max_points - 1)) * self.scan_period_us))
            points.append(p)

        out.points = points
        out.point_num = len(points)
        self.point_count_last = len(points)
        self.cloud_count += 1
        self.lidar_pub.publish(out)

    def _imu_cb(self, msg):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.imu_pub.publish(out)

    def _status(self):
        self.get_logger().info(
            f"livox bridge clouds={self.cloud_count} last_points={self.point_count_last} "
            f"scan_lines={self.scan_lines}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LivoxMid360Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
