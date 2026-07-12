import math
import struct
import time

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
        self.declare_parameter("input_imu_topic", "/mavros/imu/data_raw")
        self.declare_parameter("livox_lidar_topic", "/livox/lidar")
        self.declare_parameter("livox_imu_topic", "/livox/imu")
        self.declare_parameter("lidar_frame_id", "mid360_link")
        self.declare_parameter("imu_frame_id", "base_link")
        self.declare_parameter("restamp_imu", False)
        self.declare_parameter("scan_lines", 40)
        self.declare_parameter("frame_rate_hz", 10.0)
        self.declare_parameter("vertical_min_deg", -7.0)
        self.declare_parameter("vertical_max_deg", 52.0)
        self.declare_parameter("max_points", 65000)
        self.declare_parameter("point_stride", 1)

        self.lidar_frame_id = self.get_parameter("lidar_frame_id").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        self.restamp_imu = bool(self.get_parameter("restamp_imu").value)
        self.scan_lines = max(1, int(self.get_parameter("scan_lines").value))
        frame_rate_hz = max(0.1, float(self.get_parameter("frame_rate_hz").value))
        self.scan_period_ns = int(round(1.0e9 / frame_rate_hz))
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
        self.imu_count = 0
        self.dropped_imu_count = 0
        self.point_count_last = 0
        self.last_cloud_stamp_ns = 0
        self.last_imu_stamp_ns = 0
        self.last_status_time = time.monotonic()
        self.last_status_cloud_count = 0
        self.last_status_imu_count = 0
        self.status_timer = self.create_timer(2.0, self._status)
        self.get_logger().info(
            "MID360 Livox bridge active: raw PointCloud2 -> /livox/lidar CustomMsg, "
            f"{self.get_parameter('input_imu_topic').value} -> /livox/imu"
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
        time_field = fields.get("time")
        declared = int(msg.width) * int(msg.height)
        available = len(msg.data) // int(msg.point_step) if msg.point_step else 0
        count = min(declared, available)
        if count <= 0:
            return

        out = CustomMsg()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.lidar_frame_id
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
            if time_field is not None:
                point_time_s = _read_float(raw, time_field.offset, time_field.datatype)
                p.offset_time = max(0, min(0xFFFFFFFF, int(round(point_time_s * 1.0e9))))
            else:
                p.offset_time = int(round((src_idx / max(1, count - 1)) * self.scan_period_ns))
            points.append(p)

        out.points = points
        out.point_num = len(points)
        self.point_count_last = len(points)
        self.cloud_count += 1
        self.last_cloud_stamp_ns = self._stamp_to_ns(msg.header.stamp)
        self.lidar_pub.publish(out)

    def _imu_cb(self, msg):
        if self.restamp_imu:
            output_stamp = self.get_clock().now().to_msg()
        else:
            output_stamp = msg.header.stamp
        stamp_ns = self._stamp_to_ns(output_stamp)
        if stamp_ns <= self.last_imu_stamp_ns:
            stamp_ns = self.last_imu_stamp_ns + 1
            output_stamp.sec, output_stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            self.dropped_imu_count += 1
            if self.dropped_imu_count <= 3:
                self.get_logger().warning("Adjusted non-monotonic FCU IMU timestamp")
        out = Imu()
        out.header.stamp = output_stamp
        out.header.frame_id = self.imu_frame_id
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.imu_pub.publish(out)
        self.last_imu_stamp_ns = stamp_ns
        self.imu_count += 1

    def _status(self):
        now = time.monotonic()
        elapsed = max(now - self.last_status_time, 1.0e-6)
        cloud_hz = (self.cloud_count - self.last_status_cloud_count) / elapsed
        imu_hz = (self.imu_count - self.last_status_imu_count) / elapsed
        stamp_delta_ms = 0.0
        if self.last_cloud_stamp_ns and self.last_imu_stamp_ns:
            stamp_delta_ms = (self.last_cloud_stamp_ns - self.last_imu_stamp_ns) / 1.0e6
        self.get_logger().info(
            f"livox bridge clouds={self.cloud_count} last_points={self.point_count_last} "
            f"scan_lines={self.scan_lines} cloud_hz={cloud_hz:.1f} imu_hz={imu_hz:.1f} "
            f"cloud_minus_imu_ms={stamp_delta_ms:.1f} adjusted_imu={self.dropped_imu_count}"
        )
        self.last_status_time = now
        self.last_status_cloud_count = self.cloud_count
        self.last_status_imu_count = self.imu_count


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
