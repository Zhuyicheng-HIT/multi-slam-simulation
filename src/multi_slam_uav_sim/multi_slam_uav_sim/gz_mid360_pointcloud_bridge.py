import math
import struct
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as RosNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vec(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    qi = (-q[0], -q[1], -q[2], q[3])
    r = quat_multiply(quat_multiply(q, qv), qi)
    return (r[0], r[1], r[2])


class GzMid360PointCloudBridge(RosNode):
    def __init__(self):
        super().__init__("gz_mid360_pointcloud_bridge")
        self.declare_parameter("gz_topic", "/mid360/lidar")
        self.declare_parameter("raw_topic", "/sim/mid360/points_raw")
        self.declare_parameter("registered_topic", "/sim/mid360/cloud_registered")
        self.declare_parameter("odom_topic", "/sim/mid360/ground_truth_odom")
        self.declare_parameter("sensor_frame", "mid360_link")
        self.declare_parameter("map_frame", "camera_init")
        self.declare_parameter("gazebo_world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("gazebo_model", "apm_iris")
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_registered", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("restamp", True)

        self.gz_topic = self.get_parameter("gz_topic").value
        self.raw_topic = self.get_parameter("raw_topic").value
        self.registered_topic = self.get_parameter("registered_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.world_name = str(self.get_parameter("gazebo_world_name").value)
        self.model_name = str(self.get_parameter("gazebo_model").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.publish_registered = bool(self.get_parameter("publish_registered").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))
        self.restamp = bool(self.get_parameter("restamp").value)
        self.last_stamp_ns = 0
        self.adjusted_stamp_count = 0
        self.pose_lock = threading.Lock()
        self.latest_model_pose = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.raw_pub = self.create_publisher(PointCloud2, self.raw_topic, sensor_qos)
        self.registered_pub = self.create_publisher(PointCloud2, self.registered_topic, reliable_qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.gz_node = GzNode()
        self.gz_node.subscribe(GzLaserScan, self.gz_topic, self._scan_cb)
        self.gz_node.subscribe(
            Pose_V, f"/world/{self.world_name}/dynamic_pose/info", self._gz_pose_cb)
        self.gz_node.subscribe(
            Pose_V, f"/world/{self.world_name}/pose/info", self._gz_pose_cb)
        self.get_logger().info(
            f"Gazebo MID360 bridge active: {self.gz_topic} -> "
            f"{self.raw_topic}, {self.registered_topic}"
        )

    def _stamp(self, msg):
        stamp = self.get_clock().now().to_msg()
        if not self.restamp:
            try:
                stamp.sec = int(msg.header.stamp.sec)
                stamp.nanosec = int(msg.header.stamp.nsec)
            except Exception:
                pass
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + 1
            stamp.sec, stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            self.adjusted_stamp_count += 1
            if self.adjusted_stamp_count <= 3:
                self.get_logger().warning("Adjusted non-monotonic MID360 timestamp")
        self.last_stamp_ns = stamp_ns
        return stamp

    def _gz_pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == self.model_name or pose.name.endswith(f"::{self.model_name}"):
                with self.pose_lock:
                    self.latest_model_pose = pose
                return

    def _pose(self, msg):
        with self.pose_lock:
            pose = self.latest_model_pose
        if pose is None:
            pose = msg.world_pose
        q = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        p = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        if abs(q[3]) < 1.0e-9 and abs(q[0]) + abs(q[1]) + abs(q[2]) < 1.0e-9:
            q = (0.0, 0.0, 0.0, 1.0)
        return p, q

    def _points_from_scan(self, msg):
        h_count = max(1, int(msg.count))
        v_count = max(1, int(msg.vertical_count))
        h_step = float(msg.angle_step) if h_count > 1 else 0.0
        v_step = float(msg.vertical_angle_step) if v_count > 1 else 0.0
        h_min = float(msg.angle_min)
        v_min = float(msg.vertical_angle_min)
        r_min = float(msg.range_min)
        r_max = float(msg.range_max)
        ranges = msg.ranges
        intensities = msg.intensities
        point_count = min(len(ranges), h_count * v_count)

        points = []
        scan_period_s = 0.1
        for idx in range(0, point_count, self.point_stride):
            r = float(ranges[idx])
            if not math.isfinite(r) or r < r_min or r > r_max:
                continue
            v_idx = idx // h_count
            h_idx = idx - v_idx * h_count
            yaw = h_min + h_idx * h_step
            pitch = v_min + v_idx * v_step
            cp = math.cos(pitch)
            x = r * cp * math.cos(yaw)
            y = r * cp * math.sin(yaw)
            z = r * math.sin(pitch)
            intensity = float(intensities[idx]) if idx < len(intensities) else 0.0
            tag = 0x10
            line = v_idx
            ring = line
            point_time_s = (h_idx / max(1, h_count - 1)) * scan_period_s
            points.append((x, y, z, intensity, tag, line, ring, point_time_s))
        return points

    def _cloud_msg(self, points, stamp, frame_id):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
            PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
            PointField(name="ring", offset=18, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = b"".join(struct.pack("<ffffBBHf", *p) for p in points)
        return msg

    def _publish_pose(self, stamp, p, q):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.sensor_frame
        odom.pose.pose.position.x = p[0]
        odom.pose.pose.position.y = p[1]
        odom.pose.pose.position.z = p[2]
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.sensor_frame
            tf.transform.translation.x = p[0]
            tf.transform.translation.y = p[1]
            tf.transform.translation.z = p[2]
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(tf)

    def _scan_cb(self, msg):
        stamp = self._stamp(msg)
        sensor_points = self._points_from_scan(msg)
        pose_p, pose_q = self._pose(msg)

        if self.publish_raw:
            self.raw_pub.publish(self._cloud_msg(sensor_points, stamp, self.sensor_frame))

        self._publish_pose(stamp, pose_p, pose_q)

        if self.publish_registered:
            world_points = []
            px, py, pz = pose_p
            for point in sensor_points:
                rx, ry, rz = rotate_vec(pose_q, point[:3])
                world_points.append(
                    (rx + px, ry + py, rz + pz, point[3], point[4], point[5], point[6], point[7])
                )
            self.registered_pub.publish(self._cloud_msg(world_points, stamp, self.map_frame))


def main(args=None):
    rclpy.init(args=args)
    node = GzMid360PointCloudBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
