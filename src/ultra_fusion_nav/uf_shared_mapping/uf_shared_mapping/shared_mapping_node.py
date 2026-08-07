"""Opt-in online shared map; never publishes TF or mutates source maps."""

from collections import deque, OrderedDict
from pathlib import Path

from cv_bridge import CvBridge
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uf_interfaces.msg import ReliabilityScore

from .voxel_map import SourceAwareVoxelMap, write_ascii_pcd, write_summary


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def quaternion_rotation(q):
    values = np.asarray([q.w, q.x, q.y, q.z], dtype=float)
    values /= max(1.0e-12, np.linalg.norm(values))
    w, x, y, z = values
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SharedMappingNode(Node):
    def __init__(self):
        super().__init__("uf_shared_mapping")
        defaults = {
            "enabled": False,
            "lidar_topic": "/cloud_registered",
            "color_topic": "/sensors/rgbd/color",
            "depth_topic": "/sensors/rgbd/depth",
            "camera_info_topic": "/sensors/rgbd/camera_info",
            "pose_topic": "/fusion/unified/odom",
            "output_topic": "/mapping/shared/points",
            "map_frame": "camera_init",
            "output_directory": "shared_map_output",
            "voxel_size_m": 0.10,
            "conflict_distance_m": 0.18,
            "maximum_voxels": 500000,
            "minimum_visual_reliability": 0.35,
            "pose_tolerance_s": 0.08,
            "depth_scale": 0.001,
            "minimum_depth_m": 0.2,
            "maximum_depth_m": 12.0,
            "rgbd_pixel_stride": 4,
            "publish_period_s": 2.0,
            "rotation_body_camera": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "translation_body_camera_m": [0.0, 0.0, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.mapping = SourceAwareVoxelMap(
            self.get_parameter("voxel_size_m").value,
            self.get_parameter("conflict_distance_m").value,
            self.get_parameter("maximum_voxels").value,
            self.get_parameter("minimum_visual_reliability").value,
        )
        self.bridge = CvBridge()
        self.rotation_body_camera = np.asarray(
            self.get_parameter("rotation_body_camera").value, dtype=float
        ).reshape(3, 3)
        self.translation_body_camera = np.asarray(
            self.get_parameter("translation_body_camera_m").value, dtype=float
        )
        self.pose_buffer = deque(maxlen=400)
        self.colors = OrderedDict()
        self.depths = OrderedDict()
        self.camera_info = None
        self.visual_reliability = 0.0
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, 2
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            self._lidar,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            self.get_parameter("color_topic").value,
            self._color,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self._depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._info,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry,
            self.get_parameter("pose_topic").value,
            self._pose,
            50)
        self.create_subscription(ReliabilityScore, "/reliability/vision_score",
                                 self._score, qos_profile_sensor_data)
        self.create_service(Trigger, "/mapping/shared/export", self._export)
        self.create_timer(
            float(
                self.get_parameter("publish_period_s").value),
            self._publish)

    def _pose(self, msg):
        if not self.enabled:
            return
        pose = msg.pose.pose
        transform = np.eye(4)
        transform[:3, :3] = quaternion_rotation(pose.orientation)
        transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        self.pose_buffer.append((stamp_seconds(msg.header.stamp), transform))

    def _score(self, msg):
        self.visual_reliability = float(
            msg.reliability_weight) if msg.valid else 0.0

    def _info(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _insert(self, cache, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        cache[key] = msg
        while len(cache) > 10:
            cache.popitem(last=False)
        self._rgbd(key)

    def _color(self, msg):
        if self.enabled:
            self._insert(self.colors, msg)

    def _depth(self, msg):
        if self.enabled:
            self._insert(self.depths, msg)

    def _nearest_pose(self, stamp_s):
        if not self.pose_buffer:
            return None
        stamp, pose = min(
            self.pose_buffer, key=lambda item: abs(
                item[0] - stamp_s))
        if abs(
                stamp -
                stamp_s) > float(
                self.get_parameter("pose_tolerance_s").value):
            return None
        return pose

    def _lidar(self, msg):
        if not self.enabled:
            return
        points = np.asarray(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )), dtype=float)
        if points.size:
            self.mapping.integrate_lidar(
                points[:, :3], stamp_seconds(msg.header.stamp))

    def _rgbd(self, key):
        if key not in self.colors or key not in self.depths or self.camera_info is None:
            return
        color_msg, depth_msg = self.colors.pop(key), self.depths.pop(key)
        pose = self._nearest_pose(stamp_seconds(color_msg.header.stamp))
        if pose is None:
            return
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
        depth_raw = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough")
        depth = depth_raw.astype(np.float32) * (
            float(self.get_parameter("depth_scale").value)
            if depth_raw.dtype == np.uint16 else 1.0
        )
        stride = int(self.get_parameter("rgbd_pixel_stride").value)
        rows, columns = np.mgrid[0:depth.shape[0]
            :stride, 0:depth.shape[1]:stride]
        z = depth[rows, columns]
        valid = (
            np.isfinite(z)
            & (z >= float(self.get_parameter("minimum_depth_m").value))
            & (z <= float(self.get_parameter("maximum_depth_m").value))
        )
        rows, columns, z = rows[valid], columns[valid], z[valid]
        k = np.asarray(self.camera_info.k).reshape(3, 3)
        camera = np.c_[
            (columns - k[0, 2]) * z / k[0, 0],
            (rows - k[1, 2]) * z / k[1, 1], z,
        ]
        body = (self.rotation_body_camera @ camera.T).T + \
            self.translation_body_camera
        world = (pose[:3, :3] @ body.T).T + pose[:3, 3]
        self.mapping.integrate_rgbd(
            world, color[rows, columns], self.visual_reliability,
            stamp_seconds(color_msg.header.stamp),
        )

    def _publish(self):
        if not self.enabled:
            return
        points, colors = self.mapping.arrays("joint")
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("map_frame").value
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        rows = [(*point, (int(color[0]) << 16) | (int(color[1]) << 8)
                 | int(color[2])) for point, color in zip(points, colors)]
        self.publisher.publish(point_cloud2.create_cloud(header, fields, rows))

    def _export(self, request, response):
        del request
        root = Path(self.get_parameter(
            "output_directory").value).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        for source in ("lidar", "rgbd", "joint"):
            write_ascii_pcd(
                root / f"{source}_map.pcd",
                *self.mapping.arrays(source))
        write_summary(root / "metrics.json", self.mapping.summary())
        response.success = True
        response.message = str(root)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SharedMappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
