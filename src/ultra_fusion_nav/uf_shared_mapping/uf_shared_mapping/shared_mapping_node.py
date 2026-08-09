"""Opt-in online shared map; never publishes TF or mutates source maps."""

from collections import deque, OrderedDict
from pathlib import Path
import time

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


def structured_xyz_array(points):
    """Return contiguous XYZ rows from Humble structured or tuple points."""
    if isinstance(points, np.ndarray) and points.dtype.names:
        names = set(points.dtype.names)
        if not {"x", "y", "z"}.issubset(names):
            raise ValueError("PointCloud2 array is missing x/y/z fields")
        return np.column_stack(
            (points["x"], points["y"], points["z"])
        ).astype(float, copy=False)
    rows = list(points)
    if not rows:
        return np.empty((0, 3), dtype=float)
    array = np.asarray(rows)
    if array.dtype.names:
        return structured_xyz_array(array)
    return np.asarray(array[:, :3], dtype=float)


def structured_xyzrgb_array(points, colors, fields):
    """Pack XYZ and RGB directly into the PointCloud2 structured dtype."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    colors = np.asarray(colors, dtype=np.uint8).reshape(-1, 3)
    if points.shape[0] != colors.shape[0]:
        raise ValueError("point and color counts must match")
    rows = np.empty(
        points.shape[0], dtype=point_cloud2.dtype_from_fields(fields)
    )
    rows["x"] = points[:, 0]
    rows["y"] = points[:, 1]
    rows["z"] = points[:, 2]
    colors_u32 = colors.astype(np.uint32, copy=False)
    rows["rgb"] = (
        (colors_u32[:, 0] << 16)
        | (colors_u32[:, 1] << 8)
        | colors_u32[:, 2]
    )
    return rows


class SharedMappingNode(Node):
    def __init__(self):
        super().__init__("uf_shared_mapping")
        defaults = {
            "enabled": False,
            "lidar_enabled": True,
            "rgbd_enabled": True,
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
            "publish_when_unsubscribed": False,
            "occlusion_filter_enabled": True,
            "occlusion_lidar_tolerance_s": 0.12,
            "occlusion_azimuth_bin_deg": 0.5,
            "occlusion_elevation_bin_deg": 0.5,
            "occlusion_neighbor_bins": 0,
            "occlusion_margin_m": 0.40,
            "low_height_max_m": 1.5,
            "high_height_min_m": 2.5,
            "performance_profiling_enabled": False,
            "performance_profiling_capacity": 2048,
            "rotation_body_camera": [
                0.0, 0.0, 1.0,
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
            ],
            "translation_body_camera_m": [0.20, 0.0, 0.02],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.lidar_enabled = bool(self.get_parameter("lidar_enabled").value)
        self.rgbd_enabled = bool(self.get_parameter("rgbd_enabled").value)
        self.mapping = SourceAwareVoxelMap(
            self.get_parameter("voxel_size_m").value,
            self.get_parameter("conflict_distance_m").value,
            self.get_parameter("maximum_voxels").value,
            self.get_parameter("minimum_visual_reliability").value,
            self.get_parameter("occlusion_azimuth_bin_deg").value,
            self.get_parameter("occlusion_elevation_bin_deg").value,
            self.get_parameter("occlusion_neighbor_bins").value,
            self.get_parameter("occlusion_margin_m").value,
            self.get_parameter("low_height_max_m").value,
            self.get_parameter("high_height_min_m").value,
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
        self.latest_lidar_points = np.empty((0, 3), dtype=float)
        self.latest_lidar_stamp_s = None
        self.latest_lidar_frame = ""
        self.publish_skipped_unsubscribed = 0
        self.occlusion_frames = 0
        self.occlusion_frames_stale = 0
        self.performance_profiling_enabled = bool(
            self.get_parameter("performance_profiling_enabled").value
        )
        profile_capacity = max(
            64,
            int(self.get_parameter("performance_profiling_capacity").value),
        )
        self.profile_samples = {
            name: deque(maxlen=profile_capacity)
            for name in (
                "lidar_decode", "lidar_integrate", "rgbd_prepare",
                "rgbd_integrate", "publish_build", "publish_total",
            )
        }
        self.profile_trace = deque(maxlen=profile_capacity)
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
        if not self.rgbd_enabled:
            return
        self.visual_reliability = float(
            msg.reliability_weight) if msg.valid else 0.0

    def _info(self, msg):
        if not self.rgbd_enabled:
            return
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _insert(self, cache, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        cache[key] = msg
        while len(cache) > 10:
            cache.popitem(last=False)
        self._rgbd(key)

    def _color(self, msg):
        if self.enabled and self.rgbd_enabled:
            self._insert(self.colors, msg)

    def _depth(self, msg):
        if self.enabled and self.rgbd_enabled:
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

    def _profile_start(self):
        return time.perf_counter_ns() if self.performance_profiling_enabled else None

    def _profile_stop(self, name, started_ns, source_stamp_s=None):
        if started_ns is not None:
            duration_ms = (time.perf_counter_ns() - started_ns) * 1.0e-6
            self.profile_samples[name].append(duration_ms)
            now_s = self.get_clock().now().nanoseconds * 1.0e-9
            self.profile_trace.append({
                "kind": name,
                "ros_stamp_s": now_s,
                "source_stamp_s": source_stamp_s,
                "source_age_ms": (
                    max(0.0, (now_s - source_stamp_s) * 1000.0)
                    if source_stamp_s is not None else None
                ),
                "wall_monotonic_s": time.monotonic(),
                "duration_ms": duration_ms,
                "voxel_count": len(self.mapping.voxels),
            })

    def _profile_summary(self):
        summary = {}
        if not self.performance_profiling_enabled:
            return summary
        for name, samples in self.profile_samples.items():
            if not samples:
                continue
            values = np.fromiter(samples, dtype=float)
            summary[name] = {
                "count": int(values.size),
                "p50_ms": float(np.percentile(values, 50)),
                "p90_ms": float(np.percentile(values, 90)),
                "p95_ms": float(np.percentile(values, 95)),
                "max_ms": float(np.max(values)),
            }
        return summary

    def _lidar(self, msg):
        if not self.enabled or not self.lidar_enabled:
            return
        decode_started = self._profile_start()
        points = structured_xyz_array(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ))
        source_stamp_s = stamp_seconds(msg.header.stamp)
        self._profile_stop("lidar_decode", decode_started, source_stamp_s)
        if points.size:
            self.latest_lidar_points = points[:, :3].copy()
            self.latest_lidar_stamp_s = source_stamp_s
            self.latest_lidar_frame = str(msg.header.frame_id)
            integrate_started = self._profile_start()
            self.mapping.integrate_lidar(
                points[:, :3], stamp_seconds(msg.header.stamp))
            self._profile_stop("lidar_integrate", integrate_started, source_stamp_s)

    def _rgbd(self, key):
        if key not in self.colors or key not in self.depths or self.camera_info is None:
            return
        color_msg, depth_msg = self.colors.pop(key), self.depths.pop(key)
        pose = self._nearest_pose(stamp_seconds(color_msg.header.stamp))
        if pose is None:
            return
        prepare_started = self._profile_start()
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
        source_stamp_s = stamp_seconds(color_msg.header.stamp)
        self._profile_stop("rgbd_prepare", prepare_started, source_stamp_s)
        integrate_started = self._profile_start()
        sensor_origin = None
        occlusion_points = None
        if bool(self.get_parameter("occlusion_filter_enabled").value):
            lidar_tolerance = float(
                self.get_parameter("occlusion_lidar_tolerance_s").value)
            lidar_fresh = (
                self.latest_lidar_stamp_s is not None and
                abs(self.latest_lidar_stamp_s - source_stamp_s) <= lidar_tolerance
            )
            frame_matches = self.latest_lidar_frame.lstrip("/") == str(
                self.get_parameter("map_frame").value).lstrip("/")
            if lidar_fresh and frame_matches and len(self.latest_lidar_points):
                sensor_origin = (
                    pose[:3, :3] @ self.translation_body_camera + pose[:3, 3]
                )
                occlusion_points = self.latest_lidar_points
                self.occlusion_frames += 1
            else:
                self.occlusion_frames_stale += 1
        self.mapping.integrate_rgbd(
            world, color[rows, columns], self.visual_reliability,
            stamp_seconds(color_msg.header.stamp),
            sensor_origin=sensor_origin,
            occlusion_points=occlusion_points,
        )
        self._profile_stop("rgbd_integrate", integrate_started, source_stamp_s)

    def _publish(self):
        if not self.enabled:
            return
        if (not bool(self.get_parameter("publish_when_unsubscribed").value)
                and self.publisher.get_subscription_count() == 0):
            self.publish_skipped_unsubscribed += 1
            return
        publish_started = self._profile_start()
        build_started = self._profile_start()
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
        rows = structured_xyzrgb_array(points, colors, fields)
        self._profile_stop("publish_build", build_started)
        self.publisher.publish(point_cloud2.create_cloud(header, fields, rows))
        self._profile_stop("publish_total", publish_started)

    def _export(self, request, response):
        del request
        root = Path(self.get_parameter(
            "output_directory").value).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        for source in ("lidar", "rgbd", "joint"):
            write_ascii_pcd(
                root / f"{source}_map.pcd",
                *self.mapping.arrays(source))
        summary = self.mapping.summary()
        summary["runtime"] = {
            "publish_skipped_unsubscribed": self.publish_skipped_unsubscribed,
            "occlusion_frames": self.occlusion_frames,
            "occlusion_frames_stale_or_frame_mismatch": self.occlusion_frames_stale,
        }
        summary["performance_profile"] = self._profile_summary()
        summary["performance_trace"] = list(self.profile_trace)
        write_summary(root / "metrics.json", summary)
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
