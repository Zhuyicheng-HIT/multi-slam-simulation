import copy
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from uf_interfaces.msg import LioDiagnostics

from .geometry import (
    TemporalVoxelFilter,
    cloud_xyz,
    geometry_diagnostics,
    voxel_centroids,
    xyz_cloud,
)


class LioAdapter(Node):
    def __init__(self):
        super().__init__("lio_adapter")
        defaults = {
            "odom_input_topic": "/Odometry",
            "registered_cloud_topic": "/cloud_registered",
            "deskewed_cloud_topic": "/cloud_registered_body",
            "odom_output_topic": "/lio/odom",
            "path_output_topic": "/lio/path",
            "local_map_output_topic": "/lio/local_map",
            "deskewed_output_topic": "/lidar/points_deskewed",
            "diagnostics_topic": "/lio/diagnostics",
            "static_cloud_output_topic": "/lidar/static_cloud",
            "dynamic_cloud_output_topic": "/lidar/dynamic_cloud",
            "uncertain_cloud_output_topic": "/lidar/uncertain_cloud",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("voxel_size_m", 0.5)
        self.declare_parameter("max_diagnostic_points", 1200)
        self.declare_parameter("local_map_frames", 20)
        self.declare_parameter("local_map_publish_period_s", 1.0)
        self.declare_parameter("diagnostics_period_s", 1.0)
        self.declare_parameter("temporal_window_frames", 5)
        self.declare_parameter("temporal_min_static_support", 2)
        self.declare_parameter("temporal_neighbor_radius", 1)
        self.declare_parameter("path_publish_period_s", 1.0)
        self.declare_parameter("max_path_poses", 3000)

        self.voxel_size = float(self.get_parameter("voxel_size_m").value)
        self.max_points = int(self.get_parameter("max_diagnostic_points").value)
        self.max_path = int(self.get_parameter("max_path_poses").value)
        self.path = Path()
        self.previous_cloud = None
        self.map_frames = deque(maxlen=int(self.get_parameter("local_map_frames").value))
        self.last_cloud_header = None
        self.last_diagnostic_ns = None
        self.temporal_filter = TemporalVoxelFilter(
            window_frames=int(self.get_parameter("temporal_window_frames").value),
            min_static_support=int(self.get_parameter("temporal_min_static_support").value),
            neighbor_radius=int(self.get_parameter("temporal_neighbor_radius").value),
        )
        self.diagnostics_period_ns = int(
            float(self.get_parameter("diagnostics_period_s").value) * 1e9
        )

        self.odom_pub = self.create_publisher(Odometry, str(self.get_parameter("odom_output_topic").value), 20)
        self.path_pub = self.create_publisher(Path, str(self.get_parameter("path_output_topic").value), 10)
        self.map_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("local_map_output_topic").value), qos_profile_sensor_data
        )
        self.deskewed_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("deskewed_output_topic").value), qos_profile_sensor_data
        )
        self.diagnostic_pub = self.create_publisher(
            LioDiagnostics, str(self.get_parameter("diagnostics_topic").value), 20
        )
        self.static_cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("static_cloud_output_topic").value),
            qos_profile_sensor_data,
        )
        self.dynamic_cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("dynamic_cloud_output_topic").value),
            qos_profile_sensor_data,
        )
        self.uncertain_cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("uncertain_cloud_output_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_input_topic").value), self._odom, 20
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("registered_cloud_topic").value),
            self._registered,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("deskewed_cloud_topic").value),
            self._deskewed,
            qos_profile_sensor_data,
        )
        self.create_timer(float(self.get_parameter("local_map_publish_period_s").value), self._publish_map)
        self.create_timer(float(self.get_parameter("path_publish_period_s").value), self._publish_path)

    def _odom(self, msg):
        self.odom_pub.publish(msg)
        pose = PoseStamped()
        pose.header = copy.deepcopy(msg.header)
        pose.pose = copy.deepcopy(msg.pose.pose)
        self.path.header = copy.deepcopy(msg.header)
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_path:
            self.path.poses = self.path.poses[-self.max_path:]

    def _publish_path(self):
        if self.path.poses:
            self.path_pub.publish(self.path)

    def _deskewed(self, msg):
        self.deskewed_pub.publish(msg)

    def _registered(self, msg):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self.last_diagnostic_ns is not None:
            elapsed_ns = stamp_ns - self.last_diagnostic_ns
            if 0 <= elapsed_ns < self.diagnostics_period_ns:
                return
        self.last_diagnostic_ns = stamp_ns
        points = cloud_xyz(msg, self.max_points)
        metrics = geometry_diagnostics(points, self.previous_cloud, self.voxel_size)
        temporal = self.temporal_filter.classify(points, self.voxel_size)
        input_voxels = int(temporal["input_voxels"])
        static_count = len(temporal["static_points"])
        dynamic_count = len(temporal["dynamic_points"])
        uncertain_count = len(temporal["uncertain_points"])
        dynamic_ratio = dynamic_count / max(1, input_voxels)
        uncertain_ratio = uncertain_count / max(1, input_voxels)
        repeatability = float(temporal["feature_repeatability"])
        diagnostic = LioDiagnostics()
        diagnostic.header = copy.deepcopy(msg.header)
        diagnostic.input_points = int(msg.width) * int(msg.height)
        diagnostic.matched_points = int(metrics["matched_points"])
        diagnostic.residual_mean_m = float(metrics["residual_mean_m"])
        diagnostic.residual_median_m = float(metrics["residual_median_m"])
        diagnostic.residual_p95_m = float(metrics["residual_p95_m"])
        diagnostic.hessian_eigenvalues = [float(value) for value in metrics["hessian_eigenvalues"]]
        diagnostic.hessian_condition = float(metrics["hessian_condition"])
        diagnostic.normal_covariance_eigenvalues = [
            float(value) for value in metrics["normal_covariance_eigenvalues"]
        ]
        diagnostic.axial_penalty = float(metrics["axial_penalty"])
        diagnostic.spatial_coverage = float(metrics["spatial_coverage"])
        diagnostic.dynamic_ratio = float(dynamic_ratio)
        diagnostic.uncertain_ratio = float(uncertain_ratio)
        diagnostic.feature_repeatability = repeatability
        diagnostic.static_points = static_count
        diagnostic.dynamic_points = dynamic_count
        diagnostic.uncertain_points = uncertain_count
        temporal_quality = (1.0 - dynamic_ratio) * (0.5 + 0.5 * repeatability)
        diagnostic.map_quality = float(metrics["map_quality"] * temporal_quality)
        diagnostic.approximate = True
        diagnostic.source = "external_voxel_point_to_plane_temporal_persistence_proxy"
        self.diagnostic_pub.publish(diagnostic)
        self.static_cloud_pub.publish(xyz_cloud(temporal["static_points"], msg.header))
        self.dynamic_cloud_pub.publish(xyz_cloud(temporal["dynamic_points"], msg.header))
        self.uncertain_cloud_pub.publish(xyz_cloud(temporal["uncertain_points"], msg.header))
        self.previous_cloud = points
        if static_count:
            self.map_frames.append(temporal["static_points"])
            self.last_cloud_header = copy.deepcopy(msg.header)

    def _publish_map(self):
        if not self.map_frames or self.last_cloud_header is None:
            return
        points = np.concatenate(tuple(self.map_frames), axis=0)
        centroids = voxel_centroids(points, self.voxel_size)
        self.map_pub.publish(xyz_cloud(np.asarray(list(centroids.values())), self.last_cloud_header))


def main(args=None):
    rclpy.init(args=args)
    node = LioAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
