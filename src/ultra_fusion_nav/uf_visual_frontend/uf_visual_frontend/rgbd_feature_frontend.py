"""Exact timestamp RGB-D frontend publishing measured feature tracks."""

from collections import OrderedDict

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from uf_interfaces.msg import VisualFeatureTrack, VisualFeatureTracks

from .feature_tracker import RgbdFeatureTracker, grid_uniformity


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ExactRgbdFeatureFrontend(Node):
    def __init__(self):
        super().__init__("uf_rgbd_feature_frontend")
        defaults = {
            "color_topic": "/sensors/rgbd/color",
            "depth_topic": "/sensors/rgbd/depth",
            "camera_info_topic": "/sensors/rgbd/camera_info",
            "tracks_topic": "/vision/feature_tracks",
            "max_features": 240,
            "minimum_distance_px": 12.0,
            "forward_backward_threshold_px": 1.0,
            "pnp_reprojection_threshold_px": 3.0,
            "minimum_pnp_points": 8,
            "depth_scale": 0.001,
            "minimum_depth_m": 0.15,
            "maximum_depth_m": 12.0,
            "inverse_depth_sigma_ratio": 0.015,
            "pixel_sigma_px": 0.8,
            "keyframe_period_s": 0.10,
            "cache_size": 12,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.tracker = RgbdFeatureTracker(
            self.get_parameter("max_features").value,
            self.get_parameter("minimum_distance_px").value,
            self.get_parameter("forward_backward_threshold_px").value,
            self.get_parameter("pnp_reprojection_threshold_px").value,
            self.get_parameter("minimum_pnp_points").value,
            self.get_parameter("depth_scale").value,
            self.get_parameter("minimum_depth_m").value,
            self.get_parameter("maximum_depth_m").value,
        )
        self.cache_size = int(self.get_parameter("cache_size").value)
        self.color_cache = OrderedDict()
        self.depth_cache = OrderedDict()
        self.camera_info = None
        self.previous_header = None
        self.last_process_ns = None
        self.publisher = self.create_publisher(
            VisualFeatureTracks, self.get_parameter("tracks_topic").value, 20
        )
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

    def _info(self, msg):
        if len(msg.k) == 9 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _insert(self, cache, msg):
        cache[stamp_ns(msg.header.stamp)] = msg
        while len(cache) > self.cache_size:
            cache.popitem(last=False)
        self._try_pair(stamp_ns(msg.header.stamp))

    def _color(self, msg):
        self._insert(self.color_cache, msg)

    def _depth(self, msg):
        self._insert(self.depth_cache, msg)

    def _try_pair(self, key):
        if key not in self.color_cache or key not in self.depth_cache or self.camera_info is None:
            return
        color = self.color_cache.pop(key)
        depth = self.depth_cache.pop(key)
        period_ns = int(
            float(
                self.get_parameter("keyframe_period_s").value) *
            1.0e9)
        if self.last_process_ns is not None and key - self.last_process_ns < period_ns:
            return
        self.last_process_ns = key
        try:
            image = self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth, desired_encoding="passthrough")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            camera_matrix = np.asarray(
                self.camera_info.k,
                dtype=float).reshape(
                3,
                3)
            result = self.tracker.process(gray, depth_image, camera_matrix)
        except Exception as error:
            self.get_logger().warning(f"RGB-D pair rejected: {error}")
            return
        if result is None:
            self.previous_header = color.header
            return
        message = VisualFeatureTracks()
        message.header = color.header
        if self.previous_header is not None:
            message.previous_stamp = self.previous_header.stamp
            message.previous_frame_id = self.previous_header.frame_id
        message.image_width = int(color.width)
        message.image_height = int(color.height)
        message.camera_matrix = [float(value)
                                 for value in camera_matrix.ravel()]
        fx, fy, cx, cy = camera_matrix[0, 0], camera_matrix[1,
                                                            1], camera_matrix[0, 2], camera_matrix[1, 2]
        mean_reprojection = []
        occupied_cells = set()
        for index in range(len(result.current_pixels)):
            track = VisualFeatureTrack()
            track.feature_id = int(result.feature_ids[index])
            previous = result.previous_pixels[index]
            current = result.current_pixels[index]
            track.previous_u, track.previous_v = map(float, previous)
            track.current_u, track.current_v = map(float, current)
            track.previous_x = float((previous[0] - cx) / fx)
            track.previous_y = float((previous[1] - cy) / fy)
            track.current_x = float((current[0] - cx) / fx)
            track.current_y = float((current[1] - cy) / fy)
            track.track_age = int(result.ages[index])
            track.forward_backward_error_px = float(
                result.forward_backward_error[index])
            track.klt_inlier = True
            track.depth_valid = bool(result.depth_valid[index])
            track.geometric_inlier = bool(result.geometric_inlier[index])
            track.reprojection_error_px = float(
                result.reprojection_error[index])
            if track.depth_valid:
                track.depth_m = float(result.depth_m[index])
                track.inverse_depth = 1.0 / track.depth_m
                sigma_ratio = float(self.get_parameter(
                    "inverse_depth_sigma_ratio").value)
                track.inverse_depth_variance = max(
                    1.0e-10, (sigma_ratio * track.inverse_depth) ** 2)
            column = min(7, max(0, int(current[0] * 8 / max(1, color.width))))
            row = min(7, max(0, int(current[1] * 8 / max(1, color.height))))
            track.grid_cell = row * 8 + column
            occupied_cells.add(track.grid_cell)
            if track.geometric_inlier and track.reprojection_error_px >= 0.0:
                mean_reprojection.append(track.reprojection_error_px)
            message.tracks.append(track)
        message.feature_count = len(message.tracks)
        message.valid_depth_count = sum(
            track.depth_valid for track in message.tracks)
        message.klt_inlier_ratio = float(
            np.mean(
                result.forward_backward_error <= self.tracker.fb_threshold_px)) if len(
            result.forward_backward_error) else 0.0
        message.spatial_distribution = grid_uniformity(
            result.current_pixels, color.width, color.height
        )
        message.mean_reprojection_error_px = (
            float(np.mean(mean_reprojection)) if mean_reprojection else -1.0
        )
        message.pnp_valid = result.rotation is not None
        self.publisher.publish(message)
        self.previous_header = color.header


def main(args=None):
    rclpy.init(args=args)
    node = ExactRgbdFeatureFrontend()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
