"""Exact timestamp RGB-D frontend publishing measured feature tracks."""

from collections import OrderedDict
from dataclasses import dataclass
import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from uf_interfaces.msg import VisualFeatureTrack, VisualFeatureTracks

from .feature_tracker import RgbdFeatureTracker, grid_uniformity


CADENCE_PERIOD_S = {
    "conservative": 0.30,
    "balanced_light": 0.24,
    "balanced": 0.20,
    "balanced_plus": 0.16,
    "dense": 0.10,
}


@dataclass(frozen=True)
class CandidateQuality:
    valid: bool
    reason: str
    geometric_tracks: int
    valid_depth_tracks: int
    median_parallax_px: float
    spatial_distribution: float
    mean_reprojection_error_px: float
    pnp_inlier_ratio: float
    pnp_information_rank: int
    pnp_condition_number: float


def visual_candidate_quality(
    result,
    width,
    height,
    minimum_tracks=20,
    minimum_depth_tracks=20,
    minimum_spatial_distribution=0.08,
    minimum_parallax_px=0.15,
    maximum_reprojection_error_px=2.0,
    minimum_pnp_inlier_ratio=0.50,
    minimum_pnp_information_rank=6,
    maximum_pnp_condition_number=500.0,
    require_pnp=True,
):
    """Evaluate RGB-D measurement health without requiring camera motion.

    ``minimum_parallax_px`` remains in the API for configuration compatibility.
    Median parallax is still reported as a diagnostic, but stationary tracks are
    not a sensor fault: depth, PnP, spatial coverage and reprojection checks are
    sufficient to decide whether the candidate is usable.
    """
    _ = minimum_parallax_px
    geometric = np.asarray(result.geometric_inlier, dtype=bool)
    depth = np.asarray(result.depth_valid, dtype=bool)
    selected = geometric & depth
    geometric_tracks = int(np.count_nonzero(geometric))
    valid_depth_tracks = int(np.count_nonzero(selected))
    if valid_depth_tracks:
        displacement = (
            np.asarray(result.current_pixels)[selected]
            - np.asarray(result.previous_pixels)[selected]
        )
        median_parallax = float(np.median(np.linalg.norm(displacement, axis=1)))
        distribution = grid_uniformity(
            np.asarray(result.current_pixels)[selected], width, height
        )
        reprojection = np.asarray(result.reprojection_error)[selected]
        reprojection = reprojection[
            np.isfinite(reprojection) & (reprojection >= 0.0)
        ]
        mean_reprojection = (
            float(np.mean(reprojection)) if reprojection.size else math.inf
        )
    else:
        median_parallax = 0.0
        distribution = 0.0
        mean_reprojection = math.inf
    reason = "quality_valid"
    if require_pnp and result.rotation is None:
        reason = "pnp_invalid"
    elif geometric_tracks < int(minimum_tracks):
        reason = "insufficient_geometric_tracks"
    elif valid_depth_tracks < int(minimum_depth_tracks):
        reason = "insufficient_depth_tracks"
    elif float(result.pnp_inlier_ratio) < float(minimum_pnp_inlier_ratio):
        reason = "insufficient_pnp_inlier_ratio"
    elif int(result.pnp_information_rank) < int(minimum_pnp_information_rank):
        reason = "insufficient_pnp_information_rank"
    elif (
        not math.isfinite(float(result.pnp_condition_number))
        or float(result.pnp_condition_number)
        > float(maximum_pnp_condition_number)
    ):
        reason = "ill_conditioned_pnp_geometry"
    elif distribution < float(minimum_spatial_distribution):
        reason = "insufficient_spatial_coverage"
    elif (
        not math.isfinite(mean_reprojection)
        or mean_reprojection > float(maximum_reprojection_error_px)
    ):
        reason = "reprojection_quality"
    return CandidateQuality(
        valid=reason == "quality_valid",
        reason=reason,
        geometric_tracks=geometric_tracks,
        valid_depth_tracks=valid_depth_tracks,
        median_parallax_px=median_parallax,
        spatial_distribution=distribution,
        mean_reprojection_error_px=mean_reprojection,
        pnp_inlier_ratio=float(result.pnp_inlier_ratio),
        pnp_information_rank=int(result.pnp_information_rank),
        pnp_condition_number=float(result.pnp_condition_number),
    )


def inverse_depth_variance(depth_m, robust_depth_sigma_m, relative_sigma_ratio):
    """Propagate per-track depth uncertainty into inverse-depth variance."""
    depth = float(depth_m)
    if not math.isfinite(depth) or depth <= 0.0:
        return math.inf
    relative_sigma = max(0.0, float(relative_sigma_ratio)) * depth
    measured_sigma = float(robust_depth_sigma_m)
    if not math.isfinite(measured_sigma) or measured_sigma < 0.0:
        measured_sigma = 0.0
    depth_sigma = max(relative_sigma, measured_sigma)
    return max(1.0e-10, (depth_sigma / (depth * depth)) ** 2)


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
            "minimum_depth_m": 0.30,
            "maximum_depth_m": 6.0,
            "depth_neighborhood_radius_px": 1,
            "depth_minimum_support": 3,
            "depth_minimum_inlier_ratio": 0.60,
            "depth_inlier_absolute_tolerance_m": 0.03,
            "depth_inlier_relative_tolerance": 0.03,
            "depth_noise_floor_m": 0.005,
            "inverse_depth_sigma_ratio": 0.015,
            "pixel_sigma_px": 0.8,
            "keyframe_profile": "balanced",
            "keyframe_period_s": 0.10,
            "candidate_quality_enabled": True,
            "candidate_minimum_tracks": 20,
            "candidate_minimum_depth_tracks": 20,
            "candidate_minimum_spatial_distribution": 0.08,
            "candidate_minimum_parallax_px": 0.15,
            "candidate_maximum_reprojection_error_px": 2.0,
            "candidate_minimum_pnp_inlier_ratio": 0.50,
            "candidate_minimum_pnp_information_rank": 6,
            "candidate_maximum_pnp_condition_number": 500.0,
            "candidate_require_pnp": True,
            "diagnostic_topic": "/vision/frontend_diagnostics",
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
            self.get_parameter("depth_neighborhood_radius_px").value,
            self.get_parameter("depth_minimum_support").value,
            self.get_parameter("depth_minimum_inlier_ratio").value,
            self.get_parameter("depth_inlier_absolute_tolerance_m").value,
            self.get_parameter("depth_inlier_relative_tolerance").value,
            self.get_parameter("depth_noise_floor_m").value,
        )
        self.cache_size = int(self.get_parameter("cache_size").value)
        self.keyframe_profile = str(
            self.get_parameter("keyframe_profile").value
        ).lower()
        if self.keyframe_profile == "custom":
            self.keyframe_period_s = float(
                self.get_parameter("keyframe_period_s").value
            )
        elif self.keyframe_profile in CADENCE_PERIOD_S:
            self.keyframe_period_s = CADENCE_PERIOD_S[self.keyframe_profile]
        else:
            raise ValueError(
                "keyframe_profile must be conservative, balanced_light, "
                "balanced, balanced_plus, dense, or custom"
            )
        if self.keyframe_period_s <= 0.0:
            raise ValueError("keyframe_period_s must be positive")
        self.candidate_quality_enabled = bool(
            self.get_parameter("candidate_quality_enabled").value
        )
        self.color_cache = OrderedDict()
        self.depth_cache = OrderedDict()
        self.camera_info = None
        self.previous_header = None
        self.last_process_ns = None
        self.counts = {
            "raw_color_frames": 0,
            "raw_depth_frames": 0,
            "raw_frames": 0,
            "cadence_skipped": 0,
            "keyframe_candidates": 0,
            "tracking_initializations": 0,
            "tracked_frames": 0,
            "quality_valid_candidates": 0,
            "quality_rejected_candidates": 0,
            "published_candidates": 0,
        }
        self.quality_reasons = {}
        self.last_quality = CandidateQuality(
            False, "not_evaluated", 0, 0, 0.0, 0.0, math.inf,
            0.0, 0, math.inf
        )
        self.last_frontend_latency_s = math.inf
        self.publisher = self.create_publisher(
            VisualFeatureTracks, self.get_parameter("tracks_topic").value, 20
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostic_topic").value),
            10,
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
        self.create_timer(1.0, self._diagnostics)

    def _info(self, msg):
        if len(msg.k) == 9 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _insert(self, cache, msg):
        cache[stamp_ns(msg.header.stamp)] = msg
        while len(cache) > self.cache_size:
            cache.popitem(last=False)
        self._try_pair(stamp_ns(msg.header.stamp))

    def _color(self, msg):
        self.counts["raw_color_frames"] += 1
        self._insert(self.color_cache, msg)

    def _depth(self, msg):
        self.counts["raw_depth_frames"] += 1
        self._insert(self.depth_cache, msg)

    def _try_pair(self, key):
        if key not in self.color_cache or key not in self.depth_cache or self.camera_info is None:
            return
        color = self.color_cache.pop(key)
        depth = self.depth_cache.pop(key)
        self.counts["raw_frames"] += 1
        period_ns = int(self.keyframe_period_s * 1.0e9)
        if self.last_process_ns is not None and key - self.last_process_ns < period_ns:
            self.counts["cadence_skipped"] += 1
            return
        self.counts["keyframe_candidates"] += 1
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
            self.counts["tracking_initializations"] += 1
            self.previous_header = color.header
            return
        self.counts["tracked_frames"] += 1
        quality = visual_candidate_quality(
            result,
            color.width,
            color.height,
            minimum_tracks=self.get_parameter(
                "candidate_minimum_tracks").value,
            minimum_depth_tracks=self.get_parameter(
                "candidate_minimum_depth_tracks").value,
            minimum_spatial_distribution=self.get_parameter(
                "candidate_minimum_spatial_distribution").value,
            minimum_parallax_px=self.get_parameter(
                "candidate_minimum_parallax_px").value,
            maximum_reprojection_error_px=self.get_parameter(
                "candidate_maximum_reprojection_error_px").value,
            minimum_pnp_inlier_ratio=self.get_parameter(
                "candidate_minimum_pnp_inlier_ratio").value,
            minimum_pnp_information_rank=self.get_parameter(
                "candidate_minimum_pnp_information_rank").value,
            maximum_pnp_condition_number=self.get_parameter(
                "candidate_maximum_pnp_condition_number").value,
            require_pnp=self.get_parameter("candidate_require_pnp").value,
        )
        self.last_quality = quality
        if self.candidate_quality_enabled and not quality.valid:
            self.counts["quality_rejected_candidates"] += 1
            self.quality_reasons[quality.reason] = (
                self.quality_reasons.get(quality.reason, 0) + 1
            )
            self.previous_header = color.header
            return
        self.counts["quality_valid_candidates"] += 1
        message = VisualFeatureTracks()
        message.header = color.header
        if self.previous_header is not None:
            message.previous_stamp = self.previous_header.stamp
            message.previous_frame_id = self.previous_header.frame_id
        message.image_width = int(color.width)
        message.image_height = int(color.height)
        message.camera_matrix = [float(value)
                                 for value in camera_matrix.ravel()]
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
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
                track.inverse_depth_variance = inverse_depth_variance(
                    track.depth_m,
                    result.depth_sigma_m[index],
                    sigma_ratio,
                )
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
        if message.pnp_valid:
            message.pnp_rotation_previous_to_current = [
                float(value) for value in np.asarray(result.rotation).ravel()
            ]
            message.pnp_translation_previous_to_current_m = [
                float(value) for value in np.asarray(result.translation).ravel()
            ]
            message.pnp_inlier_count = int(
                np.count_nonzero(result.geometric_inlier)
            )
        self.publisher.publish(message)
        self.counts["published_candidates"] += 1
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        self.last_frontend_latency_s = max(
            0.0, now_s - stamp_ns(color.header.stamp) * 1.0e-9
        )
        self.previous_header = color.header

    @staticmethod
    def _key(name, value):
        return KeyValue(key=str(name), value=str(value))

    def _diagnostics(self):
        status = DiagnosticStatus()
        status.name = "uf_rgbd_feature_frontend"
        status.hardware_id = "d435i_rgbd"
        status.level = DiagnosticStatus.OK
        status.message = self.last_quality.reason
        values = dict(self.counts)
        values.update({
            "keyframe_profile": self.keyframe_profile,
            "keyframe_period_s": self.keyframe_period_s,
            "candidate_quality_enabled": self.candidate_quality_enabled,
            "last_geometric_tracks": self.last_quality.geometric_tracks,
            "last_valid_depth_tracks": self.last_quality.valid_depth_tracks,
            "last_median_parallax_px": self.last_quality.median_parallax_px,
            "last_spatial_distribution": self.last_quality.spatial_distribution,
            "last_mean_reprojection_error_px": (
                self.last_quality.mean_reprojection_error_px
            ),
            "last_pnp_inlier_ratio": self.last_quality.pnp_inlier_ratio,
            "last_pnp_information_rank": self.last_quality.pnp_information_rank,
            "last_pnp_condition_number": self.last_quality.pnp_condition_number,
            "last_frontend_latency_s": self.last_frontend_latency_s,
        })
        values.update({
            f"quality_rejected_{name}": count
            for name, count in sorted(self.quality_reasons.items())
        })
        status.values = [self._key(name, value) for name, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diagnostic_publisher.publish(array)


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
