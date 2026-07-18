import math
import os
import time
import bisect
from collections import deque

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gz.msgs10.imu_pb2 import IMU as GzImu
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from mavros_msgs.msg import OpticalFlow, OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, Range

from multi_slam_uav_sim.optical_flow_model import (
    compensated_planar_velocity,
    integrate_gyro,
    pixel_flow_to_radians,
    ros_flu_gyro_to_sensor_frd,
    sensor_displacement_frd,
    synthesize_optical_flow_from_displacement,
    track_lk_flow,
)


class GazeboOpticalFlowToMavros(Node):
    """Image, gyro and range based model of an integrated optical-flow sensor."""

    def __init__(self):
        super().__init__("gazebo_optical_flow_to_mavros")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("flow_topic", "/sim/optical_flow/raw")
        self.declare_parameter("rad_topic", "/sim/optical_flow/rad")
        self.declare_parameter("range_topic", "/sim/optical_flow/range")
        self.declare_parameter("fcu_flow_topic", "")
        self.declare_parameter("fcu_range_topic", "")
        self.declare_parameter("imu_topic", "/mavros/imu/data_raw")
        self.declare_parameter("gazebo_imu_topic", "/flow/imu")
        self.declare_parameter("gazebo_range_topic", "/flow/range")
        self.declare_parameter("camera_fov_x_rad", 1.21126)
        self.declare_parameter("camera_width_px", 640)
        self.declare_parameter("camera_height_px", 480)
        self.declare_parameter("max_rate_hz", 30.0)
        self.declare_parameter("downsample", 0.5)
        self.declare_parameter("max_corners", 160)
        self.declare_parameter("feature_quality_level", 0.01)
        self.declare_parameter("min_feature_distance_px", 7.0)
        self.declare_parameter("forward_backward_threshold_px", 1.0)
        self.declare_parameter("max_track_error", 30.0)
        self.declare_parameter("max_displacement_px", 40.0)
        self.declare_parameter("min_inliers", 8)
        self.declare_parameter("min_depth_m", 0.08)
        self.declare_parameter("max_depth_m", 12.0)
        self.declare_parameter("range_timeout_s", 0.25)
        self.declare_parameter("use_gazebo_height", False)
        self.declare_parameter("gazebo_world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("gazebo_height_model", "apm_iris")
        self.declare_parameter("ground_z_m", 0.0)
        self.declare_parameter("use_physics_flow", True)
        self.declare_parameter("sensor_offset_z_down_m", 0.35)
        self.declare_parameter("physics_flow_noise_std_rad_s", 0.002)
        self.declare_parameter("noise_seed", 29)
        self.declare_parameter("publish_low_quality", True)
        self.declare_parameter("publish_rad_topic", True)
        self.declare_parameter("publish_range_topic", True)
        self.declare_parameter("publish_to_fcu", False)
        self.declare_parameter("mtf_min_distance_m", 0.08)
        self.declare_parameter("mtf_max_distance_m", 12.0)
        self.declare_parameter("mtf_fov_rad", 0.119428926)
        self.declare_parameter("mtf_max_speed_at_1m_mps", 7.0)
        self.declare_parameter("max_vertical_speed_for_quality_mps", 1.5)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("restamp_output", False)
        self.declare_parameter("max_imu_gap_s", 0.12)
        self.declare_parameter("debug", False)

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_time = None
        self.latest_depth = None
        self.latest_range = None
        self.latest_range_monotonic = None
        self.latest_gazebo_height = None
        self.latest_vertical_speed = 0.0
        self.last_height_sample = None
        self.last_model_pose_sample = None
        self.model_pose_samples = deque(maxlen=3000)
        self.reported_gazebo_height = False
        self.last_publish_time = 0.0
        self.gazebo_imu_samples = deque(maxlen=1000)
        self.fcu_imu_samples = deque(maxlen=1000)
        self.latest_gazebo_imu_monotonic = None

        self.fov_x = float(self.get_parameter("camera_fov_x_rad").value)
        self.camera_width = int(self.get_parameter("camera_width_px").value)
        self.camera_height = int(self.get_parameter("camera_height_px").value)
        fallback_fx = self.camera_width / (2.0 * math.tan(self.fov_x * 0.5))
        self.fx_full = fallback_fx
        self.fy_full = fallback_fx
        self.have_camera_info = False
        self.min_period = 1.0 / max(float(self.get_parameter("max_rate_hz").value), 1.0)
        self.downsample = float(self.get_parameter("downsample").value)
        self.max_corners = int(self.get_parameter("max_corners").value)
        self.feature_quality_level = float(self.get_parameter("feature_quality_level").value)
        self.min_feature_distance = float(self.get_parameter("min_feature_distance_px").value)
        self.fb_threshold = float(self.get_parameter("forward_backward_threshold_px").value)
        self.max_track_error = float(self.get_parameter("max_track_error").value)
        self.max_displacement = float(self.get_parameter("max_displacement_px").value)
        self.min_inliers = int(self.get_parameter("min_inliers").value)
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.range_timeout = float(self.get_parameter("range_timeout_s").value)
        self.use_gazebo_height = bool(self.get_parameter("use_gazebo_height").value)
        self.gazebo_height_model = str(self.get_parameter("gazebo_height_model").value)
        self.ground_z = float(self.get_parameter("ground_z_m").value)
        self.use_physics_flow = bool(self.get_parameter("use_physics_flow").value)
        self.sensor_lever_arm_frd = (
            0.0,
            0.0,
            float(self.get_parameter("sensor_offset_z_down_m").value),
        )
        self.physics_flow_noise_std = float(
            self.get_parameter("physics_flow_noise_std_rad_s").value
        )
        self.rng = np.random.default_rng(int(self.get_parameter("noise_seed").value))
        self.publish_low_quality = bool(self.get_parameter("publish_low_quality").value)
        self.publish_rad_topic = bool(self.get_parameter("publish_rad_topic").value)
        self.publish_range_topic = bool(self.get_parameter("publish_range_topic").value)
        self.publish_to_fcu = bool(self.get_parameter("publish_to_fcu").value)
        self.mtf_min_distance = float(self.get_parameter("mtf_min_distance_m").value)
        self.mtf_max_distance = float(self.get_parameter("mtf_max_distance_m").value)
        self.mtf_fov = float(self.get_parameter("mtf_fov_rad").value)
        self.mtf_max_speed_at_1m = float(self.get_parameter("mtf_max_speed_at_1m_mps").value)
        self.max_vertical_speed_for_quality = float(
            self.get_parameter("max_vertical_speed_for_quality_mps").value
        )
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.restamp_output = bool(self.get_parameter("restamp_output").value)
        self.max_imu_gap = float(self.get_parameter("max_imu_gap_s").value)
        self.debug = bool(self.get_parameter("debug").value)
        self.last_debug_time = 0.0

        self.reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.flow_pub = self.create_publisher(
            OpticalFlow, self.get_parameter("flow_topic").value, qos_profile_sensor_data
        )
        self.rad_pub = None
        if self.publish_rad_topic:
            self.rad_pub = self.create_publisher(
                OpticalFlowRad, self.get_parameter("rad_topic").value, qos_profile_sensor_data
            )
        self.range_pub = None
        if self.publish_range_topic:
            self.range_pub = self.create_publisher(
                Range, self.get_parameter("range_topic").value, qos_profile_sensor_data
            )
        fcu_flow_topic = str(self.get_parameter("fcu_flow_topic").value)
        self.fcu_flow_pub = None
        if self.publish_to_fcu and fcu_flow_topic:
            self.fcu_flow_pub = self.create_publisher(OpticalFlow, fcu_flow_topic, self.reliable_qos)
        fcu_range_topic = str(self.get_parameter("fcu_range_topic").value)
        self.fcu_range_pub = None
        if self.publish_to_fcu and fcu_range_topic:
            self.fcu_range_pub = self.create_publisher(Range, fcu_range_topic, qos_profile_sensor_data)

        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self._image_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self._depth_cb, qos_profile_sensor_data
        )
        imu_topic = str(self.get_parameter("imu_topic").value)
        if imu_topic:
            self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)

        self.gz_node = GzNode()
        gazebo_imu_topic = str(self.get_parameter("gazebo_imu_topic").value)
        if gazebo_imu_topic:
            self.gz_node.subscribe(GzImu, gazebo_imu_topic, self._gz_imu_cb)
        gazebo_range_topic = str(self.get_parameter("gazebo_range_topic").value)
        if gazebo_range_topic:
            self.gz_node.subscribe(GzLaserScan, gazebo_range_topic, self._gz_range_cb)
        if self.use_gazebo_height or self.use_physics_flow:
            world_name = str(self.get_parameter("gazebo_world_name").value)
            self.gz_node.subscribe(Pose_V, f"/world/{world_name}/dynamic_pose/info", self._gz_pose_cb)
            self.gz_node.subscribe(Pose_V, f"/world/{world_name}/pose/info", self._gz_pose_cb)

        self.get_logger().info(
            "Image/gyro/range optical-flow sensor active: "
            f"{self.get_parameter('image_topic').value} -> {self.get_parameter('rad_topic').value}"
        )
        if self.publish_to_fcu:
            self.get_logger().info(
                f"FCU optical-flow injection enabled: flow={fcu_flow_topic}, "
                f"range={fcu_range_topic or 'disabled'}"
            )

    def _stamp_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _camera_info_cb(self, msg):
        if len(msg.k) >= 6 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx_full = float(msg.k[0])
            self.fy_full = float(msg.k[4])
            self.camera_width = int(msg.width)
            self.camera_height = int(msg.height)
            self.have_camera_info = True

    def _depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}")

    def _imu_cb(self, msg):
        timestamp_s = self._stamp_seconds(msg.header.stamp)
        if timestamp_s <= 0.0:
            timestamp_s = self.get_clock().now().nanoseconds * 1.0e-9
        gyro_frd = ros_flu_gyro_to_sensor_frd((
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ))
        self.fcu_imu_samples.append((timestamp_s, *gyro_frd))

    def _gz_imu_cb(self, msg):
        try:
            timestamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nsec) * 1.0e-9
        except Exception:
            timestamp_s = self.get_clock().now().nanoseconds * 1.0e-9
        self.gazebo_imu_samples.append((
            timestamp_s,
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        ))
        self.latest_gazebo_imu_monotonic = time.monotonic()

    def _gz_range_cb(self, msg):
        values = [
            float(value) for value in msg.ranges
            if math.isfinite(float(value)) and self.min_depth <= float(value) <= self.max_depth
        ]
        if not values:
            return
        now = time.monotonic()
        measured = float(np.median(values))
        if self.latest_range is not None and self.latest_range_monotonic is not None:
            dt = now - self.latest_range_monotonic
            if 0.001 < dt < 1.0:
                self.latest_vertical_speed = (measured - self.latest_range) / dt
        self.latest_range = measured
        self.latest_range_monotonic = now

    def _gz_pose_cb(self, msg):
        now = time.monotonic()
        try:
            source_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nsec) * 1.0e-9
        except Exception:
            source_stamp = now
        for pose in msg.pose:
            if pose.name == self.gazebo_height_model or pose.name.endswith(
                f"::{self.gazebo_height_model}"
            ):
                position = np.asarray([
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ])
                quaternion = (
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                )
                if self.last_model_pose_sample is not None and source_stamp <= self.last_model_pose_sample:
                    return
                self.last_model_pose_sample = source_stamp
                self.model_pose_samples.append((
                    source_stamp,
                    tuple(float(value) for value in position),
                    quaternion,
                ))

                height = max(0.0, float(pose.position.z) - self.ground_z)
                if self.last_height_sample is not None:
                    previous_time, previous_height = self.last_height_sample
                    dt = now - previous_time
                    if 0.001 < dt < 1.0:
                        self.latest_vertical_speed = (height - previous_height) / dt
                self.last_height_sample = (now, height)
                self.latest_gazebo_height = height
                if self.use_gazebo_height and not self.reported_gazebo_height:
                    self.get_logger().warning("Using Gazebo model height as a range fallback")
                    self.reported_gazebo_height = True
                return

    def _pose_at(self, timestamp_s, max_gap_s=0.20):
        samples = list(self.model_pose_samples)
        if len(samples) < 2:
            return None
        times = [sample[0] for sample in samples]
        index = bisect.bisect_left(times, timestamp_s)
        if index == 0 or index >= len(samples):
            return None
        before = samples[index - 1]
        after = samples[index]
        dt = after[0] - before[0]
        if dt <= 0.0 or dt > max_gap_s:
            return None
        ratio = (timestamp_s - before[0]) / dt
        if ratio < 0.0 or ratio > 1.0:
            return None
        position = np.asarray(before[1], dtype=float) + ratio * (
            np.asarray(after[1], dtype=float) - np.asarray(before[1], dtype=float)
        )
        q0 = np.asarray(before[2], dtype=float)
        q1 = np.asarray(after[2], dtype=float)
        if float(np.dot(q0, q1)) < 0.0:
            q1 = -q1
        quaternion = q0 + ratio * (q1 - q0)
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1.0e-9:
            return None
        quaternion /= norm
        return tuple(float(value) for value in position), tuple(float(value) for value in quaternion)

    def _distance(self):
        now = time.monotonic()
        if (
            self.latest_range is not None
            and self.latest_range_monotonic is not None
            and now - self.latest_range_monotonic <= self.range_timeout
        ):
            return self.latest_range, now - self.latest_range_monotonic
        if self.latest_depth is not None:
            depth = np.asarray(self.latest_depth)
            if depth.dtype not in (np.float32, np.float64):
                depth = depth.astype(np.float32) * 0.001
            valid = np.isfinite(depth) & (depth >= self.min_depth) & (depth <= self.max_depth)
            if np.count_nonzero(valid) >= 64:
                return float(np.median(depth[valid])), 0.0
        if self.use_gazebo_height and self.latest_gazebo_height is not None:
            return max(self.min_depth, self.latest_gazebo_height), 0.0
        return -1.0, float("nan")

    def _gray_small(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if image.ndim == 3:
            if image.shape[2] == 4:
                conversion = (
                    cv2.COLOR_RGBA2GRAY if msg.encoding.lower().startswith("rgba")
                    else cv2.COLOR_BGRA2GRAY
                )
                gray = cv2.cvtColor(image, conversion)
            else:
                conversion = (
                    cv2.COLOR_RGB2GRAY if msg.encoding.lower().startswith("rgb")
                    else cv2.COLOR_BGR2GRAY
                )
                gray = cv2.cvtColor(image, conversion)
        else:
            gray = image
        if gray.dtype != np.uint8:
            gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        if 0.05 < self.downsample < 1.0:
            gray = cv2.resize(
                gray,
                None,
                fx=self.downsample,
                fy=self.downsample,
                interpolation=cv2.INTER_AREA,
            )
        return gray

    def _limited_quality(self, quality, distance, velocity):
        if distance < self.mtf_min_distance or distance > self.mtf_max_distance:
            return 0
        q = float(max(0, min(255, int(quality))))
        speed = math.hypot(*velocity) if all(math.isfinite(value) for value in velocity) else 0.0
        max_speed = max(0.05, self.mtf_max_speed_at_1m * max(distance, self.mtf_min_distance))
        if speed > max_speed:
            q *= max(0.0, max_speed / speed)
        vertical_speed = abs(float(self.latest_vertical_speed))
        if vertical_speed > self.max_vertical_speed_for_quality:
            q *= max(0.0, self.max_vertical_speed_for_quality / vertical_speed)
        return int(round(max(0.0, min(255.0, q))))

    def _range_msg(self, stamp, distance):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = "flow_range_link"
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(self.mtf_fov)
        msg.min_range = float(self.mtf_min_distance)
        msg.max_range = float(self.mtf_max_distance)
        msg.range = float(distance)
        return msg

    def _publish(self, stamp, tracking, start_s, end_s):
        output_stamp = self.get_clock().now().to_msg() if self.restamp_output else stamp
        integration_s = end_s - start_s
        scale = self.downsample if self.downsample > 0.0 else 1.0
        fx_small = self.fx_full * scale
        fy_small = self.fy_full * scale
        image_flow = pixel_flow_to_radians(
            tracking.dx_px, tracking.dy_px, fx_small, fy_small
        )
        use_gazebo_imu = (
            self.latest_gazebo_imu_monotonic is not None
            and time.monotonic() - self.latest_gazebo_imu_monotonic <= 0.25
        )
        gyro_samples = self.gazebo_imu_samples if use_gazebo_imu else self.fcu_imu_samples
        gyro_source = "gazebo_internal" if use_gazebo_imu else "fcu_fallback"
        gyro = integrate_gyro(list(gyro_samples), start_s, end_s, max_gap_s=self.max_imu_gap)
        gyro_valid = gyro is not None
        if gyro is None:
            gyro = (float("nan"), float("nan"), float("nan"))
        distance, distance_age_s = self._distance()
        raw_flow = image_flow
        flow_source = "image_lk"
        start_pose = self._pose_at(start_s)
        end_pose = self._pose_at(end_s)
        physics_pose_valid = self.use_physics_flow and start_pose is not None and end_pose is not None
        if physics_pose_valid and gyro_valid and distance > 0.0:
            displacement_frd = sensor_displacement_frd(
                start_pose,
                end_pose,
                self.sensor_lever_arm_frd,
            )
            synthesized = synthesize_optical_flow_from_displacement(
                displacement_frd,
                gyro,
                distance,
            )
            if synthesized is not None:
                noise_std = self.physics_flow_noise_std * math.sqrt(integration_s)
                noise = self.rng.normal(0.0, noise_std, size=2)
                raw_flow = (
                    float(synthesized[0] + noise[0]),
                    float(synthesized[1] + noise[1]),
                )
                flow_source = "gazebo_physics"
        raw_flow = (raw_flow[0] * self.angular_scale, raw_flow[1] * self.angular_scale)
        velocity = compensated_planar_velocity(
            raw_flow, gyro[:2], integration_s, distance
        ) if gyro_valid else (float("nan"), float("nan"))
        quality = tracking.quality
        if not gyro_valid:
            quality = int(round(quality * 0.7))
        quality = self._limited_quality(quality, distance, velocity)

        legacy = OpticalFlow()
        legacy.header.stamp = output_stamp
        legacy.header.frame_id = "base_link"
        dx_full = math.tan(raw_flow[0]) * self.fx_full
        dy_full = math.tan(raw_flow[1]) * self.fy_full
        legacy.flow.x = float(dx_full)
        legacy.flow.y = float(-dy_full)
        legacy.flow.z = 0.0
        legacy.flow_rate.x = float(raw_flow[0] / integration_s)
        legacy.flow_rate.y = float(-raw_flow[1] / integration_s)
        legacy.flow_rate.z = 0.0
        if all(math.isfinite(value) for value in velocity):
            legacy.flow_comp_m.x = float(velocity[0])
            legacy.flow_comp_m.y = float(-velocity[1])
        legacy.flow_comp_m.z = 0.0
        legacy.quality = max(0, min(255, int(quality)))
        legacy.ground_distance = float(distance)
        self.flow_pub.publish(legacy)
        if self.fcu_flow_pub is not None:
            self.fcu_flow_pub.publish(legacy)

        range_msg = self._range_msg(output_stamp, distance)
        if self.range_pub is not None:
            self.range_pub.publish(range_msg)
        if self.fcu_range_pub is not None:
            self.fcu_range_pub.publish(range_msg)

        if self.rad_pub is not None:
            rad = OpticalFlowRad()
            rad.header.stamp = output_stamp
            rad.header.frame_id = "flow_sensor_frd"
            rad.integration_time_us = int(max(1, round(integration_s * 1.0e6)))
            rad.integrated_x = float(raw_flow[0])
            rad.integrated_y = float(raw_flow[1])
            rad.integrated_xgyro = float(gyro[0])
            rad.integrated_ygyro = float(gyro[1])
            rad.integrated_zgyro = float(gyro[2])
            rad.temperature = 2500
            rad.quality = legacy.quality
            rad.time_delta_distance_us = (
                0 if not math.isfinite(distance_age_s)
                else int(max(0, round(distance_age_s * 1.0e6)))
            )
            rad.distance = float(distance)
            self.rad_pub.publish(rad)

        if self.debug and time.monotonic() - self.last_debug_time > 2.0:
            self.get_logger().info(
                f"flow_debug q={legacy.quality} tracks={tracking.inlier_count}/"
                f"{tracking.detected_count} source={flow_source} "
                f"image_rad=({image_flow[0]:.5f},{image_flow[1]:.5f}) "
                f"output_rad=({raw_flow[0]:.5f},{raw_flow[1]:.5f}) dt={integration_s:.4f}s "
                f"range={distance:.2f}m gyro={gyro_source} gyro_valid={gyro_valid}"
            )
            self.last_debug_time = time.monotonic()

    def _image_cb(self, msg):
        now = time.monotonic()
        if now - self.last_publish_time < self.min_period:
            return
        try:
            gray = self._gray_small(msg)
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return

        stamp_seconds = self._stamp_seconds(msg.header.stamp)
        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_time = stamp_seconds
            return
        dt = stamp_seconds - self.prev_time if self.prev_time is not None else 0.0
        if not math.isfinite(dt) or dt <= 0.001 or dt > 0.5:
            self.prev_gray = gray
            self.prev_time = stamp_seconds
            return

        tracking = track_lk_flow(
            self.prev_gray,
            gray,
            max_corners=self.max_corners,
            quality_level=self.feature_quality_level,
            min_feature_distance_px=self.min_feature_distance,
            fb_threshold_px=self.fb_threshold,
            max_track_error=self.max_track_error,
            max_displacement_px=self.max_displacement,
            min_inliers=self.min_inliers,
        )
        if tracking.quality > 0 or self.publish_low_quality:
            self._publish(msg.header.stamp, tracking, self.prev_time, stamp_seconds)
            self.last_publish_time = now
        self.prev_gray = gray
        self.prev_time = stamp_seconds


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOpticalFlowToMavros()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
