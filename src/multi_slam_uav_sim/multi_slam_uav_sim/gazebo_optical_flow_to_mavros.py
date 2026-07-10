import math
import os
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from mavros_msgs.msg import OpticalFlow, OpticalFlowRad
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, Range


class GazeboOpticalFlowToMavros(Node):
    """Lightweight block-matching optical-flow chip model.

    This intentionally avoids a full visual-odometry or feature-tracking stack.
    It mimics a small optical-flow sensor: downsample, test local texture, match
    small blocks in a limited search window, then publish one aggregate flow
    measurement with a quality score.
    """

    def __init__(self):
        super().__init__("gazebo_optical_flow_to_mavros")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("flow_topic", "/sim/optical_flow/raw")
        self.declare_parameter("rad_topic", "/sim/optical_flow/rad")
        self.declare_parameter("range_topic", "/sim/optical_flow/range")
        self.declare_parameter("fcu_flow_topic", "")
        self.declare_parameter("fcu_range_topic", "")
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        self.declare_parameter("camera_fov_x_rad", 1.21126)
        self.declare_parameter("max_rate_hz", 20.0)
        self.declare_parameter("downsample", 0.25)
        self.declare_parameter("grid_cols", 9)
        self.declare_parameter("grid_rows", 7)
        self.declare_parameter("patch_px", 13)
        self.declare_parameter("search_px", 7)
        self.declare_parameter("min_texture_std", 2.0)
        self.declare_parameter("max_match_error", 0.55)
        self.declare_parameter("min_blocks", 2)
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 40.0)
        self.declare_parameter("use_gazebo_height", True)
        self.declare_parameter("gazebo_world_name", "simple_apm_rgbd_mid360")
        self.declare_parameter("gazebo_height_model", "apm_iris")
        self.declare_parameter("ground_z_m", 0.0)
        self.declare_parameter("publish_low_quality", True)
        self.declare_parameter("publish_rad_topic", True)
        self.declare_parameter("publish_range_topic", True)
        self.declare_parameter("publish_to_fcu", False)
        self.declare_parameter("mtf_min_distance_m", 0.08)
        self.declare_parameter("mtf_max_distance_m", 12.0)
        self.declare_parameter("mtf_fov_rad", 0.73303828584)
        self.declare_parameter("mtf_max_speed_at_1m_mps", 7.0)
        self.declare_parameter("max_vertical_speed_for_quality_mps", 1.5)
        self.declare_parameter("angular_scale", 0.024)
        self.declare_parameter("debug", False)

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_time = None
        self.latest_depth = None
        self.latest_gazebo_height = None
        self.latest_vertical_speed = 0.0
        self.last_height_sample = None
        self.latest_gyro = (0.0, 0.0, 0.0)
        self.reported_gazebo_height = False
        self.last_publish_time = 0.0

        self.fov_x = float(self.get_parameter("camera_fov_x_rad").value)
        self.min_period = 1.0 / max(float(self.get_parameter("max_rate_hz").value), 1.0)
        self.downsample = float(self.get_parameter("downsample").value)
        self.grid_cols = max(1, int(self.get_parameter("grid_cols").value))
        self.grid_rows = max(1, int(self.get_parameter("grid_rows").value))
        self.patch_px = max(5, int(self.get_parameter("patch_px").value) | 1)
        self.search_px = max(2, int(self.get_parameter("search_px").value))
        self.min_texture_std = float(self.get_parameter("min_texture_std").value)
        self.max_match_error = float(self.get_parameter("max_match_error").value)
        self.min_blocks = max(1, int(self.get_parameter("min_blocks").value))
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.use_gazebo_height = bool(self.get_parameter("use_gazebo_height").value)
        self.gazebo_height_model = str(self.get_parameter("gazebo_height_model").value)
        self.ground_z = float(self.get_parameter("ground_z_m").value)
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
        self.debug = bool(self.get_parameter("debug").value)
        self.last_debug_time = 0.0
        self.gz_node = None

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
        self.fcu_flow_pub = None
        fcu_flow_topic = str(self.get_parameter("fcu_flow_topic").value)
        if self.publish_to_fcu and fcu_flow_topic:
            self.fcu_flow_pub = self.create_publisher(OpticalFlow, fcu_flow_topic, self.reliable_qos)
        self.fcu_range_pub = None
        fcu_range_topic = str(self.get_parameter("fcu_range_topic").value)
        if self.publish_to_fcu and fcu_range_topic:
            self.fcu_range_pub = self.create_publisher(Range, fcu_range_topic, qos_profile_sensor_data)
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self._depth_cb,
            qos_profile_sensor_data,
        )
        imu_topic = str(self.get_parameter("imu_topic").value)
        if imu_topic:
            self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)
        if self.use_gazebo_height:
            self.gz_node = GzNode()
            world_name = str(self.get_parameter("gazebo_world_name").value)
            self.gz_node.subscribe(
                Pose_V,
                f"/world/{world_name}/dynamic_pose/info",
                self._gz_pose_cb,
            )
            self.gz_node.subscribe(
                Pose_V,
                f"/world/{world_name}/pose/info",
                self._gz_pose_cb,
            )
        self.get_logger().info(
            "Block optical-flow chip active: "
            f"{self.get_parameter('image_topic').value} -> "
            f"{self.get_parameter('flow_topic').value}"
        )
        if self.publish_to_fcu:
            self.get_logger().info(
                "FCU optical-flow injection enabled: "
                f"flow={fcu_flow_topic or 'disabled'}, range={fcu_range_topic or 'disabled'}"
            )

    def _subpixel_minimum(self, result, min_loc):
        x, y = min_loc
        h, w = result.shape[:2]

        def offset(v0, v1, v2):
            denom = float(v0 - 2.0 * v1 + v2)
            if abs(denom) < 1.0e-9:
                return 0.0
            return max(-0.5, min(0.5, 0.5 * float(v0 - v2) / denom))

        ox = 0.0
        oy = 0.0
        if 0 < x < w - 1:
            ox = offset(result[y, x - 1], result[y, x], result[y, x + 1])
        if 0 < y < h - 1:
            oy = offset(result[y - 1, x], result[y, x], result[y + 1, x])
        return ox, oy

    def _depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}")

    def _imu_cb(self, msg):
        self.latest_gyro = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )

    def _gz_pose_cb(self, msg):
        now = time.monotonic()
        for pose in msg.pose:
            if pose.name == self.gazebo_height_model or pose.name.endswith(f"::{self.gazebo_height_model}"):
                height = max(0.0, float(pose.position.z) - self.ground_z)
                if self.last_height_sample is not None:
                    prev_t, prev_h = self.last_height_sample
                    dt = now - prev_t
                    if 0.001 < dt < 1.0:
                        self.latest_vertical_speed = (height - prev_h) / dt
                self.last_height_sample = (now, height)
                self.latest_gazebo_height = height
                if not self.reported_gazebo_height:
                    self.get_logger().info(
                        f"Gazebo height source [{pose.name}] = {self.latest_gazebo_height:.2f}m"
                    )
                    self.reported_gazebo_height = True
                return

    def _stamp_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _median_depth(self):
        if self.use_gazebo_height and self.latest_gazebo_height is not None:
            if 0.0 <= self.latest_gazebo_height <= self.max_depth:
                return max(self.min_depth, self.latest_gazebo_height)
        if self.latest_depth is None:
            return float("nan")
        depth = np.asarray(self.latest_depth)
        if depth.dtype != np.float32 and depth.dtype != np.float64:
            depth = depth.astype(np.float32) * 0.001
        valid = np.isfinite(depth) & (depth >= self.min_depth) & (depth <= self.max_depth)
        if np.count_nonzero(valid) < 64:
            return float("nan")
        return float(np.median(depth[valid]))

    def _gray_small(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if 0.05 < self.downsample < 1.0:
            gray = cv2.resize(
                gray,
                None,
                fx=self.downsample,
                fy=self.downsample,
                interpolation=cv2.INTER_AREA,
            )
        return gray

    def _block_displacements(self, prev, curr):
        h, w = prev.shape[:2]
        half = self.patch_px // 2
        margin = half + self.search_px + 2
        if h <= 2 * margin or w <= 2 * margin:
            return [], [], []

        xs = np.linspace(margin, w - margin - 1, self.grid_cols).astype(int)
        ys = np.linspace(margin, h - margin - 1, self.grid_rows).astype(int)
        displacements = []
        textures = []
        errors = []

        for cy in ys:
            for cx in xs:
                patch = prev[cy - half : cy + half + 1, cx - half : cx + half + 1]
                texture = float(np.std(patch))
                if texture < self.min_texture_std:
                    continue

                roi = curr[
                    cy - half - self.search_px : cy + half + self.search_px + 1,
                    cx - half - self.search_px : cx + half + self.search_px + 1,
                ]
                if roi.shape[0] < patch.shape[0] or roi.shape[1] < patch.shape[1]:
                    continue
                result = cv2.matchTemplate(roi, patch, cv2.TM_SQDIFF_NORMED)
                min_val, _, min_loc, _ = cv2.minMaxLoc(result)
                if not math.isfinite(min_val) or min_val > self.max_match_error:
                    continue

                ox, oy = self._subpixel_minimum(result, min_loc)
                dx = float(min_loc[0] + ox - self.search_px)
                dy = float(min_loc[1] + oy - self.search_px)
                displacements.append((dx, dy))
                textures.append(texture)
                errors.append(float(min_val))

        if len(displacements) < self.min_blocks:
            return displacements, textures, errors

        disp = np.asarray(displacements, dtype=np.float32)
        med = np.median(disp, axis=0)
        residual = np.linalg.norm(disp - med, axis=1)
        keep = residual <= max(1.5, float(np.median(residual)) * 2.5 + 0.5)
        return disp[keep].tolist(), np.asarray(textures)[keep].tolist(), np.asarray(errors)[keep].tolist()

    def _quality(self, block_count, textures, errors):
        if block_count < self.min_blocks:
            return 0
        texture_score = min(1.0, max(0.0, (float(np.median(textures)) - self.min_texture_std) / 35.0))
        error_score = min(1.0, max(0.0, 1.0 - float(np.median(errors)) / self.max_match_error))
        count_score = min(1.0, block_count / float(self.grid_cols * self.grid_rows))
        return int(round(255.0 * (0.45 * texture_score + 0.35 * error_score + 0.20 * count_score)))

    def _mtf_limited_quality(self, quality, ground_distance, flow_rate_x, flow_rate_y):
        if ground_distance < self.mtf_min_distance or ground_distance > self.mtf_max_distance:
            return 0
        q = float(max(0, min(255, int(quality))))
        planar_speed = math.hypot(flow_rate_x * ground_distance, flow_rate_y * ground_distance)
        max_speed = max(0.05, self.mtf_max_speed_at_1m * max(ground_distance, self.mtf_min_distance))
        if planar_speed > max_speed:
            q *= max(0.0, max_speed / planar_speed)
        vz = abs(float(self.latest_vertical_speed))
        if vz > self.max_vertical_speed_for_quality:
            q *= max(0.0, self.max_vertical_speed_for_quality / vz)
        return int(round(max(0.0, min(255.0, q))))

    def _range_msg(self, stamp, frame_id, ground_distance):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id or "flow_camera_link"
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(self.mtf_fov)
        msg.min_range = float(self.mtf_min_distance)
        msg.max_range = float(self.mtf_max_distance)
        msg.range = float(ground_distance)
        return msg

    def _publish(self, stamp, frame_id, flow_px, dt, quality):
        ground_distance = self._median_depth()
        if not math.isfinite(ground_distance):
            ground_distance = 0.0

        # Convert downsampled-pixel displacement to full-resolution angular displacement.
        dx_full = flow_px[0] / self.downsample if self.downsample > 0 else flow_px[0]
        dy_full = flow_px[1] / self.downsample if self.downsample > 0 else flow_px[1]
        width_full = 640.0
        height_full = 480.0
        fx = width_full / (2.0 * math.tan(self.fov_x * 0.5))
        fov_y = 2.0 * math.atan(height_full / (2.0 * fx))
        fy = height_full / (2.0 * math.tan(fov_y * 0.5))

        flow_x = float(dx_full / fx) * self.angular_scale
        flow_y = float(dy_full / fy) * self.angular_scale
        flow_rate_x = flow_x / dt
        flow_rate_y = flow_y / dt
        quality = self._mtf_limited_quality(quality, ground_distance, flow_rate_x, flow_rate_y)

        out = OpticalFlow()
        out.header.stamp = stamp
        out.header.frame_id = frame_id or "camera_color_optical_frame"
        out.flow.x = flow_x
        out.flow.y = flow_y
        out.flow.z = 0.0
        out.flow_rate.x = flow_rate_x
        out.flow_rate.y = flow_rate_y
        out.flow_rate.z = 0.0
        # MAVLink OPTICAL_FLOW flow_comp_m_* fields are compensated angular flow in radians.
        out.flow_comp_m.x = flow_x
        out.flow_comp_m.y = flow_y
        out.flow_comp_m.z = 0.0
        out.quality = max(0, min(255, int(quality)))
        out.ground_distance = float(ground_distance)
        self.flow_pub.publish(out)
        if self.fcu_flow_pub is not None:
            self.fcu_flow_pub.publish(out)

        range_msg = self._range_msg(stamp, out.header.frame_id, ground_distance)
        if self.range_pub is not None:
            self.range_pub.publish(range_msg)
        if self.fcu_range_pub is not None:
            self.fcu_range_pub.publish(range_msg)

        if self.rad_pub is not None:
            gx, gy, gz = self.latest_gyro
            rad = OpticalFlowRad()
            rad.header = out.header
            rad.integration_time_us = int(max(1, round(dt * 1.0e6)))
            rad.integrated_x = out.flow_comp_m.x
            rad.integrated_y = out.flow_comp_m.y
            rad.integrated_xgyro = float(gx * dt)
            rad.integrated_ygyro = float(gy * dt)
            rad.integrated_zgyro = float(gz * dt)
            rad.temperature = 25
            rad.quality = out.quality
            rad.time_delta_distance_us = rad.integration_time_us
            rad.distance = float(ground_distance)
            self.rad_pub.publish(rad)

    def _image_cb(self, msg):
        now = time.monotonic()
        if now - self.last_publish_time < self.min_period:
            return
        try:
            gray = self._gray_small(msg)
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return

        stamp = msg.header.stamp
        stamp_seconds = self._stamp_seconds(stamp)
        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_time = stamp_seconds
            return

        dt = stamp_seconds - self.prev_time if self.prev_time is not None else 0.0
        if not math.isfinite(dt) or dt <= 0.001 or dt > 0.5:
            self.prev_gray = gray
            self.prev_time = stamp_seconds
            return

        displacements, textures, errors = self._block_displacements(self.prev_gray, gray)
        quality = self._quality(len(displacements), textures, errors)
        if self.debug and now - self.last_debug_time > 2.0:
            med_disp = np.median(np.asarray(displacements, dtype=np.float32), axis=0) if displacements else (0.0, 0.0)
            med_tex = float(np.median(textures)) if textures else 0.0
            med_err = float(np.median(errors)) if errors else 0.0
            self.get_logger().info(
                f"flow_debug blocks={len(displacements)} quality={quality} "
                f"median_px=({float(med_disp[0]):.2f},{float(med_disp[1]):.2f}) "
                f"texture={med_tex:.1f} error={med_err:.3f}"
            )
            self.last_debug_time = now
        if quality > 0:
            flow_px = np.median(np.asarray(displacements, dtype=np.float32), axis=0)
            self._publish(stamp, msg.header.frame_id, flow_px, dt, quality)
            self.last_publish_time = now
        elif self.publish_low_quality:
            self._publish(stamp, msg.header.frame_id, (0.0, 0.0), dt, 0)
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
