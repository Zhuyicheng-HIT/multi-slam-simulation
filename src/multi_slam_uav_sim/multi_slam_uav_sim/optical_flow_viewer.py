import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from mavros_msgs.msg import OpticalFlow
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class OpticalFlowViewer(Node):
    """Simple OpenCV window for checking the simulated optical-flow chain."""

    def __init__(self):
        super().__init__("optical_flow_viewer")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("flow_topic", "/sim/optical_flow/raw")
        self.declare_parameter("downsample", 0.25)
        self.declare_parameter("grid_cols", 9)
        self.declare_parameter("grid_rows", 7)
        self.declare_parameter("patch_px", 13)
        self.declare_parameter("search_px", 7)
        self.declare_parameter("min_texture_std", 2.0)
        self.declare_parameter("max_match_error", 0.55)
        self.declare_parameter("window_name", "multi-slam optical flow")

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_stamp = None
        self.last_flow_msg = None
        self.last_flow_wall_time = 0.0
        self.image_count = 0
        self.last_report_time = 0.0
        self.last_image_stamp = None

        self.downsample = float(self.get_parameter("downsample").value)
        self.grid_cols = max(1, int(self.get_parameter("grid_cols").value))
        self.grid_rows = max(1, int(self.get_parameter("grid_rows").value))
        self.patch_px = max(5, int(self.get_parameter("patch_px").value) | 1)
        self.search_px = max(2, int(self.get_parameter("search_px").value))
        self.min_texture_std = float(self.get_parameter("min_texture_std").value)
        self.max_match_error = float(self.get_parameter("max_match_error").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OpticalFlow,
            self.get_parameter("flow_topic").value,
            self._flow_cb,
            qos_profile_sensor_data,
        )
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 720)
        self.get_logger().info(
            "Optical-flow viewer active: "
            f"{self.get_parameter('image_topic').value}, "
            f"{self.get_parameter('flow_topic').value}"
        )

    def _flow_cb(self, msg):
        self.last_flow_msg = msg
        self.last_flow_wall_time = time.monotonic()

    def _stamp_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _gray_small(self, bgr):
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
            return []

        xs = np.linspace(margin, w - margin - 1, self.grid_cols).astype(int)
        ys = np.linspace(margin, h - margin - 1, self.grid_rows).astype(int)
        matches = []

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

                dx = float(min_loc[0] - self.search_px)
                dy = float(min_loc[1] - self.search_px)
                matches.append((cx, cy, dx, dy, texture, float(min_val)))
        return matches

    def _draw_status(self, image, dt, matches):
        quality = 0
        ground_distance = 0.0
        flow_x = 0.0
        flow_y = 0.0
        age = float("inf")
        if self.last_flow_msg is not None:
            quality = int(self.last_flow_msg.quality)
            ground_distance = float(self.last_flow_msg.ground_distance)
            flow_x = float(self.last_flow_msg.flow.x)
            flow_y = float(self.last_flow_msg.flow.y)
            age = time.monotonic() - self.last_flow_wall_time

        med = np.median(np.asarray([(m[2], m[3]) for m in matches], dtype=np.float32), axis=0) if matches else (0.0, 0.0)
        text = [
            f"blocks={len(matches)} median_px=({float(med[0]):.2f},{float(med[1]):.2f}) dt={dt:.3f}s",
            f"sim quality={quality} flow_rad=({flow_x:.5f},{flow_y:.5f}) dist={ground_distance:.2f}m age={age:.2f}s",
            "q/esc: close window",
        ]
        x0, y0 = 14, 28
        for i, line in enumerate(text):
            y = y0 + 26 * i
            cv2.putText(image, line, (x0 + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, line, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)

    def _image_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return

        gray = self._gray_small(bgr)
        self.image_count += 1
        stamp = self._stamp_seconds(msg.header.stamp)
        now_wall = time.monotonic()
        if now_wall - self.last_report_time > 3.0:
            stamp_delta = 0.0 if self.last_image_stamp is None else stamp - self.last_image_stamp
            self.get_logger().info(
                f"viewer images received: {self.image_count}, "
                f"stamp={stamp:.3f}, dt={stamp_delta:.3f}, gray_std={float(np.std(gray)):.1f}"
            )
            self.last_report_time = now_wall
        self.last_image_stamp = stamp
        matches = []
        dt = 0.0
        if self.prev_gray is not None and self.prev_stamp is not None:
            dt = stamp - self.prev_stamp
            if math.isfinite(dt) and 0.001 < dt <= 0.5:
                matches = self._block_displacements(self.prev_gray, gray)

        vis = bgr.copy()
        inv_scale = 1.0 / self.downsample if self.downsample > 0 else 1.0
        for cx, cy, dx, dy, texture, err in matches:
            x0 = int(round(cx * inv_scale))
            y0 = int(round(cy * inv_scale))
            x1 = int(round((cx + dx) * inv_scale))
            y1 = int(round((cy + dy) * inv_scale))
            color = (0, 255, 0) if err < self.max_match_error * 0.5 else (0, 210, 255)
            cv2.arrowedLine(vis, (x0, y0), (x1, y1), color, 2, cv2.LINE_AA, tipLength=0.35)
            cv2.circle(vis, (x0, y0), 3, (255, 0, 0), -1)

        self._draw_status(vis, dt, matches)
        cv2.imshow(self.window_name, vis)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            rclpy.shutdown()

        self.prev_gray = gray
        self.prev_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
