#!/usr/bin/env python3
import copy
import csv
import os
import threading
import time
from pathlib import Path

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.imu_pb2 import IMU as GzImu
from gz.transport13 import Node as GzNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


PIXEL_ENCODINGS = {
    1: "mono8", 2: "mono16", 3: "rgb8", 4: "rgba8",
    5: "bgra8", 8: "bgr8", 11: "16FC1", 13: "32FC1",
}


class D435iSimBridge(Node):
    def __init__(self):
        super().__init__("d435i_sim_bridge")
        self.declare_parameter("gz_prefix", "/front/d435i/gz")
        self.declare_parameter("ros_prefix", "/front/d435i")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("pointcloud_hz", 10.0)
        self.declare_parameter("pointcloud_stride", 4)
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("depth_encoding", "16UC1")
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("enable_pointcloud", True)
        self.declare_parameter("performance_stats_enabled", False)
        self.declare_parameter("performance_stats_period_s", 5.0)
        self.declare_parameter("performance_csv_path", "")
        self.gz_prefix = str(self.get_parameter("gz_prefix").value).rstrip("/")
        self.ros_prefix = str(self.get_parameter("ros_prefix").value).rstrip("/")
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.pointcloud_interval = 1.0 / max(float(self.get_parameter("pointcloud_hz").value), 0.1)
        self.pointcloud_stride = max(int(self.get_parameter("pointcloud_stride").value), 1)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.depth_encoding = str(
            self.get_parameter("depth_encoding").value).upper()
        qos_reliability = str(
            self.get_parameter("qos_reliability").value).lower()
        qos_depth = max(int(self.get_parameter("qos_depth").value), 1)
        if self.depth_encoding not in ("16UC1", "32FC1"):
            raise ValueError("depth_encoding must be 16UC1 or 32FC1")
        if qos_reliability not in ("best_effort", "reliable"):
            raise ValueError(
                "qos_reliability must be best_effort or reliable")
        self.enable_pointcloud = bool(self.get_parameter("enable_pointcloud").value)
        self.performance_stats_enabled = bool(
            self.get_parameter("performance_stats_enabled").value)
        self.performance_stats_period_s = max(
            float(self.get_parameter("performance_stats_period_s").value), 1.0)
        self.performance_csv_path = str(
            self.get_parameter("performance_csv_path").value).strip()

        self.color_frame = "front_d435i_color_optical_frame"
        self.depth_frame = "front_d435i_depth_optical_frame"
        self.imu_frame = "front_d435i_imu_frame"
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=(ReliabilityPolicy.RELIABLE
                         if qos_reliability == "reliable"
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
        )
        self.color_pub = self.create_publisher(Image, f"{self.ros_prefix}/color/image_raw", self.qos)
        self.color_info_pub = self.create_publisher(CameraInfo, f"{self.ros_prefix}/color/camera_info", self.qos)
        self.depth_pub = self.create_publisher(Image, f"{self.ros_prefix}/depth/image_rect_raw", self.qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, f"{self.ros_prefix}/depth/camera_info", self.qos)
        self.aligned_pub = self.create_publisher(
            Image, f"{self.ros_prefix}/aligned_depth_to_color/image_raw", self.qos)
        self.points_pub = None
        if self.enable_pointcloud:
            self.points_pub = self.create_publisher(
                PointCloud2, f"{self.ros_prefix}/depth/color/points", self.qos)
        self.gyro_pub = self.create_publisher(Imu, f"{self.ros_prefix}/gyro/sample", self.qos)
        self.accel_pub = self.create_publisher(Imu, f"{self.ros_prefix}/accel/sample", self.qos)
        self.imu_pub = self.create_publisher(Imu, f"{self.ros_prefix}/imu", self.qos)

        self.lock = threading.Lock()
        self.color = None
        self.depth = None
        self.info = None
        self.color_seq = self.depth_seq = 0
        self.published_color_seq = self.published_depth_seq = 0
        self.last_cloud_time = -1.0
        self.last_report_time = 0.0
        self.stats_lock = threading.Lock()
        self.stats_window_start = time.perf_counter()
        self.stats = self._empty_stats()
        self.stats_csv_fields = [
            "wall_time", "sim_time_s", "window_s",
            "gazebo_color_callback_hz", "gazebo_depth_callback_hz",
            "rgbd_pair_hz", "ros_color_publish_hz", "ros_depth_publish_hz",
            "skip_color_not_updated", "skip_depth_not_updated",
            "color_conversion_mean_ms", "depth_conversion_mean_ms",
            "deepcopy_mean_ms", "camera_info_mean_ms",
            "pointcloud_mean_ms", "publish_mean_ms",
            "pointcloud_count", "color_seq", "depth_seq",
            "seq_delta_mean", "seq_delta_max", "conversion_errors",
        ]
        self._initialize_stats_csv()
        self.gz_node = GzNode()
        self.gz_topics = [
            f"{self.gz_prefix}/image",
            self.gz_prefix,
            f"{self.gz_prefix}/depth_image",
            f"{self.gz_prefix}/camera_info",
            f"{self.gz_prefix}/imu",
        ]
        self.gz_node.subscribe(GzImage, self.gz_topics[0], self._color_cb)
        self.gz_node.subscribe(GzImage, self.gz_topics[1], self._color_cb)
        self.gz_node.subscribe(GzImage, self.gz_topics[2], self._depth_cb)
        self.gz_node.subscribe(GzCameraInfo, self.gz_topics[3], self._info_cb)
        self.gz_node.subscribe(GzImu, self.gz_topics[4], self._imu_cb)
        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._publish_images)
        if self.performance_stats_enabled:
            self.create_timer(
                self.performance_stats_period_s, self._report_performance)
        self._publish_static_tf()
        self.get_logger().info(
            f"D435i simulation bridge active: {self.gz_prefix} -> {self.ros_prefix}")

    def _stamp(self, msg):
        stamp = self.get_clock().now().to_msg()
        try:
            stamp.sec = msg.header.stamp.sec
            stamp.nanosec = msg.header.stamp.nsec
        except Exception:
            pass
        return stamp

    def _color_cb(self, msg):
        if not rclpy.ok():
            return
        with self.lock:
            self.color = msg
            self.color_seq += 1
        self._increment_stat("color_callbacks")

    def _depth_cb(self, msg):
        if not rclpy.ok():
            return
        with self.lock:
            self.depth = msg
            self.depth_seq += 1
        self._increment_stat("depth_callbacks")

    def _info_cb(self, msg):
        if not rclpy.ok():
            return
        with self.lock:
            self.info = msg

    def _image(self, msg, frame_id):
        out = Image()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = frame_id
        out.height = int(msg.height)
        out.width = int(msg.width)
        out.encoding = PIXEL_ENCODINGS.get(int(msg.pixel_format_type), "passthrough")
        out.is_bigendian = 0
        out.step = int(msg.step)
        out.data = bytes(msg.data)
        return out

    def _depth(self, msg):
        width, height = int(msg.width), int(msg.height)
        if int(msg.step) == width * 4:
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(height, width)
        elif int(msg.step) == width * 2:
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
            depth_m = raw.astype(np.float32) * 0.001
        else:
            raise ValueError(f"unsupported depth step {msg.step} for width {width}")
        valid = np.isfinite(depth_m) & (depth_m >= 0.105) & (depth_m <= self.max_depth_m)
        out = Image()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = self.depth_frame
        out.height, out.width = height, width
        out.is_bigendian = 0
        if self.depth_encoding == "32FC1":
            depth_out = np.full(depth_m.shape, np.nan, dtype=np.float32)
            depth_out[valid] = depth_m[valid]
            out.encoding = "32FC1"
            out.step = width * 4
            out.data = depth_out.tobytes()
        else:
            depth_mm = np.zeros(depth_m.shape, dtype=np.uint16)
            depth_mm[valid] = np.rint(
                depth_m[valid] * 1000.0).astype(np.uint16)
            out.encoding = "16UC1"
            out.step = width * 2
            out.data = depth_mm.tobytes()
        return out, depth_m, valid

    def _camera_info(self, msg, frame_id):
        out = CameraInfo()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = frame_id
        out.width, out.height = int(msg.width), int(msg.height)
        out.distortion_model = "plumb_bob"
        try:
            out.k = list(msg.intrinsics.k)
            out.p = list(msg.projection.p)
            out.r = list(msg.rectification_matrix)
            out.d = list(msg.distortion.k)
        except Exception:
            out.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return out

    def _pointcloud(self, depth_m, valid, info, stamp):
        if self.points_pub is None:
            return
        stride = self.pointcloud_stride
        depth_m = depth_m[::stride, ::stride]
        valid = valid[::stride, ::stride]
        height, width = depth_m.shape
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if fx <= 0.0 or fy <= 0.0:
            return
        u = (np.arange(width, dtype=np.float32) * stride)[None, :]
        v = (np.arange(height, dtype=np.float32) * stride)[:, None]
        z = depth_m.astype(np.float32, copy=False)
        z_safe = np.where(valid, z, 0.0)
        xyz = np.empty((height, width, 3), dtype=np.float32)
        xyz[..., 0] = (u - cx) * z_safe / fx
        xyz[..., 1] = (v - cy) * z_safe / fy
        xyz[..., 2] = z_safe
        xyz[~valid] = np.nan
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.color_frame
        cloud.height, cloud.width = height, width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = width * cloud.point_step
        cloud.is_dense = bool(valid.all())
        cloud.data = xyz.tobytes()
        self.points_pub.publish(cloud)

    def _publish_images(self):
        with self.lock:
            color, depth, info = self.color, self.depth, self.info
            color_seq, depth_seq = self.color_seq, self.depth_seq

        new_color = color is not None and color_seq != self.published_color_seq
        new_depth = depth is not None and depth_seq != self.published_depth_seq
        if not new_color:
            self._increment_stat("skip_color")
        if not new_depth:
            self._increment_stat("skip_depth")

        # RGB and depth belong to the same Gazebo RGB-D sensor.
        # Publish only a fresh pair and give both messages one timestamp.
        if not (new_color and new_depth):
            return

        try:
            start = time.perf_counter()
            color_msg = self._image(color, self.color_frame)
            self._record_duration("color_conversion_s", start)

            start = time.perf_counter()
            depth_msg, depth_m, valid = self._depth(depth)
            self._record_duration("depth_conversion_s", start)
            shared_stamp = color_msg.header.stamp

            depth_msg.header.stamp = shared_stamp
            start = time.perf_counter()
            aligned = copy.deepcopy(depth_msg)
            self._record_duration("deepcopy_s", start)
            aligned.header.frame_id = self.color_frame

            start = time.perf_counter()
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            self.aligned_pub.publish(aligned)
            self._record_duration("publish_s", start)
            self.published_color_seq = color_seq
            self.published_depth_seq = depth_seq
            self._increment_stat("pairs")
            self._increment_stat("color_publishes")
            self._increment_stat("depth_publishes")
            self._record_sequence_delta(color_seq, depth_seq)

            if info is not None:
                start = time.perf_counter()
                color_info = self._camera_info(info, self.color_frame)
                depth_info = self._camera_info(info, self.depth_frame)
                color_info.header.stamp = shared_stamp
                depth_info.header.stamp = shared_stamp
                self._record_duration("camera_info_s", start)

                start = time.perf_counter()
                self.color_info_pub.publish(color_info)
                self.depth_info_pub.publish(depth_info)
                self._record_duration("publish_s", start)

                now = self.get_clock().now().nanoseconds * 1e-9
                if (self.points_pub is not None and
                        self.points_pub.get_subscription_count() > 0 and
                        now - self.last_cloud_time >= self.pointcloud_interval):
                    start = time.perf_counter()
                    self._pointcloud(
                        depth_m, valid, depth_info, shared_stamp)
                    self._record_duration("pointcloud_s", start)
                    self._increment_stat("pointclouds")
                    self.last_cloud_time = now
        except Exception as exc:
            self._increment_stat("conversion_errors")
            self.get_logger().error(
                f"D435i image conversion failed: {exc}")

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_report_time >= 3.0:
            self.get_logger().info(
                f"frames received: color={color_seq}, depth={depth_seq}")
            self.last_report_time = now

    @staticmethod
    def _empty_stats():
        return {
            "color_callbacks": 0,
            "depth_callbacks": 0,
            "pairs": 0,
            "color_publishes": 0,
            "depth_publishes": 0,
            "skip_color": 0,
            "skip_depth": 0,
            "color_conversion_s": 0.0,
            "depth_conversion_s": 0.0,
            "deepcopy_s": 0.0,
            "camera_info_s": 0.0,
            "pointcloud_s": 0.0,
            "publish_s": 0.0,
            "pointclouds": 0,
            "seq_delta_sum": 0,
            "seq_delta_count": 0,
            "seq_delta_max": 0,
            "conversion_errors": 0,
        }

    def _increment_stat(self, key, amount=1):
        if not self.performance_stats_enabled:
            return
        with self.stats_lock:
            self.stats[key] += amount

    def _record_duration(self, key, started_at):
        if not self.performance_stats_enabled:
            return
        with self.stats_lock:
            self.stats[key] += time.perf_counter() - started_at

    def _record_sequence_delta(self, color_seq, depth_seq):
        if not self.performance_stats_enabled:
            return
        delta = abs(int(color_seq) - int(depth_seq))
        with self.stats_lock:
            self.stats["seq_delta_sum"] += delta
            self.stats["seq_delta_count"] += 1
            self.stats["seq_delta_max"] = max(
                self.stats["seq_delta_max"], delta)

    def _initialize_stats_csv(self):
        if not self.performance_stats_enabled or not self.performance_csv_path:
            return
        path = Path(self.performance_csv_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists() or path.stat().st_size == 0:
            with path.open("w", newline="", encoding="utf-8") as handle:
                csv.DictWriter(handle, fieldnames=self.stats_csv_fields).writeheader()

    @staticmethod
    def _mean_ms(total_s, count):
        return 1000.0 * total_s / max(int(count), 1)

    def _report_performance(self):
        now_wall = time.perf_counter()
        with self.stats_lock:
            window_s = max(now_wall - self.stats_window_start, 1.0e-6)
            stats = self.stats
            self.stats = self._empty_stats()
            self.stats_window_start = now_wall

        pair_count = stats["pairs"]
        info_count = pair_count
        publish_operations = pair_count + info_count
        seq_count = stats["seq_delta_count"]
        with self.lock:
            color_seq = self.color_seq
            depth_seq = self.depth_seq

        row = {
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "sim_time_s": round(
                self.get_clock().now().nanoseconds * 1.0e-9, 6),
            "window_s": round(window_s, 6),
            "gazebo_color_callback_hz": round(
                stats["color_callbacks"] / window_s, 4),
            "gazebo_depth_callback_hz": round(
                stats["depth_callbacks"] / window_s, 4),
            "rgbd_pair_hz": round(pair_count / window_s, 4),
            "ros_color_publish_hz": round(
                stats["color_publishes"] / window_s, 4),
            "ros_depth_publish_hz": round(
                stats["depth_publishes"] / window_s, 4),
            "skip_color_not_updated": stats["skip_color"],
            "skip_depth_not_updated": stats["skip_depth"],
            "color_conversion_mean_ms": round(self._mean_ms(
                stats["color_conversion_s"], pair_count), 4),
            "depth_conversion_mean_ms": round(self._mean_ms(
                stats["depth_conversion_s"], pair_count), 4),
            "deepcopy_mean_ms": round(self._mean_ms(
                stats["deepcopy_s"], pair_count), 4),
            "camera_info_mean_ms": round(self._mean_ms(
                stats["camera_info_s"], info_count), 4),
            "pointcloud_mean_ms": round(self._mean_ms(
                stats["pointcloud_s"], stats["pointclouds"]), 4),
            "publish_mean_ms": round(self._mean_ms(
                stats["publish_s"], publish_operations), 4),
            "pointcloud_count": stats["pointclouds"],
            "color_seq": color_seq,
            "depth_seq": depth_seq,
            "seq_delta_mean": round(
                stats["seq_delta_sum"] / max(seq_count, 1), 4),
            "seq_delta_max": stats["seq_delta_max"],
            "conversion_errors": stats["conversion_errors"],
        }

        self.get_logger().info(
            "D435I_PERF "
            f"gz_color={row['gazebo_color_callback_hz']:.2f}Hz "
            f"gz_depth={row['gazebo_depth_callback_hz']:.2f}Hz "
            f"pair={row['rgbd_pair_hz']:.2f}Hz "
            f"skip_color={row['skip_color_not_updated']} "
            f"skip_depth={row['skip_depth_not_updated']} "
            f"convert_ms={row['color_conversion_mean_ms'] + row['depth_conversion_mean_ms']:.2f} "
            f"deepcopy_ms={row['deepcopy_mean_ms']:.2f} "
            f"publish_ms={row['publish_mean_ms']:.2f} "
            f"seq_delta={row['seq_delta_mean']:.2f}/{row['seq_delta_max']}")

        if self.performance_csv_path:
            path = Path(self.performance_csv_path).expanduser()
            with path.open("a", newline="", encoding="utf-8") as handle:
                csv.DictWriter(
                    handle, fieldnames=self.stats_csv_fields).writerow(row)


    def _imu_cb(self, msg):
        if not rclpy.ok():
            return
        stamp = self._stamp(msg)
        combined = Imu()
        combined.header.stamp = stamp
        combined.header.frame_id = self.imu_frame
        combined.orientation.x = msg.orientation.x
        combined.orientation.y = msg.orientation.y
        combined.orientation.z = msg.orientation.z
        combined.orientation.w = msg.orientation.w
        combined.angular_velocity.x = msg.angular_velocity.x
        combined.angular_velocity.y = msg.angular_velocity.y
        combined.angular_velocity.z = msg.angular_velocity.z
        combined.linear_acceleration.x = msg.linear_acceleration.x
        combined.linear_acceleration.y = msg.linear_acceleration.y
        combined.linear_acceleration.z = msg.linear_acceleration.z
        combined.angular_velocity_covariance = [4e-8, 0.0, 0.0, 0.0, 4e-8, 0.0, 0.0, 0.0, 4e-8]
        combined.linear_acceleration_covariance = [4e-6, 0.0, 0.0, 0.0, 4e-6, 0.0, 0.0, 0.0, 4e-6]
        self.imu_pub.publish(combined)
        gyro = copy.deepcopy(combined)
        gyro.orientation_covariance[0] = -1.0
        gyro.linear_acceleration_covariance[0] = -1.0
        self.gyro_pub.publish(gyro)
        accel = copy.deepcopy(combined)
        accel.orientation_covariance[0] = -1.0
        accel.angular_velocity_covariance[0] = -1.0
        self.accel_pub.publish(accel)

    @staticmethod
    def _transform(parent, child, xyz=(0.0, 0.0, 0.0), optical=False):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        if optical:
            t.transform.rotation.x = -0.5
            t.transform.rotation.y = 0.5
            t.transform.rotation.z = -0.5
            t.transform.rotation.w = 0.5
        else:
            t.transform.rotation.w = 1.0
        return t

    def _publish_static_tf(self):
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        transforms = [
            self._transform("base_link", "front_d435i_link", (0.30, 0.0, 0.02)),
            self._transform("front_d435i_link", "front_d435i_color_frame"),
            self._transform("front_d435i_color_frame", self.color_frame, optical=True),
            self._transform("front_d435i_link", "front_d435i_depth_frame"),
            self._transform("front_d435i_depth_frame", self.depth_frame, optical=True),
            self._transform("front_d435i_link", self.imu_frame),
        ]
        stamp = self.get_clock().now().to_msg()
        for transform in transforms:
            transform.header.stamp = stamp
        self.tf_broadcaster.sendTransform(transforms)

    def close_transport(self):
        for topic in self.gz_topics:
            try:
                self.gz_node.unsubscribe(topic)
            except Exception as exc:
                self.get_logger().warning(
                    f"Could not unsubscribe Gazebo topic {topic}: {exc}")


def main():
    rclpy.init()
    node = D435iSimBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_transport()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
