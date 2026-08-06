#!/usr/bin/env python3
import copy
import os
import threading

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
        self.declare_parameter("publish_pointcloud", False)
        self.declare_parameter("pointcloud_hz", 10.0)
        self.declare_parameter("pointcloud_stride", 4)
        self.declare_parameter("max_depth_m", 10.0)
        self.gz_prefix = str(self.get_parameter("gz_prefix").value).rstrip("/")
        self.ros_prefix = str(self.get_parameter("ros_prefix").value).rstrip("/")
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.publish_pointcloud = bool(self.get_parameter("publish_pointcloud").value)
        self.pointcloud_interval = 1.0 / max(float(self.get_parameter("pointcloud_hz").value), 0.1)
        self.pointcloud_stride = max(int(self.get_parameter("pointcloud_stride").value), 1)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.color_frame = "front_d435i_color_optical_frame"
        self.depth_frame = "front_d435i_depth_optical_frame"
        self.imu_frame = "front_d435i_imu_frame"
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.color_pub = self.create_publisher(Image, f"{self.ros_prefix}/color/image_raw", self.qos)
        self.color_info_pub = self.create_publisher(CameraInfo, f"{self.ros_prefix}/color/camera_info", self.qos)
        self.depth_pub = self.create_publisher(Image, f"{self.ros_prefix}/depth/image_rect_raw", self.qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, f"{self.ros_prefix}/depth/camera_info", self.qos)
        self.aligned_pub = self.create_publisher(
            Image, f"{self.ros_prefix}/aligned_depth_to_color/image_raw", self.qos)
        self.points_pub = None
        if self.publish_pointcloud:
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
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, f"{self.gz_prefix}/image", self._color_cb)
        self.gz_node.subscribe(GzImage, self.gz_prefix, self._color_cb)
        self.gz_node.subscribe(GzImage, f"{self.gz_prefix}/depth_image", self._depth_cb)
        self.gz_node.subscribe(GzCameraInfo, f"{self.gz_prefix}/camera_info", self._info_cb)
        self.gz_node.subscribe(GzImu, f"{self.gz_prefix}/imu", self._imu_cb)
        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._publish_images)
        self._publish_static_tf()
        self.get_logger().info(
            f"D435i simulation bridge active: {self.gz_prefix} -> {self.ros_prefix}; "
            f"pointcloud={'enabled' if self.publish_pointcloud else 'disabled'}")

    def _stamp(self, msg):
        stamp = self.get_clock().now().to_msg()
        try:
            stamp.sec = msg.header.stamp.sec
            stamp.nanosec = msg.header.stamp.nsec
        except Exception:
            pass
        return stamp

    def _color_cb(self, msg):
        with self.lock:
            self.color = msg
            self.color_seq += 1

    def _depth_cb(self, msg):
        with self.lock:
            self.depth = msg
            self.depth_seq += 1

    def _info_cb(self, msg):
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

    def _depth_mm(self, msg):
        width, height = int(msg.width), int(msg.height)
        if int(msg.step) == width * 4:
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(height, width)
        elif int(msg.step) == width * 2:
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
            depth_m = raw.astype(np.float32) * 0.001
        else:
            raise ValueError(f"unsupported depth step {msg.step} for width {width}")
        valid = np.isfinite(depth_m) & (depth_m >= 0.105) & (depth_m <= self.max_depth_m)
        depth_mm = np.zeros(depth_m.shape, dtype=np.uint16)
        depth_mm[valid] = np.rint(depth_m[valid] * 1000.0).astype(np.uint16)
        out = Image()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = self.depth_frame
        out.height, out.width = height, width
        out.encoding = "16UC1"
        out.is_bigendian = 0
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
        try:
            if new_color:
                self.color_pub.publish(self._image(color, self.color_frame))
                self.published_color_seq = color_seq
            if new_depth:
                depth_msg, depth_m, valid = self._depth_mm(depth)
                self.depth_pub.publish(depth_msg)
                aligned = copy.deepcopy(depth_msg)
                aligned.header.frame_id = self.color_frame
                self.aligned_pub.publish(aligned)
                self.published_depth_seq = depth_seq
                if info is not None:
                    depth_info = self._camera_info(info, self.depth_frame)
                    self.depth_info_pub.publish(depth_info)
                    now = self.get_clock().now().nanoseconds * 1e-9
                    if (self.points_pub is not None and
                            self.points_pub.get_subscription_count() > 0 and
                            now - self.last_cloud_time >= self.pointcloud_interval):
                        self._pointcloud(depth_m, valid, depth_info, depth_msg.header.stamp)
                        self.last_cloud_time = now
            if info is not None and (new_color or new_depth):
                self.color_info_pub.publish(self._camera_info(info, self.color_frame))
        except Exception as exc:
            self.get_logger().error(f"D435i image conversion failed: {exc}")
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_report_time >= 3.0:
            self.get_logger().info(f"frames received: color={color_seq}, depth={depth_seq}")
            self.last_report_time = now

    def _imu_cb(self, msg):
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
            # Keep the bridge TF identical to the Gazebo model mount.
            self._transform("base_link", "front_d435i_link", (0.20, 0.0, 0.02)),
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


def main():
    rclpy.init()
    node = D435iSimBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
