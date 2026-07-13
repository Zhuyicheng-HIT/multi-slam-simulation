import copy
import math
from collections import deque

import numpy as np
import rclpy
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
from uf_interfaces.msg import GnssIntegrity, LioDiagnostics, ReliabilityScore

from .scoring import gnss_score, imu_score, lidar_score, optical_flow_score, vision_score


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def vector_norm(vector):
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def distance_m(a, b):
    lat_scale = 111_320.0
    lon_scale = lat_scale * math.cos(math.radians(0.5 * (a.latitude + b.latitude)))
    dx = (a.longitude - b.longitude) * lon_scale
    dy = (a.latitude - b.latitude) * lat_scale
    dz = a.altitude - b.altitude
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def depth_valid_ratio(msg):
    if msg.height == 0 or msg.width == 0:
        return 0.0
    encoding = msg.encoding.upper()
    if encoding == "32FC1":
        values = np.frombuffer(msg.data, dtype=np.float32)
        valid = np.isfinite(values) & (values > 0.05)
    elif encoding in ("16UC1", "MONO16"):
        values = np.frombuffer(msg.data, dtype=np.uint16)
        valid = values > 0
    else:
        return 0.0
    return float(np.mean(valid)) if len(valid) else 0.0


def image_feature_support(msg):
    channels = 3 if msg.encoding.lower() in ("rgb8", "bgr8") else 1
    values = np.frombuffer(msg.data, dtype=np.uint8)
    needed = int(msg.height) * int(msg.width) * channels
    if needed == 0 or len(values) < needed:
        return 0, 0.0, -1.0
    image = values[:needed].reshape((msg.height, msg.width, channels))
    gray = image[::4, ::4].mean(axis=2)
    if min(gray.shape) < 3:
        return 0, 0.0, -1.0
    laplacian = -4.0 * gray[1:-1, 1:-1]
    laplacian += gray[:-2, 1:-1] + gray[2:, 1:-1]
    laplacian += gray[1:-1, :-2] + gray[1:-1, 2:]
    blur_energy = float(np.sqrt(np.mean(laplacian * laplacian)))
    gradient = np.hypot(np.diff(gray, axis=1)[:-1, :], np.diff(gray, axis=0)[:, :-1])
    rows = np.array_split(gradient, 8, axis=0)
    cell_counts = []
    for row in rows:
        for cell in np.array_split(row, 8, axis=1):
            cell_counts.append(int(np.count_nonzero(cell > 12.0)))
    feature_count = sum(min(3, count) for count in cell_counts)
    spatial_uniformity = sum(count > 0 for count in cell_counts) / 64.0
    return feature_count, spatial_uniformity, blur_energy


class ReliabilityMonitor(Node):
    def __init__(self):
        super().__init__("reliability_monitor")
        parameters = {
            "lidar.match_reference": 1000,
            "lidar.tau_lambda": 1.0,
            "lidar.tau_kappa": 1.0e5,
            "lidar.tau_normal": 0.02,
            "lidar.weights": [0.35, 0.20, 0.20, 0.25],
            "gnss.tau_covariance": 25.0,
            "gnss.tau_innovation": 5.0,
            "gnss.weights": [0.25, 0.20, 0.55],
            "imu.tau_preintegration": 5.0,
            "imu.accel_excitation_scale": 0.5,
            "imu.gyro_excitation_scale": 0.15,
            "imu.accel_saturation": 55.0,
            "imu.gyro_saturation": 10.0,
            "imu.weights": [0.35, 0.45, 0.20],
            "optical_flow.tau_translation": 0.30,
            "optical_flow.weights": [0.60, 0.25, 0.15],
            "vision.feature_reference": 150,
            "vision.tau_reprojection_px": 3.0,
            "vision.weights": [0.30, 0.25, 0.25, 0.20],
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        self.score_publishers = {
            name: self.create_publisher(ReliabilityScore, f"/reliability/{name}_score", 20)
            for name in ("lidar", "gnss", "imu", "optical_flow", "vision")
        }
        self.gnss_integrity_pub = self.create_publisher(GnssIntegrity, "/reliability/gnss_integrity", 20)
        self.last_imu = None
        self.last_imu_ns = None
        self.last_imu_publish_ns = None
        self.imu_window = deque(maxlen=100)
        self.last_gnss = None
        self.last_gnss_ns = None
        self.last_gnss_arrival_ns = None
        self.last_gnss_lio_position = None
        self.lio_speed = 0.0
        self.lio_position = None
        self.latest_depth_ratio = -1.0
        self.latest_blur_energy = -1.0
        self.latest_feature_count = 0
        self.latest_spatial_uniformity = 0.0
        self.last_vision_publish_ns = None

        self.create_subscription(LioDiagnostics, "/lio/diagnostics", self._lidar, 20)
        self.create_subscription(NavSatFix, "/sensors/gnss/fix", self._gnss, qos_profile_sensor_data)
        self.create_subscription(Imu, "/sensors/imu", self._imu, qos_profile_sensor_data)
        self.create_subscription(OpticalFlowRad, "/sensors/optical_flow/rad", self._flow, qos_profile_sensor_data)
        self.create_subscription(Image, "/sensors/rgbd/depth", self._depth, qos_profile_sensor_data)
        self.create_subscription(Image, "/sensors/rgbd/color", self._color, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/lio/odom", self._odom, 20)
        self.create_timer(0.5, self._outage_timer)

    def _publish(self, modality, header, result, valid=True):
        score, evidence, reasons = result
        msg = ReliabilityScore()
        msg.header = copy.deepcopy(header)
        msg.modality = modality
        msg.degradation_score = float(score)
        msg.reliability_weight = float(1.0 - score)
        msg.valid = bool(valid)
        msg.reasons = reasons
        msg.evidence_names = list(evidence.keys())
        msg.evidence_values = [float(value) for value in evidence.values()]
        self.score_publishers[modality].publish(msg)

    def _lidar(self, msg):
        result = lidar_score(
            msg.hessian_eigenvalues, msg.normal_covariance_eigenvalues,
            msg.axial_penalty, msg.matched_points,
            self.get_parameter("lidar.match_reference").value,
            self.get_parameter("lidar.tau_lambda").value,
            self.get_parameter("lidar.tau_kappa").value,
            self.get_parameter("lidar.tau_normal").value,
            tuple(self.get_parameter("lidar.weights").value),
        )
        if msg.approximate:
            result[2].append("approximate_external_geometry")
        self._publish("lidar", msg.header, result, msg.input_points > 0)

    def _odom(self, msg):
        v = msg.twist.twist.linear
        self.lio_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        p = msg.pose.pose.position
        self.lio_position = np.asarray([p.x, p.y, p.z], dtype=float)

    def _gnss(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        current_ns = stamp_ns(msg.header)
        covariance = float(msg.position_covariance[0] + msg.position_covariance[4] + msg.position_covariance[8])
        innovation = -1.0
        innovation_mahalanobis = -1.0
        jump = False
        if self.last_gnss is not None and self.last_gnss_ns is not None:
            dt = max(1.0e-3, (current_ns - self.last_gnss_ns) * 1.0e-9)
            gnss_delta = distance_m(msg, self.last_gnss)
            jump = gnss_delta / dt > 15.0 or gnss_delta > 20.0
            if self.lio_position is not None and self.last_gnss_lio_position is not None:
                lio_delta = float(np.linalg.norm(self.lio_position - self.last_gnss_lio_position))
                innovation = abs(gnss_delta - lio_delta)
                innovation_mahalanobis = innovation * innovation / max(0.01, covariance)
        if jump and innovation_mahalanobis < 0.0:
            innovation_mahalanobis = 10.0
        q_fix = 1.0 if msg.status.status >= 0 else 0.0
        result = gnss_score(
            q_fix, covariance, innovation_mahalanobis,
            self.get_parameter("gnss.tau_covariance").value,
            self.get_parameter("gnss.tau_innovation").value,
            tuple(self.get_parameter("gnss.weights").value),
        )
        result[1]["satellite_count_unavailable"] = -1.0
        result[1]["dop_unavailable"] = -1.0
        self._publish("gnss", msg.header, result, True)
        integrity = GnssIntegrity()
        integrity.header = copy.deepcopy(msg.header)
        integrity.fix_status = int(msg.status.status)
        integrity.service = int(msg.status.service)
        integrity.satellite_count = 0
        integrity.hdop = -1.0
        integrity.vdop = -1.0
        integrity.covariance_trace_m2 = covariance
        integrity.innovation_norm_m = innovation
        integrity.outage_duration_s = 0.0
        integrity.jump_detected = jump
        integrity.synthetic_metadata = True
        self.gnss_integrity_pub.publish(integrity)
        self.last_gnss = copy.deepcopy(msg)
        self.last_gnss_ns = current_ns
        self.last_gnss_arrival_ns = now_ns
        self.last_gnss_lio_position = None if self.lio_position is None else self.lio_position.copy()

    def _imu(self, msg):
        current_ns = stamp_ns(msg.header)
        accel = np.asarray([
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ], dtype=float)
        jerk = 0.0
        if self.last_imu is not None and self.last_imu_ns is not None:
            dt = (current_ns - self.last_imu_ns) * 1.0e-9
            if dt > 1.0e-4:
                jerk = float(np.linalg.norm(accel - self.last_imu) / dt)
        self.last_imu = accel
        self.last_imu_ns = current_ns
        gyro = np.asarray([
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ], dtype=float)
        self.imu_window.append((accel, gyro))
        if self.last_imu_publish_ns is not None and current_ns - self.last_imu_publish_ns < 100_000_000:
            return
        self.last_imu_publish_ns = current_ns
        accel_values = np.asarray([sample[0] for sample in self.imu_window])
        gyro_values = np.asarray([sample[1] for sample in self.imu_window])
        accel_excitation = float(np.mean(np.std(accel_values, axis=0))) if len(accel_values) > 2 else 0.0
        gyro_excitation = float(np.mean(np.std(gyro_values, axis=0))) if len(gyro_values) > 2 else 0.0
        excitation = min(1.0, 0.5 * (
            accel_excitation / self.get_parameter("imu.accel_excitation_scale").value
            + gyro_excitation / self.get_parameter("imu.gyro_excitation_scale").value
        ))
        accel_norm = float(np.linalg.norm(accel))
        gyro_norm = float(np.linalg.norm(gyro))
        saturation = (
            accel_norm >= self.get_parameter("imu.accel_saturation").value
            or gyro_norm >= self.get_parameter("imu.gyro_saturation").value
        )
        result = imu_score(
            excitation, -1.0, saturation,
            self.get_parameter("imu.tau_preintegration").value,
            tuple(self.get_parameter("imu.weights").value),
        )
        result[1]["accel_norm_mps2"] = accel_norm
        result[1]["gyro_norm_radps"] = gyro_norm
        result[1]["jerk_mps3_diagnostic"] = jerk
        result[1]["preintegration_residual_unavailable"] = -1.0
        self._publish("imu", msg.header, result, True)

    def _flow(self, msg):
        integration_s = float(msg.integration_time_us) * 1.0e-6
        delta_flow = 0.0
        delta_prediction = 0.0
        if integration_s > 1.0e-5 and msg.distance > 0.0:
            delta_flow = math.hypot(msg.integrated_x, msg.integrated_y) * msg.distance
            delta_prediction = self.lio_speed * integration_s
        self._publish(
            "optical_flow", msg.header,
            optical_flow_score(
                delta_flow, delta_prediction, msg.quality, msg.distance,
                self.get_parameter("optical_flow.tau_translation").value,
                tuple(self.get_parameter("optical_flow.weights").value),
            ), True
        )

    def _depth(self, msg):
        self.latest_depth_ratio = depth_valid_ratio(msg)
        self._publish_vision(msg.header)

    def _color(self, msg):
        (self.latest_feature_count, self.latest_spatial_uniformity,
         self.latest_blur_energy) = image_feature_support(msg)
        self._publish_vision(msg.header)

    def _publish_vision(self, header):
        current_ns = stamp_ns(header)
        if self.last_vision_publish_ns is not None and current_ns - self.last_vision_publish_ns < 200_000_000:
            return
        self.last_vision_publish_ns = current_ns
        result = vision_score(
            self.latest_feature_count, self.get_parameter("vision.feature_reference").value,
            self.latest_spatial_uniformity, -1.0, self.latest_depth_ratio,
            self.get_parameter("vision.tau_reprojection_px").value,
            tuple(self.get_parameter("vision.weights").value),
        )
        result[1]["blur_energy_diagnostic"] = self.latest_blur_energy
        result[1]["projection_consistency_unavailable"] = -1.0
        self._publish("vision", header, result, self.latest_depth_ratio >= 0.0)

    def _outage_timer(self):
        if self.last_gnss_arrival_ns is None or self.last_gnss is None:
            return
        outage_s = (self.get_clock().now().nanoseconds - self.last_gnss_arrival_ns) * 1.0e-9
        if outage_s <= 0.75:
            return
        covariance = float(
            self.last_gnss.position_covariance[0] + self.last_gnss.position_covariance[4]
            + self.last_gnss.position_covariance[8]
        )
        result = gnss_score(
            0.0, covariance, -1.0,
            self.get_parameter("gnss.tau_covariance").value,
            self.get_parameter("gnss.tau_innovation").value,
            tuple(self.get_parameter("gnss.weights").value),
        )
        result[1]["outage_duration_s"] = outage_s
        result[2].append("outage_forces_q_fix_zero")
        self._publish("gnss", self.last_gnss.header, result, outage_s < 5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ReliabilityMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
