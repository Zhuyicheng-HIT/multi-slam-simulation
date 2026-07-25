import bisect
import copy
import math
from collections import deque
import time

import numpy as np
import rclpy
from mavros_msgs.msg import GPSRAW, OpticalFlowRad
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
from uf_interfaces.msg import GnssIntegrity, LioDiagnostics, ReliabilityScore

from .scoring import (
    gnss_integrity_quality,
    gnss_score,
    imu_score,
    lidar_score,
    optical_flow_displacement_frd,
    optical_flow_score,
    vision_score,
)


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


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def interpolate_lio(samples, timestamp_s, maximum_gap_s):
    if len(samples) < 2:
        return None
    times = [sample[0] for sample in samples]
    index = bisect.bisect_left(times, timestamp_s)
    if index == 0 or index >= len(samples):
        return None
    before = samples[index - 1]
    after = samples[index]
    span_s = after[0] - before[0]
    if span_s <= 0.0 or span_s > maximum_gap_s:
        return None
    ratio = (timestamp_s - before[0]) / span_s
    yaw_delta = math.atan2(
        math.sin(after[3] - before[3]), math.cos(after[3] - before[3])
    )
    return np.asarray([
        before[1] + ratio * (after[1] - before[1]),
        before[2] + ratio * (after[2] - before[2]),
        before[3] + ratio * yaw_delta,
    ])


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
            "lidar.minimum_matches": 50,
            "gnss.tau_covariance": 25.0,
            "gnss.tau_innovation": 5.0,
            "gnss.weights": [0.25, 0.20, 0.55],
            "gnss.raw_topic": "/fcu/gnss/raw",
            "gnss.raw_timeout_s": 1.0,
            "gnss.minimum_satellites": 5,
            "gnss.good_satellites": 10,
            "gnss.good_hdop": 1.0,
            "gnss.maximum_hdop": 4.0,
            "imu.tau_preintegration": 5.0,
            "imu.accel_excitation_scale": 0.5,
            "imu.gyro_excitation_scale": 0.15,
            "imu.accel_saturation": 55.0,
            "imu.gyro_saturation": 10.0,
            "imu.weights": [0.35, 0.45, 0.20],
            "imu.minimum_window_samples": 20,
            "optical_flow.tau_translation": 0.30,
            "optical_flow.weights": [0.60, 0.25, 0.15],
            "optical_flow.lio_max_gap_s": 0.5,
            "optical_flow.lio_wait_s": 0.4,
            "vision.feature_reference": 150,
            "vision.tau_reprojection_px": 3.0,
            "vision.weights": [0.30, 0.25, 0.25, 0.20],
            "vision.minimum_features": 20,
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
        self.last_gps_raw = None
        self.last_gps_raw_arrival_ns = None
        self.lio_position = None
        self.lio_samples = deque(maxlen=1000)
        self.pending_flows = deque(maxlen=100)
        self.latest_depth_ratio = -1.0
        self.latest_blur_energy = -1.0
        self.latest_feature_count = 0
        self.latest_spatial_uniformity = 0.0
        self.last_vision_publish_ns = None

        self.create_subscription(LioDiagnostics, "/lio/diagnostics", self._lidar, 20)
        self.create_subscription(NavSatFix, "/sensors/gnss/fix", self._gnss, qos_profile_sensor_data)
        self.create_subscription(
            GPSRAW, str(self.get_parameter("gnss.raw_topic").value),
            self._gps_raw, qos_profile_sensor_data,
        )
        self.create_subscription(Imu, "/sensors/imu", self._imu, qos_profile_sensor_data)
        self.create_subscription(OpticalFlowRad, "/sensors/optical_flow/rad", self._flow, qos_profile_sensor_data)
        self.create_subscription(Image, "/sensors/rgbd/depth", self._depth, qos_profile_sensor_data)
        self.create_subscription(Image, "/sensors/rgbd/color", self._color, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/lio/odom", self._odom, 20)
        self.create_timer(0.5, self._outage_timer)

    def _publish(
        self,
        modality,
        header,
        result,
        valid=True,
        observation_count=1,
        minimum_observation_count=1,
    ):
        score, evidence, reasons = result
        complete = evidence.get("score_complete", 0.0) >= 0.5
        usable = bool(valid and complete)
        msg = ReliabilityScore()
        msg.header = copy.deepcopy(header)
        msg.modality = modality
        msg.degradation_score = float(score)
        msg.reliability_weight = float(1.0 - score) if usable else 0.0
        msg.valid = usable
        msg.observation_count = max(0, int(observation_count))
        msg.minimum_observation_count = max(1, int(minimum_observation_count))
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
        self._publish(
            "lidar",
            msg.header,
            result,
            msg.input_points > 0,
            msg.matched_points,
            self.get_parameter("lidar.minimum_matches").value,
        )

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.lio_position = np.asarray([p.x, p.y, p.z], dtype=float)
        timestamp_s = stamp_ns(msg.header) * 1.0e-9
        if not self.lio_samples or timestamp_s > self.lio_samples[-1][0]:
            self.lio_samples.append((
                timestamp_s,
                float(p.x),
                float(p.y),
                yaw_from_quaternion(msg.pose.pose.orientation),
            ))
        self._flush_flows()

    def _gps_raw(self, msg):
        self.last_gps_raw = copy.deepcopy(msg)
        self.last_gps_raw_arrival_ns = self.get_clock().now().nanoseconds

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
        raw = None
        if self.last_gps_raw is not None and self.last_gps_raw_arrival_ns is not None:
            age_s = (now_ns - self.last_gps_raw_arrival_ns) * 1.0e-9
            if age_s <= float(self.get_parameter("gnss.raw_timeout_s").value):
                raw = self.last_gps_raw
        hdop = None
        vdop = None
        if raw is not None:
            if int(raw.eph) != 65535:
                hdop = float(raw.eph) * 0.01
            if int(raw.epv) != 65535:
                vdop = float(raw.epv) * 0.01
        integrity_quality, integrity_evidence, integrity_reasons = gnss_integrity_quality(
            None if raw is None else raw.fix_type,
            None if raw is None else raw.satellites_visible,
            hdop,
            self.get_parameter("gnss.minimum_satellites").value,
            self.get_parameter("gnss.good_satellites").value,
            self.get_parameter("gnss.good_hdop").value,
            self.get_parameter("gnss.maximum_hdop").value,
        )
        q_fix = (
            (1.0 if msg.status.status >= 0 else 0.0)
            if integrity_quality is None else integrity_quality
        )
        result = gnss_score(
            q_fix, covariance, innovation_mahalanobis,
            self.get_parameter("gnss.tau_covariance").value,
            self.get_parameter("gnss.tau_innovation").value,
            tuple(self.get_parameter("gnss.weights").value),
            hard_jump=jump,
        )
        result[1].update(integrity_evidence)
        result[1]["vdop"] = -1.0 if vdop is None else vdop
        result[1]["fcu_metadata_fresh"] = 0.0 if raw is None else 1.0
        result[2].extend(integrity_reasons)
        self._publish("gnss", msg.header, result, True)
        integrity = GnssIntegrity()
        integrity.header = copy.deepcopy(msg.header)
        integrity.fix_status = int(msg.status.status)
        integrity.service = int(msg.status.service)
        integrity.satellite_count = 0 if raw is None else int(raw.satellites_visible)
        integrity.hdop = -1.0 if hdop is None else hdop
        integrity.vdop = -1.0 if vdop is None else vdop
        integrity.covariance_trace_m2 = covariance
        integrity.innovation_norm_m = innovation
        integrity.outage_duration_s = 0.0
        integrity.jump_detected = jump
        integrity.synthetic_metadata = raw is None
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
        self._publish(
            "imu",
            msg.header,
            result,
            True,
            len(self.imu_window),
            self.get_parameter("imu.minimum_window_samples").value,
        )

    def _flow(self, msg):
        self.pending_flows.append((time.monotonic(), copy.deepcopy(msg)))
        self._flush_flows()

    def _flow_prediction(self, msg):
        integration_s = float(msg.integration_time_us) * 1.0e-6
        if integration_s <= 1.0e-5:
            return None
        end_s = stamp_ns(msg.header) * 1.0e-9
        maximum_gap = float(self.get_parameter("optical_flow.lio_max_gap_s").value)
        current = interpolate_lio(self.lio_samples, end_s, maximum_gap)
        previous = interpolate_lio(self.lio_samples, end_s - integration_s, maximum_gap)
        if current is None or previous is None:
            return None
        world_delta = current[:2] - previous[:2]
        yaw = current[2]
        body_flu_x = math.cos(yaw) * world_delta[0] + math.sin(yaw) * world_delta[1]
        body_flu_y = -math.sin(yaw) * world_delta[0] + math.cos(yaw) * world_delta[1]
        return float(body_flu_x), float(-body_flu_y)

    def _flush_flows(self):
        now = time.monotonic()
        wait_s = float(self.get_parameter("optical_flow.lio_wait_s").value)
        while self.pending_flows:
            queued_at, msg = self.pending_flows[0]
            prediction = self._flow_prediction(msg)
            if prediction is None and now - queued_at < wait_s:
                break
            self.pending_flows.popleft()
            self._publish_flow(msg, prediction)

    def _publish_flow(self, msg, prediction):
        flow_displacement = optical_flow_displacement_frd(
            msg.integrated_x, msg.integrated_y,
            msg.integrated_xgyro, msg.integrated_ygyro,
            msg.distance,
        )
        result = optical_flow_score(
            flow_displacement, prediction, msg.quality, msg.distance,
            self.get_parameter("optical_flow.tau_translation").value,
            tuple(self.get_parameter("optical_flow.weights").value),
        )
        gyro_available = flow_displacement is not None
        result[1]["gyro_compensation_available"] = 1.0 if gyro_available else 0.0
        result[1]["lio_increment_available"] = 0.0 if prediction is None else 1.0
        result[1]["translation_vector_residual_eq22_adapted"] = 1.0
        if not gyro_available:
            result[2].append("gyro_compensation_unavailable")
        self._publish(
            "optical_flow", msg.header, result, gyro_available
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
        self._publish(
            "vision",
            header,
            result,
            self.latest_depth_ratio >= 0.0,
            self.latest_feature_count,
            self.get_parameter("vision.minimum_features").value,
        )

    def _outage_timer(self):
        self._flush_flows()
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
        self._publish(
            "gnss",
            self.last_gnss.header,
            result,
            outage_s < 5.0,
            0,
            1,
        )


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
