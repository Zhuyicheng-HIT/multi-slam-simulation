import bisect
import copy
import math
from collections import OrderedDict, deque

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import GPSRAW, OpticalFlowRad
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
from uf_interfaces.msg import (
    GnssIntegrity, LioDiagnostics, ReliabilityScore, VisualFeatureTracks,
)

from .flow_rotation_gate import (
    FlowRotationGateConfig,
    OpticalFlowRotationGate,
    interval_mean_absolute_yaw_rate,
    interval_mean_vector,
)
from .scoring import (
    gnss_integrity_quality,
    gnss_score,
    imu_health_admission,
    imu_score,
    lidar_factor_score,
    lidar_innovation_score,
    lidar_map_score,
    lidar_score,
    optical_flow_displacement_frd,
    optical_flow_lever_arm_displacement_flu,
    optical_flow_score,
    vision_score,
)


VISION_HEALTH_TOPIC = "/reliability/vision_score"
VISION_FACTOR_SCORE_TOPIC = "/reliability/vision_factor_score"


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def score_operationally_usable(
    valid,
    evidence,
    observation_count,
    minimum_observation_count,
    minimum_evidence_coverage=1.0,
):
    coverage = float(evidence.get("evidence_weight_coverage", 0.0))
    return bool(
        valid
        and int(observation_count) >= int(minimum_observation_count)
        and coverage + 1.0e-9 >= float(minimum_evidence_coverage)
    )


def conservative_partial_score(score, evidence_coverage):
    """Treat unavailable evidence as fully degraded during bootstrap."""
    bounded_score = min(1.0, max(0.0, float(score)))
    bounded_coverage = min(1.0, max(0.0, float(evidence_coverage)))
    return 1.0 - bounded_coverage * (1.0 - bounded_score)


def vector_norm(vector):
    return math.sqrt(
        vector.x *
        vector.x +
        vector.y *
        vector.y +
        vector.z *
        vector.z)


def distance_m(a, b):
    lat_scale = 111_320.0
    lon_scale = lat_scale * \
        math.cos(math.radians(0.5 * (a.latitude + b.latitude)))
    dx = (a.longitude - b.longitude) * lon_scale
    dy = (a.latitude - b.latitude) * lat_scale
    dz = a.altitude - b.altitude
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def nonnegative_diagnostic_value(message, status_name, key):
    """Read one finite non-negative scalar from a DiagnosticArray-like message."""
    for status in message.status:
        if str(status.name) != str(status_name):
            continue
        for item in status.values:
            if str(item.key) != str(key):
                continue
            try:
                value = float(item.value)
            except (TypeError, ValueError):
                return None
            return value if math.isfinite(value) and value >= 0.0 else None
    return None


def flow_observation_valid(gyro_available, yaw_rate, rotation_gate):
    return bool(
        gyro_available
        and yaw_rate is not None
        and not rotation_gate.hard_disabled
    )


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                        quaternion.z * quaternion.z)
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


def depth_valid_ratio(msg, minimum_depth_m=0.30, maximum_depth_m=6.0):
    if msg.height == 0 or msg.width == 0:
        return 0.0
    encoding = msg.encoding.upper()
    if encoding == "32FC1":
        values = np.frombuffer(msg.data, dtype=np.float32)
        valid = (
            np.isfinite(values)
            & (values >= float(minimum_depth_m) - 1.0e-6)
            & (values <= float(maximum_depth_m) + 1.0e-6)
        )
    elif encoding in ("16UC1", "MONO16"):
        values = np.frombuffer(msg.data, dtype=np.uint16)
        minimum_mm = max(0, int(math.ceil(
            float(minimum_depth_m) * 1000.0 - 1.0e-9)))
        maximum_mm = max(0, int(math.floor(
            float(maximum_depth_m) * 1000.0 + 1.0e-9)))
        valid = (values >= minimum_mm) & (values <= maximum_mm)
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
    gradient = np.hypot(
        np.diff(
            gray,
            axis=1)[
            :-1,
            :],
        np.diff(
            gray,
            axis=0)[
            :,
            :-1])
    rows = np.array_split(gradient, 8, axis=0)
    cell_counts = []
    for row in rows:
        for cell in np.array_split(row, 8, axis=1):
            cell_counts.append(int(np.count_nonzero(cell > 12.0)))
    feature_count = sum(min(3, count) for count in cell_counts)
    spatial_uniformity = sum(count > 0 for count in cell_counts) / 64.0
    return feature_count, spatial_uniformity, blur_energy


def visual_factor_track_metrics(message):
    """Measure exactly the visual tracks eligible for backend factor creation."""
    tracks = list(message.tracks)
    geometry_eligible = []
    selected = []
    for track in tracks:
        coordinates_finite = all(math.isfinite(float(value)) for value in (
            track.previous_x,
            track.previous_y,
            track.current_x,
            track.current_y,
        ))
        geometry_valid = bool(
            track.klt_inlier
            and track.geometric_inlier
            and int(track.track_age) >= 2
            and coordinates_finite
        )
        if not geometry_valid:
            continue
        geometry_eligible.append(track)
        if bool(
            track.depth_valid
            and float(track.inverse_depth) > 0.0
            and math.isfinite(float(track.inverse_depth))
        ):
            selected.append(track)
    reprojection = [
        float(track.reprojection_error_px) for track in selected
        if math.isfinite(float(track.reprojection_error_px))
        and float(track.reprojection_error_px) >= 0.0
    ]
    occupied_cells = {
        int(track.grid_cell) for track in selected
        if 0 <= int(track.grid_cell) < 64
    }
    selected_count = len(selected)
    geometry_count = len(geometry_eligible)
    return {
        "raw_track_count": len(tracks),
        "geometry_eligible_count": geometry_count,
        "selected_track_count": selected_count,
        "selected_track_ratio": selected_count / max(1, len(tracks)),
        "depth_valid_ratio": selected_count / max(1, geometry_count),
        "spatial_distribution": len(occupied_cells) / 64.0,
        "mean_reprojection_error_px": (
            float(np.mean(reprojection)) if reprojection else -1.0
        ),
    }


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
            "lidar.extension.tau_residual_m": 0.15,
            "lidar.extension.tau_dynamic_ratio": 0.20,
            "lidar.extension.tau_uncertain_ratio": 0.25,
            "lidar.extension.reference": 0.35,
            "lidar.extension.paper_weight": 0.70,
            "lidar.extension.weights": [0.20, 0.15, 0.20, 0.10, 0.15, 0.20],
            "lidar.factor.approximate_geometry_weight": 0.20,
            "lidar.factor.native_geometry_weight": 0.60,
            "lidar.factor.tau_position_innovation_m": 0.50,
            "lidar.factor.tau_yaw_innovation_rad": 0.35,
            "lidar.factor.innovation_weights": [0.70, 0.30],
            "lidar.factor.innovation_timeout_s": 2.0,
            "lidar.map.tau_residual_m": 0.15,
            "lidar.map.tau_dynamic_ratio": 0.20,
            "lidar.map.tau_uncertain_ratio": 0.25,
            "lidar.map.weights": [0.25, 0.20, 0.20, 0.15, 0.20],
            "gnss.tau_covariance": 25.0,
            "gnss.tau_innovation": 5.0,
            "gnss.weights": [0.25, 0.20, 0.55],
            "gnss.raw_topic": "/fcu/gnss/raw",
            "gnss.raw_timeout_s": 1.0,
            "gnss.minimum_satellites": 5,
            "gnss.good_satellites": 10,
            "gnss.good_hdop": 1.0,
            "gnss.maximum_hdop": 4.0,
            "gnss.minimum_startup_evidence_coverage": 0.45,
            "gnss.prefit_nis_timeout_s": 2.0,
            "imu.tau_preintegration": 5.0,
            "imu.accel_excitation_scale": 0.5,
            "imu.gyro_excitation_scale": 0.15,
            "imu.accel_saturation": 55.0,
            "imu.gyro_saturation": 10.0,
            "imu.weights": [0.35, 0.45, 0.20],
            "imu.minimum_window_samples": 20,
            "imu.minimum_startup_evidence_coverage": 0.55,
            "imu.backend_diagnostic_topic": "/fusion/unified/diagnostics",
            "imu.preintegration_residual_timeout_s": 2.0,
            "optical_flow.tau_translation": 0.30,
            "optical_flow.weights": [0.60, 0.25, 0.15],
            "optical_flow.allow_prediction_fallback": True,
            "optical_flow.prediction_fallback_min_quality": 120,
            "optical_flow.lio_max_gap_s": 0.5,
            "optical_flow.lio_wait_s": 0.4,
            "optical_flow.rotation_gate.lower_yaw_rate_radps": 0.08,
            "optical_flow.rotation_gate.upper_yaw_rate_radps": 0.30,
            "optical_flow.rotation_gate.recovery_dwell_s": 0.8,
            "optical_flow.rotation_gate.recovery_ramp_s": 1.5,
            "optical_flow.rotation_gate.minimum_translation_m": 0.01,
            "optical_flow.rotation_gate.minimum_translation_speed_mps": 0.08,
            "optical_flow.rotation_gate.recovery_max_base_score": 0.55,
            "optical_flow.rotation_gate.imu_max_gap_s": 0.12,
            # APM's FCU optical-flow path compensates body rotation before
            # fusion.  Keep the same behavior for the direct companion path.
            "optical_flow.rotation_gate.allow_compensated": True,
            "optical_flow.lever_arm_compensation_enabled": True,
            "optical_flow.flow_sensor_offset_body_m": [0.0, 0.0, -0.35],
            "vision.feature_reference": 150,
            "vision.tau_reprojection_px": 3.0,
            "vision.weights": [0.30, 0.25, 0.25, 0.20],
            "vision.minimum_features": 20,
            "vision.track_evidence_enabled": True,
            "vision.klt_weight": 0.15,
            "vision.factor_score_topic": VISION_FACTOR_SCORE_TOPIC,
            "vision.camera_cache_size": 4,
            "vision.minimum_camera_evidence_coverage": 0.75,
            "vision.minimum_depth_m": 0.30,
            "vision.maximum_depth_m": 6.0,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        self.score_publishers = {
            name: self.create_publisher(
                ReliabilityScore,
                (
                    VISION_HEALTH_TOPIC
                    if name == "vision"
                    else f"/reliability/{name}_score"
                ),
                20) for name in (
                "lidar",
                "lidar_map",
                "gnss",
                "imu",
                "optical_flow",
                "vision")}
        vision_factor_topic = str(
            self.get_parameter("vision.factor_score_topic").value)
        if vision_factor_topic == VISION_HEALTH_TOPIC:
            raise ValueError(
                "vision.factor_score_topic must differ from camera health topic"
            )
        self.vision_factor_publisher = self.create_publisher(
            ReliabilityScore,
            vision_factor_topic,
            20,
        )
        self.gnss_integrity_pub = self.create_publisher(
            GnssIntegrity, "/reliability/gnss_integrity", 20)
        self.last_imu = None
        self.last_imu_ns = None
        self.last_imu_publish_ns = None
        self.imu_window = deque(maxlen=100)
        self.flow_imu_yaw_samples = deque(maxlen=2000)
        self.flow_imu_samples = deque(maxlen=3000)
        self.flow_lever_arm_compensated = 0
        self.flow_lever_arm_unavailable = 0
        self.imu_preintegration_residual = None
        self.imu_preintegration_residual_arrival = None
        self.lidar_innovation_position = None
        self.lidar_innovation_yaw = None
        self.lidar_innovation_arrival = None
        self.gnss_prefit_nis = None
        self.gnss_prefit_residual_norm_m = None
        self.gnss_prefit_stamp_s = None
        self.last_gnss = None
        self.last_gnss_ns = None
        self.last_gnss_arrival_ns = None
        self.last_gnss_lio_position = None
        self.last_gps_raw = None
        self.last_gps_raw_arrival_ns = None
        self.lio_position = None
        self.lio_samples = deque(maxlen=1000)
        self.pending_flows = deque(maxlen=100)
        self.flow_rotation_gate = OpticalFlowRotationGate(
            FlowRotationGateConfig(
                lower_yaw_rate_radps=float(self.get_parameter(
                    "optical_flow.rotation_gate.lower_yaw_rate_radps").value),
                upper_yaw_rate_radps=float(self.get_parameter(
                    "optical_flow.rotation_gate.upper_yaw_rate_radps").value),
                recovery_dwell_s=float(self.get_parameter(
                    "optical_flow.rotation_gate.recovery_dwell_s").value),
                recovery_ramp_s=float(self.get_parameter(
                    "optical_flow.rotation_gate.recovery_ramp_s").value),
                minimum_translation_m=float(self.get_parameter(
                    "optical_flow.rotation_gate.minimum_translation_m").value),
                minimum_translation_speed_mps=float(self.get_parameter(
                    "optical_flow.rotation_gate.minimum_translation_speed_mps").value),
                allow_compensated_rotation=bool(self.get_parameter(
                    "optical_flow.rotation_gate.allow_compensated").value),
            )
        )
        self.flow_lever_arm_compensation_enabled = bool(
            self.get_parameter(
                "optical_flow.lever_arm_compensation_enabled"
            ).value
        )
        flow_sensor_offset = tuple(
            float(value) for value in self.get_parameter(
                "optical_flow.flow_sensor_offset_body_m"
            ).value
        )
        if len(flow_sensor_offset) != 3 or not all(
            math.isfinite(value) for value in flow_sensor_offset
        ):
            raise ValueError(
                "optical_flow.flow_sensor_offset_body_m must be a finite 3-vector"
            )
        self.flow_sensor_offset_body_m = tuple(flow_sensor_offset)
        self.latest_depth_ratio = -1.0
        self.latest_blur_energy = -1.0
        self.latest_feature_count = 0
        self.latest_spatial_uniformity = 0.0
        self.last_vision_publish_ns = None
        self.last_visual_tracks_ns = None
        self.vision_color_metrics = OrderedDict()
        self.vision_depth_metrics = OrderedDict()

        self.create_subscription(
            LioDiagnostics,
            "/lio/diagnostics",
            self._lidar,
            20)
        self.create_subscription(
            NavSatFix,
            "/sensors/gnss/fix",
            self._gnss,
            qos_profile_sensor_data)
        self.create_subscription(
            GPSRAW, str(self.get_parameter("gnss.raw_topic").value),
            self._gps_raw, qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/sensors/imu",
            self._imu,
            qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("imu.backend_diagnostic_topic").value),
            self._backend_diagnostics,
            10,
        )
        self.create_subscription(
            OpticalFlowRad,
            "/sensors/optical_flow/rad",
            self._flow,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            "/sensors/rgbd/depth",
            self._depth,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            "/sensors/rgbd/color",
            self._color,
            qos_profile_sensor_data)
        self.create_subscription(
            VisualFeatureTracks, "/vision/feature_tracks",
            self._visual_tracks, qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/lio/odom", self._odom, 20)
        self.create_timer(0.5, self._outage_timer)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _age_s(now_s, received_s):
        if received_s is None or received_s > now_s:
            return math.inf
        return now_s - received_s

    def _publish(
        self,
        modality,
        header,
        result,
        valid=True,
        observation_count=1,
        minimum_observation_count=1,
        minimum_evidence_coverage=1.0,
        publisher=None,
    ):
        score, evidence, reasons = result
        usable = score_operationally_usable(
            valid,
            evidence,
            observation_count,
            minimum_observation_count,
            minimum_evidence_coverage,
        )
        evidence["minimum_operational_evidence_coverage"] = float(
            minimum_evidence_coverage
        )
        evidence["score_operationally_usable"] = 1.0 if usable else 0.0
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
        target = self.score_publishers[modality] if publisher is None else publisher
        target.publish(msg)

    def _lidar(self, msg):
        paper_result = lidar_score(
            msg.hessian_eigenvalues, msg.normal_covariance_eigenvalues,
            msg.axial_penalty, msg.matched_points,
            self.get_parameter("lidar.match_reference").value,
            self.get_parameter("lidar.tau_lambda").value,
            self.get_parameter("lidar.tau_kappa").value,
            self.get_parameter("lidar.tau_normal").value,
            tuple(self.get_parameter("lidar.weights").value),
        )
        innovation_position = self.lidar_innovation_position
        innovation_yaw = self.lidar_innovation_yaw
        innovation_age_s = -1.0
        if self.lidar_innovation_arrival is not None:
            innovation_age_s = self._age_s(
                self._now_s(), self.lidar_innovation_arrival
            )
            if innovation_age_s > float(
                self.get_parameter("lidar.factor.innovation_timeout_s").value
            ):
                innovation_position = None
                innovation_yaw = None
        innovation_result = lidar_innovation_score(
            innovation_position,
            innovation_yaw,
            self.get_parameter("lidar.factor.tau_position_innovation_m").value,
            self.get_parameter("lidar.factor.tau_yaw_innovation_rad").value,
            tuple(self.get_parameter("lidar.factor.innovation_weights").value),
        )
        result = lidar_factor_score(
            paper_result,
            innovation_result,
            bool(msg.approximate),
            self.get_parameter("lidar.factor.approximate_geometry_weight").value,
            self.get_parameter("lidar.factor.native_geometry_weight").value,
        )
        result[1]["lidar_innovation_age_s"] = innovation_age_s
        result[1]["residual_mean_m_diagnostic"] = float(msg.residual_mean_m)
        self._publish(
            "lidar",
            msg.header,
            result,
            msg.input_points > 0,
            msg.matched_points,
            self.get_parameter("lidar.minimum_matches").value,
        )
        map_result = lidar_map_score(
            msg.residual_p95_m,
            msg.spatial_coverage,
            msg.dynamic_ratio,
            msg.uncertain_ratio,
            msg.feature_repeatability,
            self.get_parameter("lidar.map.tau_residual_m").value,
            self.get_parameter("lidar.map.tau_dynamic_ratio").value,
            self.get_parameter("lidar.map.tau_uncertain_ratio").value,
            tuple(self.get_parameter("lidar.map.weights").value),
            map_quality_diagnostic=msg.map_quality,
        )
        self._publish(
            "lidar_map",
            msg.header,
            map_result,
            msg.input_points > 0,
            msg.input_points,
            1,
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
        source_ns = stamp_ns(msg.header)
        self.last_gps_raw_arrival_ns = source_ns if source_ns > 0 else None

    def _backend_diagnostics(self, msg):
        source_s = stamp_ns(msg.header) * 1.0e-9
        if source_s <= 0.0:
            source_s = None
        value = nonnegative_diagnostic_value(
            msg,
            "unified_backend_fusion",
            "imu_preintegration_residual_mahalanobis",
        )
        self.imu_preintegration_residual = value
        self.imu_preintegration_residual_arrival = (
            source_s if value is not None else None
        )
        position_innovation = nonnegative_diagnostic_value(
            msg,
            "unified_backend_fusion",
            "lidar_prediction_position_innovation_m",
        )
        yaw_innovation = nonnegative_diagnostic_value(
            msg,
            "unified_backend_fusion",
            "lidar_prediction_yaw_innovation_rad",
        )
        self.lidar_innovation_position = position_innovation
        self.lidar_innovation_yaw = yaw_innovation
        self.lidar_innovation_arrival = (
            source_s
            if position_innovation is not None and yaw_innovation is not None
            else None
        )
        gnss_prefit_nis = nonnegative_diagnostic_value(
            msg,
            "unified_backend_fusion",
            "gnss_prefit_nis",
        )
        gnss_prefit_residual = nonnegative_diagnostic_value(
            msg,
            "unified_backend_fusion",
            "gnss_prefit_residual_norm_m",
        )
        gnss_prefit_stamp = nonnegative_diagnostic_value(
            msg,
            "unified_backend_fusion",
            "gnss_prefit_stamp_s",
        )
        if (
            gnss_prefit_nis is not None
            and gnss_prefit_residual is not None
            and gnss_prefit_stamp is not None
        ):
            self.gnss_prefit_nis = gnss_prefit_nis
            self.gnss_prefit_residual_norm_m = gnss_prefit_residual
            self.gnss_prefit_stamp_s = gnss_prefit_stamp

    def _gnss(self, msg):
        current_ns = stamp_ns(msg.header)
        covariance = float(
            msg.position_covariance[0] +
            msg.position_covariance[4] +
            msg.position_covariance[8])
        innovation = -1.0
        innovation_mahalanobis = -1.0
        backend_prefit_age_s = -1.0
        backend_prefit_used = False
        jump = False
        if self.last_gnss is not None and self.last_gnss_ns is not None:
            dt = max(1.0e-3, (current_ns - self.last_gnss_ns) * 1.0e-9)
            gnss_delta = distance_m(msg, self.last_gnss)
            jump = gnss_delta / dt > 15.0 or gnss_delta > 20.0
            if self.lio_position is not None and self.last_gnss_lio_position is not None:
                lio_delta = float(
                    np.linalg.norm(
                        self.lio_position -
                        self.last_gnss_lio_position))
                innovation = abs(gnss_delta - lio_delta)
                innovation_mahalanobis = innovation * \
                    innovation / max(0.01, covariance)
        if jump and innovation_mahalanobis < 0.0:
            innovation_mahalanobis = 10.0
        current_s = current_ns * 1.0e-9
        if self.gnss_prefit_stamp_s is not None:
            backend_prefit_age_s = current_s - self.gnss_prefit_stamp_s
            if (
                0.0 <= backend_prefit_age_s
                <= float(self.get_parameter("gnss.prefit_nis_timeout_s").value)
                and self.gnss_prefit_nis is not None
                and self.gnss_prefit_residual_norm_m is not None
            ):
                innovation_mahalanobis = float(self.gnss_prefit_nis)
                innovation = float(self.gnss_prefit_residual_norm_m)
                backend_prefit_used = True
        raw = None
        if self.last_gps_raw is not None and self.last_gps_raw_arrival_ns is not None:
            age_s = (current_ns - self.last_gps_raw_arrival_ns) * 1.0e-9
            if 0.0 <= age_s <= float(
                self.get_parameter("gnss.raw_timeout_s").value
            ):
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
        score, evidence, reasons = gnss_score(
            q_fix, covariance, innovation_mahalanobis,
            self.get_parameter("gnss.tau_covariance").value,
            self.get_parameter("gnss.tau_innovation").value,
            tuple(self.get_parameter("gnss.weights").value),
            hard_jump=jump,
        )
        evidence.update(integrity_evidence)
        evidence["vdop"] = -1.0 if vdop is None else vdop
        evidence["fcu_metadata_fresh"] = 0.0 if raw is None else 1.0
        evidence["innovation_source_backend_prefit"] = (
            1.0 if backend_prefit_used else 0.0
        )
        evidence["backend_prefit_age_s"] = backend_prefit_age_s
        reasons.extend(integrity_reasons)
        evidence_coverage = float(evidence["evidence_weight_coverage"])
        if innovation_mahalanobis < 0.0:
            evidence["partial_score_eq23"] = float(score)
            score = conservative_partial_score(score, evidence_coverage)
            evidence["provisional_score_missing_as_degraded"] = float(score)
            evidence["hard_gate_allowed"] = 0.0
            reasons.append("provisional_gnss_direct_evidence_only")
        direct_evidence_valid = bool(
            msg.status.status >= 0
            and math.isfinite(covariance)
            and covariance >= 0.0
        )
        self._publish(
            "gnss",
            msg.header,
            (score, evidence, reasons),
            direct_evidence_valid,
            minimum_evidence_coverage=self.get_parameter(
                "gnss.minimum_startup_evidence_coverage"
            ).value,
        )
        integrity = GnssIntegrity()
        integrity.header = copy.deepcopy(msg.header)
        integrity.fix_status = int(msg.status.status)
        integrity.service = int(msg.status.service)
        integrity.satellite_count = 0 if raw is None else int(
            raw.satellites_visible)
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
        self.last_gnss_arrival_ns = current_ns
        self.last_gnss_lio_position = (
            None if self.lio_position is None else self.lio_position.copy()
        )

    def _imu(self, msg):
        current_ns = stamp_ns(msg.header)
        accel = np.asarray([msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z],
                           dtype=float)
        gyro = np.asarray([msg.angular_velocity.x,
                           msg.angular_velocity.y,
                           msg.angular_velocity.z],
                          dtype=float)
        sample_finite = bool(
            np.all(np.isfinite(accel)) and np.all(np.isfinite(gyro))
        )
        timestamp_valid = bool(
            current_ns > 0
            and (self.last_imu_ns is None or current_ns > self.last_imu_ns)
        )
        jerk = 0.0
        if (
            sample_finite
            and timestamp_valid
            and self.last_imu is not None
            and self.last_imu_ns is not None
        ):
            dt = (current_ns - self.last_imu_ns) * 1.0e-9
            if dt > 1.0e-4:
                jerk = float(np.linalg.norm(accel - self.last_imu) / dt)
        if sample_finite and timestamp_valid:
            self.last_imu = accel
            self.last_imu_ns = current_ns
            timestamp_s = current_ns * 1.0e-9
            self.flow_imu_yaw_samples.append((timestamp_s, float(gyro[2])))
            self.flow_imu_samples.append(
                (timestamp_s, tuple(float(value) for value in gyro)))
            cutoff_s = timestamp_s - 5.0
            while (
                self.flow_imu_yaw_samples
                and self.flow_imu_yaw_samples[0][0] < cutoff_s
            ):
                self.flow_imu_yaw_samples.popleft()
            while (
                self.flow_imu_samples
                and self.flow_imu_samples[0][0] < cutoff_s
            ):
                self.flow_imu_samples.popleft()
            self.imu_window.append((accel, gyro))
        health_failure = not sample_finite or not timestamp_valid
        if (
            not health_failure
            and self.last_imu_publish_ns is not None
            and current_ns - self.last_imu_publish_ns < 100_000_000
        ):
            return
        if timestamp_valid:
            self.last_imu_publish_ns = current_ns
        accel_values = np.asarray(
            [sample[0] for sample in self.imu_window], dtype=float
        )
        gyro_values = np.asarray(
            [sample[1] for sample in self.imu_window], dtype=float
        )
        accel_excitation = float(
            np.mean(
                np.std(
                    accel_values,
                    axis=0))) if len(accel_values) > 2 else 0.0
        gyro_excitation = float(
            np.mean(
                np.std(
                    gyro_values,
                    axis=0))) if len(gyro_values) > 2 else 0.0
        excitation = min(1.0, 0.5 *
                         (accel_excitation /
                          self.get_parameter("imu.accel_excitation_scale").value +
                             gyro_excitation /
                             self.get_parameter("imu.gyro_excitation_scale").value))
        accel_norm = float(np.linalg.norm(accel)) if sample_finite else -1.0
        gyro_norm = float(np.linalg.norm(gyro)) if sample_finite else -1.0
        saturation = (
            sample_finite
            and (
                accel_norm >= self.get_parameter("imu.accel_saturation").value
                or gyro_norm >= self.get_parameter("imu.gyro_saturation").value
            )
        )
        residual = -1.0
        residual_age_s = -1.0
        if self.imu_preintegration_residual_arrival is not None:
            residual_age_s = self._age_s(
                self._now_s(), self.imu_preintegration_residual_arrival
            )
            if residual_age_s <= float(self.get_parameter(
                    "imu.preintegration_residual_timeout_s").value):
                residual = float(self.imu_preintegration_residual)
        paper_result = imu_score(
            excitation, residual, saturation,
            self.get_parameter("imu.tau_preintegration").value,
            tuple(self.get_parameter("imu.weights").value),
        )
        result = imu_health_admission(
            paper_result,
            sample_finite=sample_finite,
            saturation=saturation,
            timestamp_valid=timestamp_valid,
        )
        result[1]["accel_norm_mps2"] = accel_norm
        result[1]["gyro_norm_radps"] = gyro_norm
        result[1]["jerk_mps3_diagnostic"] = jerk
        result[1]["preintegration_residual_age_s"] = residual_age_s
        result[1]["preintegration_residual_available"] = (
            1.0 if residual >= 0.0 else 0.0
        )
        if residual < 0.0:
            result[2].append(
                "diagnostic_only:preintegration_residual_unavailable_eq21"
            )
        self._publish(
            "imu",
            msg.header,
            result,
            not health_failure and not saturation,
            len(self.imu_window),
            self.get_parameter("imu.minimum_window_samples").value,
            self.get_parameter(
                "imu.minimum_startup_evidence_coverage"
            ).value,
        )

    def _flow_lever_arm_displacement(self, msg):
        """Return the FLU sensor-point displacement for one flow exposure."""
        if not self.flow_lever_arm_compensation_enabled:
            return np.zeros(3, dtype=float), "disabled"
        integration_s = float(msg.integration_time_us) * 1.0e-6
        end_s = stamp_ns(msg.header) * 1.0e-9
        if integration_s <= 0.0 or end_s <= 0.0:
            self.flow_lever_arm_unavailable += 1
            return None, "invalid_integration"
        angular_velocity = interval_mean_vector(
            list(self.flow_imu_samples),
            end_s - integration_s,
            end_s,
            float(self.get_parameter(
                "optical_flow.rotation_gate.imu_max_gap_s").value),
        )
        if angular_velocity is None:
            self.flow_lever_arm_unavailable += 1
            return None, "imu_interval_unavailable"
        correction = optical_flow_lever_arm_displacement_flu(
            angular_velocity,
            self.flow_sensor_offset_body_m,
            integration_s,
        )
        if correction is None:
            self.flow_lever_arm_unavailable += 1
            return None, "invalid_gyro_or_mount"
        self.flow_lever_arm_compensated += 1
        return np.asarray(correction, dtype=float), "per_exposure_imu"

    def _flow(self, msg):
        self.pending_flows.append((self._now_s(), copy.deepcopy(msg)))
        self._flush_flows()

    def _flow_prediction(self, msg):
        integration_s = float(msg.integration_time_us) * 1.0e-6
        if integration_s <= 1.0e-5:
            return None
        end_s = stamp_ns(msg.header) * 1.0e-9
        maximum_gap = float(self.get_parameter(
            "optical_flow.lio_max_gap_s").value)
        current = interpolate_lio(self.lio_samples, end_s, maximum_gap)
        previous = interpolate_lio(
            self.lio_samples,
            end_s - integration_s,
            maximum_gap)
        if current is None or previous is None:
            return None
        world_delta = current[:2] - previous[:2]
        yaw = current[2]
        body_flu_x = math.cos(
            yaw) * world_delta[0] + math.sin(yaw) * world_delta[1]
        body_flu_y = -math.sin(yaw) * \
            world_delta[0] + math.cos(yaw) * world_delta[1]
        return float(body_flu_x), float(-body_flu_y)

    def _flush_flows(self):
        now = self._now_s()
        wait_s = float(self.get_parameter("optical_flow.lio_wait_s").value)
        while self.pending_flows:
            queued_at, msg = self.pending_flows[0]
            if queued_at > now:
                self.pending_flows.clear()
                return
            prediction = self._flow_prediction(msg)
            if prediction is None and now - queued_at < wait_s:
                break
            self.pending_flows.popleft()
            self._publish_flow(msg, prediction)

    def _publish_flow(self, msg, prediction):
        raw_flow_displacement = optical_flow_displacement_frd(
            msg.integrated_x, msg.integrated_y,
            msg.integrated_xgyro, msg.integrated_ygyro,
            msg.distance,
        )
        flow_displacement = raw_flow_displacement
        lever_correction = None
        lever_reason = "not_attempted"
        if raw_flow_displacement is not None:
            lever_correction, lever_reason = self._flow_lever_arm_displacement(
                msg)
            if lever_correction is not None:
                # Convert the FLU correction to FRD before removing it from the
                # APM-compatible sensor displacement.
                flow_displacement = (
                    float(
                        raw_flow_displacement[0]) -
                    float(
                        lever_correction[0]),
                    float(
                        raw_flow_displacement[1]) +
                    float(
                        lever_correction[1]),
                )
        prediction_fallback_allowed = (
            prediction is None
            and flow_displacement is not None
            and int(msg.quality) >= int(self.get_parameter(
                "optical_flow.prediction_fallback_min_quality").value)
            and 0.10 <= float(msg.distance) <= 30.0
            and bool(self.get_parameter(
                "optical_flow.allow_prediction_fallback").value)
        )
        score, evidence, reasons = optical_flow_score(
            flow_displacement, prediction, msg.quality, msg.distance,
            self.get_parameter("optical_flow.tau_translation").value,
            tuple(self.get_parameter("optical_flow.weights").value),
            allow_prediction_fallback=prediction_fallback_allowed,
        )
        integration_s = float(msg.integration_time_us) * 1.0e-6
        end_s = stamp_ns(msg.header) * 1.0e-9
        yaw_rate = interval_mean_absolute_yaw_rate(
            self.flow_imu_yaw_samples,
            end_s - integration_s,
            end_s,
            float(self.get_parameter(
                "optical_flow.rotation_gate.imu_max_gap_s").value),
        )
        translation_norm = (
            None if flow_displacement is None else math.hypot(
                float(
                    flow_displacement[0]), float(
                    flow_displacement[1])))
        # The gyro-compensated displacement is an observation in its own
        # right.  A missing LIO prediction must not turn a valid flow sample
        # into a rotation-gate failure; its quality and distance still enter
        # the normal score and scheduler below.
        rotation_compensated = flow_displacement is not None
        observation_healthy = (
            rotation_compensated
            and int(msg.quality) > 0
            and 0.10 <= float(msg.distance) <= 30.0
        )
        rotation_gate = self.flow_rotation_gate.update(
            end_s,
            yaw_rate,
            translation_norm,
            observation_healthy,
            translation_interval_s=integration_s,
            rotation_compensated=rotation_compensated,
        )
        rotation_term = 1.0 - rotation_gate.weight
        score = max(float(score), rotation_term)
        evidence.update({
            "fcu_yaw_rate_abs_radps": rotation_gate.yaw_rate_abs_radps,
            "rotation_gate_weight": rotation_gate.weight,
            "rotation_gate_term": rotation_term,
            "rotation_gate_phase_code": OpticalFlowRotationGate.PHASE_CODES[
                rotation_gate.phase
            ],
            "rotation_gate_translation_ready": (
                1.0 if rotation_gate.translation_ready else 0.0
            ),
            "rotation_gate_translation_speed_mps": (
                -1.0
                if translation_norm is None or integration_s <= 0.0
                else translation_norm / integration_s
            ),
        })
        if rotation_gate.phase != "ACTIVE":
            reasons.append(rotation_gate.reason)
        if rotation_gate.hard_disabled:
            reasons.append("flow_rotation_gate_marks_observation_unavailable")
        if lever_reason not in {
            "per_exposure_imu",
            "disabled",
                "not_attempted"}:
            reasons.append(f"flow_lever_arm_{lever_reason}")
        evidence.update({
            "flow_lever_arm_compensation_enabled": (
                1.0 if self.flow_lever_arm_compensation_enabled else 0.0
            ),
            "flow_lever_arm_compensation_valid": (
                1.0 if lever_correction is not None else 0.0
            ),
            "flow_lever_arm_source": (
                1.0 if lever_reason == "per_exposure_imu" else 0.0
            ),
            "flow_lever_arm_displacement_x_m": (
                -1.0 if lever_correction is None else float(lever_correction[0])
            ),
            "flow_lever_arm_displacement_y_m": (
                -1.0 if lever_correction is None else float(lever_correction[1])
            ),
            "flow_delta_sensor_x_m": (
                -1.0 if raw_flow_displacement is None
                else float(raw_flow_displacement[0])
            ),
            "flow_delta_sensor_y_m": (
                -1.0 if raw_flow_displacement is None
                else float(raw_flow_displacement[1])
            ),
            "flow_delta_compensated_x_m": (
                -1.0 if flow_displacement is None
                else float(flow_displacement[0])
            ),
            "flow_delta_compensated_y_m": (
                -1.0 if flow_displacement is None
                else float(flow_displacement[1])
            ),
        })
        result = score, evidence, reasons
        gyro_available = flow_displacement is not None
        result[1]["gyro_compensation_available"] = 1.0 if gyro_available else 0.0
        result[1]["lio_increment_available"] = 0.0 if prediction is None else 1.0
        result[1]["translation_vector_residual_eq22_adapted"] = 1.0
        if not gyro_available:
            result[2].append("gyro_compensation_unavailable")
        self._publish(
            "optical_flow", msg.header, result,
            flow_observation_valid(gyro_available, yaw_rate, rotation_gate),
        )

    def _depth(self, msg):
        current_ns = stamp_ns(msg.header)
        self.vision_depth_metrics[current_ns] = depth_valid_ratio(
            msg,
            self.get_parameter("vision.minimum_depth_m").value,
            self.get_parameter("vision.maximum_depth_m").value,
        )
        self._trim_vision_cache(self.vision_depth_metrics)
        self._publish_vision_pair(current_ns, msg.header)

    def _color(self, msg):
        current_ns = stamp_ns(msg.header)
        self.vision_color_metrics[current_ns] = image_feature_support(msg)
        self._trim_vision_cache(self.vision_color_metrics)
        self._publish_vision_pair(current_ns, msg.header)

    def _trim_vision_cache(self, cache):
        maximum_size = max(
            1, int(self.get_parameter("vision.camera_cache_size").value))
        while len(cache) > maximum_size:
            cache.popitem(last=False)

    def _publish_vision_pair(self, current_ns, header):
        if (
            current_ns not in self.vision_color_metrics
            or current_ns not in self.vision_depth_metrics
        ):
            return
        feature_count, spatial_uniformity, blur_energy = (
            self.vision_color_metrics.pop(current_ns)
        )
        depth_ratio = self.vision_depth_metrics.pop(current_ns)
        if self.last_vision_publish_ns is not None and current_ns - \
                self.last_vision_publish_ns < 200_000_000:
            return
        self.last_vision_publish_ns = current_ns
        result = vision_score(
            feature_count, self.get_parameter("vision.feature_reference").value,
            spatial_uniformity, -1.0, depth_ratio,
            self.get_parameter("vision.tau_reprojection_px").value,
            tuple(self.get_parameter("vision.weights").value),
        )
        result[1]["blur_energy_diagnostic"] = blur_energy
        result[1]["projection_consistency_unavailable"] = -1.0
        result[1]["camera_health_exact_rgbd_pair"] = 1.0
        self._publish(
            "vision",
            header,
            result,
            depth_ratio >= 0.0,
            feature_count,
            self.get_parameter("vision.minimum_features").value,
            self.get_parameter(
                "vision.minimum_camera_evidence_coverage").value,
        )

    def _visual_tracks(self, msg):
        if not bool(self.get_parameter("vision.track_evidence_enabled").value):
            return
        current_ns = stamp_ns(msg.header)
        if self.last_visual_tracks_ns is not None and current_ns <= self.last_visual_tracks_ns:
            return
        self.last_visual_tracks_ns = current_ns
        metrics = visual_factor_track_metrics(msg)
        feature_count = int(metrics["selected_track_count"])
        depth_ratio = float(metrics["depth_valid_ratio"])
        result = vision_score(
            feature_count,
            self.get_parameter("vision.feature_reference").value,
            float(metrics["spatial_distribution"]),
            float(metrics["mean_reprojection_error_px"]),
            depth_ratio,
            self.get_parameter("vision.tau_reprojection_px").value,
            tuple(self.get_parameter("vision.weights").value),
        )
        score, evidence, reasons = result
        selection_ratio = max(
            0.0, min(1.0, float(metrics["selected_track_ratio"])))
        klt_weight = max(
            0.0, min(
                0.5, float(
                    self.get_parameter("vision.klt_weight").value)))
        score = (
            (1.0 - klt_weight) * score
            + klt_weight * (1.0 - selection_ratio)
        )
        evidence["factor_raw_track_count"] = float(metrics["raw_track_count"])
        evidence["factor_geometry_eligible_count"] = float(
            metrics["geometry_eligible_count"])
        evidence["factor_selected_track_count"] = float(feature_count)
        evidence["factor_selected_track_ratio"] = selection_ratio
        evidence["klt_extension_weight"] = klt_weight
        evidence["pnp_geometric_verification"] = 1.0 if msg.pnp_valid else 0.0
        evidence["valid_depth_track_ratio"] = depth_ratio
        evidence["factor_candidate_score"] = 1.0
        if selection_ratio < 0.5:
            reasons.append("weak_selected_track_retention")
        if not msg.pnp_valid:
            reasons.append("pnp_geometric_verification_failed")
        self._publish(
            "vision",
            msg.header,
            (score,
             evidence,
             reasons),
            bool(
                msg.pnp_valid
                and feature_count >= self.get_parameter(
                    "vision.minimum_features"
                ).value
            ),
            feature_count,
            self.get_parameter("vision.minimum_features").value,
            publisher=self.vision_factor_publisher,
        )

    def _outage_timer(self):
        self._flush_flows()
        if self.last_gnss_arrival_ns is None or self.last_gnss is None:
            return
        outage_s = (self.get_clock().now().nanoseconds -
                    self.last_gnss_arrival_ns) * 1.0e-9
        if outage_s < 0.0:
            self.last_gnss_arrival_ns = None
            return
        if outage_s <= 0.75:
            return
        covariance = float(
            self.last_gnss.position_covariance[0] +
            self.last_gnss.position_covariance[4] +
            self.last_gnss.position_covariance[8])
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
    except Exception:
        # Humble may surface RCLError instead of ExternalShutdownException
        # when launch invalidates the context while the executor is waiting.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
