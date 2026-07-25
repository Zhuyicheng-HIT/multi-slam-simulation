"""Extract timestamped backend factors from a ROS 2 sensor bag.

The extractor deliberately keeps the estimator/reference boundary explicit:
``/lio/odom`` becomes a LiDAR pose factor, while ``/Odometry`` is copied only
as an evaluation reference. No FCU fused position or Gazebo truth is used as a
factor.
"""

from bisect import bisect_left, bisect_right
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from uf_interfaces.msg import LioDiagnostics

from .imu_preintegration import ImuSample, preintegrate
from uf_reliability.scoring import (
    gnss_score,
    lidar_score,
    optical_flow_displacement_frd,
    optical_flow_score,
)


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
DISABLE_THRESHOLD = 0.80
MAX_COVARIANCE_INFLATION = 20.0
MIN_FLOW_QUALITY = 20


def message_stamp(message: Any) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(quaternion) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9 or not math.isfinite(norm):
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float):
    latitude = math.radians(float(latitude_deg))
    longitude = math.radians(float(longitude_deg))
    sin_latitude = math.sin(latitude)
    prime_vertical = WGS84_A_M / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )
    return (
        (prime_vertical + altitude_m) * math.cos(latitude) * math.cos(longitude),
        (prime_vertical + altitude_m) * math.cos(latitude) * math.sin(longitude),
        (prime_vertical * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
    )


class LocalEnuProjector:
    def __init__(self, latitude_deg: float, longitude_deg: float, altitude_m: float):
        self.latitude = math.radians(float(latitude_deg))
        self.longitude = math.radians(float(longitude_deg))
        self.origin = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)

    def project(self, latitude_deg: float, longitude_deg: float, altitude_m: float):
        x, y, z = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        dx, dy, dz = x - self.origin[0], y - self.origin[1], z - self.origin[2]
        sin_latitude, cos_latitude = math.sin(self.latitude), math.cos(self.latitude)
        sin_longitude, cos_longitude = math.sin(self.longitude), math.cos(self.longitude)
        return (
            -sin_longitude * dx + cos_longitude * dy,
            -sin_latitude * cos_longitude * dx
            - sin_latitude * sin_longitude * dy
            + cos_latitude * dz,
            cos_latitude * cos_longitude * dx
            + cos_latitude * sin_longitude * dy
            + sin_latitude * dz,
        )


def frd_to_enu_delta(forward_m: float, right_m: float, yaw_enu_rad: float):
    left_m = -float(right_m)
    cosine, sine = math.cos(float(yaw_enu_rad)), math.sin(float(yaw_enu_rad))
    return (
        cosine * float(forward_m) - sine * left_m,
        sine * float(forward_m) + cosine * left_m,
    )


def scheduler_decision(score: float) -> dict[str, float | bool]:
    score = max(0.0, min(1.0, float(score)))
    reliability = max(0.0, 1.0 - score)
    enabled = score < DISABLE_THRESHOLD
    inflation = min(
        MAX_COVARIANCE_INFLATION,
        1.0 / max(reliability, 0.05),
    )
    return {
        "factor_enabled": enabled,
        "reliability_weight": reliability if enabled else 0.0,
        "covariance_inflation": inflation,
        "degradation_score": score,
    }


def optical_flow_decision(score: float, evidence: dict[str, float],
                          reasons: list[str], quality: float):
    """Apply the paper-inspired score plus the sensor's hard validity gate.

    Optical-flow quality is a validity field, not only a soft confidence term.
    A zero-quality MTF01P sample must not become a usable backend constraint
    merely because its displacement happens to agree with the prediction.
    """
    decision = scheduler_decision(score)
    decision["evidence"] = evidence
    decision["reasons"] = list(reasons)
    decision["quality_gate_passed"] = float(quality) >= MIN_FLOW_QUALITY
    if float(quality) < MIN_FLOW_QUALITY:
        decision["factor_enabled"] = False
        decision["reliability_weight"] = 0.0
        decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
        if "quality_below_minimum_extension" not in decision["reasons"]:
            decision["reasons"].append("quality_below_minimum_extension")
    return decision


def _nearest(records: list[dict[str, Any]], stamps: list[float], stamp: float, tolerance: float):
    if not records:
        return None
    index = min(len(records) - 1, bisect_left(stamps, stamp))
    candidates = [index]
    if index > 0:
        candidates.append(index - 1)
    selected = min(candidates, key=lambda item: abs(stamps[item] - stamp))
    return records[selected] if abs(stamps[selected] - stamp) <= tolerance else None


def _read_streams(bag_path: str) -> dict[str, list[dict[str, Any]]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {
        "/lio/odom": (Odometry, "lio"),
        "/Odometry": (Odometry, "reference"),
        "/sensors/gnss/fix": (NavSatFix, "gnss"),
        "/sensors/optical_flow/rad": (OpticalFlowRad, "flow"),
        "/sensors/imu": (Imu, "imu"),
        "/lio/diagnostics": (LioDiagnostics, "diagnostics"),
    }
    streams = {name: [] for _, name in wanted.values()}
    deserialize_skipped = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in wanted or topic not in types:
            continue
        message_type, stream_name = wanted[topic]
        try:
            message = deserialize_message(data, message_type)
        except Exception:
            # A bag may contain an older message ABI. Keep other streams usable
            # and expose the loss in the extraction report instead of fabricating
            # fields that were not present in the recording.
            deserialize_skipped += 1
            continue
        stamp = message_stamp(message)
        if not math.isfinite(stamp) or stamp <= 0.0:
            continue
        if stream_name in ("lio", "reference"):
            pose = message.pose.pose
            streams[stream_name].append({
                "stamp_s": stamp,
                "position": [
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ],
                "yaw_rad": yaw_from_quaternion(pose.orientation),
            })
        elif stream_name == "gnss":
            covariance = [
                float(message.position_covariance[0]),
                float(message.position_covariance[4]),
                float(message.position_covariance[8]),
            ]
            streams[stream_name].append({
                "stamp_s": stamp,
                "latitude": float(message.latitude),
                "longitude": float(message.longitude),
                "altitude": float(message.altitude),
                "covariance": covariance,
                "status": int(message.status.status),
            })
        elif stream_name == "imu":
            streams[stream_name].append({
                "stamp_s": stamp,
                "acceleration": [
                    float(message.linear_acceleration.x),
                    float(message.linear_acceleration.y),
                    float(message.linear_acceleration.z),
                ],
                "angular_velocity": [
                    float(message.angular_velocity.x),
                    float(message.angular_velocity.y),
                    float(message.angular_velocity.z),
                ],
            })
        elif stream_name == "flow":
            streams[stream_name].append({
                "stamp_s": stamp,
                "integration_time_us": int(message.integration_time_us),
                "integrated_x": float(message.integrated_x),
                "integrated_y": float(message.integrated_y),
                "integrated_xgyro": float(message.integrated_xgyro),
                "integrated_ygyro": float(message.integrated_ygyro),
                "quality": int(message.quality),
                "distance_m": float(message.distance),
            })
        else:
            streams[stream_name].append({
                "stamp_s": stamp,
                "input_points": int(message.input_points),
                "matched_points": int(message.matched_points),
                "residual_mean_m": float(message.residual_mean_m),
                "residual_p95_m": float(message.residual_p95_m),
                "hessian_eigenvalues": [float(value) for value in message.hessian_eigenvalues],
                "normal_covariance_eigenvalues": [
                    float(value) for value in message.normal_covariance_eigenvalues
                ],
                "axial_penalty": float(message.axial_penalty),
                "spatial_coverage": float(message.spatial_coverage),
            })
    for records in streams.values():
        records.sort(key=lambda item: item["stamp_s"])
    return streams, deserialize_skipped


def _align_relative_flow_clock(streams: dict[str, list[dict[str, Any]]]) -> float:
    """Align a relative camera clock to the absolute LIO clock when unambiguous."""
    if not streams["lio"] or not streams["flow"]:
        return 0.0
    lio_start = streams["lio"][0]["stamp_s"]
    lio_end = streams["lio"][-1]["stamp_s"]
    flow_start = streams["flow"][0]["stamp_s"]
    flow_end = streams["flow"][-1]["stamp_s"]
    lio_span = lio_end - lio_start
    flow_span = flow_end - flow_start
    if abs(lio_start - flow_start) < 1000.0:
        return 0.0
    # The camera may only be present for a prefix of the LIO recording. A
    # shorter positive flow span is still alignable; a longer span is not.
    if lio_span <= 0.0 or flow_span <= 5.0 or flow_span > lio_span + 5.0:
        return 0.0
    offset = lio_start - flow_start
    for record in streams["flow"]:
        record["stamp_s"] += offset
    streams["flow"].sort(key=lambda item: item["stamp_s"])
    return offset


def _lidar_decision(diagnostic: dict[str, Any] | None):
    if diagnostic is None:
        decision = scheduler_decision(0.35)
        decision["diagnostic_available"] = False
        decision["reasons"] = ["diagnostics_unavailable_legacy_bag"]
        return decision
    score, evidence, reasons = lidar_score(
        diagnostic["hessian_eigenvalues"],
        diagnostic["normal_covariance_eigenvalues"],
        diagnostic["axial_penalty"],
        diagnostic["matched_points"],
    )
    decision = scheduler_decision(score)
    decision["evidence"] = evidence
    decision["reasons"] = reasons
    return decision


def extract_factors(bag_path: str, output_path: str, tolerance_s: float = 0.30):
    streams, deserialize_skipped = _read_streams(bag_path)
    flow_stamp_offset_s = _align_relative_flow_clock(streams)
    lio = streams["lio"]
    if len(lio) < 2:
        raise RuntimeError("bag has fewer than two /lio/odom messages")
    lio_stamps = [record["stamp_s"] for record in lio]
    reference_stamps = [record["stamp_s"] for record in streams["reference"]]
    gnss_stamps = [record["stamp_s"] for record in streams["gnss"]]
    diagnostic_stamps = [record["stamp_s"] for record in streams["diagnostics"]]
    flow_stamps = [record["stamp_s"] for record in streams["flow"]]
    imu_stamps = [record["stamp_s"] for record in streams["imu"]]

    valid_gnss = [
        record for record in streams["gnss"]
        if all(math.isfinite(record[key]) for key in ("latitude", "longitude", "altitude"))
    ]
    projector = None
    if valid_gnss:
        origin = valid_gnss[0]
        projector = LocalEnuProjector(
            origin["latitude"], origin["longitude"], origin["altitude"]
        )
    first_position = np.asarray(lio[0]["position"], dtype=float)
    frames = []
    previous_stamp = None
    for index, record in enumerate(lio):
        stamp = record["stamp_s"]
        position = np.asarray(record["position"], dtype=float)
        frame = {
            "index": index,
            "stamp_s": stamp,
            "lidar_pose": {
                "position": record["position"],
                "rotation": [0.0, 0.0, record["yaw_rad"]],
                "decision": _lidar_decision(
                    _nearest(streams["diagnostics"], diagnostic_stamps, stamp, tolerance_s)
                ),
            },
            "reference_position": None,
            "gnss": None,
            "optical_flow": None,
            "imu_preintegration": None,
            "imu_preintegration_status": None,
        }
        reference = _nearest(streams["reference"], reference_stamps, stamp, tolerance_s)
        if reference is not None:
            frame["reference_position"] = reference["position"]
        gnss = _nearest(streams["gnss"], gnss_stamps, stamp, tolerance_s)
        if gnss is not None and projector is not None:
            enu = np.asarray(
                projector.project(gnss["latitude"], gnss["longitude"], gnss["altitude"]),
                dtype=float,
            )
            gnss_position = (first_position + enu).tolist()
            covariance = np.maximum(np.asarray(gnss["covariance"], dtype=float), 0.04).tolist()
            innovation = position - np.asarray(gnss_position, dtype=float)
            mahalanobis = float(np.sum(innovation * innovation / np.asarray(covariance)))
            score, evidence, reasons = gnss_score(
                1.0 if gnss["status"] >= 0 else 0.0,
                float(sum(covariance)),
                mahalanobis,
            )
            decision = scheduler_decision(score)
            decision["evidence"] = evidence
            decision["reasons"] = reasons
            frame["gnss"] = {
                "position": gnss_position,
                "covariance": covariance,
                "decision": decision,
            }
        if previous_stamp is not None:
            imu_start = max(0, bisect_left(imu_stamps, previous_stamp) - 1)
            imu_end = min(len(imu_stamps), bisect_right(imu_stamps, stamp) + 1)
            if imu_end - imu_start >= 2:
                imu_samples = [
                    ImuSample(
                        item["stamp_s"],
                        tuple(item["acceleration"]),
                        tuple(item["angular_velocity"]),
                    )
                    for item in streams["imu"][imu_start:imu_end]
                ]
                preintegrated = preintegrate(imu_samples, previous_stamp, stamp)
                frame["imu_preintegration_status"] = {
                    "valid": preintegrated.valid,
                    "reason": preintegrated.reason,
                    "sample_count": preintegrated.sample_count,
                    "max_gap_s": preintegrated.max_gap_s,
                }
                if preintegrated.valid:
                    frame["imu_preintegration"] = {
                        "dt_s": preintegrated.dt_s,
                        "delta_position": list(preintegrated.delta_position),
                        "delta_velocity": list(preintegrated.delta_velocity),
                        "delta_quaternion": list(preintegrated.delta_quaternion),
                        "covariance": list(preintegrated.covariance),
                        "sample_count": preintegrated.sample_count,
                        "max_gap_s": preintegrated.max_gap_s,
                        "reason": preintegrated.reason,
                    }
            flow_start = bisect_right(flow_stamps, previous_stamp)
            flow_end = bisect_right(flow_stamps, stamp)
            flow_records = streams["flow"][flow_start:flow_end]
            if flow_records:
                delta_enu = np.zeros(2, dtype=float)
                qualities, distances = [], []
                for flow in flow_records:
                    distance = flow["distance_m"]
                    if distance <= 0.0 or not math.isfinite(distance):
                        continue
                    delta_frd = optical_flow_displacement_frd(
                        flow["integrated_x"], flow["integrated_y"],
                        flow["integrated_xgyro"], flow["integrated_ygyro"],
                        distance,
                    )
                    if delta_frd is None:
                        continue
                    delta_enu += np.asarray(
                        frd_to_enu_delta(delta_frd[0], delta_frd[1], record["yaw_rad"]),
                        dtype=float,
                    )
                    qualities.append(flow["quality"])
                    distances.append(distance)
                if qualities:
                    lio_delta = position - np.asarray(lio[index - 1]["position"], dtype=float)
                    flow_delta = [float(delta_enu[0]), float(delta_enu[1])]
                    score, evidence, reasons = optical_flow_score(
                        flow_delta,
                        [float(lio_delta[0]), float(lio_delta[1])],
                        float(np.mean(qualities)),
                        float(np.mean(distances)),
                    )
                    decision = optical_flow_decision(
                        score, evidence, reasons, float(np.mean(qualities))
                    )
                    frame["optical_flow"] = {
                        "delta_position": [float(delta_enu[0]), float(delta_enu[1]), 0.0],
                        "covariance": [0.10 ** 2, 0.10 ** 2, 1.0],
                        "decision": decision,
                        "sample_count": len(qualities),
                    }
        frames.append(frame)
        previous_stamp = stamp

    imu_statuses = [
        frame["imu_preintegration_status"]
        for frame in frames
        if frame["imu_preintegration_status"] is not None
    ]
    invalid_reasons = {}
    for status in imu_statuses:
        if not status["valid"]:
            reason = status["reason"]
            invalid_reasons[reason] = invalid_reasons.get(reason, 0) + 1
    result = {
        "schema_version": 1,
        "source_bag": str(Path(bag_path).resolve()),
        "factor_frame": "lio_odom_assumed_local_enu",
        "reference_topic": "/Odometry",
        "reference_is_evaluation_only": True,
        "frame_count": len(frames),
        "streams": {name: len(records) for name, records in streams.items()},
        "deserialize_skipped_messages": deserialize_skipped,
        "lidar_diagnostics_available": bool(streams["diagnostics"]),
        "flow_stamp_offset_s": flow_stamp_offset_s,
        "imu_preintegration_summary": {
            "interval_count": len(imu_statuses),
            "valid_count": sum(status["valid"] for status in imu_statuses),
            "invalid_count": sum(not status["valid"] for status in imu_statuses),
            "invalid_reasons": invalid_reasons,
        },
        "frames": frames,
    }
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result
