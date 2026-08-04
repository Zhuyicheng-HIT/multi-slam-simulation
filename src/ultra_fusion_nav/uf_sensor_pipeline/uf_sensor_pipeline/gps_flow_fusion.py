import math
from dataclasses import dataclass


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    latitude = math.radians(float(latitude_deg))
    longitude = math.radians(float(longitude_deg))
    sin_latitude = math.sin(latitude)
    prime_vertical = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    x = (prime_vertical + altitude_m) * math.cos(latitude) * math.cos(longitude)
    y = (prime_vertical + altitude_m) * math.cos(latitude) * math.sin(longitude)
    z = (prime_vertical * (1.0 - WGS84_E2) + altitude_m) * sin_latitude
    return x, y, z


class LocalEnuProjector:
    def __init__(self, latitude_deg, longitude_deg, altitude_m):
        self.latitude = math.radians(float(latitude_deg))
        self.longitude = math.radians(float(longitude_deg))
        self.origin_ecef = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)

    def project(self, latitude_deg, longitude_deg, altitude_m):
        x, y, z = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        dx = x - self.origin_ecef[0]
        dy = y - self.origin_ecef[1]
        dz = z - self.origin_ecef[2]
        sin_latitude = math.sin(self.latitude)
        cos_latitude = math.cos(self.latitude)
        sin_longitude = math.sin(self.longitude)
        cos_longitude = math.cos(self.longitude)
        east = -sin_longitude * dx + cos_longitude * dy
        north = (
            -sin_latitude * cos_longitude * dx
            - sin_latitude * sin_longitude * dy
            + cos_latitude * dz
        )
        up = (
            cos_latitude * cos_longitude * dx
            + cos_latitude * sin_longitude * dy
            + sin_latitude * dz
        )
        return east, north, up


def yaw_from_quaternion(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compensated_flow_velocity_frd(
        integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
        integration_s, distance_m):
    values = (
        integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
        integration_s, distance_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    if integration_s <= 1.0e-4 or distance_m <= 0.0:
        return None
    translation_x = (float(integrated_x) - float(integrated_xgyro)) / integration_s
    translation_y = (float(integrated_y) - float(integrated_ygyro)) / integration_s
    return translation_y * distance_m, -translation_x * distance_m


def velocity_frd_to_enu(forward_mps, right_mps, yaw_enu_rad):
    left_mps = -float(right_mps)
    cosine = math.cos(float(yaw_enu_rad))
    sine = math.sin(float(yaw_enu_rad))
    east_mps = cosine * float(forward_mps) - sine * left_mps
    north_mps = sine * float(forward_mps) + cosine * left_mps
    return east_mps, north_mps


def velocity_enu_to_flu(east_mps, north_mps, yaw_enu_rad):
    cosine = math.cos(float(yaw_enu_rad))
    sine = math.sin(float(yaw_enu_rad))
    forward_mps = cosine * float(east_mps) + sine * float(north_mps)
    left_mps = -sine * float(east_mps) + cosine * float(north_mps)
    return forward_mps, left_mps


@dataclass
class FusionUpdate:
    accepted: bool
    innovation_m: float = 0.0
    reason: str = "ok"


class GpsFlowComplementaryFilter:
    def __init__(
            self, gps_position_gain=0.35, flow_velocity_gain=0.65,
            gps_jump_gate_m=20.0, maximum_predict_step_s=0.5):
        self.gps_position_gain = float(gps_position_gain)
        self.flow_velocity_gain = float(flow_velocity_gain)
        self.gps_jump_gate_m = float(gps_jump_gate_m)
        self.maximum_predict_step_s = float(maximum_predict_step_s)
        self.initialized = False
        self.position = [0.0, 0.0, 0.0]
        self.velocity_enu = [0.0, 0.0]
        self.last_stamp_s = None
        self.last_gnss_variance_m2 = 4.0
        self.last_flow_weight = 0.0

    def predict_to(self, stamp_s):
        stamp_s = float(stamp_s)
        if not self.initialized:
            self.last_stamp_s = stamp_s
            return 0.0
        if self.last_stamp_s is None:
            self.last_stamp_s = stamp_s
            return 0.0
        dt = stamp_s - self.last_stamp_s
        if not math.isfinite(dt) or dt <= 0.0:
            return 0.0
        if dt > self.maximum_predict_step_s:
            self.last_stamp_s = stamp_s
            return 0.0
        self.position[0] += self.velocity_enu[0] * dt
        self.position[1] += self.velocity_enu[1] * dt
        self.last_stamp_s = stamp_s
        return dt

    def update_gnss(self, position_enu, variance_m2, stamp_s, weight=1.0):
        position = [float(value) for value in position_enu]
        if not all(math.isfinite(value) for value in position):
            return FusionUpdate(False, reason="nonfinite_gnss")
        variance_m2 = max(0.01, float(variance_m2))
        weight = max(0.0, min(1.0, float(weight)))
        if not self.initialized:
            self.position = position
            self.initialized = True
            self.last_stamp_s = float(stamp_s)
            self.last_gnss_variance_m2 = variance_m2
            return FusionUpdate(True)
        self.predict_to(stamp_s)
        innovation = [position[axis] - self.position[axis] for axis in range(3)]
        innovation_xy = math.hypot(innovation[0], innovation[1])
        if innovation_xy > self.gps_jump_gate_m:
            return FusionUpdate(False, innovation_xy, "gps_jump_gate")
        covariance_scale = 1.0 / (1.0 + math.sqrt(variance_m2))
        gain = max(0.0, min(1.0, self.gps_position_gain * weight * covariance_scale * 2.0))
        for axis in range(3):
            self.position[axis] += gain * innovation[axis]
        self.last_gnss_variance_m2 = variance_m2
        return FusionUpdate(True, innovation_xy)

    def update_flow(self, forward_mps, right_mps, yaw_enu_rad, stamp_s, weight=1.0):
        values = (forward_mps, right_mps, yaw_enu_rad, stamp_s, weight)
        if not all(math.isfinite(float(value)) for value in values):
            return FusionUpdate(False, reason="nonfinite_flow")
        self.predict_to(stamp_s)
        if not self.initialized:
            return FusionUpdate(False, reason="waiting_for_gnss_origin")
        measured = velocity_frd_to_enu(forward_mps, right_mps, yaw_enu_rad)
        weight = max(0.0, min(1.0, float(weight)))
        gain = max(0.0, min(1.0, self.flow_velocity_gain * weight))
        self.velocity_enu[0] += gain * (measured[0] - self.velocity_enu[0])
        self.velocity_enu[1] += gain * (measured[1] - self.velocity_enu[1])
        self.last_flow_weight = weight
        return FusionUpdate(True)
