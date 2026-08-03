import math
import struct


OPTICAL_FLOW_MESSAGE_ID = 100
DISTANCE_SENSOR_MESSAGE_ID = 132
MAVLINK_DPIX_PER_PIXEL = 10.0
MTF01P_WIDTH_PX = 100
MTF01P_FOV_RAD = math.radians(42.0)


def focal_length_px(width_px=MTF01P_WIDTH_PX, fov_rad=MTF01P_FOV_RAD):
    width_px = float(width_px)
    fov_rad = float(fov_rad)
    if width_px <= 0.0 or not 0.0 < fov_rad < math.pi:
        raise ValueError("invalid MTF01P image geometry")
    return width_px / (2.0 * math.tan(0.5 * fov_rad))


def clamp_int16(value):
    return max(-32768, min(32767, int(round(float(value)))))


def integrated_radians_to_pixels(integrated_x, integrated_y, fx_px, fy_px):
    """Encode sensor-FRD angles as MAVLink OPTICAL_FLOW decipixels."""
    if fx_px <= 0.0 or fy_px <= 0.0:
        raise ValueError("focal lengths must be positive")
    return (
        clamp_int16(
            math.tan(float(integrated_x)) * float(fx_px) * MAVLINK_DPIX_PER_PIXEL
        ),
        clamp_int16(
            math.tan(float(integrated_y)) * float(fy_px) * MAVLINK_DPIX_PER_PIXEL
        ),
    )


def pixels_to_integrated_radians(flow_x, flow_y, fx_px, fy_px):
    """Decode MAVLink1 OPTICAL_FLOW decipixels to integrated angles."""
    if fx_px <= 0.0 or fy_px <= 0.0:
        raise ValueError("focal lengths must be positive")
    return (
        math.atan2(float(flow_x) / MAVLINK_DPIX_PER_PIXEL, float(fx_px)),
        math.atan2(float(flow_y) / MAVLINK_DPIX_PER_PIXEL, float(fy_px)),
    )


def sensor_frd_to_ros_flu(flow_x, flow_y):
    return float(flow_x), -float(flow_y)


def mavros_payload_bytes(payload64, payload_length):
    packed = b"".join(
        struct.pack("<Q", int(word) & 0xFFFFFFFFFFFFFFFF) for word in payload64
    )
    return packed[:int(payload_length)]


def decode_optical_flow_payload(payload):
    if len(payload) < 26:
        raise ValueError("OPTICAL_FLOW payload is shorter than MAVLink1 base fields")
    fields = struct.unpack_from("<QfffhhBB", payload)
    return {
        "time_usec": fields[0],
        "flow_comp_m_x": fields[1],
        "flow_comp_m_y": fields[2],
        "ground_distance": fields[3],
        "flow_x": fields[4],
        "flow_y": fields[5],
        "sensor_id": fields[6],
        "quality": fields[7],
    }


def decode_distance_sensor_payload(payload):
    if len(payload) < 14:
        raise ValueError("DISTANCE_SENSOR payload is shorter than MAVLink1 base fields")
    fields = struct.unpack_from("<IHHHBBBB", payload)
    return {
        "time_boot_ms": fields[0],
        "min_distance_cm": fields[1],
        "max_distance_cm": fields[2],
        "current_distance_cm": fields[3],
        "sensor_type": fields[4],
        "sensor_id": fields[5],
        "orientation": fields[6],
        "covariance": fields[7],
    }


def compensated_planar_velocity(
    integrated_x, integrated_y, gyro_x, gyro_y, integration_s, distance_m
):
    if integration_s <= 0.0 or distance_m <= 0.0:
        return float("nan"), float("nan")
    translational_x = (float(integrated_x) - float(gyro_x)) / float(integration_s)
    translational_y = (float(integrated_y) - float(gyro_y)) / float(integration_s)
    return (
        translational_y * float(distance_m),
        -translational_x * float(distance_m),
    )


class SensorClock:
    def __init__(self, initial_time_usec=1_000_000):
        self.time_usec = max(1, int(initial_time_usec))

    def advance(self, integration_time_us):
        increment = max(1, int(integration_time_us))
        self.time_usec += increment
        return self.time_usec
