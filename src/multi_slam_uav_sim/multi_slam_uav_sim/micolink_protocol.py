from dataclasses import dataclass
import math
import struct


MICOLINK_HEADER = 0xEF
MICOLINK_MAX_PAYLOAD_LENGTH = 64
MICOLINK_RANGE_SENSOR_MESSAGE_ID = 0x51
MICOLINK_MTF01_DEVICE_ID = 0x0F
MICOLINK_RANGE_PAYLOAD_LENGTH = 20
MICOLINK_FRAME_OVERHEAD = 7
MICOLINK_RANGE_PAYLOAD = struct.Struct("<IIBBBBhhBBH")


def clamp_int16(value):
    return max(-32768, min(32767, int(round(float(value)))))


def clamp_uint8(value):
    return max(0, min(255, int(round(float(value)))))


def micolink_checksum(data):
    return sum(bytes(data)) & 0xFF


@dataclass(frozen=True)
class MicoLinkFrame:
    device_id: int
    system_id: int
    message_id: int
    sequence: int
    payload: bytes


@dataclass(frozen=True)
class Mtf01RangeFlow:
    time_ms: int
    distance_mm: int
    strength: int
    precision: int
    tof_status: int
    reserved1: int
    flow_velocity_x: int
    flow_velocity_y: int
    flow_quality: int
    flow_status: int
    reserved2: int


def encode_frame(
    payload,
    *,
    device_id=MICOLINK_MTF01_DEVICE_ID,
    system_id=0,
    message_id=MICOLINK_RANGE_SENSOR_MESSAGE_ID,
    sequence=0,
):
    payload = bytes(payload)
    if len(payload) > MICOLINK_MAX_PAYLOAD_LENGTH:
        raise ValueError("MicoLink payload is too long")
    prefix = bytes(
        (
            MICOLINK_HEADER,
            clamp_uint8(device_id),
            clamp_uint8(system_id),
            clamp_uint8(message_id),
            clamp_uint8(sequence),
            len(payload),
        )
    ) + payload
    return prefix + bytes((micolink_checksum(prefix),))


def decode_frame(data):
    data = bytes(data)
    if len(data) < MICOLINK_FRAME_OVERHEAD:
        raise ValueError("MicoLink frame is too short")
    if data[0] != MICOLINK_HEADER:
        raise ValueError("invalid MicoLink frame header")
    payload_length = data[5]
    expected_length = payload_length + MICOLINK_FRAME_OVERHEAD
    if len(data) != expected_length:
        raise ValueError("MicoLink frame length does not match payload length")
    if micolink_checksum(data[:-1]) != data[-1]:
        raise ValueError("invalid MicoLink checksum")
    return MicoLinkFrame(
        device_id=data[1],
        system_id=data[2],
        message_id=data[3],
        sequence=data[4],
        payload=data[6:-1],
    )


def encode_range_flow_payload(observation):
    return MICOLINK_RANGE_PAYLOAD.pack(
        int(observation.time_ms) & 0xFFFFFFFF,
        max(0, min(0xFFFFFFFF, int(observation.distance_mm))),
        clamp_uint8(observation.strength),
        clamp_uint8(observation.precision),
        clamp_uint8(observation.tof_status),
        clamp_uint8(observation.reserved1),
        clamp_int16(observation.flow_velocity_x),
        clamp_int16(observation.flow_velocity_y),
        clamp_uint8(observation.flow_quality),
        clamp_uint8(observation.flow_status),
        max(0, min(0xFFFF, int(observation.reserved2))),
    )


def decode_range_flow_payload(payload):
    payload = bytes(payload)
    if len(payload) != MICOLINK_RANGE_PAYLOAD_LENGTH:
        raise ValueError("MTF-01 MicoLink range payload must be 20 bytes")
    values = MICOLINK_RANGE_PAYLOAD.unpack(payload)
    return Mtf01RangeFlow(*values)


def encode_range_flow_frame(observation, *, sequence=0, device_id=0x0F, system_id=0):
    return encode_frame(
        encode_range_flow_payload(observation),
        device_id=device_id,
        system_id=system_id,
        message_id=MICOLINK_RANGE_SENSOR_MESSAGE_ID,
        sequence=sequence,
    )


def integrated_radians_to_flow_velocity(integrated_x, integrated_y, integration_s):
    """Convert integrated image angles to MicoLink cm/s-at-1m rate units."""
    if not math.isfinite(float(integration_s)) or integration_s <= 0.0:
        raise ValueError("integration time must be positive")
    return (
        clamp_int16(float(integrated_x) / float(integration_s) * 100.0),
        clamp_int16(float(integrated_y) / float(integration_s) * 100.0),
    )


def flow_velocity_to_integrated_radians(flow_velocity_x, flow_velocity_y, integration_s):
    """Convert MicoLink cm/s-at-1m rates to small-angle integrated radians."""
    if not math.isfinite(float(integration_s)) or integration_s <= 0.0:
        raise ValueError("integration time must be positive")
    return (
        float(flow_velocity_x) * 0.01 * float(integration_s),
        float(flow_velocity_y) * 0.01 * float(integration_s),
    )


def sensor_interval_seconds(previous_time_ms, current_time_ms, nominal_rate_hz=100.0):
    if previous_time_ms is None:
        return 1.0 / max(float(nominal_rate_hz), 1.0)
    delta_ms = (int(current_time_ms) - int(previous_time_ms)) & 0xFFFFFFFF
    if delta_ms == 0:
        return None
    return delta_ms * 1.0e-3


class MicoLinkSensorClock:
    def __init__(self, initial_time_ms=1_000):
        self.time_ms = max(1, int(initial_time_ms)) & 0xFFFFFFFF

    def advance(self, integration_time_us):
        increment_ms = max(1, int(round(float(integration_time_us) * 1.0e-3)))
        self.time_ms = (self.time_ms + increment_ms) & 0xFFFFFFFF
        return self.time_ms


class MicoLinkParser:
    def __init__(self):
        self.buffer = bytearray()
        self.frames_decoded = 0
        self.checksum_errors = 0
        self.length_errors = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(bytes(data))
        frames = []
        while True:
            header_index = self.buffer.find(bytes((MICOLINK_HEADER,)))
            if header_index < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if header_index > 0:
                self.discarded_bytes += header_index
                del self.buffer[:header_index]
            if len(self.buffer) < 6:
                break
            payload_length = self.buffer[5]
            if payload_length > MICOLINK_MAX_PAYLOAD_LENGTH:
                self.length_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            frame_length = payload_length + MICOLINK_FRAME_OVERHEAD
            if len(self.buffer) < frame_length:
                break
            candidate = bytes(self.buffer[:frame_length])
            if micolink_checksum(candidate[:-1]) != candidate[-1]:
                self.checksum_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            frames.append(decode_frame(candidate))
            self.frames_decoded += 1
            del self.buffer[:frame_length]
        return frames
