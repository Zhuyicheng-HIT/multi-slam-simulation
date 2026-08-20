"""Pure contracts for bounded active-relocalization observation motions."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass


MOTION_PROFILES = ("hold", "yaw_scan", "circle", "figure8")


@dataclass(frozen=True)
class MotionObservation:
    forward_m: float
    left_m: float
    yaw_offset_rad: float


@dataclass(frozen=True)
class MotionCommand:
    sequence_id: int
    profile: str
    step_index: int
    step_count: int


@dataclass(frozen=True)
class MotionStatus:
    sequence_id: int
    profile: str
    step_index: int
    step_count: int
    state: str
    reason: str
    distance_m: float = 0.0
    duration_s: float = 0.0


def normalize_motion_profile(value):
    profile = str(value).strip().lower()
    if profile not in MOTION_PROFILES:
        raise ValueError(
            "relocalization motion profile must be one of "
            + ", ".join(MOTION_PROFILES)
        )
    return profile


def motion_observations(profile, radius_m=0.6, yaw_step_deg=45.0):
    """Return observation stops relative to the frozen FCU-local anchor."""
    profile = normalize_motion_profile(profile)
    radius = float(radius_m)
    yaw_step = math.radians(float(yaw_step_deg))
    if not math.isfinite(radius) or not 0.1 <= radius <= 2.0:
        raise ValueError("relocalization motion radius must be in [0.1, 2.0] m")
    if not math.isfinite(yaw_step) or not math.radians(10.0) <= yaw_step <= math.pi:
        raise ValueError("relocalization yaw step must be in [10, 180] deg")

    if profile == "hold":
        return (MotionObservation(0.0, 0.0, 0.0),)
    if profile == "yaw_scan":
        count = max(2, int(math.ceil(2.0 * math.pi / yaw_step)))
        return tuple(
            MotionObservation(
                0.0,
                0.0,
                math.atan2(math.sin(yaw_step * index), math.cos(yaw_step * index)),
            )
            for index in range(1, count + 1)
        )
    if profile == "circle":
        # The anchor is tangent to a circle whose center is radius metres left.
        return tuple(
            MotionObservation(
                radius * math.sin(theta),
                radius * (1.0 - math.cos(theta)),
                0.0,
            )
            for theta in (0.5 * math.pi, math.pi, 1.5 * math.pi, 2.0 * math.pi)
        )

    # A fixed-yaw Gerono figure-eight starts, crosses, and ends at the anchor.
    return tuple(
        MotionObservation(
            radius * math.sin(theta),
            0.5 * radius * math.sin(2.0 * theta),
            0.0,
        )
        for theta in (0.5 * math.pi, math.pi, 1.5 * math.pi, 2.0 * math.pi)
    )


def body_offset_to_local(anchor_x, anchor_y, anchor_yaw, forward_m, left_m):
    values = tuple(float(value) for value in (
        anchor_x, anchor_y, anchor_yaw, forward_m, left_m
    ))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("motion anchor and offset must be finite")
    x, y, yaw, forward, left = values
    return (
        x + math.cos(yaw) * forward - math.sin(yaw) * left,
        y + math.sin(yaw) * forward + math.cos(yaw) * left,
    )


def _integer(payload, key):
    value = payload.get(key)
    if isinstance(value, bool):
        raise ValueError(f"{key} must be an integer")
    converted = int(value)
    if converted != value:
        raise ValueError(f"{key} must be an integer")
    return converted


def encode_motion_command(command):
    return json.dumps({
        "profile": normalize_motion_profile(command.profile),
        "sequence_id": int(command.sequence_id),
        "step_count": int(command.step_count),
        "step_index": int(command.step_index),
    }, sort_keys=True)


def decode_motion_command(payload):
    try:
        data = json.loads(str(payload))
    except json.JSONDecodeError as error:
        raise ValueError("invalid relocalization motion command JSON") from error
    if not isinstance(data, dict):
        raise ValueError("relocalization motion command must be an object")
    sequence_id = _integer(data, "sequence_id")
    step_index = _integer(data, "step_index")
    step_count = _integer(data, "step_count")
    profile = normalize_motion_profile(data.get("profile", ""))
    if sequence_id <= 0:
        raise ValueError("motion sequence_id must be positive")
    if profile == "hold" and step_count != 1:
        raise ValueError("hold motion step_count must be one")
    if profile in ("circle", "figure8") and step_count != 4:
        raise ValueError("translation motion step_count must be four")
    if profile == "yaw_scan" and not 2 <= step_count <= 36:
        raise ValueError("yaw scan step_count must be in [2, 36]")
    if not 0 <= step_index < step_count:
        raise ValueError("motion step_index is outside the profile")
    return MotionCommand(sequence_id, profile, step_index, step_count)


def encode_motion_status(status):
    state = str(status.state).strip().lower()
    if state not in ("started", "settled", "failed"):
        raise ValueError("motion status state is invalid")
    return json.dumps({
        "distance_m": float(status.distance_m),
        "duration_s": float(status.duration_s),
        "profile": normalize_motion_profile(status.profile),
        "reason": str(status.reason),
        "sequence_id": int(status.sequence_id),
        "state": state,
        "step_count": int(status.step_count),
        "step_index": int(status.step_index),
    }, sort_keys=True)


def decode_motion_status(payload):
    try:
        data = json.loads(str(payload))
    except json.JSONDecodeError as error:
        raise ValueError("invalid relocalization motion status JSON") from error
    if not isinstance(data, dict):
        raise ValueError("relocalization motion status must be an object")
    command = decode_motion_command(json.dumps({
        "profile": data.get("profile"),
        "sequence_id": data.get("sequence_id"),
        "step_count": data.get("step_count"),
        "step_index": data.get("step_index"),
    }))
    state = str(data.get("state", "")).strip().lower()
    if state not in ("started", "settled", "failed"):
        raise ValueError("motion status state is invalid")
    distance = float(data.get("distance_m", 0.0))
    duration = float(data.get("duration_s", 0.0))
    if (
        not math.isfinite(distance) or distance < 0.0
        or not math.isfinite(duration) or duration < 0.0
    ):
        raise ValueError("motion status metrics must be finite and non-negative")
    return MotionStatus(
        command.sequence_id,
        command.profile,
        command.step_index,
        command.step_count,
        state,
        str(data.get("reason", "")),
        distance,
        duration,
    )
