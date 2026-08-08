"""Validated, deterministic fault-profile loading for Robustness V3.

Profiles intentionally describe faults, not expected outcomes.  In particular,
the loader does not expose estimator thresholds and cannot weaken the integrity
gate.  A profile can include other profiles to form deterministic double-fault
experiments without duplicating values.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Tuple

import yaml


SUPPORTED_CHANNELS = {
    "native_lidar", "imu", "gnss", "optical_flow", "vision"
}
SUPPORTED_FAULTS = {
    "outage", "time_offset", "correspondence_dropout",
    "extrinsic_error", "bias", "saturation", "jump",
    "covariance_scale", "low_quality", "scale", "track_dropout",
    "reprojection_bias",
}


@dataclass(frozen=True)
class FaultSpec:
    channel: str
    fault_type: str
    start_s: float
    duration_s: float
    magnitude: float = 0.0
    secondary_magnitude: float = 0.0
    score_floor: float = 0.0
    seed_offset: int = 0
    temporal_scope: str = "factor_only"

    @property
    def modality(self) -> str:
        return "lidar" if self.channel == "native_lidar" else self.channel


@dataclass(frozen=True)
class FaultProfile:
    name: str
    description: str
    seed: int
    faults: Tuple[FaultSpec, ...]
    calibration: Mapping[str, Any]


def _finite_number(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if result != result or abs(result) == float("inf"):
        raise ValueError(f"{name} must be finite")
    return result


def _profile_rows(
    name: str,
    profiles: Mapping[str, Mapping[str, Any]],
    stack: Tuple[str, ...] = (),
) -> Iterable[Mapping[str, Any]]:
    if name not in profiles:
        raise KeyError(f"unknown fault profile: {name}")
    if name in stack:
        raise ValueError("fault profile include cycle: " + " -> ".join(stack + (name,)))
    row = profiles[name] or {}
    for parent in row.get("include", []) or []:
        yield from _profile_rows(str(parent), profiles, stack + (name,))
    yield row


def load_fault_profile(path: str, name: str) -> FaultProfile:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if int(document.get("schema_version", 0)) != 1:
        raise ValueError("fault profile schema_version must be 1")
    profiles = document.get("profiles", {})
    if not isinstance(profiles, dict):
        raise ValueError("profiles must be a mapping")
    defaults = document.get("defaults", {}) or {}
    rows = list(_profile_rows(name, profiles))
    faults = []
    calibration: Dict[str, Any] = {}
    description = ""
    seed = int(defaults.get("seed", 73))
    for row in rows:
        description = str(row.get("description", description))
        seed = int(row.get("seed", seed))
        calibration.update(row.get("calibration", {}) or {})
        for index, raw in enumerate(row.get("faults", []) or []):
            channel = str(raw.get("channel", ""))
            fault_type = str(raw.get("type", ""))
            if channel not in SUPPORTED_CHANNELS:
                raise ValueError(f"unsupported channel {channel!r} in {name}")
            if fault_type not in SUPPORTED_FAULTS:
                raise ValueError(f"unsupported fault {fault_type!r} in {name}")
            start_s = _finite_number(
                raw.get("start_s", defaults.get("start_s", 30.0)), "start_s"
            )
            duration_s = _finite_number(
                raw.get("duration_s", defaults.get("duration_s", 20.0)),
                "duration_s",
            )
            score_floor = _finite_number(raw.get("score_floor", 0.0), "score_floor")
            if start_s < 0.0 or duration_s < 0.0:
                raise ValueError("fault start/duration must be non-negative")
            if not 0.0 <= score_floor <= 1.0:
                raise ValueError("score_floor must be in [0, 1]")
            faults.append(FaultSpec(
                channel=channel,
                fault_type=fault_type,
                start_s=start_s,
                duration_s=duration_s,
                magnitude=_finite_number(raw.get("magnitude", 0.0), "magnitude"),
                secondary_magnitude=_finite_number(
                    raw.get("secondary_magnitude", 0.0), "secondary_magnitude"
                ),
                score_floor=score_floor,
                seed_offset=int(raw.get("seed_offset", index)),
                temporal_scope=str(raw.get("temporal_scope", "factor_only")),
            ))
            if faults[-1].fault_type == "time_offset" and faults[-1].channel == "native_lidar":
                if faults[-1].temporal_scope not in {
                    "factor_only", "coherent_frontend_contract"
                }:
                    raise ValueError(
                        "native_lidar time_offset temporal_scope must be "
                        "factor_only or coherent_frontend_contract"
                    )
    return FaultProfile(
        name=name,
        description=description,
        seed=seed,
        faults=tuple(faults),
        calibration=calibration,
    )


def profile_backend_overrides(profile: FaultProfile) -> Dict[str, Any]:
    """Return only documented calibration parameters for the replay backend."""
    allowed = {
        "visual_time_offset_s",
        "visual_rotation_body_camera",
        "visual_translation_body_camera_m",
    }
    unknown = set(profile.calibration) - allowed
    if unknown:
        raise ValueError("unsupported calibration override(s): " + ", ".join(sorted(unknown)))
    return {key: profile.calibration[key] for key in allowed if key in profile.calibration}
