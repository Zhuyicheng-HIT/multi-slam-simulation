"""Mission-checkpoint serialization for relocalization experiments."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class MissionCheckpoint:
    index: int
    label: str
    distance_m: float
    position: tuple[float, float, float]


def parse_checkpoint_indices(value: str) -> tuple[int, ...]:
    parts = [part.strip() for part in str(value).split(",") if part.strip()]
    if not parts:
        raise ValueError("at least one relocalization checkpoint is required")
    try:
        indices = tuple(int(part) for part in parts)
    except ValueError as error:
        raise ValueError("checkpoint indices must be integers") from error
    if any(index <= 0 for index in indices):
        raise ValueError("checkpoint indices must be positive")
    if tuple(sorted(set(indices))) != indices:
        raise ValueError("checkpoint indices must be unique and increasing")
    return indices


def encode_checkpoint(checkpoint: MissionCheckpoint) -> str:
    if checkpoint.index <= 0:
        raise ValueError("checkpoint index must be positive")
    values = (checkpoint.distance_m, *checkpoint.position)
    if any(not math.isfinite(float(value)) for value in values):
        raise ValueError("checkpoint geometry must be finite")
    if not checkpoint.label:
        raise ValueError("checkpoint label must not be empty")
    return json.dumps(
        {
            "index": int(checkpoint.index),
            "label": str(checkpoint.label),
            "distance_m": float(checkpoint.distance_m),
            "position": [float(value) for value in checkpoint.position],
        },
        sort_keys=True,
        separators=(",", ":"),
    )


def decode_checkpoint(payload: str) -> MissionCheckpoint:
    try:
        data = json.loads(payload)
        position = tuple(float(value) for value in data["position"])
        checkpoint = MissionCheckpoint(
            index=int(data["index"]),
            label=str(data["label"]),
            distance_m=float(data["distance_m"]),
            position=position,
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError("invalid mission checkpoint payload") from error
    if len(checkpoint.position) != 3:
        raise ValueError("checkpoint position must have three components")
    # Reuse the encoder as the canonical semantic validation pass.
    encode_checkpoint(checkpoint)
    return checkpoint
