"""Source-owned relocalization request leases.

The core is deliberately independent of ROS so ownership, expiry, restart, and
ordering semantics can be tested deterministically.
"""

from dataclasses import dataclass, field
import math


@dataclass
class SourceLease:
    instance_id: str
    sequence: int
    episode_id: int
    source_stamp_s: float
    active: bool
    deadline_s: float
    reason: str


@dataclass
class ArbiterDecision:
    accepted: bool
    reason: str
    output_active: bool
    output_changed: bool
    active_sources: tuple = ()
    expired_sources: tuple = ()


@dataclass
class RelocalizationRequestArbiterCore:
    allowed_sources: tuple = (
        "reliability_scheduler",
        "localization_safety",
        "manual_control",
    )
    minimum_lease_s: float = 0.20
    maximum_lease_s: float = 5.0
    maximum_stamp_age_s: float = 2.0
    maximum_future_skew_s: float = 0.50
    leases: dict = field(default_factory=dict)
    retired_instances: dict = field(default_factory=dict)
    output_active: bool = False
    accepted_intents: int = 0
    rejected_intents: int = 0
    duplicate_intents: int = 0
    expired_leases: int = 0
    source_restarts: int = 0
    output_transitions: int = 0

    def __post_init__(self):
        self.allowed_sources = tuple(
            str(value) for value in self.allowed_sources
        )
        if not self.allowed_sources or any(
            not value for value in self.allowed_sources
        ):
            raise ValueError(
                "allowed_sources must contain nonempty source ids"
            )
        if len(set(self.allowed_sources)) != len(self.allowed_sources):
            raise ValueError("allowed_sources must be unique")
        values = (
            self.minimum_lease_s,
            self.maximum_lease_s,
            self.maximum_stamp_age_s,
            self.maximum_future_skew_s,
        )
        if not all(
            math.isfinite(float(value)) and float(value) >= 0.0
            for value in values
        ):
            raise ValueError(
                "arbiter timing limits must be finite and nonnegative"
            )
        if (
            self.minimum_lease_s <= 0.0
            or self.maximum_lease_s < self.minimum_lease_s
        ):
            raise ValueError("invalid lease range")
        self.retired_instances = {
            source: set() for source in self.allowed_sources
        }

    def _active_sources(self):
        return tuple(sorted(
            source for source, lease in self.leases.items() if lease.active
        ))

    def _finish(self, previous, accepted, reason, expired=()):
        self.output_active = bool(self._active_sources())
        changed = self.output_active != previous
        if changed:
            self.output_transitions += 1
        return ArbiterDecision(
            accepted=accepted,
            reason=reason,
            output_active=self.output_active,
            output_changed=changed,
            active_sources=self._active_sources(),
            expired_sources=tuple(expired),
        )

    def _expire(self, steady_now_s):
        expired = []
        for source, lease in self.leases.items():
            if lease.active and steady_now_s >= lease.deadline_s:
                lease.active = False
                expired.append(source)
        self.expired_leases += len(expired)
        return expired

    def update(
        self,
        *,
        source_id,
        instance_id,
        sequence,
        episode_id,
        active,
        lease_duration_s,
        source_stamp_s,
        steady_now_s,
        ros_now_s,
        reason="",
    ):
        previous = self.output_active
        expired = self._expire(float(steady_now_s))
        source_id = str(source_id)
        instance_id = str(instance_id)
        numeric = (
            float(lease_duration_s),
            float(source_stamp_s),
            float(steady_now_s),
            float(ros_now_s),
        )
        if source_id not in self.allowed_sources:
            return self._reject(previous, "unknown_source", expired)
        if not instance_id:
            return self._reject(previous, "empty_instance", expired)
        if not all(math.isfinite(value) for value in numeric):
            return self._reject(previous, "nonfinite_time", expired)
        lease_duration_s, source_stamp_s, steady_now_s, ros_now_s = numeric
        if source_stamp_s < 0.0 or steady_now_s < 0.0 or ros_now_s < 0.0:
            return self._reject(previous, "negative_time", expired)
        if source_stamp_s > ros_now_s + self.maximum_future_skew_s:
            return self._reject(previous, "future_stamp", expired)
        if ros_now_s - source_stamp_s > self.maximum_stamp_age_s:
            return self._reject(previous, "stale_stamp", expired)
        sequence = int(sequence)
        episode_id = int(episode_id)
        if sequence <= 0 or episode_id < 0:
            return self._reject(previous, "invalid_sequence", expired)
        if bool(active) and not (
            self.minimum_lease_s <= lease_duration_s <= self.maximum_lease_s
        ):
            return self._reject(previous, "invalid_lease", expired)

        current = self.leases.get(source_id)
        retired = self.retired_instances[source_id]
        if current is not None and instance_id != current.instance_id:
            if instance_id in retired:
                return self._reject(previous, "retired_instance", expired)
            retired.add(current.instance_id)
            while len(retired) > 8:
                retired.pop()
            self.source_restarts += 1
            current = None
        if current is not None:
            if sequence <= current.sequence:
                self.duplicate_intents += 1
                return self._reject(
                    previous, "duplicate_or_reordered", expired
                )
            if source_stamp_s < current.source_stamp_s:
                return self._reject(previous, "timestamp_regression", expired)

        self.leases[source_id] = SourceLease(
            instance_id=instance_id,
            sequence=sequence,
            episode_id=episode_id,
            source_stamp_s=source_stamp_s,
            active=bool(active),
            deadline_s=(
                steady_now_s + lease_duration_s
                if bool(active)
                else steady_now_s
            ),
            reason=str(reason),
        )
        self.accepted_intents += 1
        return self._finish(previous, True, "accepted", expired)

    def _reject(self, previous, reason, expired):
        self.rejected_intents += 1
        return self._finish(previous, False, reason, expired)

    def tick(self, steady_now_s):
        steady_now_s = float(steady_now_s)
        if not math.isfinite(steady_now_s) or steady_now_s < 0.0:
            raise ValueError("steady_now_s must be finite and nonnegative")
        previous = self.output_active
        expired = self._expire(steady_now_s)
        return self._finish(previous, True, "tick", expired)

    def reset(self, reason="clock_reset"):
        previous = self.output_active
        self.leases.clear()
        self.retired_instances = {
            source: set() for source in self.allowed_sources
        }
        return self._finish(previous, True, str(reason))
