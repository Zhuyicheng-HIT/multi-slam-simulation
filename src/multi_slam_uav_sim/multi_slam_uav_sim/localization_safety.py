"""Pure state machine for conservative mission behavior during pose loss."""

from __future__ import annotations

import math
from dataclasses import dataclass


TRACKING = "TRACKING"
LOSS_PENDING = "LOSS_PENDING"
HOLDING = "HOLDING"
RELOCALIZING_HOLD = "RELOCALIZING_HOLD"
RECOVERY_PENDING = "RECOVERY_PENDING"


def diagnostic_level_value(value) -> int:
    """Normalize ROS diagnostic levels across integer and byte bindings."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        raw = bytes(value)
        if len(raw) != 1:
            raise ValueError("diagnostic level byte value must have length one")
        return raw[0]
    return int(value)


def mission_hold_required(
    safety_hold: bool,
    localization_lost: bool,
    relocalization_request_active: bool,
) -> bool:
    """Hold for either pose-loss recovery or an explicit relocalization."""
    return bool(
        safety_hold or localization_lost or relocalization_request_active
    )


def scheduler_localization_loss(
    health_state: str,
    estimator_support: float,
    capability_names,
    capability_observable,
    minimum_support: float,
    fresh: bool = True,
    estimator_fresh: bool = True,
    estimator_finite: bool = True,
    external_nav_gate_fresh: bool = True,
    external_nav_gate_healthy: bool = True,
    external_nav_gate_reason: str = "publishing",
):
    """Return only clear loss-of-pose evidence, not a generic degraded state."""
    if not fresh:
        return True, "scheduler_stale"
    if not estimator_fresh:
        return True, "unified_odom_stale"
    if not estimator_finite:
        return True, "unified_odom_nonfinite"
    if not external_nav_gate_fresh:
        return True, "external_nav_gate_stale"
    if not external_nav_gate_healthy:
        reason = str(external_nav_gate_reason).strip() or "rejected"
        return True, f"external_nav_gate_{reason}"
    capabilities = {
        name: bool(observable)
        for name, observable in zip(capability_names, capability_observable)
    }
    required = ("propagation", "horizontal_motion", "yaw_tracking")
    absent = [name for name in required if name not in capabilities]
    if absent:
        return True, "missing_capability_status_" + "+".join(absent)
    missing = [
        name for name in required if not capabilities[name]
    ]
    if missing:
        return True, "unobservable_" + "+".join(missing)
    if (
        not math.isfinite(float(estimator_support))
        or float(estimator_support) < float(minimum_support)
    ):
        return True, "estimator_support_low"
    # FAILSAFE can also describe a failed recovery service. When the estimator
    # output is live and all critical capabilities remain observable, it is not
    # evidence of pose loss by itself.
    return False, "observable"


@dataclass(frozen=True)
class SafetyDecision:
    state: str
    hold: bool
    request_relocalization: bool = False
    clear_relocalization_request: bool = False


class LocalizationSafetyStateMachine:
    """Confirm pose loss, hold for at least one second, then recover safely."""

    def __init__(
        self,
        loss_dwell_s: float = 0.30,
        minimum_hold_s: float = 1.0,
        recovery_dwell_s: float = 0.75,
    ):
        self.loss_dwell_s = max(0.0, float(loss_dwell_s))
        self.minimum_hold_s = max(0.0, float(minimum_hold_s))
        self.recovery_dwell_s = max(0.0, float(recovery_dwell_s))
        self.state = TRACKING
        self.state_since = 0.0

    def _transition(self, state: str, now_s: float) -> None:
        self.state = state
        self.state_since = float(now_s)

    def update(self, obvious_loss: bool, now_s: float) -> SafetyDecision:
        now_s = float(now_s)
        request = False
        clear = False

        if self.state == TRACKING:
            if obvious_loss:
                self._transition(LOSS_PENDING, now_s)

        elif self.state == LOSS_PENDING:
            if not obvious_loss:
                self._transition(TRACKING, now_s)
            elif now_s - self.state_since >= self.loss_dwell_s:
                self._transition(HOLDING, now_s)

        elif self.state == HOLDING:
            if now_s - self.state_since >= self.minimum_hold_s:
                if obvious_loss:
                    self._transition(RELOCALIZING_HOLD, now_s)
                    request = True
                else:
                    self._transition(RECOVERY_PENDING, now_s)

        elif self.state == RELOCALIZING_HOLD:
            if not obvious_loss:
                self._transition(RECOVERY_PENDING, now_s)

        elif self.state == RECOVERY_PENDING:
            if obvious_loss:
                self._transition(RELOCALIZING_HOLD, now_s)
            elif now_s - self.state_since >= self.recovery_dwell_s:
                self._transition(TRACKING, now_s)
                clear = True

        return SafetyDecision(
            state=self.state,
            hold=self.state in (HOLDING, RELOCALIZING_HOLD, RECOVERY_PENDING),
            request_relocalization=request,
            clear_relocalization_request=clear,
        )
