"""Pure state machine for conservative mission behavior during pose loss."""

from __future__ import annotations

from dataclasses import dataclass


TRACKING = "TRACKING"
LOSS_PENDING = "LOSS_PENDING"
HOLDING = "HOLDING"
RELOCALIZING_HOLD = "RELOCALIZING_HOLD"
RECOVERY_PENDING = "RECOVERY_PENDING"


def scheduler_localization_loss(
    health_state: str,
    estimator_support: float,
    capability_names,
    capability_observable,
    minimum_support: float,
    fresh: bool = True,
):
    """Return only clear loss-of-pose evidence, not a generic degraded state."""
    if not fresh:
        return True, "scheduler_stale"
    if str(health_state) == "FAILSAFE":
        return True, "scheduler_failsafe"
    if float(estimator_support) < float(minimum_support):
        return True, "estimator_support_low"
    capabilities = {
        name: bool(observable)
        for name, observable in zip(capability_names, capability_observable)
    }
    missing = [
        name for name in ("propagation", "horizontal_motion", "yaw_tracking")
        if name in capabilities and not capabilities[name]
    ]
    if missing:
        return True, "unobservable_" + "+".join(missing)
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
                request = True

        elif self.state == HOLDING:
            if now_s - self.state_since >= self.minimum_hold_s:
                if obvious_loss:
                    self._transition(RELOCALIZING_HOLD, now_s)
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
