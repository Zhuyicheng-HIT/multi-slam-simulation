from collections import deque
from dataclasses import dataclass

import numpy as np


UNINITIALIZED = "UNINITIALIZED"
ACTIVE = "ACTIVE"
OUTAGE = "OUTAGE"
REANCHORING = "REANCHORING"
REJECTED_JUMP = "REJECTED_JUMP"


@dataclass(frozen=True)
class AidingResult:
    state: str
    accepted: bool
    position: object
    blend: float
    innovation_m: float
    reason: str


class SmoothGnssReanchor:
    def __init__(self, jump_threshold_m=3.0, outage_timeout_s=1.5,
                 reacquire_samples=5, reanchor_duration_s=3.0,
                 max_degradation_score=0.45, anchor_stability_m=0.75):
        self.jump_threshold_m = float(jump_threshold_m)
        self.outage_timeout_s = float(outage_timeout_s)
        self.reacquire_samples = max(2, int(reacquire_samples))
        self.reanchor_duration_s = max(1.0e-3, float(reanchor_duration_s))
        self.max_degradation_score = float(max_degradation_score)
        self.anchor_stability_m = float(anchor_stability_m)
        self.state = UNINITIALIZED
        self.anchor = None
        self.last_valid_time = None
        self.reanchor_started = None
        self.anchor_candidates = deque(maxlen=self.reacquire_samples)

    def _invalid(self, timestamp_s, reason, innovation=float("nan")):
        if self.last_valid_time is None or timestamp_s - self.last_valid_time >= self.outage_timeout_s:
            if self.state != OUTAGE:
                self.anchor = None
                self.anchor_candidates.clear()
                self.reanchor_started = None
            self.state = OUTAGE
        return AidingResult(self.state, False, None, 0.0, innovation, reason)

    def _collect_anchor(self, timestamp_s, candidate):
        if self.anchor_candidates:
            center = np.median(np.asarray(self.anchor_candidates), axis=0)
            if float(np.linalg.norm(candidate - center)) > self.anchor_stability_m:
                self.anchor_candidates.clear()
        self.anchor_candidates.append(candidate)
        if len(self.anchor_candidates) < self.reacquire_samples:
            self.state = REANCHORING
            return False
        self.anchor = np.median(np.asarray(self.anchor_candidates), axis=0)
        self.anchor_candidates.clear()
        self.reanchor_started = timestamp_s
        self.state = REANCHORING
        return True

    def update(self, timestamp_s, gnss_position, reference_position,
               degradation_score, valid_fix=True):
        timestamp_s = float(timestamp_s)
        if (not valid_fix or gnss_position is None or reference_position is None
                or float(degradation_score) > self.max_degradation_score):
            return self._invalid(timestamp_s, "unreliable_or_missing_fix")

        gnss = np.asarray(gnss_position, dtype=float)
        reference = np.asarray(reference_position, dtype=float)
        candidate_anchor = reference - gnss

        if self.anchor is None or self.state == OUTAGE:
            ready = self._collect_anchor(timestamp_s, candidate_anchor)
            self.last_valid_time = timestamp_s
            if not ready:
                return AidingResult(
                    self.state, False, None, 0.0, float("nan"), "collecting_stable_anchor"
                )

        corrected = gnss + self.anchor
        innovation = float(np.linalg.norm(corrected - reference))
        if self.state == ACTIVE and innovation > self.jump_threshold_m:
            self.state = REJECTED_JUMP
            return AidingResult(
                self.state, False, None, 0.0, innovation, "jump_rejected"
            )

        if self.state == REJECTED_JUMP:
            if innovation > self.jump_threshold_m:
                return self._invalid(timestamp_s, "jump_still_present", innovation)
            self.state = ACTIVE

        self.last_valid_time = timestamp_s
        if self.state == REANCHORING:
            elapsed = max(0.0, timestamp_s - self.reanchor_started)
            blend = min(1.0, elapsed / self.reanchor_duration_s)
            if blend >= 1.0:
                self.state = ACTIVE
            return AidingResult(
                self.state, True, corrected, blend, innovation, "smooth_reanchor"
            )
        self.state = ACTIVE
        return AidingResult(self.state, True, corrected, 1.0, innovation, "accepted")
