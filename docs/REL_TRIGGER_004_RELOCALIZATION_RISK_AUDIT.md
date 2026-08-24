# REL-TRIGGER-004: Relocalization Trigger Audit and Shadow Risk Design

Date: 2026-08-24

## Scope and baseline

- Stable baseline: `integration/current-complete-pr14-20260824` at `054a6744cf2265bd4dc1bd4cee0be6287cd2dbc1`.
- Development branch: `feat/relocalization-trigger-risk-shadow-v1`.
- The directional LiDAR experiment at `90e8ff3e429cbf873c94b51ca65cb03d2aacdb0e` remains `DO_NOT_PROMOTE` and default-off.
- This stage adds a diagnostic-only evaluator. It is not part of a production launch, publishes no relocalization request or flight command, and does not modify Safety Slice, Local Avoidance, command-arbiter priority, Active Relocalization actions, Dynamic, Z-COV, or estimator mathematics.
- Ground truth appears only in the offline matrix scorer. The online risk model has no truth input.

## Existing production trigger paths

| Producer / consumer | Trigger contract | Dwell / cooldown | Clear, recovery, and failure |
|---|---|---|---|
| Reliability Scheduler automatic request | LiDAR reliability below `0.85` or disabled, horizontal-position support below `0.15`, three distinct LiDAR observations, relocalization ready | startup grace `10 s`; candidate hold `1.0 s`; cooldown `15 s`; request commit timeout `2.0 s` | Request remains asserted until the matching `FusionEpoch` transaction/candidate commits. Failure or commit timeout clears it. Scheduler enters `RELOCALIZING` while requested. Core scheduler transition/recovery dwell is `0.5/1.5 s`, with `1 s` recovered hold. |
| Mission Localization Safety | Scheduler stale `1.0 s`, unified odometry stale `0.6 s` or nonfinite, ExternalNav gate stale `1.5 s` or unhealthy, missing propagation/horizontal-motion/yaw capabilities, or estimator support below `0.15` | loss dwell `0.30 s`; minimum HOLD `1.0 s`; retry cooldown `5.0 s` | Recovery must remain valid `0.75 s`; then its request is cleared and navigation may recover. |
| Passive relocalization | Automatic loop-closure search only in scheduler `NORMAL/RECOVERED`, LiDAR enabled, keyframe age at least `20 s`, nearby distance no more than `3 m` | cooldown `15 s`; at most three attempts | It does not independently assert the shared fault request. A manual/fault request can adopt an already-running search. |
| Relocalization node | A false-to-true edge on `/relocalization/request` opens one transaction; database readiness requires at least six entries | search timeout `6 s`; up to ten attempts | Success requires the configured multi-query consistency and backend integrity gates. Backend accepts only a matching transaction and candidate, commits `FusionEpoch`, and reanchors/reset-bounds its buffers. |
| Active Relocalization controller | Existing request/recovery chain drives `NORMAL_NAVIGATION -> HOLD -> ACTIVE_RELOCALIZATION -> RECOVERY_VALIDATION -> RESUME`, otherwise `FAILSAFE` | initial hold `1 s`; active timeout `20 s`; recovery dwell `0.75 s`; resume `0.25 s`; maximum failures `2` | Passive attempts precede yaw scan and bounded safe motion. Raw Obstacle Safety can veto motion. Recovery requires matching epoch plus healthy scheduler/capabilities. Intents go through the flight command arbiter. |
| Obstacle Safety / arbiter interaction | Obstacle BRAKE/HOVER has higher priority than localization hold, active relocalization, planner, and mission | freshness and fail-closed contracts are owned by the existing Safety Slice | Relocalization motion is suppressed while obstacle safety is not clear. No priority was changed here. |

### Actual ownership defect

Both Reliability Scheduler and Mission Localization Safety publish a source-less level-triggered `std_msgs/Bool` on `/relocalization/request`. The consumer suppresses a second transaction while one is active, but the transport cannot express per-source ownership: one publisher's `false` can clear another publisher's still-valid `true`. Thus multi-source degradation can create competing/duplicate requests even though transaction duplication is partially contained downstream. A production fix should use source-owned leases or an explicit request-arbiter message; changing thresholds alone cannot repair this semantic defect.

## Problems found

1. Production reacts well to hard LiDAR/horizontal-support failure but has no explicit WATCH/DEGRADED representation for accumulating drift.
2. Sustained small position or yaw inconsistency can remain below hard health thresholds and never request relocalization.
3. A single directional weakness is not represented separately from whole-source health.
4. The shared Bool request has no producer identity, lease, reason, risk level, or transaction ownership.
5. Scheduler cooldown (`15 s`) and mission retry cooldown (`5 s`) are independent, so recovery/retrigger semantics are not globally coordinated.
6. Obstacle veto is correctly higher priority in the arbiter, but existing request messages do not encode whether safe relocalization motion is unavailable; the active controller resolves this later rather than at risk-classification time.
7. Hard failures and single-frame transients are largely debounced already; replacing the existing dwell with an immediate threshold would regress this property.

## Shadow risk model

The new model is causal, stateful, diagnostic-only, and uses hysteresis plus dwell:

| Level | Meaning | Default entry semantics | Shadow recommendation |
|---|---|---|---|
| 0 `NORMAL` | Healthy, mutually consistent sources | score below `0.25`, no persistent integrity issue | none |
| 1 `WATCH` | Short or directional weakness, early cross-source inconsistency | score at least `0.25` for `0.30 s`; directional weakness may contribute but is capped here | record and expose risk; no request |
| 2 `DEGRADED` | Persistent degradation or slow-drift evidence | score at least `0.48` for `1.0 s` | recommend HOLD/slowdown/observation, but do not publish control |
| 3 `RELOCALIZE` | Persistent production-eligible loss of integrity | score at least `0.72` for `1.5 s`, readiness valid, cooldown clear | shadow request edge once per episode |
| 4 `FAILSAFE` | Localization unavailable, repeated/failed relocalization, or no safe motion while relocalization is required | failed result, repeated failures, or BRAKE/HOVER conflict at relocalize risk | shadow HOLD/HOVER requirement |

Recovery must remain healthy for `2.0 s`; a successful matching result plus `FusionEpoch` starts a `15 s` cooldown. Timestamp regression/nonfinite time fails closed. External request sources are observed but never republished by the shadow node.

## Explainable slow-drift evidence

The model does not use truth. It forms bounded normalized evidence from:

- persistent position and yaw innovation across independent sources;
- residual/NIS that remains elevated but has not reached a hard reject;
- covariance growth;
- pose jump and velocity/position inconsistency;
- per-source reliability, factor admission/rejection, capability support, and estimator support;
- relocalization result/history and matching epoch state.

A causal time-based EWMA (default time constant `3 s`) is applied to drift indicators. The two strongest independent indicators and the second-largest source degradation are used so one noisy modality cannot normally force LEVEL 3. This exposes slowly accumulating inconsistency before global health is already lost while retaining persistence and hysteresis.

## Directional LiDAR future integration

Directional weakness is disabled by default. When explicitly enabled for shadow experiments it is capped to a `0.15` score contribution and LEVEL 1, is marked non-production-eligible, and cannot by itself request relocalization. Promotion would require the directional branch to pass its same-input A/B/C trajectory replay and then require corroboration from covariance growth, innovation/residual, or an independent source. It must not be converted directly into a production trigger while LIDAR-DIR-001 remains `DO_NOT_PROMOTE`.

## Deterministic shadow matrix

The matrix ran 12 scenarios, five deterministic seeds each (60 runs, 10 Hz, 30 s). Truth degradation time was supplied only to the scorer. Times below are representative seed-0 state-entry times in seconds.

| Scenario | WATCH | DEGRADED | RELOCALIZE | FAILSAFE | Existing production request | Result |
|---|---:|---:|---:|---:|---:|---|
| Normal flight | - | - | - | - | - | no false trigger |
| Short LiDAR degradation then recovery | - | - | - | - | - | transient rejected |
| Sustained LiDAR geometry degradation | - | 11.0 | 12.6 | - | 11.0 | detected |
| Single-direction weak constraint | 10.3 | - | - | - | - | shadow-only warning |
| GNSS degradation, LiDAR healthy | 10.3 | - | - | - | - | warning, no unnecessary request |
| Visual degradation | 10.3 | - | - | - | - | warning, no unnecessary request |
| Slow position drift (offline boundary 18.0) | 11.4 | 16.5 | 21.2 | - | - | WATCH 6.6 s and DEGRADED 1.5 s early; request 3.2 s after boundary |
| Yaw drift (offline boundary 17.0) | 11.8 | 15.7 | 19.7 | - | - | WATCH 5.2 s and DEGRADED 1.3 s early; request 2.7 s after boundary |
| Multiple sources degraded | - | - | - | 10.0 | 11.0 | fail-safe classification; competing producers observed |
| Successful relocalization and epoch recovery | - | 11.0 | 12.6 | - | 11.0 | matching success/epoch clears, cooldown prevents premature retrigger |
| Relocalization failure | - | 11.0 | 12.6 | 14.0 | 11.0 | failure latched fail-safe |
| Obstacle safety and relocalization conflict | - | 11.0 | - | 11.1 | 11.0 | obstacle veto wins; no unsafe shadow request |

Aggregate results:

- Existing production: `0` false triggers, `10` missed triggers (all five slow-position and all five yaw-drift trials), and `5` competing-producer episodes (all multi-source trials).
- Shadow model: `0` false triggers and `0` missed triggers for the matrix expectations. It observed the same five producer competitions but emitted at most one shadow edge per episode.
- Evaluation runtime: P50/P95/P99 `8.300/13.125/30.568 us` per update in the final run.

These results establish logic behavior on normalized deterministic inputs, not field sensitivity. They do not justify direct production enablement without same-input real/simulation log replay.

## Build, tests, and runtime contract

- Full workspace: 20 ROS 2 packages built successfully after sourcing the existing Livox dependency underlay at `/home/zyc/multi-slam-deps/mid360_ws/install`.
- Full `colcon test-result`: 192 tests, 0 errors, 0 failures, 0 skipped.
- New pure-model tests: 12 passed, covering dwell, hysteresis, slow drift, directional cap, cooldown, epoch recovery, duplicate suppression, fail-safe, and invalid time.
- ROS smoke/test: diagnostic publication passed; the shadow node had zero publishers on `/relocalization/request` and only subscribed to it.
- Flake8, Python compile, YAML parse, and `git diff --check` passed.
- The node is not referenced by any production launch. Configuration must be selected explicitly.

## Recommendation

Choose **B: one more shadow round**, using frozen same-input logs from normal flight, hard degradations, slow position/yaw drift, recovery, and obstacle conflict. The risk hierarchy is ready for observation, but production should not change yet because:

1. the 60-run matrix uses deterministic normalized signals rather than actual online distributions;
2. the source-less shared Bool request ownership defect needs a dedicated request-arbitration/lease design;
3. thresholds need false-alarm and lead-time characterization on real callback timing and covariance/NIS distributions;
4. directional LiDAR evidence remains experimental and default-off.

The next stage should run this node shadow-only against identical recorded inputs, compare its state timeline to production requests and evaluator-only truth, then separately design a source-aware request arbiter. No production trigger threshold should be changed as part of that replay.
