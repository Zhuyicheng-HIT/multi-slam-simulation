# REL-ACT-003 Active Relocalization Production Flight Candidate

## Scope and baseline

- Baseline: `feat/local-avoidance-nav-avoid-002` at
  `4e12ef947ae5ffa847e469af31e4fabc1ff1946d`.
- Development branch: `feat/active-relocalization-production-v1`.
- The localization estimator, relocalization registration, Dynamic stack and
  Z-axis candidate algorithms are unchanged.
- This slice converts the existing active-relocalization policy into an
  arbiter-owned flight intent. It is a production/hardware-validation
  candidate, not an autonomous substitute for the FCU failsafe or pilot.

## Reused implementation

The controller directly uses `ActiveRelocalizationPolicy`. The policy retains
the existing health gates and progression through passive search, stationary
yaw observations and opt-in safe motion. It also uses the existing
`RelocalizationResult` identity and `FusionEpoch` transaction contract. A
candidate match alone is insufficient: the same transaction and candidate
must appear in an applied epoch, followed by continuously healthy scheduler
capabilities for the configured recovery dwell.

The old S-curve experiment executor is not used. It was an experiment-only
route executor with legacy setpoint ownership. The safe yaw geometry is reused
conceptually; figure-eight motion is intentionally excluded from this first
production candidate because four stationary yaw views plus a bounded 0.35 m
motion provide the required observability with a smaller swept volume.

## Flight state machine

```text
NORMAL_NAVIGATION
  -> HOLD                    relocalization request + valid local FCU pose
  -> ACTIVE_RELOCALIZATION   1.0 s stable hold complete
  -> RECOVERY_VALIDATION     accepted result with nonzero transaction/candidate
  -> RESUME                  matching applied epoch + 0.75 s healthy dwell
  -> NORMAL_NAVIGATION       0.25 s resume dwell

Any invalid clock, unavailable stabilization, explicit failure, cancellation
during HOLD/ACTIVE, or 20 s timeout -> latched FAILSAFE HOLD.
```

An old transient-local epoch cannot release a new request: its arrival must be
after both the current request and the accepted result. A wrong transaction or
candidate remains in `RECOVERY_VALIDATION`. The matching epoch also cannot
start the healthy dwell until the upstream relocalization request is released;
this prevents a still-active request from being mistaken for completed recovery.

## Actions and admission

| Action | Admission | Flight behavior |
| --- | --- | --- |
| HOLD / passive search | valid FCU-local pose; propagation, vertical and yaw capabilities observable | Arbiter holds the current pose; no scan motion |
| YAW_SCAN | passive budget exhausted; local odometry and Raw obstacle state fresh | Four 90-degree stationary yaw targets, each rechecked against Raw safety before authorization |
| EGO_SAFE_MOTION | yaw views exhausted; policy enabled; fresh Raw map | Up to four 0.35 m body-relative targets; each candidate is published for Raw trajectory checking before it becomes an intent |
| FAILSAFE | result failure, timeout, invalid/stale stabilization or repeated/reinitialized failure | Safe HOLD/HOVER; no mission/planner forwarding until manual intervention/restart |

`UNKNOWN`, stale, nonfinite or missing safety input never authorizes motion.
The safe-motion targets use no future estimator state and no truth input.

## Command ownership

```text
mission intent -----+
local planner ------+
active relocalization controller -- intent + status --> flight_command_arbiter
Localization Safety -------------------------------> flight_command_arbiter
Raw obstacle safety -------------------------------> flight_command_arbiter
manual / FCU failsafe ------------------------------> flight_command_arbiter
                                                     |
                                                     +--> MAVROS setpoint
```

Only `flight_command_arbiter` publishes
`/mavros/setpoint_position/local`. The active controller publishes only:

- `/autonomy/intent/relocalization/pose`
- `/autonomy/relocalization_candidate_path`
- `/safety/active_relocalization_status`

It does not share or race the existing `/safety/localization_hold` publisher.
The arbiter derives the active HOLD directly from the status message. Priority
remains manual/FCU > Raw obstacle > LAND/RETURN > localization/active HOLD >
authorized active relocalization > local planner > mission.

The Raw safety monitor evaluates the active candidate path before authorization
and retains a per-cycle veto after authorization. BRAKE/HOVER therefore wins
over yaw, safe motion, planner and mission at all times.

## NAV-AVOID interface closure

Runtime A/B exposed one pre-existing recovery defect: after a detour or hold,
the local planner returned to `NAVIGATING` but stopped refreshing its planner
intent. The arbiter correctly treated the historical intent as stale and held
forever. `NAVIGATING` now republishes the direct mission segment only after the
same Raw obstacle verification. This is an ownership/interface correction; it
does not change planning geometry, clearance, factor definitions or priorities.

## Validation

Deterministic ROS closed-loop results:

- Normal degrade -> HOLD -> YAW_SCAN -> matching epoch -> resume: 3/3.
- Representative normal recovery time: 3.223 s from request to RESUME; original
  goal reached in 5.075 s total.
- Full four-view yaw scan followed by 0.35 m safe motion: 2/2; recovery
  6.124-6.325 s; original goal reached.
- Sudden Raw obstacle during active relocalization: 1/1 safe veto, 21 obstacle
  ownership cycles, zero continued relocalization motion while blocked; clear
  input restored active motion and the original goal was reached.
- Explicit candidate failure: 1/1 latched FAILSAFE HOLD.
- Wrong epoch: 4/4 representative success/safe-motion runs remained in recovery
  validation until the matching epoch arrived.
- Single MAVROS setpoint publisher: all runs reported exactly
  `flight_command_arbiter`; no dual publisher observed.
- Local avoidance regression: wall detour completed with one replan, zero
  collision, full blocked/brake/replan/verify/resume state sequence.
- Safety slice smoke: wall brake, Raw dropout hover, localization hold,
  nonfinite planner containment and relocalization/obstacle conflict all pass.

Runtime on the WSL validation host at the 20 Hz control cadence:

- `active_relocalization_controller`: approximately 0.9% of one CPU and
  26.3 MiB RSS during the closed-loop safe-motion test.
- `flight_command_arbiter`: approximately 0.8% CPU and 27.2 MiB RSS.
- No queue overflow, lost setpoint ownership, collision or process crash was
  observed in the deterministic runs.

Regression:

- 19 ROS packages build successfully.
- `colcon test-result`: 189 tests, 0 errors, 0 failures, 0 skipped.
- Relocalization policy: 8/8; flight core: 6/6; command arbiter: 16/16;
  command ownership: 5/5.
- `git diff --check` and Python syntax checks pass.

## Remaining production gates

- Hardware MID360 obstacle freshness and FCU-local pose latency must be measured
  on a disarmed bench before propeller-on testing.
- Real vehicle yaw braking, body envelope and 0.35 m safe-motion clearance need
  cage/tether validation; the deterministic ROS test is kinematic, not a claim
  of real-flight certification.
- A hard FAILSAFE is intentionally latched and requires operator intervention;
  automatic retries are not introduced in this slice.
- Figure-eight active motion is not admitted in V1.

## Decision

`PROMOTE_ACTIVE_RELOCALIZATION`: the software production candidate satisfies
the closed-loop ownership, Raw safety veto, transaction/epoch recovery and
fail-closed requirements. Promotion means proceed to disarmed bench and then
tethered validation; it does not mean unrestricted autonomous flight release.
