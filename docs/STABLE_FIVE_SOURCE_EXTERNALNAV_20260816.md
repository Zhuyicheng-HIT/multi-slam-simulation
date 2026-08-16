# Five-source ExternalNav stable candidate

Version scope: `2026-08-16-five-source-externalnav-candidate`

This milestone integrates FCU IMU, MID360 point-to-plane factors, GNSS/BDS,
MTF-01P-style optical flow, and D435i RGB-D feature reprojection in one sliding
window. It publishes one fused odometry stream to ArduPilot EKF3. Gazebo truth
is used only by the evaluator.

This is a simulation-validated research candidate. It is not a hardware flight
release and does not claim that automatic online time-offset application is
production-ready.

## Stable one-command validation

After entering the workspace with `cuf_ws`, run:

```bash
bash tools/run_stable_five_source_validation.sh
```

The command owns the complete headless graph, flies one short rectangle, lets
EKF3 consume `/mavros/odometry/out`, records a replay bag, evaluates the unified
state against Gazebo truth, checks landing/disarm, and stops its child processes.
It exits nonzero when a strict gate fails.

An isolated ROS domain can be selected without editing the script:

```bash
VALIDATION_ROS_DOMAIN_ID=42 bash tools/run_stable_five_source_validation.sh
```

## Verified live result

Evidence directory:

```text
logs/externalnav_rectangle_20260816_v20_five_source
```

The v20 closed-loop run passed every strict gate:

| Metric | Result |
|---|---:|
| causal 3D RMSE | 0.0482 m |
| causal 3D P95 | 0.0785 m |
| causal 3D maximum | 0.0944 m |
| endpoint error | 0.0235 m |
| horizontal RMSE | 0.0160 m |
| vertical RMSE | 0.0455 m |
| unified odometry source rate | 10.0 Hz |
| ExternalNav output rate | 20.0 Hz |
| visual factors | 412 |
| visual solver acceptance | 412 / 412 |
| optimizer errors / rollbacks | 0 / 0 |

The evaluator freezes a 10 s initial yaw/translation alignment and does not use
the future trajectory. It matched 1528 samples and observed 576 motion samples.
EKF3 ExternalNav consumption, the MAVROS NED/FRD TF contract, all five factor
families, route completion, landing, and disarm were independently gated.

## ExternalNav covariance and continuity policy

The backend computes a 15-state marginal covariance from the active factor
graph. Unobservable eigen-directions receive a large finite variance. Between
optimization states, IMU propagation applies `F P F^T + Q`. The state covariance
is then mapped into ROS pose and body-velocity covariance before publishing
`/fusion/unified/odom`.

The ExternalNav gate republishes at 20 Hz and adds finite process uncertainty as
the optimized state ages. It also scales the covariance by estimator capability
support, up to a configured finite maximum. ArduPilot therefore receives a
continuous but less confident measurement during a bounded degradation instead
of a stream that silently claims nominal precision.

In v20 the final maximum pose-position diagonal was `0.00667 m^2`, the maximum
orientation diagonal was `0.00635 rad^2`, and the maximum linear-velocity
diagonal was `0.189 m^2/s^2`. The largest position/orientation diagonals seen
during the run were `1.053 m^2` and `0.0309 rad^2`. Covariance came from the
window marginal or its IMU-propagated anchor; fallback covariance was not used.
These are estimator outputs, not hand-filled constants or accuracy results.

Output admission uses OR semantics for redundant sensors:

- one fresh, usable modality is enough to keep the estimator out of `FAILSAFE`;
- a missing required IMU changes the scheduler to `RISK`, but does not by itself
  kill the state stream;
- missing horizontal/yaw/propagation capability is reported to the safety state
  machine, which may hold or request relocalization;
- `DEGRADED`, `RISK`, and `RELOCALIZING` may continue to publish ExternalNav;
- no usable source, stale state, non-finite values, invalid quaternion or
  covariance, timestamp regression, and physically implausible jumps remain
  hard output failures.

This separates "the estimate is degraded" from "there is no safe state to
publish". The safety controller remains responsible for flight behavior while
the estimator preserves continuity.

## ArduPilot interface boundary

The stable ROS output is:

```text
/mavros/odometry/out
```

MAVROS converts the unified `camera_init -> body` odometry through the explicit
`camera_init_ned` and `body_frd` transforms. The current SITL EKF3 profile uses
ExternalNav for horizontal position and horizontal velocity. Barometer remains
the vertical source and compass remains the yaw source in this first closed-loop
profile. Hardware deployment must read back the actual EKF3 source parameters,
frames, message rate, covariance, quality, and reset counter before flight.

No FAST-LIO odometry, FCU local position, or Gazebo truth is allowed to become a
second authoritative state input to the unified backend.

## Time calibration policy

Both LiDAR-IMU and visual-IMU offset estimators run online for observability and
diagnostics. The stable defaults are:

```text
calibration_apply_locked_values: false
calibration_apply_locked_time_offset: false
calibration_apply_locked_rotation: false
visual_time_calibration_apply_locked: false
visual_initialization_require_time_lock: false
```

The v20 LiDAR-IMU estimator produced candidates but did not obtain a repeatable
lock. The visual estimator locked during part of the motion, but its final
correlation was low. Earlier controlled runs produced offsets in the approximate
`-5 ms` to `-15 ms` range, which is useful evidence but not sufficient to enable
automatic timestamp mutation.

Production policy is therefore:

1. measure and configure fixed sensor offsets;
2. keep online estimators in shadow mode and monitor correlation, peak margin,
   independent agreement, and lock revocation;
3. enable automatic application only in a separate A/B experiment with injected
   offsets and a repeatability gate.

### Online calibration follow-up

A later calibration-only simulation locked the visual-to-IMU offset at
approximately `+16 ms`. Applying that locked visual offset retained `0.0396 m`
causal 3D RMSE, `0.0778 m` P95 and `0.0911 m` maximum error. The same run also
recorded two transactional optimizer rollbacks, one latest-only native queue
discard and a `0.29 s` unified-odometry gap. The automatic visual offset path is
therefore implemented and accuracy-compatible, but is still experimental.

The LiDAR-IMU spatiotemporal calibrator and the visual-IMU calibrator are
separate mechanisms. Reports and promotion gates must name them separately;
locking or applying one is not evidence that the other has been applied.

## Known limits

- The live five-source run meets the 0.20 m target on the short rectangle; the
  large figure-eight and real hardware remain separate acceptance milestones.
- D435i simulation depth is idealized and uses a 10 m experimental range. The
  hardware profile must return to a calibrated range, initially 0.3-6.0 m, with
  per-point depth quality and realistic holes/noise.
- D435i/MID360 online shared mapping is a separate map consumer. It is not an
  additional dense factor in this stable sliding window.
- Current EKF3 closed-loop evidence covers ExternalNav horizontal position and
  velocity, not an ExternalNav takeover of barometric height or compass yaw.
- The solver maintained the required 10 Hz source output, but the full simulated
  stack ran slower than wall time. Hardware CPU profiling is still required.

## Promotion gates for hardware

Before removing the simulation-only label:

1. replay real FCU IMU, MID360, MTF-01P, GNSS/BDS, and D435i bags with source
   timestamps and measured extrinsics;
2. validate fixed time offsets and keep automatic calibration shadow-only;
3. verify no factor double-counts FAST-LIO or FCU fused local position;
4. bench-test `/mavros/odometry/out` with EKF3 parameter readback and no propellers;
5. test source outages while confirming covariance inflation, continuous output,
   HOLD behavior, and reset-counter handling;
6. repeat the 0.20 m accuracy and continuity gates on a motion-capture or surveyed
   reference trajectory before free flight.
