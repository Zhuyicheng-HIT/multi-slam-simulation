# Raw-obstacle safety and command arbitration slice

## Scope and baseline

This branch starts from `integration/dynamic-current-complete-v2` at
`cf8ec51335aec11ce60fb0af4bd36a5d950eebf3`.  It does not change the estimator,
Dynamic Clean ownership, ExternalNav, EKF3, or any Z-axis algorithm.  Dynamic
Clean remains a localization input only; obstacle safety always observes the
raw MID360 representation.

## Production command ownership

```text
manual / FCU native failsafe ------------------------------> FCU owns control
Raw MID360 ---> raw_obstacle_safety_monitor --+             (arbiter releases)
mission intent --------------------------------+---> flight_command_arbiter
planner intent --------------------------------+              |
localization HOLD ------------------------------+              +--> MAVROS pose setpoint
active relocalization intent -------------------+              +--> mode intent
LAND / RETURN intent ---------------------------+
```

`flight_command_arbiter` is the only project source that creates a publisher
for `/mavros/setpoint_position/local`.  The fixed priority is:

1. manual / FCU failsafe (release automatic ownership)
2. obstacle BRAKE or HOVER_REQUIRED
3. LAND or RETURN
4. localization HOLD
5. safe active relocalization
6. local planner
7. mission

Automatic route entry points start or reuse one safety slice.  They reject an
unknown existing setpoint publisher instead of creating a second owner.

## Obstacle-state contract

| State | Trigger | Flight behavior |
|---|---|---|
| CLEAR | raw scan and motion are fresh and the swept corridor is clear | selected safe intent passes |
| CAUTION | clearance or TTC enters the caution envelope | intent step is capped at 0.30 m |
| BRAKE | clearance is within stopping distance or TTC <= 1.25 s | current finite FCU-local pose is held |
| HOVER_REQUIRED | raw dropout/stale, timestamp regression/future stamp, nonfinite point/motion, or internal unhealthy state | fail-closed hold; never drop through to mission |

The default swept body envelope uses front/half-width/half-height
`0.32/0.32/0.18 m`, lateral/vertical margins `0.28/0.22 m`, and clearance
margin `0.35 m`.  Stopping distance is:

```text
0.35 + 0.25 * forward_speed + forward_speed^2 / (2 * 2.0)
```

This gives 0.85 m at 1 m/s and 3.35 m at 3 m/s.  A separate TTC gate catches
sudden intrusions.  If unified localization is unavailable, raw body-frame
points plus finite MAVROS body motion still produce BRAKE; localization loss
does not blind obstacle safety.

## Fault and conflict validation

Core tests cover a wall/column in the commanded corridor, high-speed approach,
sudden appearance, raw stale/dropout, timestamp errors, nonfinite raw points,
localization loss with body-frame braking, caution, and braking simulation.
Arbiter tests cover every priority level, planner timeout/nonfinite/jump,
relocalization versus obstacle conflict, manual/FCU takeover, and missing
intent/current-pose fail-closed behavior.

The ROS smoke test exercised real Livox `CustomMsg`, odometry, mission/planner/
relocalization intents and MAVROS state.  All checks passed: CLEAR, wall BRAKE,
dropout HOVER, localization HOLD, mission ownership, nonfinite planner HOLD,
obstacle-over-relocalization, and exactly one setpoint publisher.

The deterministic braking model produced minimum clearances of 0.353 m for a
1 m/s wall approach, 0.352 m for a 3 m/s approach, and 0.353 m for a sudden
obstacle.  Collision count was 0/3.  These are software safety-contract tests,
not a substitute for propeller-off hardware braking calibration.

## Runtime

With 20,000 raw points and 500 iterations on this WSL host:

| Component | P50 | P95 | P99 |
|---|---:|---:|---:|
| raw obstacle monitor core | 15.289 ms | 16.035 ms | 16.593 ms |
| command arbiter core | 0.0029 ms | 0.0042 ms | 0.0101 ms |

The monitor operates directly on the incoming scan without building a map or
running dynamic classification.  The final hardware rate and braking margin
must be verified using the actual MID360 mounting extrinsic and measured vehicle
deceleration.

## Verification result and boundary

- final full workspace: 19 packages and 164 xUnit, zero errors/failures/skips
- C++ core, Python ownership, shell syntax, YAML/XML and `git diff --check` pass
- ROS/Livox smoke pass; no residual safety processes
- no direct MAVROS setpoint publisher remains outside the arbiter

This slice is suitable for promotion as a reversible safety candidate.  Before
untethered flight it still requires propellers-off/tethered validation of the
real braking response, MID360 extrinsic/body envelope, raw-link dropout, FCU
mode executor behavior, and manual takeover latency.
