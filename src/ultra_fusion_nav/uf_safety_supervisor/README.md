# Ultra-Fusion Safety Slice

Active relocalization is integrated as an arbiter-owned production candidate.
See `docs/REL_ACT_003_ACTIVE_RELOCALIZATION_FLIGHT.md` at the repository root
for its state, epoch/recovery, Raw-obstacle veto and validation contracts.

This opt-in package closes the minimum low-level obstacle-safety loop without
changing estimator mathematics or Dynamic Clean ownership.

## Sensor and command ownership

`raw_obstacle_safety_monitor` subscribes to the original `/livox/lidar`
`livox_ros_driver2/CustomMsg`. It never subscribes to a clean/static/dynamic
classification output. Clean scans remain localization-only, so a person or
animal removed from FAST-LIO remains visible to obstacle safety.

`flight_command_arbiter` is the only project node that publishes automatic
`/mavros/setpoint_position/local`. Mission, planner, and active relocalization
publish source-specific candidate topics.

`local_avoidance_planner` implements a bounded, conservative 2.5-D local A*
detour over the same Raw MID360 scan. It publishes only a planner intent and a
candidate path. The raw obstacle monitor independently checks that candidate
before the arbiter may select it. Planning uses 0.80 m obstacle inflation while
the independent trajectory gate continues to require 0.65 m clearance; this
provides tracking margin without weakening the safety contract.

Priority is fixed:

1. manual or FCU failsafe (release automatic publication)
2. raw-obstacle BRAKE/HOVER
3. LAND/RETURN
4. localization HOLD
5. safe active relocalization
6. local planner
7. mission

## Obstacle states

- `CLEAR`: selected command may pass unchanged.
- `CAUTION`: selected displacement is bounded by `caution_step_m`.
- `BRAKE`: publish the current finite local pose as a hold setpoint.
- `HOVER_REQUIRED`: fail-closed hold for stale/dropout, timestamp regression,
  non-finite raw points, stale motion, or an internal health failure.

The braking envelope is
`margin + reaction_time * speed + speed^2 / (2 * maximum_deceleration)`.
TTC provides an independent trigger. Body-frame velocity and raw points remain
usable when world localization is degraded.

## Local replanning states

The local planner follows the fail-closed sequence
`NAVIGATING -> PATH_BLOCKED -> BRAKE_HOLD -> REPLAN -> TRAJECTORY_VERIFY -> RESUME`.
A stale Raw scan, stale/non-finite mission or odometry, planner timeout, failed
search, or candidate-path conflict transitions to `HOVER_REQUIRED`. It never
forces progress through an invalid path. Global exploration and long-horizon
dynamic prediction remain out of scope.

## Run

```bash
ros2 launch uf_safety_supervisor safety_slice.launch.py
```

Observer-only stacks do not start this package.  Every project automatic-route
entry point starts or reuses exactly one instance before emitting an intent;
the helper refuses to coexist with an unknown direct MAVROS setpoint publisher.
No global exploration, dynamic prediction, estimator, ExternalNav, EKF3,
Z-axis, or Dynamic Clean algorithm is changed.
