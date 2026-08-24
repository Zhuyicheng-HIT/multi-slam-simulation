# Current complete PR #14 synchronization (2026-08-24)

## Source and history

- Upstream branch: `feat/core-algorithm-cleanup-20260817`
- Exact PR #14 head: `5f15ab032949e24b539375b4bfa6349e6b562b3b`
- GitHub update time: 2026-08-24 13:52:17 +08:00
- Previous upstream baseline: `e934132ffdd991b0dd59a752eead93d2e0313b40`
- Relationship: `e934132` is the direct ancestor of `5f15ab0` (0 behind, 1 ahead).
- Upstream delta: 8 files, 499 insertions, 136 deletions.

The upstream increment restores the tunnel/straight route contract, replaces
the simulated MicoLink optical-flow bridge with a ROS 2 C++ package, simplifies
sensor startup and MAVROS stream requests, and expands the simulated MID360
body exclusion envelope to `x/y=[-0.45,0.45] m`, `z=[-0.35,0.15] m`.

## Selective integration

The integration was rebuilt from the exact PR #14 head. It did not merge PR
#15. The following validated local changes were replayed in order:

1. Dynamic observer v1 and v2
2. fail-open Clean Gateway
3. long-term `STATIC_CONFIRMED` refinement
4. dynamic-localization and Hybrid relocalization integration
5. current-complete integration documentation
6. raw obstacle safety and sole command arbitration
7. local obstacle avoidance/replanning
8. production active-relocalization flight loop
9. WSLg EGL/OpenGL hardware-renderer preflight

There were no textual cherry-pick conflicts. Upstream retained ownership of
sensor startup, MicoLink transport, MAVROS stream configuration, tunnel
validation and the enlarged MID360 body envelope. Local code retained
ownership of Dynamic Raw/Clean separation, obstacle safety, command
arbitration, local replanning and active relocalization.

## Compatibility fixes found by runtime validation

1. ArduPilot 4.5.7 rejects the `HIGHRES_IMU` message-rate request. The minimal
   requester now falls back to the legacy `RAW_SENSORS` stream at the requested
   IMU rate while leaving other streams minimal.
2. External dependency workspaces are sourced through `local_setup.bash`.
   Sourcing their generated `setup.bash` could restore an obsolete project
   underlay and silently run old executables.
3. During GUIDED takeoff, MAVROS position setpoints switch ArduPilot out of its
   takeoff submode. A fresh `TAKEOFF` intent now makes the arbiter release
   setpoint ownership to the FCU takeoff controller. Obstacle BRAKE/HOVER still
   has higher priority, and an expired intent returns to fail-closed behavior.
4. Gazebo publishes both dynamic poses and a full pose inventory. The latter
   can contain the vehicle's initial pose. Both the C++ truth bridge and the
   evaluator now permanently prefer dynamic truth after it appears. This fixes
   a false multi-metre FAST-LIO drift report without changing estimator data.
5. The straight route now has an explicit one-waypoint validation contract.

## Runtime evidence

### Low-indoor rectangle strict run

- route: 4/4 legs, all turns, LAND and FCU disarm
- Native LiDAR / IMU / GNSS / optical-flow factors: 1123 / 1134 / 571 / 293
- optimization errors / rollbacks / Native worker overflow: 0 / 0 / 0
- unified odometry: 9.999 Hz; maximum source gap 0.200 s
- causal 3-D RMSE / P95 / maximum: 2.45 / 4.32 / 6.13 cm
- endpoint error: 2.79 cm
- strict acceptance: PASS, no failed gates

### Tunnel straight basic startup

- latest tunnel world and enlarged PR #14 body envelope were used
- Raw MID360, MicoLink C++ bridge, Native LiDAR factor and unified odometry
  all started
- straight route: 1/1, 3 m; LAND and disarm confirmed
- optimization errors / rollbacks / overflow: 0 / 0 / 0
- strict accuracy is intentionally **not** claimed: horizontal RMSE was 1.57 m
  and no optical-flow factor was accepted during this short tunnel run

The tunnel result is a valid startup/contract smoke test and direct evidence
for LIDAR-DIR-001. It must not be quoted as a tunnel localization pass.

### Build, test and targeted runtime regression

- clean overlay build: 20 packages PASS
- aggregate `colcon test-result --all`: 192 xUnit tests, 0 errors, 0 failures,
  0 skipped
- Dynamic observer ROS smoke: PASS; production FAST-LIO input remained Raw
- Clean Gateway ROS smoke: PASS, including exact-Raw fail-open for stale state,
  missing IMU coverage, queue overflow and timestamp regression
- long-term static-map ROS smoke: PASS; `STATIC_CONFIRMED_ONLY`, no future pose,
  fail-open map hold, and shadow-only semantic evidence were all observed
- Safety Slice ROS smoke: PASS for clear, wall brake, Raw dropout hover,
  localization hold, non-finite planner rejection and obstacle veto
- local avoidance ROS smoke: PASS; detour reached the goal without collision
- active relocalization ROS smoke: PASS for successful recovery, obstacle veto,
  and terminal fail-closed behavior; a wrong epoch never released recovery
- every safety/avoidance/relocalization smoke observed exactly one production
  MAVROS setpoint publisher: `flight_command_arbiter`
- Python bytecode compilation, 45 YAML documents, 42 XML/SDF/URDF documents,
  all tracked shell scripts, and `git diff --check`: PASS

## Ownership and defaults

- `/livox/lidar` remains the production Raw MID360 input.
- observer, Clean Gateway and long-term static-map launches default disabled.
- unknown points remain fail-open; Clean is for localization, not obstacle
  visibility.
- only `flight_command_arbiter` publishes the production MAVROS automatic
  position setpoint. Test-only smoke tools are excluded from deployment.
- priority remains obstacle BRAKE/HOVER > active relocalization > local planner
  > mission; FCU/manual failsafe remains above project automation.
- active relocalization continues to use `FusionEpoch`, validated
  `RelocalizationResult`, and recovery transaction gates.
- ExternalNav and one-observation-one-factor ownership are unchanged.
- axis/subspace LiDAR handoff remains default-off pending LIDAR-DIR-001.

Runtime evidence lives under ignored `logs/` directories and is not committed.
