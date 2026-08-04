# S-curve map and mission-safety audit (2026-08-01)

## Scope

This iteration expands the ordinary Gazebo test world, makes the long S-curve
mission conservative, and connects confirmed localization loss to a hold and
relocalization request. It does not claim that the unified estimator or online
relocalization has passed a closed-loop flight.

## Map coverage

- Ground collision remains one 42 x 42 m box.
- Optical-flow texture is visual-only: 19 x-lines and 19 y-lines cover
  -18..18 m, plus 32 asymmetric patches.
- The LiDAR model contains 22 asymmetric collision landmarks. New geometry is
  placed beside, not on, the S route and extends into the outer test area.
- The nominal 12 m x 4.5 m S route has a maximum nearest-landmark distance of
  4.123 m. A 5 x 5 audit grid over -16..16 m has a maximum nearest-landmark
  distance of 7.280 m.
- `tools/plot_s_curve_world_audit.py` writes a top-down PNG and JSON report.

Gazebo loaded the installed world with the project resource path and exposed
the MID360, flow, world pose, and world-statistics topics. No model URI error
occurred with the project `env.sh` environment.

Short RTX 5060 headless probes measured approximately 0.49 RTF for the Git
baseline world and 0.46 RTF for the expanded world. This startup-window probe
is not a flight benchmark; it indicates that the new visuals are a secondary
cost while the existing 1 ms physics and rendered sensors remain dominant.

## Conservative mission behavior

- Default S speed is reduced from 0.8 to 0.45 m/s.
- Endpoint hold increases from 2 to 3 s.
- The path pauses about every 3 m for at least 1 s and waits until FCU local
  position stays within 0.60 m for 0.50 s. It never advances merely because a
  fixed transit duration elapsed.
- Safety supervision requires a fresh `SchedulerState` before takeoff.
- A transient loss must persist for 0.30 s before the route is interrupted.
- Confirmed loss freezes the last commanded safe setpoint for at least 1 s and
  publishes `/relocalization/request=true`.
- Persistent loss remains in `RELOCALIZING_HOLD`; the route does not continue.
- Recovery requires propagation, horizontal-motion and yaw observability plus
  a 0.75 s stable dwell, then clears the relocalization request and resumes.

`RELOCALIZING` is not itself treated as continuing pose loss. Otherwise the
request would force Scheduler into `RELOCALIZING` and deadlock the mission even
after sensor observability recovered.

## Relocalization status

Implemented but not end-to-end accepted:

- in-memory static keyframe admission using map quality, repeatability,
  dynamic ratio and LiDAR degradation;
- ESF candidate retrieval, NDT initialization and ICP verification;
- `/relocalization/request` and `RelocalizationResult` messages;
- successful-result validation, `map_from_lio` alignment, reset counter, and
  unified-window reset with a recovered-state prior;
- after reset, `last_lio_stamp` is set to the relocalization timestamp, so the
  next manifold factor calls IMU preintegration over the new timestamp-to-next
  LiDAR interval. Preintegration is stateless, so this is the current
  implementation of restarting it.

Remaining gaps:

- no persistent relocalization map or Scan Context database;
- no validated automatic trigger outside the S-mission safety node;
- no explicit stale pending-factor flush and no integration test proving IMU
  coverage across a live relocalization reset;
- no measured success rate, recovery time, or false-relocalization rate;
- MAVROS ExternalNav still does not carry the project reset counter/quality to
  the FCU with verified semantics;
- the current dual-state FAST-LIO/backend map-ownership defect must be removed
  before relocalization can be accepted in closed loop.

## Verification

- `colcon build --packages-select uf_interfaces multi_slam_uav_sim`: passed.
- `colcon test --packages-select multi_slam_uav_sim`: 45 tests passed.
- focused path/safety/coverage tests: 12 tests passed after the final safety
  deadlock regression was added.
- XML parse, Python compile, shell syntax, and `git diff --check`: passed.
