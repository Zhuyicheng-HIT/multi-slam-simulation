# Manual Relocalization Session Validation

## Candidate

- Worktree: `feat/manual-relocalization-sitl-session-v1`
- Frozen base: `d77ba581ea3ae0e7b8bb2fef51515d6d118f8305`
- Product gateway: `feat/manual-relocalization-control-v1` (kept in its existing worktree)
- Validation logs: `/tmp/manual-relocalization-control-v1/ros-smoke`

## No-flight ROS validation

Five independent ROS domains ran the real service-to-controller path. Each run
accepted `START` and `CANCEL`, produced the state sequence
`NORMAL -> HOLD -> ACTIVE_RELOCALIZATION -> RECOVERY_VALIDATION -> RESUME -> NORMAL`,
rejected a wrong FusionEpoch before the matching epoch, and reported exactly one
MAVROS setpoint publisher (`flight_command_arbiter`). Recovery took 3.16--3.27 s.

The obstacle run observed 21 Raw Obstacle Safety veto samples while preserving
the same recovery state sequence. The failure run reached `FAILSAFE` with the
arbiter as the only setpoint owner. Unit and ROS tests cover duplicate START,
cancel, stale/future/regressed timestamps, lease/source loss, timeout, missing
candidate, repeated epoch, and concurrent automatic/manual intents.

## SITL evidence

Gazebo low-indoor headless and ArduPilot SITL produced valid `/clock`, sensor
topics, MAVROS heartbeats, and command acknowledgements. The vehicle remained
on ground: actuator outputs stayed idle and Gazebo repeatedly logged controller
resets/input-frame misses. Thus no takeoff, route, LAND, or in-flight recovery
claim is made; this is an environment/control synchronization blocker, not a
manual-request ownership failure. Logs are under
`/tmp/manual-reloc-session-sim-20260903`.

## Verification

Full workspace build and `colcon test` completed with **266 tests, 0 errors,
0 failures, 0 skipped**. The validation smoke does not publish the final
`/relocalization/request` or MAVROS setpoint directly.
