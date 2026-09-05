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

## Flight-candidate validation

On `feat/manual-relocalization-flight-v1`, the frozen takeoff stack was
exercised with real Gazebo, ArduPilot SITL, MAVROS, the MID360 bridge,
FAST-LIO and the flight arbiter. Session A completed a 3 m x 2 m rectangle,
LAND and disarm; Session B completed the same route and LAND/disarm. Evidence
was written under `/tmp/manual-reloc-flight-A` and `/tmp/manual-reloc-flight-B`
by the runner.

The Session-B manual recovery attempt did not produce a valid production
relocalization transaction. The existing `d435i_paper_visual_integration`
launch starts the backend but does not include the request arbiter,
`relocalization_node`, or active controller. When those existing nodes were
started explicitly, the relocalization node reported `database ready=0
keyframes=0`; `/lio/local_map` synchronization also expired without a valid
map-frame keyframe stream. The manual service accepted START/CANCEL, and the
deterministic ROS smoke passed with one `flight_command_arbiter` setpoint owner,
but this is not flight-recovery evidence. The current in-memory
`StaticKeyframeDatabase` has no Session-A persistence/load contract.

Therefore cross-session map warm-up, candidate matching and in-flight recovery
remain open gates. The no-flight matrix is authoritative for state transitions,
duplicate/cancel/epoch/obstacle failure handling and publisher ownership.
Status: `DO_NOT_PROMOTE`.

## Continued candidate validation (2026-09-05)

The integration launch exposes `relocalization_source_lio_pose_topic`, defaulting
in simulation to `/fusion/unified/frontend_activation_odom`. The validated
FAST-LIO compatibility mode does not publish `/Odometry`; this backend topic is
timestamp-aligned and carries the `camera_init` to `body` pose contract.

Session A4 produced `/tmp/manual-reloc-map-A4-db` with 19 keyframes and per-PCD
SHA256 entries in its manifest. Archive loading initially SIGSEGVed because
readiness was published before its publisher was constructed; that ordering bug
was fixed and standalone archive-load smoke now remains alive with `ready=1`.

Session B1 used the real PR6 Gazebo/SITL stack and the persistent archive. Manual
START was accepted; candidate 0 passed descriptor, forward, reciprocal, and
three-query consistency checks. The relocalizer then logged `candidate accepted
... awaiting unified backend reset acknowledgement`, but no backend reset
acknowledgement was observed. The active controller consequently retained HOLD
ownership and the route did not resume. Evidence is preserved in
`/tmp/manual-reloc-session-B1-fixed-run`. This is an integration acknowledgement
blocker, not a candidate-quality failure; status remains `DO_NOT_PROMOTE` until
the result/epoch acknowledgement contract is traced and three real recoveries
complete.

## Final recovery boundary evidence

After the acknowledgement diagnostics were corrected, B3/B4/B5 all showed the
backend accepting the result and applying the matching FusionEpoch. The remaining
hold is the existing recovery gate: `recovery_healthy` requires a fresh scheduler
state in `NORMAL` or `RECOVERED`, propagation/horizontal/vertical/yaw capabilities,
and estimator support at least 0.15. With the reduced PR6 sensor profile used for
these trials, that health predicate did not become true, so the active controller
correctly retained HOLD and the route remained at its pre-relocalization position.
This is an exact gate condition visible in `active_relocalization_controller.cpp`,
not a candidate or epoch failure. A full five-source Session B profile must be run
to establish scheduler recovery before claiming the requested 3/3 flight result.

The full five-source B6 startup did reach a ready database (28 keyframes), accepted
candidate 19, and applied its FusionEpoch, but the visual stack failed its upstream
`/sensors/rgbd/color` readiness contract and the run exited before route execution.
The reduced-profile B3/B4/B5 routes completed takeoff and LAND/disarm but remained
at the HOLD position after recovery validation. No run is counted as a successful
cross-session recovery.
