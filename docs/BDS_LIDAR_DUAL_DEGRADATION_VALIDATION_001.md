# BDS + LiDAR Dual-Degradation Validation

## Baseline

- Branch: `feat/bds-lidar-dual-degradation-v1`
- Frozen algorithm baseline: `02935070bf7177efe6b07d01db9e2e6beac6f0dc`
- Validation commits: `6a51791e`, `570b129`, `191b80b`, `e37ad64`, `5ce18c7`
- External Livox workspace: `$HOME/multi-slam-deps/mid360_ws`
- Runs: `/tmp/bds-nominal-006-1788674481`, `/tmp/bds-nominal-007-1788674849`

## Metric Audit

The initial ~1.34 m ATE values are not valid absolute localization evidence.
The evaluator was matching against an all-zero truth stream: the C++ MID360
bridge subscribed to its default `simple_apm_rgbd_mid360` pose topic while the
campaign world was `low_indoor_apm_rgbd_mid360`. The estimate moved to the
actual rectangle (about x=2.0 m, y=1.2 m, z=2.1 m), while the recorded truth
remained zero. The bridge launch now passes the selected `WORLD_NAME` and
`gazebo_model` explicitly (`755d345`). Existing ATE results must therefore be
recomputed from a successful post-fix run; they are not used as a 20 cm pass
claim.

The first post-fix nominal smoke (`/tmp/bds-postfix-nominal-001-1788691200`)
recorded non-zero truth over the full rectangle. Using the offline
`evaluate_lio_phases.py` scorer, which estimates one yaw-plus-translation
transform from samples before the event window, XY error was 0.0189 m RMSE,
0.0326 m P95 and 0.0474 m max; Z error was 0.0222 m RMSE, 0.0320 m P95 and
0.0623 m max. These are valid nominal evidence; no fault-period score is
claimed from this run.

The first post-fix dual-medium run (`/tmp/bds-postfix-dual-medium-001-1788692000`)
also has non-zero truth and completed the route, but its fault windows were
not concurrent: GNSS outage was active at 33.533--48.483 s and LiDAR
correspondence dropout began at 50.700 s. It is therefore a sequential
GNSS-then-LiDAR run, not evidence for the concurrent dual condition. With a
single pre-event yaw/translation alignment, raw XY RMSE was 0.0215 m (P95
0.0364 m, max 0.0495 m); aligned DURING XY RMSE was 0.0155 m and aligned
DURING Z RMSE was 0.0225 m. No aligned XY sample exceeded 0.20 m after the
first event. The phase scorer now records raw XY/Z and incremental drift
without refitting the trajectory.

## Real Gazebo/SITL Nominal Runs

Both runs used the repository `run_pr6_d435i_visual_headless.sh` entry,
`low_indoor_apm_rgbd_mid360.sdf`, Gazebo headless rendering, ArduPilot SITL,
MID360 direct Livox bridge, and the guided small rectangle. Each run used a
unique `ROS_DOMAIN_ID`, and `rmw_fastrtps_cpp` because the restored host's
CycloneDDS participant limit was exhausted by stale processes.

| Run | ROS domain | Result | ATE RMSE (m) | RPE translation RMSE (m) | Native LiDAR | IMU factors | GNSS factors | Rollbacks |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| nominal-006 | 75 | takeoff, 4/4 waypoints, LAND, disarm | 1.3705 | 0.0368 | 619 | 618 | 310 | 0 |
| nominal-007 | 76 | takeoff, 4/4 waypoints, LAND, disarm | 1.3645 | 0.0389 | 621 | 620 | 293 | 0 |

GNSS-outage profile (15 s outage beginning at source time 20 s) was run three
times. Two valid trials completed the route; one attempt was an
`ENV_START_FAILURE` before `/clock` became available.

| Run | Result | ATE RMSE (m) | RPE translation RMSE (m) | Native LiDAR | IMU factors | GNSS factors | Rollbacks |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| gnss-outage-001 | takeoff, 4/4 waypoints, LAND, disarm | 1.3800 | 0.0352 | 622 | 621 | 224 | 0 |
| gnss-outage-002 | ENV_START_FAILURE (`/clock` timeout) | - | - | - | - | - | - |
| gnss-outage-003 | takeoff, 4/4 waypoints, LAND, disarm | 1.3531 | 0.0385 | 618 | 617 | 233 | 0 |

The runtime evidence reports LiDAR at about 10 Hz, IMU at about 10x the LiDAR
factor cadence, output source-age P95 of 30 ms, zero backend queue overflow,
zero optimization errors/rejections/rollbacks, and feature repeatability
median about 99.8%. The route logs contain the actual climb confirmation,
mission phases, LAND command, and confirmed motor disarm.

## Concurrent BDS + LiDAR Replay Runs

Three valid `lidar_medium` plus 15 s GNSS outage trials completed the Gazebo/SITL
small rectangle (takeoff, 4/4 waypoints, LAND, disarm) with zero rollback,
optimization rejection and queue overflow:

| Run | ATE RMSE (m) | RPE RMSE (m) | LiDAR | IMU | GNSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| dual-medium-007 | 1.3676 | 0.0387 | 626 | 625 | 237 |
| dual-medium-009 | 1.3494 | 0.0411 | 624 | 623 | 211 |
| dual-medium-011 | 1.3549 | 0.0406 | 614 | 613 | 213 |

Two valid `lidar_light` trials also completed (ATE 1.3700 m and 1.3639 m);
one additional attempt failed before `/clock` and is excluded.

`lidar_heavy` completed two valid trials (ATE 1.3655 m and 1.3535 m); a third
attempt failed before `/clock` and is excluded. The valid heavy runs likewise
completed the route with zero rollback, rejection and queue overflow.

## Fault Profiles

The repository's deterministic `robustness_v3_profiles.yaml` defines the
required `lidar_light`, `lidar_medium`, `lidar_heavy`,
`gnss_denial_light/medium/heavy`, and `dual_lidar_gnss_medium` profiles. The
profile parser/injector tests pass (9/9). However, the formal PR6 Gazebo launch
The native-factor replay route is explicit and single-publisher: FAST-LIO emits
`/robustness/raw/native_lidar_factor`, the selected-channel injector emits
`/fast_lio/native_lidar_factor`, and the backend consumes only the latter.
Remaining light/heavy, LiDAR-stop, total-MID360-fault and manual recovery trials
remain pending.

## Environment Findings

Initial attempts were invalidated by stale Gazebo/ROS processes occupying UDP
9002 and exhausting CycloneDDS participant slots. Their logs are retained in
`/tmp/bds-nominal-002-1788673889` through `...005...`. The lifecycle fix now
records PID start ticks, checks `/proc/<pid>/cmdline`, validates project-owned
commands before signaling, and removes the run SITL PID file. Nominal runs
were then repeated successfully with Fast DDS.

## Verification

- Full `colcon build --symlink-install`: 22/22 packages.
- Full `colcon test --return-code-on-test-failure`: 268 tests, 0 errors,
  0 failures, 0 skipped.
- Robustness profile tests: 9/9 passed.
- Shell syntax and `git diff --check`: passed.
- No hardware, flight-controller, or field validation was performed.

## Status

`DO_NOT_PROMOTE`: medium concurrent trials are repeatable, but the complete
light/medium/heavy matrix, sensor-stop boundaries and manual recovery remain
unexecuted. Startup `/clock` failures were traced to stale concurrent trial
processes; cleanup now scopes and converges exact run-owned processes. The
full test suite passes when the external Livox workspace is sourced (`268/268`).
The post-truth-binding smoke reached sensor and backend readiness but exited
before a complete route, so no post-fix non-zero truth trajectory is claimed.
The later smoke reached `/clock`, LiDAR, IMU, NativeLidarFactor and unified
odom, but stopped at `/mavros/global_position/raw/fix`: MAVROS connected
intermittently while the SITL telemetry request did not produce a NavSatFix.
That is an environment/startup boundary, not a valid localization trial. The
early cleanup path is guarded by `f5eac4f`; the full suite remains `268/268`.
No tag was created.

The corrected concurrent-profile attempt
(`/tmp/bds-postfix-dual-sync-medium-001-1788694000`) stopped at backend launch
because an empty `relocalization_database_path:=` argument is invalid. A retry
with an explicit temporary database
(`/tmp/bds-postfix-dual-sync-medium-002-1788694300`) reached sensor-stack
startup but failed the `/clock` readiness gate; its bridge log shows the
correct world clock before teardown. Both are `ENV_START_FAILURE`s, not
algorithm trials.

Before the injector contract correction, a third post-fix run
(`/tmp/bds-postfix-dual-sync-medium-003-1788695200`) completed takeoff,
4/4 waypoints, LAND and disarm. Its trajectory had 521 matched poses and
legacy whole-trajectory ATE RMSE 0.02035 m (max 0.05535 m; RPE translation
RMSE 0.01951 m). GNSS was active at 46.987--58.937 s and LiDAR dropout at
56.300--80.200 s, now with the intended shared origin (12.0 s windows; overlap
is the configured interval after the common start). Runtime recorded 686
native LiDAR and 685 IMU factors, 611 GNSS factors, 685 flow attempts/150
enabled, zero optimization rejects/rollbacks and zero worker queue overflow.
Its GNSS and LiDAR windows were still channel-relative (GNSS 46.987--58.937 s;
LiDAR 56.300--80.200 s), so it is not concurrent dual evidence. The subsequent
`22be899` fix synchronizes included channels; a fresh post-fix concurrent
campaign remains pending after the next two startup failures. The raw evidence
and phase-scored trajectory remain under the run directory.
