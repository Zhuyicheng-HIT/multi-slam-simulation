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

`DO_NOT_PROMOTE`: the synchronized light, medium and heavy dual-degradation
sets now each have three valid completed Gazebo/SITL trials. Light and medium
remain below 0.20 m XY during the joint fault; heavy is not compliant because
all three repetitions contain a horizontal threshold crossing after roughly
0.9--10.4 s without both GNSS and NativeLidarFactor. One nominal post-truth
run is valid; additional nominal repetitions are still needed for a complete
baseline set. Sensor-stop trials are not valid because the current launch
does not connect the injector's IMU input. Manual relocalization and mission
recovery were not executed in this campaign. Visual rates/factors are
unmeasured because the visual bridge/frontend were explicitly disabled.
Startup `/clock` failures were retained as `ENV_START_FAILURE` evidence and
were not counted as algorithm trials. The full test suite passes when the
external Livox workspace is sourced (`268/268`). No tag was created.

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
56.300--80.200 s, leaving only about 2.64 s of overlap. Runtime recorded 686
native LiDAR and 685 IMU factors, 611 GNSS factors, 685 flow attempts/150
enabled, zero optimization rejects/rollbacks and zero worker queue overflow.
It is not concurrent dual evidence.

After the `22be899` shared-origin fix, the first valid synchronized trial
(`/tmp/bds-postfix-dual-sync-medium-fixed-002-1788697400`) completed takeoff,
4/4 waypoints, LAND and disarm. GNSS was active at 45.668--57.554 s and LiDAR
dropout at 45.700--69.700 s, providing the intended concurrent 12 s interval.
The fixed pre-event scorer reported XY RMSE 0.0189 m (P95 0.0352 m, max
0.0581 m) and Z RMSE 0.0242 m (P95 0.0343 m, max 0.0513 m). During the
concurrent interval, XY RMSE was 0.0223 m and Z RMSE was 0.0223 m; no aligned
XY sample exceeded 0.20 m. Runtime recorded 684 LiDAR, 683 IMU, 448 GNSS and
683 flow attempts, with zero optimization rejection, rollback or worker queue
overflow. The raw evidence and phase-scored trajectory remain under the run
directory.

A second post-fix synchronized dual-medium trial (`/tmp/bds-postfix-dual-sync-medium-fixed-004-1788700000`) also completed the full rectangle. GNSS was active at 46.095--58.045 s and LiDAR at 46.100--70.100 s. Fixed pre-event scoring gave all-trajectory XY RMSE 0.0178 m (P95 0.0325 m, max 0.0497 m), Z RMSE 0.0265 m (P95 0.0364 m, max 0.0757 m); concurrent DURING XY RMSE was 0.0208 m and Z RMSE 0.0273 m. No XY sample exceeded 0.20 m. Runtime recorded 691 LiDAR factors, 683 IMU factors, 622 GNSS factors, zero rollback and zero queue overflow.

A third post-fix attempt (`/tmp/bds-postfix-dual-sync-medium-fixed-005-1788701000`)
failed before `/clock` readiness and is excluded as `ENV_START_FAILURE`. At
that point the concurrent medium set had 2 valid trials; the clean retry below
supplied the third repetition.

The next clean retry (`/tmp/bds-postfix-dual-sync-medium-fixed-006-1788701500`)
completed the rectangle and supplied the third valid synchronized repetition:
GNSS active 48.254--60.203 s and LiDAR active 48.300--72.600 s. During the
concurrent interval the fixed pre-event scorer reported XY RMSE 0.0143 m,
P95 0.0288 m, max 0.0381 m and Z RMSE 0.0100 m, P95 0.0171 m, max 0.0539 m.
No XY sample exceeded 0.20 m; rollback and queue overflow were zero.

The post-fix medium repetition set is now complete at 3 valid trials: fixed
medium-002, -004 and -006. All three completed the route with overlapping
GNSS/LiDAR fault windows, XY DURING RMSE of 0.0223 m, 0.0208 m and 0.0143 m,
respectively, and no XY threshold violation or rollback. A first post-fix
LiDAR-heavy attempt (`/tmp/bds-postfix-lidar-heavy-fixed-001-1788703500`)
did not reach factor startup: the Livox ownership stability gate timed out
despite one publisher per topic. It is recorded as `ENV_START_FAILURE` and is
not counted as a heavy trial.

A first valid post-fix LiDAR-heavy trial (`/tmp/bds-postfix-lidar-heavy-fixed-003-1788705000`) completed the full route. During the 25 s outage, fixed pre-event scoring gave XY RMSE 0.0822 m, P95 0.2166 m, max 0.3414 m; Z RMSE 0.0208 m, P95 0.0266 m. XY first exceeded 0.20 m 8.29 s after outage start and recovered below it afterward; Z remained below 0.06 m. A second heavy trial (`/tmp/bds-postfix-lidar-heavy-fixed-004-1788706500`) completed as well, with DURING XY RMSE 0.0469 m, P95 0.1204 m, max 0.1546 m and Z RMSE 0.0228 m; no 0.20 m XY crossing. Both had zero rollback and queue overflow. Heavy therefore shows run-to-run XY sensitivity under LiDAR-only loss and is not yet a 3-trial set.

The third valid LiDAR-heavy trial (`/tmp/bds-postfix-lidar-heavy-fixed-006-1788709500`) completed the route with zero rollback. DURING XY RMSE was 0.0692 m, P95 0.1789 m and max 0.2129 m; Z RMSE was 0.0209 m. XY first exceeded 0.20 m 8.49 s after outage start. Across the three heavy trials, two exceeded 0.20 m and one did not; the repeatable failure mode is horizontal drift after roughly 8.4 s without NativeLidarFactor, while Z remains within 0.06 m.

Three delayed-profile GNSS-only medium trials completed the full route with enough pre-event trajectory for fixed alignment. Runs `gnss-medium-delayed-002`, `-003` and `-004` had DURING XY RMSE 0.0216 m, 0.0187 m and 0.0206 m, respectively; P95 stayed at 0.0317--0.0333 m and max at 0.0411--0.0481 m. Z DURING RMSE was 0.0098--0.0131 m. None crossed 0.20 m or rolled back. The earlier delayed-001 run completed but began recording about 9 s into the outage and is excluded from phase scoring.

Three full-window LiDAR-medium trials (`bds-postfix-lidar-medium-scored-001` through `-003`) completed the route. DURING XY RMSE was 0.0245, 0.0254 and 0.0246 m (P95 0.0395--0.0422 m; max 0.0502--0.0602 m), and Z RMSE was 0.0200, 0.0190 and 0.0188 m. None crossed 0.20 m or rolled back. The earlier late-window medium run had one `excessive_accel_bias_correction` rollback at stamp 95.0 s and is retained as an anomalous non-matrix run.

Three valid delayed `dual_lidar_gnss_light` trials completed the full rectangle:
`/tmp/bds-postfix-dual-light-scored-002-1788733000`,
`/tmp/bds-postfix-dual-light-scored-004-1788737000` and
`/tmp/bds-postfix-dual-light-scored-006-1788741000`. The shared source-time
fault intervals were verified in `runtime_evidence.json`; all three had zero
optimization errors/rejections, rollback and worker overflow. Fixed pre-event
scoring gave DURING XY RMSE of 0.0242, 0.0102 and 0.0167 m, with maxima
0.0485, 0.0218 and 0.0322 m; DURING Z RMSE was 0.0154, 0.0038 and 0.0189 m.
Attempts `-003` and `-005` failed at `/clock` readiness and are recorded as
`ENV_START_FAILURE`.

The first valid delayed `dual_lidar_gnss_heavy` trial
(`/tmp/bds-postfix-dual-heavy-scored-002-1788745000`) completed takeoff, 4/4
waypoints, LAND and disarm. Both GNSS and NativeLidarFactor outages were
active concurrently for the planned 25 s. DURING fixed-alignment XY RMSE was
0.0863 m, P95 0.2399 m and max 0.2961 m; Z RMSE was 0.0173 m, P95 0.0242 m
and max 0.0321 m. XY first crossed 0.20 m 0.88 s after the joint outage;
rollback, rejection and queue overflow stayed zero. Attempts `-001` and
`-003` were `/clock` startup failures and are excluded.

The third valid delayed `dual_lidar_gnss_heavy` trial
(`/tmp/bds-postfix-dual-heavy-scored-006-1788759000`) completed the route.
The joint outage was concurrent from approximately 59.8 s to 84.8 s. Fixed
pre-event scoring gave DURING XY RMSE 0.0809 m, P95 0.2349 m and max 0.3410 m;
Z RMSE was 0.0192 m, P95 0.0254 m and max 0.0387 m. The first aligned XY
sample above 0.20 m occurred 10.39 s after the outage and there were no
optimization errors, rejections, rollbacks or queue overflows. The heavy set
is therefore complete at three valid trials; all three show horizontal
threshold violations in at least one repetition, while Z remains below 0.06 m.

An external `lidar_imu_stop` profile was exercised once at
`/tmp/bds-postfix-lidar-imu-stop-002-1788749000`. It is not valid evidence for
simultaneous LiDAR/IMU loss: the launch supplied no publisher for the
injector's default `/robustness/raw/imu`, and `runtime_evidence.json` contains
LiDAR fault events but no IMU fault event. This identified a test-chain
contract gap; no physical MID360 IMU-stop conclusion was claimed. The gap is
now closed by the test-only `robustness_external_sensor_relay` launch switch.
With that switch enabled, the sensor relay writes IMU/GNSS/flow and raw Livox
CustomMsg into `/robustness/raw/*`; the robustness injector republishes the
canonical topics. The injector has an explicit `livox_lidar` CustomMsg channel
with RELIABLE QoS, so raw Livox can be stopped without a reliability mismatch.
The production default remains unchanged (`robustness_external_sensor_relay=false`),
and raw `/livox/lidar` and `/livox/imu` contracts are not altered.

The corresponding deterministic profiles are `lidar_cloud_outage` and
`mid360_total_outage`. A valid raw cloud-only replay
(`/tmp/bds-postfix-lidar-cloud-stop-002-1788712238`) completed the route with
LiDAR disabled while IMU continued: GNSS factors 607, IMU factors 608, native
LiDAR factors 495 before the outage, zero rollback and zero queue overflow. In
the 25 s event window the aligned XY RMSE/P95/max were 0.0174/0.0243/0.0258 m
and Z RMSE/P95/max were 0.0121/0.0164/0.0166 m. This is valid cloud-only
evidence; the post-event Z maximum includes the landing segment.

A valid raw total-outage replay
(`/tmp/bds-postfix-mid360-total-stop-002-1788712766`) delivered both verified
outage events (IMU and Livox at about 63.8 s), but the estimator then recorded
745 `ill_conditioned_latest_information` rejections and 745 rollbacks, with
only 32 committed states; the trajectory recorder stopped at the fault window
and the launcher returned 7. It is confirmed failure-mode evidence, not a
pass. A prior raw total-outage attempt was an environment startup failure, so
no claim of three total-outage repetitions is made.

The current scored runs use `VISUAL_BRIDGE_ENABLED=0` and
`VISUAL_FRONTEND_ENABLED=0`; visual frame/factor counts are therefore zero by
configuration. RGB/Depth/CameraInfo rates and visual factor adoption remain
unmeasured in this campaign and must not be inferred from these runs.

## Final Verification

The post-fix server verification completed a full 22/22-package build and
268/268 tests (0 errors, 0 failures, 0 skipped). Shell syntax and
`git diff --check` passed. The changes in this addendum are limited to
test-chain launch routing, raw CustomMsg fault injection, profiles and their
tests; no estimator, fusion weight, HXY, Z-axis, Dynamic or relocalization
algorithm was changed.

## Status

`DO_NOT_PROMOTE`: nominal, GNSS-medium, LiDAR-medium, dual-light and
dual-heavy have valid repetitions, but dual-heavy exceeds the 0.20 m XY
P95/max criterion, simultaneous IMU+LiDAR loss drives repeated ill-conditioned
rollback, visual/manual-relocalization coverage is absent, and the requested
full matrix repetitions are not all available. Intermittent `/clock` startup
failures are retained as `ENV_START_FAILURE` and excluded from metric trials.

## Continued Validation

The installed PR6 runner was exercised again from the frozen launch entry.
Two independent nominal repetitions completed the rectangle route and cleaned
their process groups:

| Run | ROS duration | Route | Native LiDAR | IMU | GNSS | Rollback | Queue overflow |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| `bds-final-nominal-db-1788715353` | 71.94 s | complete (`small_rectangle_exit=0`) | 750 | 749 | 375 | 0 | 0 |
| `bds-final-nominal-db-005-1788716175` | 73.86 s | complete (`small_rectangle_exit=0`) | 762 | 762 | 382 | 0 | 0 |

These runs used an explicit temporary relocalization database path and are
valid nominal runtime evidence. A third nominal attempt
(`bds-final-nominal-db-004-1788715722`) and a no-database attempt
(`bds-final-nominal-no-db-1788716567`) reached backend startup but failed the
intermittent `/clock` readiness gate; both are retained as `ENV_START_FAILURE`
and excluded from algorithm metrics.

The no-database launch failure exposed a test-entry contract defect: the
runner passed the empty token `relocalization_database_path:=`, which ROS 2
launch rejects before constructing the backend. The runner now appends that
argument only when `RELOCALIZATION_DATABASE_PATH` is non-empty. The default
production behavior is otherwise unchanged; a regression test checks that an
empty value is omitted. A subsequent explicit-database run completed, and the
package build plus test suite passed.

A third valid simultaneous MID360 total-outage replay
(`bds-final-mid360-total-003-1788716736`) verified both IMU and Livox outage
events in one source-time window. The route reached the fault boundary, then
the backend recorded 661 `ill_conditioned_latest_information` rejections and
661 rollbacks, with 168 committed states; the trajectory stopped at the
outage. This confirms the previous failure boundary and does not constitute a
pass. The three valid total-outage runs consistently show estimator loss of
progress when both raw MID360 channels disappear.
