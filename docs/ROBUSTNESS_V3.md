# Ultra-Fusion Robustness V3

Robustness V3 freezes the Performance V2 estimator and tests its failure
behavior from outside the fusion core.  It adds no estimator factor, changes no
integrity threshold, and never substitutes `/Odometry` for
`NativeLidarFactor`.

## Architecture

The frozen rosbag is replayed onto `/robustness/raw/*`.  The profile-driven
injector republishes canonical sensor/factor topics and matching reliability
evidence.  A fresh reliability scheduler produces the live scheduler state.
The backend then runs in either dynamic (`FRS ON`) or fixed unit-weight
(`FRS OFF`, A/B control only) mode.  Recorded scheduler states are isolated and
cannot leak into the experiment.

Fault timing always uses the source header clock.  A `time_offset` fault shifts
the source timestamp by its configured physical offset; no output is restamped
to make association pass.  All random selection uses the profile seed.  The
input bag, profile, FRS mode, output directory, metrics and resource record are
saved for every run.

The original capture's rosbag storage timestamps include WSL/Gazebo software-
rendering slowdown.  `tools/normalize_robustness_replay_clock.py` may create a
derived functional-test bag scheduled by the bag's recorded `/clock`.  It
copies every CDR payload unchanged and changes only storage receive times; it
does not rewrite any sensor timestamp.  Performance/RTF claims must continue
to use the original bag, while fault-boundary tests may use the normalized bag
to avoid spending time replaying a simulator bottleneck.

## Profiles and engineering levels

`uf_sensor_pipeline/config/robustness_v3_profiles.yaml` contains light, medium
and heavy visual, Native LiDAR, GNSS denial/jump, optical-flow and IMU faults;
camera/LiDAR time offsets; D435i/MID360 rotation and translation errors; double
faults; and an endurance cycle.  The values are explicit V3 engineering test
levels, not thresholds attributed to the paper.

Native LiDAR correspondence dropout reduces the raw point-plane rows delivered
to the backend.  LiDAR outages drop NativeLidarFactor messages.  No run enables
the pose fallback.  D435i calibration cases use the backend's documented
`visual_rotation_body_camera` and `visual_translation_body_camera_m`
parameters.  MID360 cases perturb the `T_body_sensor` carried by each native
factor.

The frozen replay contains recorded reliability evidence rather than every
front-end diagnostic input.  During an active injected fault, the adapter
raises the corresponding degradation evidence to the profile's declared
floor; the scheduler itself remains live and unchanged.  Detection behavior of
the sensor-specific monitors continues to be covered by their unit tests.  A
full Gazebo/hardware run should use the same injector before the live monitors
when raw diagnostic evidence is available.

## Commands

Build the new validation package and run its deterministic tests:

```bash
colcon build --packages-select uf_sensor_pipeline
colcon test --packages-select uf_sensor_pipeline --event-handlers console_direct+
colcon test-result --verbose
```

Run one frozen-input A/B pair:

```bash
PROFILE=visual_medium FRS_MODE=on tools/run_robustness_v3_replay.sh
PROFILE=visual_medium FRS_MODE=off tools/run_robustness_v3_replay.sh
```

Run matrices without selecting the best trial:

```bash
RUN_SET=singles tools/run_robustness_v3_matrix.sh
RUN_SET=calibration tools/run_robustness_v3_matrix.sh
RUN_SET=doubles tools/run_robustness_v3_matrix.sh
RUN_SET=endurance tools/run_robustness_v3_matrix.sh
```

`RUN_SET=all` executes all of the above.  `RUN_SET=smoke` is the bounded entry
check.  Outputs go only to `logs/tmp` and are ignored by Git.
Set `RESUME_EXISTING=1` to retain already-complete reports and run only missing
matrix entries.  The runner still validates each retained report before use.

## Recorded evidence

Each run records aligned ATE/RPE, trajectory span completeness, modality factor
counts and rejections, scheduler weights/scores, FRS switch and recovery times,
solver/callback latency, replay RTF proxy, backend CPU/RSS/page-fault/context-
switch counters, optimization errors, integrity rejects and rollbacks.  When a
capture has no same-run ground-truth stream, the matrix uses the frozen nominal
backend trajectory as its reference and labels the result as delta-ATE/RPE.  It
does not present cross-run simulator truth as an absolute accuracy result.

The matrix summary applies a declared V3 continuity criterion: at least 90%
route span, no unified-odom gap above 1.0 second, zero transaction/integrity failures, and aligned ATE no worse than
`max(2 * nominal, nominal + 0.10 m)`.  This criterion is labeled as an
engineering decision and is not presented as a paper result.

Joint-map stress remains a full-stack test because the frozen backend bag does
not contain RGB-D images or map updates.  Map evidence must include voxel count,
RGB coverage, supplementary volume, conflict ratio and ghosting proxy from the
existing source-aware mapper; replay metrics must not be used to claim map
stability.

Run the managed long-route joint-map check with:

```bash
tools/run_robustness_v3_joint_map_stress.sh
```

It reuses the existing lifecycle-safe headless harness, a 20 m by 12 m route,
the balanced paper-reprojection frontend, and the source-aware joint mapper.
Its ghosting proxy is explicitly the fraction of RGB-D updates classified as
geometry conflicts; it is not mislabeled as absolute surface ground truth.

The measured campaign outcome, including failed boundaries and the full-stack
joint-map failure, is recorded in `docs/ROBUSTNESS_V3_RESULTS.md`.
