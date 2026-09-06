# BDS + LiDAR Dual-Degradation Validation

## Baseline

- Branch: `feat/bds-lidar-dual-degradation-v1`
- Frozen algorithm baseline: `02935070bf7177efe6b07d01db9e2e6beac6f0dc`
- Validation commit: `6a51791e4b24e5f061b60de04ce706b239668e0e`
- External Livox workspace: `$HOME/multi-slam-deps/mid360_ws`
- Runs: `/tmp/bds-nominal-006-1788674481`, `/tmp/bds-nominal-007-1788674849`

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

The runtime evidence reports LiDAR at about 10 Hz, IMU at about 10x the LiDAR
factor cadence, output source-age P95 of 30 ms, zero backend queue overflow,
zero optimization errors/rejections/rollbacks, and feature repeatability
median about 99.8%. The route logs contain the actual climb confirmation,
mission phases, LAND command, and confirmed motor disarm.

## Fault Profiles

The repository's deterministic `robustness_v3_profiles.yaml` defines the
required `lidar_light`, `lidar_medium`, `lidar_heavy`,
`gnss_denial_light/medium/heavy`, and `dual_lidar_gnss_medium` profiles. The
profile parser/injector tests pass (9/9). However, the formal PR6 Gazebo launch
currently includes `sensor_pipeline.launch.py` without enabling the robustness
overlay or remapping its `/robustness/raw/*` inputs. Starting the injector on
top of the production topics would create duplicate publishers and violate the
one-observation/one-factor contract. Therefore no live BDS+LiDAR degraded run
is claimed yet.

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

`DO_NOT_PROMOTE`: nominal simulation is repeatable, but the requested
concurrent BDS/LiDAR degradation matrix and recovery runs remain unexecuted
until the existing robustness injector is wired into a single production-safe
replay/launch path. No tag was created.
