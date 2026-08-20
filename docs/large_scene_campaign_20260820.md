# Large-scene dynamic-object and relocalization campaign

Date: 2026-08-20

Branch: `feat/core-algorithm-cleanup-20260817`

Campaign rollback point: `checkpoint/large-scene-campaign-start-20260820`
(`23feb7d`)

## Scope and frozen boundaries

The campaign uses the frozen five-source ExternalNav stack. Algorithm weights,
factor ownership, fixed sensor extrinsics, and shadow-only time calibration are
unchanged. The MID360 bridge removes returns inside the body-centred
`0.50 m x 0.50 m x 0.10 m` aircraft envelope before FAST-LIO receives them.

The matrix runner is `tools/run_large_scene_validation.sh`. Its profiles are:

| Environment | Static | Dynamic | Dynamic + relocalization |
| --- | --- | --- | --- |
| Outdoor city, 54.52 m figure eight | `city_static` | `city_dynamic` | `city_dynamic_relocalization` |
| 96 m repetitive indoor tunnel, >=140 m route | `tunnel_static` | `tunnel_dynamic` | `tunnel_dynamic_relocalization` |

Every profile performs one route, records a replay bag, and stops observers
after confirmed landing and disarm. A route failure also stops all collectors,
while preserving partial evidence. CPU, RSS, system memory, and global NVIDIA
GPU load are sampled into `resource_metrics.json` and `resource_samples.csv`.

## Infrastructure failure baseline

Evidence: `logs/large_scene_city_static_baseline_20260820`

The first city run was rejected at route distance 8 m after EKF variance caused
an automatic LAND. At 66 s simulation time, unified causal 3D RMSE was about
`1.00 m`; FAST-LIO RMSE was about `6.50 m`. The world contained the full
`iris_apm_rgbd` sensor rig plus three fixed `sensor_only` models. Both the
aircraft MID360 and a fixed world MID360 published `/mid360/lidar`, producing an
impossible interleaved scan stream at roughly 20 Hz. The run is invalid as an
algorithm baseline, but is retained as causal infrastructure evidence.

The single correction was removal of the redundant fixed sensors from
`apm_city_rgbd_mid360.sdf`. A contract test now requires one full aircraft rig
and no `sensor_only` includes. The corrected stream had exactly one publisher
and approximately 10 Hz MID360 / native LiDAR factors.

## Corrected city-static screening baseline

Evidence: `logs/large_scene_city_static_sensor_rig_fix_20260820`

Commit under test: `23feb7d` plus the campaign infrastructure changes in this
report. Dynamic agents and explicit relocalization triggers were disabled.

| Result | Observed |
| --- | ---: |
| Takeoff / route / landing / disarm | pass / pass / pass / pass |
| Planned route / checkpoints | 54.52 m / 27 |
| Maximum displacement | 12.93 m |
| Causal 3D RMSE / P95 / max | 0.1201 / 0.2112 / 0.5429 m |
| Causal XY RMSE | 0.0993 m |
| Causal Z RMSE | 0.0676 m |
| Endpoint error | 0.1144 m |
| Solver P50 / P95 / max | 5.60 / 35.32 / 79.85 ms |
| Backend callback P50 / P95 / max | 31.55 / 83.29 / 101.45 ms |
| Unified output age P50 / P95 / max | 34 / 74 / 133 ms |
| ExternalNav rate / age P95 | 20.0 Hz / 1 ms |
| Observed RTF | 0.370 |

The route contract and all graph-integrity gates passed. EKF3 IMU0 and IMU1
both reported that they consumed external navigation. ExternalNav had zero
duplicate, regressing, zero, or stale stamps. The strict acceptance gate still
failed because causal P95 exceeded 0.20 m by 11 mm and the maximum exceeded
0.20 m. The largest transient occurred at simulation stamp 147.51 s near the
upper turn of the second lobe (truth approximately
`[-12.06, 0.75, 4.65] m`). It recovered before route completion; this is not a
claim that the tail error is acceptable.

Latest factor counters were:

| Source | Received / attempted | Accepted | Rejected or superseded |
| --- | ---: | ---: | ---: |
| Native LiDAR | 1774 | 1774 | 0 queue drops / overflows |
| IMU | 17914 | 1772 factors | 1 invalid, 1 pair timeout |
| GNSS | 891 / 887 | 886 | 4 superseded, 0 NIS rejects |
| Optical flow | 2252 / 1773 | 1310 | 394 scheduler-disabled, 56 quality-disabled |
| RGB-D direct | 557 / 542 | 493 | 47 prefit, 2 track, 12 superseded |

Source rates were IMU 99.97 Hz, GNSS 5.00 Hz, optical flow 12.57 Hz, RGB and
depth about 15.1 Hz, and native LiDAR about 10 Hz. No source stamp regressions
or duplicates were observed. Resource sampling was added after this screening
run, so per-process CPU/RSS and GPU numbers are intentionally reported as not
captured for this run rather than reconstructed after the fact.

## Repeatability and dynamic screening

The corrected static run was repeated with resource sampling at
`logs/large_scene_city_static_repeat_resource_20260820`. The dynamic screening
run is `logs/large_scene_city_dynamic_screening_20260820`; it used three
pedestrians at `0.75 m/s` and three cars at `1.6 m/s`. Both completed takeoff,
the 54.52 m route, landing, and disarm.

| Result | Static repeat | Dynamic screening |
| --- | ---: | ---: |
| Causal 3D RMSE / P95 / max | 0.1500 / 0.2851 / 0.7226 m | 0.0992 / 0.1581 / 0.4084 m |
| Causal XY / Z RMSE | 0.1346 / 0.0662 m | 0.0802 / 0.0585 m |
| Endpoint error | 0.1217 m | 0.0860 m |
| Solver P50 / P95 / max | 5.76 / 34.83 / 209.40 ms | 5.70 / 40.39 / 173.06 ms |
| Callback P50 / P95 / max | 32.81 / 76.14 / 355.12 ms | 32.63 / 84.65 / 333.21 ms |
| Validation RSS P50 / max | 3973 / 4114 MiB | about 4.0 / 4.1 GiB |

The static repeat is materially worse than the first corrected static run, so
the lower error in the single dynamic run cannot be attributed to dynamic
filtering. The existing temporal voxel filter reported scene diagnostics but
did not own FAST-LIO matching or map insertion. These two runs establish
screening and repeatability evidence only.

## Relocalization failure and candidate safety gate

Evidence: `logs/large_scene_city_dynamic_relocalization_screening_20260820`

At checkpoint 8 the database contained six keyframes. Candidate 4 passed the
existing descriptor, registration, cycle, and absolute alignment checks, then
committed a false epoch. The applied correction was `9.9703 m` and
`1.5681 rad`; unified odometry stepped by the same 9.97 m with approximately a
90 degree yaw jump. The backend subsequently recorded four optimization
rollbacks, 26 native queue discards/supersessions, a 3.018 s maximum IMU gap,
an EKF lane switch, and variance failsafe LAND at route distance 16 m. This run
is a failed relocalization experiment, not a completed capability claim.

A single-variable candidate now checks the proposed `map_from_lio` epoch
against the time-aligned current epoch before publishing success or reset.
Manual/active relocalization is bounded to `1.0 m` translation and `0.5 rad`
rotation; routine automatic loop closure retains its existing stricter bounds.
The observed 9.9703 m / 1.5681 rad false candidate is rejected by unit tests.
The fix is pushed as a review candidate but is not a stable campaign baseline
until a complete runtime run proves that the false epoch is rejected without
breaking the route or EKF3 consumption.

## Current runtime blocker

Three new attempts on 2026-08-20 did not reach a relocalization checkpoint:

| Evidence directory | Outcome |
| --- | --- |
| `large_scene_city_dynamic_relocalization_epoch_gate_20260820` | Aborted before arming: no continuous unified trajectory in the readiness window; observed RTF 0.025 |
| `large_scene_city_dynamic_relocalization_epoch_gate_retry_20260820` | GUIDED and armed, first takeoff rejected, then MAVROS/unified odometry became stale and FCU disarmed |
| `large_scene_tunnel_static_baseline_20260820` | Navigation ready, then MAVROS disconnected and unified odometry became stale before GUIDED confirmation |

Memory and disk had ample headroom. The renderer was the expected WSLg D3D12
NVIDIA OGRE2 path, but validation CPU dropped to roughly 200--500 percent
during stalls versus roughly 1300--1500 percent in successful city runs. Global
GPU utilization was variable and included a host-side PID not visible inside
this WSL distribution. The matching failure in both city and tunnel means this
is currently classified as an external simulation/runtime blocker, not an
algorithm or scene regression. No Windows or other WSL distribution was
accessed. An attempted software-render diagnostic was rejected by the frozen
GPU requirement before Gazebo started and is not counted as a scene run.

Validation now stops the relocalization trigger immediately after route
termination, writes `interrupted_after_route_end`, and records
`/lio/local_map` plus `/lidar/points_deskewed` so future bags can recompute the
actual relocalization candidate. Collectors still stop after landing/disarm.
Large-scene profiles also require a preflight wall/source rate ratio of at
least `0.12` on the 20 Hz ExternalNav stream. A slower simulation now exits
before GUIDED/arming instead of spending a route attempt on stale MAVLink and
estimator traffic. The ordinary frozen validator leaves this gate disabled.

## Current interpretation

The campaign validates the large-scene launcher, one static repeat, and one
dynamic screening route. It is not a stable accuracy release: static
repeatability is weak, maximum-error gates remain red, and no relocalization or
tunnel profile has completed.

The existing `TemporalVoxelFilter` produces dynamic/static diagnostics and a
filtered cloud for downstream consumers. It does not remove dynamic points
from FAST-LIO's internal matching and map update path. Dynamic profile results
must therefore not be described as FAST-LIO map-level dynamic-object removal.

## Next controlled runs

1. Restore stable Gazebo/MAVROS wall-time progress and prove a complete static
   takeoff/route/landing run before consuming more relocalization runs.
2. Run `city_dynamic_relocalization` at checkpoints 8 and 16, requiring clean
   transaction, epoch, native queue, latency, and post-reset integrity evidence.
3. Repeat static/dynamic/relocalization in the long repetitive tunnel.
4. Use the newly recorded relocalization point clouds for deterministic replay
   of candidate acceptance and rejection.
5. Change at most one core variable per A/B. Retain each failed run and use the
   campaign tag for rollback if a candidate regresses.

## Verification

- `colcon build --symlink-install --packages-select multi_slam_uav_sim`: pass.
- `colcon test --packages-select multi_slam_uav_sim`: 155 tests pass.
- `colcon test --packages-select uf_relocalization`: 8 test executables pass.
- Combined recorded result: 76 tests, zero errors or failures.
- Shell syntax, Python compile, and `git diff --check`: pass.
