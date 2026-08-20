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

## Current interpretation

This run validates the large-scene launcher and identifies a scene ownership
bug, but it is only screening evidence. It is not a stable accuracy release:
the city-static P95/max gates remain red, and no dynamic or relocalization
profile has yet completed.

The existing `TemporalVoxelFilter` produces dynamic/static diagnostics and a
filtered cloud for downstream consumers. It does not remove dynamic points
from FAST-LIO's internal matching and map update path. Dynamic profile results
must therefore not be described as FAST-LIO map-level dynamic-object removal.

## Next controlled runs

1. Re-run `city_static` with resource sampling to establish the resource and
   repeatability baseline.
2. Run `city_dynamic` with the identical route and compare error tails,
   reliability, point-cloud repeatability, and process load.
3. Run `city_dynamic_relocalization` at checkpoints 8 and 16, requiring clean
   transaction, epoch, native queue, latency, and post-reset integrity evidence.
4. Repeat static/dynamic/relocalization in the long repetitive tunnel.
5. Change at most one core variable per A/B. Retain each failed run and use the
   campaign tag for rollback if a candidate regresses.

## Verification

- `colcon build --symlink-install --packages-select multi_slam_uav_sim`: pass.
- `colcon test --packages-select multi_slam_uav_sim`: 151 tests pass.
- Focused world and runner contracts: 34 tests pass.
- Shell syntax, Python compile, and `git diff --check`: pass.
