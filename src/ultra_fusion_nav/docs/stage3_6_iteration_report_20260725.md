# Stage 3-6 Iteration Report

Date: 2026-07-25

## Scope

This iteration validates the paper-aligned reliability gate and the LIO input
path in the simple Gazebo world. It does not claim a unified sliding-window
backend yet. All runs used the simple map, FCU HIGHRES_IMU for FAST-LIO, and
Gazebo truth only for evaluation.

## Verified software changes

- ReliabilityScheduler now treats missing optional factors independently. A
  valid GNSS/LIO path can remain `DEGRADED`; only the all-active-missing case is
  `FAILSAFE`.
- Eq. (15) minimum observation counts are carried in `ReliabilityScore.msg` and
  gate each factor before weighting.
- `FASTLIO_INPUT_MODE=filtered_pointcloud` selects a separate configuration whose
  `lid_topic` is `/sensors/lidar/points`. This topic contains body removal and
  fault injection. The default `pointcloud` mode remains unchanged.
- `run_lio_baseline_experiment.sh` records `simulation_performance.json` by
  default. Set `ENABLE_PERFORMANCE_MONITOR=0` to disable it. A reduced profile
  can record deliberately missing modalities with `record_reliability_scores.py
  --allow-missing`.

## Component verification

The affected workspace rebuilt successfully. The current component and ROS node
tests pass: 3 `uf_aiding`, 6 `uf_lio_adapter`, 22 `uf_reliability`, 14
`uf_sensor_pipeline`, and 9 `multi_slam_uav_sim` tests (54 total).

## Fixed-route evidence

| Run | LIO input | Position RMSE | Yaw RMSE | Yaw/gyro corr. | RTF median | Gate |
|---|---|---:|---:|---:|---:|---|
| `uf_stage3_scheduler_eq15_repeat_20260725` | raw | 0.109 m | 0.638 deg | 0.571 | not recorded | analyzer failed only on corr. |
| `uf_stage3_scheduler_eq15_filtered_20260725` | body-filtered | 0.088 m | 0.819 deg | 0.707 | not recorded | passed |
| `uf_stage3_perf_filtered_20260725` | body-filtered | 0.040 m | 0.139 deg | 0.789 | 0.032 | LIO passed; sim too slow |
| `uf_stage3_perf_filtered_nod435_20260725` | body-filtered | 0.040 m | 0.107 deg | 0.914 | 0.928 | passed |

All successful runs reported zero raw/registered/IMU timestamp regressions and
completed the rectangle and landing. The first failed run in the table is not a
trajectory divergence; its only failed criterion is the yaw/gyro correlation.

## Runtime conclusion

The 640x480@30 Hz D435 bridge is the dominant live-simulation load in this
configuration. Turning off only the D435 ROS bridge raised RTF from 0.032 to
0.928 and increased flow from about 13.3 to 19.5 Hz. This is a bridge/CPU load
effect, not evidence that the LIO equations are unstable. The D435 bridge should
remain enabled for visual experiments, but LIO/scheduler regressions should use
the explicit lightweight profile until the camera model is rate-limited or
rendering is reduced.

## Boundary and next step

The online path now has independent sensor scores, Eq. (15) gating, factor
enable/weight/inflation decisions, and a stable body-filtered LIO front end. IMU
preintegration residuals, RGB-D reprojection residuals, and the unified
sliding-window state are still incomplete. The next algorithm milestone is a
small offline weighted window using recorded `/lio/odom`, GNSS, optical-flow and
reliability streams, followed by fixed-weight versus scheduler-weighted replay.
