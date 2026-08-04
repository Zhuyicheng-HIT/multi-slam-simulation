# Repository and Environment Audit

Audit date: 2026-07-12

## 1. Workspaces and Git State

| Purpose | Path | State |
|---|---|---|
| Runtime experiments and historical logs | `$HOME/multi-slam` | Not a Git worktree; contains local build/install/log data |
| GitHub publication worktree | `$HOME/multi-slam-github-staging` | Clean `main` at `6d878b7`, tracking `origin/main` |
| Target remote | `https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git` | Fetch/push configured; `gh` authenticated as `Zhuyicheng-HIT` |
| Existing LiDAR dependency workspace | `$HOME/mid360_flight_ws` | Symlink to a local extracted workspace; build works but source directories have no Git metadata |

Algorithm code belongs in the publication worktree. Runtime bags, logs, maps, and build products remain ignored. No algorithm code should be developed in the non-Git runtime directory and copied back without a diff review.

No repository-level `AGENTS.md` was found.

## 2. Confirmed Environment

| Component | Confirmed version/state |
|---|---|
| OS | Ubuntu 22.04.5 LTS under WSL2 |
| ROS | ROS 2 Humble, `ros-humble-ros-base` 0.10.0 |
| MAVROS | 2.14.0 |
| Gazebo | Harmonic metapackage 1.0.0 |
| rosbag2 | 0.15.16; `sqlite3` storage plugin available |
| MCAP storage | Not installed |
| Compiler | GCC 11.4.0 |
| CMake | 3.22.1 |
| Python | 3.10.12 |
| Eigen / PCL / OpenCV | 3.4.0 / 1.12.1 / 4.5.4 |
| Ceres | Not installed; Ubuntu candidate is 2.0.0 |
| GTSAM | Not installed and no Jammy candidate was found in the configured Ubuntu repositories |
| Free WSL filesystem space | About 924 GiB at audit time |

## 3. Existing Repository Capabilities

The repository currently owns three ROS 2 packages:

- `multi_slam_uav_sim`: Gazebo/ArduPilot/MAVROS integration, D435i bridge, MID360 bridge, optical flow, flight-state bridge, and automated rectangle flight.
- `mid360_reliable_mapper`: FAST-LIO registered-cloud filtering and occupancy-map publication.
- `multi_slam_worlds`: project worlds and models.

The simulation already publishes the main data needed for the sensor-layer phase:

- MID360: `/sim/mid360/points_raw`, `/sim/mid360/cloud_registered`, and evaluator-only `/sim/mid360/ground_truth_odom`.
- FCU IMU: `/mavros/imu/data_raw`, bridged to `/livox/imu` with the source timestamp preserved except for a one-nanosecond monotonic repair.
- GNSS: `/mavros/global_position/raw/fix` and normalized `/uav/global_fix`.
- Optical flow: `/sim/optical_flow/raw`, `/sim/optical_flow/rad`, and `/sim/optical_flow/range`.
- RGB-D: color, depth, aligned depth, point cloud, individual accel/gyro, and combined D435i IMU topics.
- FCU comparison signals: `/uav/state`, `/uav/local_pose`, `/uav/local_odom`, `/uav/velocity`, and `/uav/interface_status`.

The repository does not currently contain a rosbag2 recording profile or replay regression test.

## 4. LiDAR-IMU Dependency Audit

`dependencies.repos` pins FAST-LIO commit `a4743b095409588842a5b30ddfa27e29d2f99164`. That commit is the ROS2 merge commit from the upstream repository, but the working local source identifies itself as the `Ericsii/FAST_LIO_ROS2` fork and is an `ament_cmake` package. Because the local extracted source has no `.git` directory, its exact correspondence to the pinned commit cannot currently be proven from the machine state alone.

The launch script defaults to `$HOME/multi-slam-deps/mid360_ws`, but that path does not exist on this machine. Successful runs require the explicit override:

```bash
LIDAR_WS="$HOME/mid360_flight_ws"
```

This is a clean-install reproducibility gap, not a localization tuning issue.

The default FAST-LIO path consumes `/sim/mid360/points_raw` as `PointCloud2`. The Gazebo bridge writes a `time` field by spreading horizontal samples over 0.1 s, while the FAST-LIO configuration comments describe the Gazebo cloud as an instantaneous snapshot. One of these models must be selected and tested before deskew or a future `LidarPlaneFactor` can be considered physically meaningful.

FAST-LIO currently publishes pose and registered clouds but does not expose the scan-to-map match count, point-to-plane residual distribution, Jacobians/Hessian, or local plane support required by `D_L`. Stage 2 therefore needs a small, version-pinned instrumentation patch or an external diagnostic front end.

## 5. Pre-Change Baseline

The required clean-main baseline was run headless using the existing GPS rectangle state machine and a 125 s analyzer window. Raw outputs are local and ignored by Git:

```text
logs/ultra_fusion_baseline_20260712_225309/
  baseline.json
  sim/
  lio/
  rectangle/
```

| Metric | 2026-07-12 clean-main result | Previous stated reference |
|---|---:|---:|
| Position RMSE | 0.111 m | about 0.196 m |
| Max position error | 0.366 m | about 0.522 m |
| Final position error | 0.025 m | not supplied |
| Yaw RMSE | 1.38 deg | about 7.31 deg |
| Max yaw error | 10.18 deg | about 13.69 deg |
| Final yaw error | 0.013 deg | not supplied |
| Estimated FCU IMU lag | 60 ms | about 40 ms |
| FAST-LIO yaw vs FCU gyro correlation | 0.262 | about 0.695 |
| Raw cloud timestamp regressions / duplicates | 0 / 0 | regressions 0 |
| Registered cloud timestamp regressions / duplicates | 0 / 12 | regressions 0 |
| IMU timestamp regressions / duplicates | 0 / 0 | regressions 0 |
| Registered-cloud voxel overlap median | 0.532 | not supplied |
| Registered-cloud centroid jump P95 / max | 3.01 m / 6.21 m | not supplied |

The analyzer returned `passed=false` because yaw/gyro correlation was below 0.65. The smaller ATE and yaw error do not justify ignoring that failure. The next iteration must first distinguish an estimator defect from a metric/time-association defect and establish run-to-run repeatability.

A second unchanged run after the documentation-only build also returned `passed=false`:

| Repeatability metric | Run 1 | Run 2 | Run 3 |
|---|---:|---:|---:|
| Position RMSE | 0.111 m | 0.159 m | 0.045 m |
| Yaw RMSE | 1.38 deg | 2.60 deg | 0.124 deg |
| FAST-LIO yaw vs FCU gyro correlation | 0.262 | 0.535 | 0.524 |
| Truth yaw vs FCU gyro correlation | 0.665 | 0.699 | 0.515 |
| Estimated FCU IMU lag | 60 ms | 20 ms | 100 ms |
| Registered timestamp duplicates | 12 | 9 | 13 |
| Centroid jump P95 | 3.01 m | 2.91 m | 2.98 m |
| Centroid jump max | 6.21 m | 10.02 m | 5.97 m |

All three runs failed the 0.65 FAST-LIO yaw/gyro correlation threshold. The centroid-jump P95 is repeatable within about 0.10 m and registered timestamp duplicates persist, so both are real diagnostic targets. In contrast, the estimated lag spans 20-100 ms, FAST-LIO correlation spans 0.262-0.535, and even truth-yaw/gyro correlation spans 0.515-0.699. The current correlation/lag association metric is therefore not repeatable enough for parameter acceptance and must be instrumented before estimator tuning.

The same flight produced an optical-flow comparison with 289 accepted samples, RMSE 0.205 m/s, MAE 0.160 m/s, `corr_x=0.027`, and `corr_y=0.068`. This is not yet a valid optical-flow localization baseline; the low correlation requires calibration/metric review before defining normal `D_OF` thresholds.

## 6. Missing or Simulation-Limited Information

### Blocking Before Stage 1 Acceptance

1. A repeatable clean-install FAST-LIO ROS2 source URL, commit, patches, and default `LIDAR_WS` path.
2. A canonical TF tree and numeric extrinsics for body, FCU IMU, MID360, front D435i, and downward optical-flow camera.
3. A consistent LiDAR acquisition-time model: instantaneous GPU snapshot or simulated rotating scan, not both.
4. A parameterized body/self-return exclusion volume in the LiDAR sensor frame.
5. A rosbag2 topic/QoS profile plus record, replay, and topic-rate regression scripts.
6. GNSS metadata beyond `NavSatFix`: satellite count, DOP, detailed fix type, outage state, and fault labels.
7. The exact source and licensing of the intended YOLO segmentation model and weights.

### Cannot Be Claimed From the Current Simulator

- Gazebo NavSat provides generic GNSS position, not a BDS RF chain. Real constellation geometry, ephemeris, carrier phase, ionosphere/troposphere, receiver clock behavior, and urban multipath are not modeled.
- Satellite count and DOP can be generated as controlled integrity metadata, but this validates scheduler logic rather than real BDS integrity performance.
- PPS/PTP hardware synchronization cannot be reproduced physically in WSL simulation. Only timestamp offsets, drift, jitter, and lock-state interfaces can be emulated.
- A GPU LiDAR frame cannot prove a real MID360 firing pattern or motion distortion unless the scan-time model is explicitly simulated.

### Available but Not Yet Implemented

- Sensor fault injection for time offset, extrinsic perturbation, noise, outage, jump, point dropout, depth holes, and low-texture optical flow.
- Independent reliability scores and their validation plots.
- Static/dynamic cloud separation and feature-repeatability measurement.
- ReliabilityScheduler, unified offline backend, relocalization, and automated ablations.

## 7. Stage 0 Verdict

The machine and repository are sufficient to start the simulation-only program. The work is not blocked by compute, ROS, Gazebo, MAVROS, or Git access. It is blocked from claiming reproducibility until the external FAST-LIO source/path and rosbag profile are fixed, and blocked from claiming a stable LIO baseline until the failed correlation/timestamp/jump metrics are explained or corrected by a single-variable iteration.

## 8. 2026-07-17 Closure Update

The FAST-LIO reproducibility blocker is closed for the simple-map baseline. A clean, manifest-pinned workspace now exists at `$HOME/multi-slam-deps/mid360_ws`; FAST-LIO, its `ikd-Tree` submodule, and Livox ROS2 all build from recorded immutable commits. Three consecutive unchanged fixed-route runs passed, and a fourth run passed after a reliability-only change. Online position RMSE was `0.043-0.056 m`, yaw RMSE was `0.145-0.176 deg`, TUM ATE was `0.053-0.079 m`, and all recorded timestamp regressions were zero.

The earlier extracted `$HOME/mid360_flight_ws` produced two large-drift failures and one near-pass after WSL restart. It is not accepted for future milestone claims. This does not prove the source-tree difference caused the drift; it establishes that only the clean pinned workspace has met the repeatability gate.

Stage 1 rosbag2 infrastructure and Stage 3 scoring now exist. Remaining simulator limitations from Section 6 still apply, especially generic GNSS versus real BDS, lack of hardware PPS/PTP, approximate LiDAR timing, and unavailable true FAST-LIO internal Hessian.
