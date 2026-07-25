# Optical-Flow Sensor Report

Status: image-LK optical flow accepted on three repeated simple-map flights;
companion GPS/flow aiding is active through one ExternalNav output, while direct
FCU optical-flow injection remains disabled.

## Sensor Boundary

The estimator-facing observation is `/sensors/optical_flow/rad`. The Gazebo source publishes `/sim/optical_flow/rad` with MAVLink `OPTICAL_FLOW_RAD` semantics:

- `integrated_x/y`: raw angular image flow over one sensor integration window;
- `integrated_x/y/zgyro`: co-timed internal gyro integration in sensor FRD axes;
- `distance`: noisy downward single-beam range;
- `quality`: image-derived texture, LK survival, forward/backward consistency, spatial coverage, range, and motion-limit score.

The default ExternalNav profile derives the vector from rendered 100 by 100 images
using LK tracking and the flow-module gyro. The optional physics-derived vector is
diagnostic-only and must remain disabled for algorithm-quality evaluation. Low
texture or failed tracking produces quality zero. Simulator pose and velocity are
never published to the estimator, FCU, scheduler, or bag contract; explicit
ground-truth topics remain evaluator-only.

## Implementation Changes

- Uses best-effort, keep-last depth 1 at both image endpoints and a 15 Hz latest-frame bridge.
- Replaced grid block matching with pyramidal Lucas-Kanade tracking, forward/backward checks, robust inlier filtering, and coverage-aware quality.
- Removed the empirical `angular_scale=0.024`; the default scale is `1.0`.
- Added a noisy 30 Hz downward single-beam range sensor.
- Added a noisy 200 Hz internal flow-module IMU; FCU IMU is fallback only.
- Corrected legacy `OPTICAL_FLOW` units: pixels, rad/s, compensated m/s, and distance.
- Added a dual-clock policy: Gazebo source time defines image/gyro/pose integration, while the published ROS header uses wall time for association with the current non-`use_sim_time` LIO stack.
- Added independent Gazebo sensor and LIO cross-check gates.
- Corrected the evaluator to align truth with `integration_time_us`, while reporting callback arrival gaps separately.

## Iteration Evidence

| Run | Main change | Scale | Correlation | Normalized RMSE | Result |
| --- | --- | ---: | ---: | ---: | --- |
| 12 | LK plus physical intrinsics | 0.083 | 0.161 | 1.285 | failed |
| 14 | internal Gazebo IMU | 0.020 | 0.067 | 1.367 | failed |
| 15 | physics velocity synthesis | 0.325 | 0.534 | 1.105 | failed |
| 16 | pose-window displacement | 0.642 | 0.604 | 0.913 | failed |
| 17 | dual source/wall clocks | 0.972 | 0.850 | 0.705 | LIO cross-check passed |
| 19 | clean WSL repeat, Gazebo sensor gate | 0.951 | 0.840 | 0.617 | sensor gate passed |
| 20a | image-LK ExternalNav, corrected interval, repeat 1 | 0.940 | 0.973 | 0.363 | passed |
| 20b | image-LK ExternalNav, corrected interval, repeat 2 | 0.935 | 0.974 | 0.363 | passed |
| 20c | image-LK ExternalNav, corrected interval, repeat 3 | 0.933 | 0.974 | 0.294 | passed |

Run 19 used 455 observable Gazebo displacement samples and recovered the identity sensor-axis mapping. The concurrent LIO cross-check produced scale `0.937` and correlation `0.735`, but normalized RMSE `0.889`; its LIO reference was independently invalid (`position_rmse=0.142 m`, `yaw_rmse=1.11 deg`, coupling reference false). The combined gate therefore reports `sensor_passed_lio_crosscheck_inconclusive`, not a false LIO pass.

Acceptance thresholds for an observable-motion segment are:

- at least 80 paired samples;
- expected axis mapping;
- scale in `[0.70, 1.30]`;
- correlation at least `0.50`;
- normalized RMSE at most `0.75`.

## Decision and Remaining Gates

- The Gazebo optical-flow sensor data layer is accepted for rosbag2 recording, fault injection, and reliability-score work.
- Use the pure image-LK vector as the default algorithm measurement; physics-derived flow is diagnostic-only.
- Do not feed `/uav/local_pose`, `/uav/local_odom`, or Gazebo ground-truth topics into the navigation algorithm.
- Companion GPS/flow aiding is enabled through `/mavros/odometry/out`; direct FCU optical-flow injection remains disabled to avoid double fusion.
- `D_OF` consumes quality, range validity, timing diagnostics, and a vector-increment residual. Live scheduler admission still requires an independent LIO increment.
