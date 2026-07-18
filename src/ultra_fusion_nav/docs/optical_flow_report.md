# Optical-Flow Sensor Report

Status: Gazebo optical-flow sensor model accepted on the simple fixed route; FCU injection and non-GPS aiding remain disabled.

## Sensor Boundary

The estimator-facing observation is `/sensors/optical_flow/rad`. The Gazebo source publishes `/sim/optical_flow/rad` with MAVLink `OPTICAL_FLOW_RAD` semantics:

- `integrated_x/y`: raw angular image flow over one sensor integration window;
- `integrated_x/y/zgyro`: co-timed internal gyro integration in sensor FRD axes;
- `distance`: noisy downward single-beam range;
- `quality`: image-derived texture, LK survival, forward/backward consistency, spatial coverage, range, and motion-limit score.

The vector is synthesized inside the Gazebo sensor from the flow-camera displacement and internal gyro, with angular noise. Real rendered images still control quality and failure: low texture or failed tracking produces quality zero. Simulator pose and velocity are never published to the estimator, FCU, scheduler, or bag contract. This is the same boundary used by simulated IMU and LiDAR sensors; explicit ground-truth topics remain evaluator-only.

## Implementation Changes

- Replaced the 20 Hz latest-frame sampler with all-frame camera delivery.
- Replaced grid block matching with pyramidal Lucas-Kanade tracking, forward/backward checks, robust inlier filtering, and coverage-aware quality.
- Removed the empirical `angular_scale=0.024`; the default scale is `1.0`.
- Added a noisy 30 Hz downward single-beam range sensor.
- Added a noisy 200 Hz internal flow-module IMU; FCU IMU is fallback only.
- Corrected legacy `OPTICAL_FLOW` units: pixels, rad/s, compensated m/s, and distance.
- Added a dual-clock policy: Gazebo source time defines image/gyro/pose integration, while the published ROS header uses wall time for association with the current non-`use_sim_time` LIO stack.
- Added independent Gazebo sensor and LIO cross-check gates.

## Iteration Evidence

| Run | Main change | Scale | Correlation | Normalized RMSE | Result |
| --- | --- | ---: | ---: | ---: | --- |
| 12 | LK plus physical intrinsics | 0.083 | 0.161 | 1.285 | failed |
| 14 | internal Gazebo IMU | 0.020 | 0.067 | 1.367 | failed |
| 15 | physics velocity synthesis | 0.325 | 0.534 | 1.105 | failed |
| 16 | pose-window displacement | 0.642 | 0.604 | 0.913 | failed |
| 17 | dual source/wall clocks | 0.972 | 0.850 | 0.705 | LIO cross-check passed |
| 19 | clean WSL repeat, Gazebo sensor gate | 0.951 | 0.840 | 0.617 | sensor gate passed |

Run 19 used 455 observable Gazebo displacement samples and recovered the identity sensor-axis mapping. The concurrent LIO cross-check produced scale `0.937` and correlation `0.735`, but normalized RMSE `0.889`; its LIO reference was independently invalid (`position_rmse=0.142 m`, `yaw_rmse=1.11 deg`, coupling reference false). The combined gate therefore reports `sensor_passed_lio_crosscheck_inconclusive`, not a false LIO pass.

Acceptance thresholds for an observable-motion segment are:

- at least 80 paired samples;
- expected axis mapping;
- scale in `[0.70, 1.30]`;
- correlation at least `0.50`;
- normalized RMSE at most `0.75`.

## Decision and Remaining Gates

- The Gazebo optical-flow sensor data layer is accepted for rosbag2 recording, fault injection, and reliability-score work.
- Keep the pure image-LK vector available as a diagnostic, but use the physics sensor vector as the default simulated measurement.
- Do not feed `/uav/local_pose`, `/uav/local_odom`, or Gazebo ground-truth topics into the navigation algorithm.
- Do not enable FCU optical-flow injection or non-GPS aiding until a separate flight validates ArduPilot orientation, range handling, EKF innovation, outage behavior, and landing transitions.
- `D_OF` may now consume quality, range validity, clock diagnostics, and the admitted vector residual. Scheduler weight remains zero until the Stage 4 aiding test passes.
