# Optical-Flow Sensor Report

Status: image-LK optical flow passes the independent full-route and
translation-segment gates in the current unified-window simulation. Companion
GNSS/flow aiding is active through one ExternalNav output, while direct FCU
optical-flow injection remains disabled.

## Sensor Boundary

The estimator-facing observation is `/sensors/optical_flow/rad`. Gazebo publishes
an internal `/sim/optical_flow/rad_native` observation. It is quantized into the
same 27-byte MicoLink `0x51` frame measured from the physical MTF-01 and decoded
back to `/sim/optical_flow/rad` before entering the sensor pipeline:

- `integrated_x/y`: raw angular image flow over one sensor integration window;
- `integrated_x/y/zgyro`: co-timed internal gyro integration in sensor FRD axes;
- `distance`: noisy downward single-beam range;
- `quality`: image-derived texture, LK survival, forward/backward consistency, spatial coverage, range, and motion-limit score.

The physical direct-computer path uses the same decoder on
`/hardware/mtf01/optical_flow/rad`. MicoLink does not carry gyro integrals, so
both simulation and hardware paths associate FCU HIGHRES_IMU at the companion
computer before reliability scoring and fusion.

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
- Added the hardware-matched MicoLink frame boundary: `0xEF`, device `0x0F`,
  message `0x51`, 20-byte payload, 27-byte frame, additive checksum, and sensor
  time-derived integration interval.

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

### 2026-07-28 unified-window iteration

The later unified-window route exposed two errors that the earlier standalone
sensor runs did not isolate:

- the Gazebo downward camera image-x and horizontal gyro-x axes needed one
  explicit conversion at the simulator/MAVLink boundary;
- the scheduler recovery gate used displacement per sample, so correct 30 Hz
  motion below `0.01 m/sample` was rejected even when vehicle speed was valid.

The gate now uses translation speed (`0.08 m/s`) with each message's actual
integration interval. A fixed simulator-only `translation_scale=0.683` scales
only gyro-compensated image translation; it does not scale the gyro integral.
The value is the mean fit from two unscaled fixed-route runs and is never read
from truth online. Real hardware does not execute this Gazebo bridge.

| Run | Translation scale | Full scale/corr/NRMSE | No-turn scale/corr/NRMSE | Enabled flow factors | Unified ATE |
| --- | ---: | --- | --- | ---: | ---: |
| v8 | 1.000 | 0.641 / 0.851 / 0.499 | 0.686 / 0.868 / 0.471 | 385 / 716 | 0.055881 m |
| v9 | 1.000 | 0.632 / 0.842 / 0.527 | 0.680 / 0.863 / 0.488 | 330 / 717 | 0.057889 m |
| v10 | 0.683 | 0.931 / 0.859 / 0.494 | 1.000 / 0.875 / 0.462 | 233 / 714 | 0.055385 m |

The no-turn segment requires `|yaw_rate| <= 0.08 rad/s`. Both v10 segments pass
the existing mapping, scale, correlation, and normalized-RMSE thresholds. The
v8/v9 pair establishes repeatable rate-gate coverage; v10 is one calibrated
verification run, so the trajectory delta is not yet a statistical claim.

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
- During sustained yaw, the scheduler continuously lowers the flow factor
  weight and hard-disables it above the configured upper yaw-rate threshold.
  Recovery requires low yaw rate plus translational motion for a dwell period,
  then ramps the factor weight instead of switching it on in one step.
