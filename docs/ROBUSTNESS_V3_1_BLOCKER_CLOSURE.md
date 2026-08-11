# Robustness V3.1 Blocker Closure

## Decision summary

The two requested blockers are causally resolved without changing FRS,
integrity/rollback thresholds, the 0.065 s visual window or sensor physics.

1. Native LiDAR `+2 ms` was an interface-contract mismatch, not a measured
   physical tolerance.  Coherent FAST-LIO/backend-boundary timing passed through
   ±10 ms and failed at ±20 ms on the frozen deterministic replay.  Factor-only
   mismatch passed through ±1 ms and failed at ±2 ms.
2. Joint-map computation was not the cause of the ArduPilot Crash.  Crashes
   occurred in all three prescribed map-OFF runs, including a zero-rollback
   run.  In the direct-clock joint run, every rollback followed FCU Crash.

Detailed evidence is in
`docs/LIDAR_TEMPORAL_CONTRACT_ANALYSIS.md` and
`docs/JOINT_MAP_LONG_RUN_CAUSAL_ANALYSIS.md`.

## Changes made

- Added explicit coherent/factor-only temporal scopes and request-topic
  remapping for deterministic replay.
- Matched `FrontendScanRequest` reliable/transient-local QoS.
- Added scan cache, first-gap, state-association and integrity-correction
  diagnostics.
- Added bounded mapping operation traces and map-mode matrix runners.
- Added bounded retry for the ROS parameter discovery readiness probe; the
  required visual factor mode is still read and checked before flight.
- Moved the matrix to low ROS domain IDs (31–39) to avoid Fast DDS discovery
  stalls in WSL's Linux ephemeral-port range.
- Added direct wall-monotonic timestamps to FCU StatusText diagnostics.

No estimator mathematics, physical model, FRS/safety threshold, rollback rule
or timestamp tolerance was changed.

## Final verification

- All 15 ROS 2 packages built successfully with `--symlink-install`.
- `colcon test-result --all --verbose`: 57 result files, 0 errors, 0 failures,
  0 skipped.  The backend suite's 160 internal unit cases and the temporal
  fault-profile suite's 9 cases passed.
- Active-run lifecycle and visual/fault launch argument parsing passed.
- Static parsing passed for 213 Python, 31 YAML, 33 XML/SDF and 61 Shell files;
  `git diff --check` passed.
- Final coherent-zero replay: 575 Native LiDAR, 574 IMU, 574 GNSS, 101 optical
  flow and 11 visual factors; 0 optimization errors, 0 integrity rejects,
  0 rollback; completeness 1.0, maximum odometry gap 0.232 s, ATE RMSE
  0.00231 m and translation/rotation RPE 0.00313 m / 0.0134 degrees.

The generic Performance V2 replay wrapper's older default capture is not the
V3.1 frozen-clock dataset and fails its own invariants (duplicate/stale request
stream and mismatched historical truth).  It was retained as incompatible
legacy evidence and was not substituted for the coherent-zero control.

## Required matrix result

The prescribed map-OFF/LiDAR-only/joint runs did not meet 3/3 LAND because the
FCU independently declared Crash in all nine runs.  This is classified as
`SIM_FCU_DYNAMICS_BLOCKED`, not as a fusion pass.  The allowed independent-FCU
exception is supported by:

- Crash with map OFF and zero backend rollback;
- FCU `navigation_source=gps`, not the unified estimator;
- direct-clock joint evidence showing Crash before rollback;
- a later identical LiDAR-only route completing LAND/disarm with zero backend
  error/reject/rollback.

All modes maintained NativeLidarFactor, IMU, GNSS, optical flow and visual
traffic until FCU termination.  Optimization errors remained zero in every
reported run.  The maximum backend ROS-state gap was 0.594 s, below the old
greater-than-one-second interruption.

## Hardware-only items

The following cannot be closed honestly in the present simulation:

- raw MID360 packet/per-point/deskew temporal tolerance and production online
  LiDAR-IMU time calibration;
- GNSS FRS validation with FCU innovation metadata;
- production D435i/MID360 hardware clocks, trigger behavior and transport
  latency;
- real vibration, magnetic interference and airframe extrinsic stability;
- GPU-rendered Gazebo behavior while `/dev/dri/renderD128` is unavailable and
  `kms_swrast` is used.

## Readiness

- **Prop-off bench integration:** YES, conditionally.  Use direct packet/point
  timestamps, record FCU innovations and verify all time/extrinsic parameters
  before enabling any control use of unified odometry.
- **Tethered flight:** NO.  The simulated long route achieved only 1/12
  LAND/disarm attempts across all retained runs.  First reproduce repeated
  stable FCU attitude/altitude behavior on the target compute stack, then run a
  prop-off estimator soak with hardware clocks and a supervised low-energy
  tether protocol.
