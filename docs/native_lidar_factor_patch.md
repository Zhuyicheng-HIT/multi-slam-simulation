# FAST-LIO native LiDAR factor export

This package carries a reproducible patch for the pinned FAST-LIO2 ROS 2 source
checkout. The patch is intentionally additive: FAST-LIO keeps its original
iterated EKF update, map update, odometry output, and IMU input chain.

## Version boundary

- Upstream: `https://github.com/hku-mars/FAST_LIO.git`
- Expected commit: `a4743b095409588842a5b30ddfa27e29d2f99164`
- External checkout: `$HOME/multi-slam-deps/mid360_ws/src/FAST_LIO_ROS2`

Apply it with:

```bash
bash tools/apply_fast_lio_native_factor_patch.sh
```

The script refuses a different commit or a dirty checkout. It never commits
external source into this repository.

## Packet contract

The patched node publishes `fast_lio/msg/NativeLidarFactor` when enabled.
Each packet contains the frozen point-to-plane correspondences used by one
FAST-LIO update, the signed residuals, the native 12-column measurement
Jacobian, and the unscaled normal equations:

```text
H = J^T J
g = J^T r
```

The linearization pose is `map_T_body`, while the matched points remain in
`sensor_frame`. The packet therefore also carries `T_body_sensor` and the
ordered names of all 12 Jacobian columns. A consumer must not apply the body
pose directly to LiDAR points without this extrinsic transform.

The backend must apply `measurement_variance` when forming information. The
`pose_covariance` field is the FAST-LIO posterior pose covariance for logging
and reliability diagnostics only; it must not be added as an independent
LiDAR measurement covariance.

The packet is a scan-local linearization. It is not yet a complete reusable
sliding-window factor: a later backend must decide when to relinearize the
correspondences and how to eliminate or retain FAST-LIO state variables.

## Runtime switch

The export is disabled by default. For the existing MID360 simulation launcher:

```bash
FASTLIO_NATIVE_FACTOR_EXPORT=1 \
FASTLIO_NATIVE_FACTOR_TOPIC=/fast_lio/native_lidar_factor \
FASTLIO_NATIVE_FACTOR_SENSOR_FRAME=mid360_link \
bash src/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
```

The launch wrapper exposes the same three arguments and keeps `RVIZ` and the
existing FAST-LIO input mode unchanged.

## Validator

After sourcing both ROS 2 and the external FAST-LIO install overlay:

```bash
ros2 run uf_lio_adapter native_factor_validator
```

The validator checks array dimensions and finite values, reconstructs every
point-to-plane residual from the exported geometry, independently rebuilds the
six-column pose Jacobian from Ultra-Fusion Eq. (18), and verifies `H = J^T J`,
`g = J^T r`, Hessian symmetry/PSD, and posterior covariance symmetry. It writes
per-packet metrics to JSONL and a final dynamic-range summary when `output_path`
and `summary_path` are configured. It does not alter the estimator or feed data
back to the FCU.

The validator forces BLAS to one thread. Its small matrix checks otherwise
spawn many workers and disturb the simulation being measured.

## Simulation evidence

The 2026-07-26 fixed rectangle run at
`/tmp/multi_slam_gpu_residual_route_20260726_201853` produced:

- 1,051 packets, 1,051 valid, zero malformed, and no sequence gaps;
- median 302 matched points, with a 149 to 1,544 range;
- median point-to-plane residual RMS 0.0271 m and 95th percentile 0.0468 m;
- maximum reconstructed geometry error `1.91e-12` m;
- maximum relative `J^T J` error `9.54e-16` and `J^T r` error `1.37e-14`;
- pose-Hessian minimum eigenvalue 0.699 to 336.0;
- FAST-LIO position RMSE 0.0586 m and yaw RMSE 0.0978 degrees;
- zero point-cloud or IMU timestamp regressions.

Use the upper-left 6 by 6 position/rotation block for LiDAR observability and
degeneration metrics. When online extrinsic estimation is disabled, the final
six columns of the full 12-column Jacobian are zero by design, so the minimum
eigenvalue of the full 12 by 12 Hessian is zero and is not a LiDAR-degeneration
signal.

Raw eigenvalues also scale with the number of accepted correspondences. The
validator therefore emits `pose_hessian_min_eigenvalue_per_match` and
`pose_hessian_condition_number`; reliability scoring should use those together
with match count and residual statistics instead of thresholding the raw
minimum eigenvalue alone.

The adapter now prefers the native factor packet for `/lio/diagnostics`. It
publishes `approximate=false` with native residual, pose-Hessian, normal
distribution, and spatial-coverage evidence. Temporal static/dynamic map
statistics continue to come from registered-cloud persistence. If native
packets time out, the previous voxel point-to-plane proxy remains an explicit
`approximate=true` fallback.

The 2026-07-26 nominal, 75% dropout, and 90% dropout matrix is documented in
`docs/native_lidar_factor_test_report.md`. The strongest run validated 1404 of
1404 packets with the independent pose-Jacobian geometry check enabled and
showed scheduler continuous down-weighting, binary disable, and recovery.
