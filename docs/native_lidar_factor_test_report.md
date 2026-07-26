# Native FAST-LIO residual runtime report

Date: 2026-07-26

## Scope

This report validates the patched FAST-LIO2 scan-to-map measurement export. It
does not claim that the unified sliding-window backend already consumes the
native point-to-plane factors.

The implementation follows the paper's LiDAR model:

```text
r_i = n_i^T (p_i^w - s_i)                         Eq. (6)
J_i = [n_i^T, -n_i^T [p_i]_x]                    Eq. (18)
H_k = sum_i J_i^T J_i + 1e-8 I_6                 Eq. (18)
D_L = w_h phi_h + w_n normal_term + w_a phi_a
      + w_c (1 - min(1, M_k / M_ref))            Eq. (19)
```

FAST-LIO uses a right SO(3) perturbation. The equivalent rotation block checked
by the validator is `(p_body x R_WB^T n_world)^T`.

## Validation layers

1. Reconstruct every signed point-to-plane residual from the exported point,
   plane, body pose, and LiDAR-to-body extrinsic.
2. Reconstruct the first six Jacobian columns independently from geometry.
3. Recompute `J^T J` and `J^T r`; check symmetry and positive semidefiniteness.
4. Check posterior pose covariance structure, packet dimensions, frames,
   sequence behavior, and finite values.
5. Feed the native six-dimensional pose Hessian and residual statistics into
   `/lio/diagnostics`, then into the reliability scheduler.

The original FAST-LIO iterated EKF and map update remain unchanged. The adapter
falls back to the external voxel proxy only if native packets time out.

## Fixed-route results

All runs used `simple_apm_rgbd_mid360`, the same rectangle flight, headless
Gazebo GPU rendering, FCU HIGHRES_IMU, and one-thread OpenBLAS for Python nodes.
The table reports medians in the active evaluation interval.

| Run | Native packets | Matches | Residual P95 (m) | lambda_1 | lambda_1 / match | condition | D_L output | Scheduler action |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| Nominal | 1415 | 288 | 0.0512 | 29.67 | 0.1103 | 460.7 | 0.186 | enabled, continuous weight |
| 75% dropout | 421 | 89 | 0.0609 | 2.790 | 0.0335 | 1083 | 0.250 | enabled, weight 0.750, inflation 1.33 |
| 90% dropout | 387 | 38 | 0.0636 | 0.410 | 0.0108 | 3410 | 0.326 | disabled, weight 0, inflation 20 |
| 90% recovery | 540 | 1148 | 0.0510 | 248.8 | 0.2159 | 31.4 | 0.0196 | re-enabled, weight 0.932 |

The 90% dropout run is the first runtime run with the independent pose-Jacobian
geometry check enabled. It received 1404 packets and accepted 1404; no packet
failed residual, Jacobian, normal-equation, covariance, or metadata checks.

## Trajectory and runtime gates

| Run | Aligned ATE RMSE (m) | RPE translation RMSE (m) | Drift yaw RMSE (deg) | RTF median | RTF P10 | Stamp regressions |
|---|---:|---:|---:|---:|---:|---:|
| Nominal | 0.0439 | 0.00933 | 0.0988 | 0.9997 | 0.815 | 0 |
| 75% dropout | 0.0472 | 0.01104 | 0.1011 | 0.9994 | 0.781 | 0 |
| 90% dropout | 0.0600 | 0.01367 | 0.1150 | 0.9995 | 0.774 | 0 |

The simple test map remains localizable even during heavy random point loss.
The scheduler therefore applies continuous down-weighting at 75% loss and only
uses the Eq. (15) minimum-observation gate when the median match count falls
below 50 in the 90% run.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
source install/setup.bash

python3 src/ultra_fusion_nav/scripts/analyze_native_lidar_experiment.py \
  /tmp/native_lidar_dropout90_20260726_210614 \
  --output /tmp/native_lidar_dropout90_20260726_210614/native_factor_report.json
```

Evidence directories:

```text
/tmp/native_lidar_nominal_v2_20260726_205300
/tmp/native_lidar_dropout_20260726_205731
/tmp/native_lidar_dropout90_20260726_210614
```

## Remaining integration boundary

The exporter is now suitable for the next backend step, but the backend must
not add a FAST-LIO pose anchor and the same native point-to-plane information at
the same timestamp. The next implementation should use one of these policies:

1. Consume native LiDAR pose blocks while using separate IMU preintegration,
   and remove the proxy LIO pose factor for those states.
2. Keep FAST-LIO odometry only as initialization/output continuity, not as an
   additional information factor.
3. Treat exported posterior covariance as diagnostics only; use
   `measurement_variance` with the native residual/Jacobian for factor weight.
