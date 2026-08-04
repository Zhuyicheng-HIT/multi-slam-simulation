# Stage 7 IMU Residual and Guarded LiDAR Bypass Report

Date: 2026-07-26

## Scope

This iteration closes the missing IMU evidence term in Ultra-Fusion Eq. (21)
and tests whether the online tangent-space backend can remove a LiDAR pose
factor without losing yaw observability. Gazebo truth remains evaluator-only.
No FCU fused local pose is fed into the estimator.

## Implementation

- The bounded backend evaluates the newest preintegrated IMU factor as
  `r_imu^T Omega_imu r_imu`, even when that factor is currently disabled.
- Reliability evidence uses nominal preintegration covariance. The separate
  factor covariance scale remains an optimizer tuning parameter.
- The reliability monitor accepts only finite, non-negative, fresh residuals.
  Missing evidence keeps the IMU score invalid.
- Marginalization carries an approximate boundary-state prior. This is not a
  Schur-complement marginal prior.
- `preserve_lio_anchor` remains `true` by default. An explicit launch argument
  can disable it only after fresh LiDAR evidence and an enabled, fresh IMU
  backup are both present.
- Backend diagnostics separate optimization failures from residual-evidence
  failures and reject non-finite solver output.

## Fixed-route evidence

All retained runs used `simple_apm_rgbd_mid360`, the automated rectangle,
headless Gazebo, no D435 point-cloud bridge, and the physics optical-flow path
disabled. The median simulation real-time factor stayed near 1.0, so time
dilation was not used.

| Run | Mode | Unified ATE | Translation RPE | Rotation RPE | Result |
| --- | --- | ---: | ---: | ---: | --- |
| `imu_residual_clean_v4` | default anchor protection | 0.0517 m | 0.0267 m | 0.432 deg | retained |
| `imu_nominal_cov_bias_v2` | 15 s IMU bias | 0.0666 m | 0.0320 m | 0.752 deg | scoring validation |
| `lidar_bypass_clean_v5` | explicit LiDAR bypass | 0.0659 m | 0.0222 m | 0.493 deg | experimental only |

The retained clean run had FAST-LIO position RMSE `0.0387 m`, yaw RMSE
`0.110 deg`, yaw-rate/FCU-gyro correlation `0.924`, zero timestamp regressions,
zero optimizer errors, zero residual errors, 761 residual updates, ExternalNav
rate `7.42 Hz`, and RTF median `0.9993`.

Clean IMU residual median/P95 were `0.0360/0.6396`; all 998 scheduler samples
kept IMU enabled. Under `gyro_z +0.5 rad/s` and `accel_x +2.0 m/s^2`, the
aligned fault-window residual median rose from `0.0227` to `2.4603`, and median
`D_I` rose from `0.1696` to `0.3833`. The 150 fault-window scheduler samples
split into 82 `DEGRADED` and 68 `RISK`; the factor was continuously downweighted
but did not cross the binary disable threshold.

## Rejected and bounded results

The first true-bypass run was rejected: LiDAR was disabled while IMU evidence
was incomplete, producing rotation RPE `19.68 deg`. The corrected experimental
bypass reduced this to `0.493 deg` with 380 actually disabled LiDAR factors and
zero backend errors. It is still not the default because the independent
FAST-LIO yaw-rate correlation gate failed in that run (`0.442 < 0.65`) and the
clean LiDAR classifier issued 561 disabled decisions out of 990 scheduler
samples.

A three-signal dynamic-map candidate (`dynamic_ratio`, repeatability, and map
quality) also remains rejected. It fired on 18/93 clean samples and 7/14 fault
samples. Temporal persistence separated this one route only narrowly: the
longest clean run was `3.26 s`, versus `5.19 s` during the injected fault. That
margin is not sufficient for a default hard gate.

## Reproduction

Default protected run:

```bash
RUN_ID=uf_stage7_imu_residual_clean \
ENABLE_UNIFIED_BACKEND=1 ENABLE_RELIABILITY=1 \
ENABLE_RELIABILITY_TIMELINE=1 PRESERVE_LIO_ANCHOR=true \
ENABLE_D435_BRIDGE=0 RVIZ=0 HEADLESS=1 \
bash src/ultra_fusion_nav/scripts/run_lio_baseline_experiment.sh
```

The same command with `PRESERVE_LIO_ANCHOR=false` enables the guarded research
mode. It is not an accepted default configuration.

## Remaining gate

The backend is still a local linear prototype triggered by LIO timestamps. A
complete Ultra-Fusion reproduction still requires manifold SE(3) states,
proper marginalization, point-to-plane factors, online calibration, reliable
dynamic-map admission, and scheduler-triggered relocalization.
