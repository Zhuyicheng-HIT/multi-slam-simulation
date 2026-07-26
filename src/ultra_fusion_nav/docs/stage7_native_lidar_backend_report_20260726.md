# Stage 7 Native FAST-LIO Factor Backend Report

Date: 2026-07-26
Workspace: `/home/zyc/multi-slam-github-staging`
Branch: `feature/ultra-fusion-stage3`

## Scope

This iteration replaces the backend's per-scan `lidar_pose` proxy with the
native FAST-LIO point-to-plane information exported by the patched frontend.
The packet contains the exact residuals, right-perturbation Jacobian,
`H = J^T J`, `g = J^T r`, and the point measurement variance. The backend
consumes only the fixed-extrinsic 6-DoF pose block. FAST-LIO's posterior pose
covariance is diagnostics only and is not added as another measurement.

For the backend's ZYX roll/pitch/yaw coordinates, the rotational columns are
converted with the right-perturbation map `delta_theta_body = E_RPY delta_RPY`.
The condensed factor is inserted as:

```text
r(x) ~= r0 + J (x - x0)
H x = H x0 - g
```

Scheduler weighting is applied as `s_L / covariance_inflation` to this normal
equation. A state is prohibited from containing both `lidar_point_plane` and
`lidar_pose`, including when one of them is disabled.

## Tests

```text
uf_backend_fusion: 34 tests passed
colcon test --packages-select uf_backend_fusion: 34 tests passed
```

The new tests cover non-zero RPY right-tangent conversion, native normal
equation recovery, timestamp-nearest pairing, incompatible packet rejection,
and the no-duplicate-LiDAR-factor invariant.

## Fixed-route online result

Command family: `run_lio_baseline_experiment.sh`, simple map, 125 s route,
FAST-LIO native export enabled, ReliabilityScheduler enabled, unified backend
enabled, no Gazebo truth feedback.

Output: `/tmp/native_backend_tight_v2`

| Metric | Result |
|---|---:|
| Rectangle state machine | pass |
| Unified output ATE RMSE | 0.0715 m |
| Unified output RPE translation RMSE | 0.0517 m |
| Unified output RPE rotation RMSE | 0.489 deg |
| Native factor packets validated | 863 / 863 |
| Backend native packets received | 692 |
| Backend native factors inserted | 692 |
| Invalid native packets | 0 |
| Pose fallbacks | 3 |
| Pair timeouts | 3 |
| Median factor/odom stamp error | 0 ms |
| Backend optimization errors | 0 |
| IMU residual errors | 0 |
| Maximum LiDAR anchor overrides | 1 (startup only) |
| Maximum native hard-disabled count | 0 |

The three fallbacks occurred at the startup pairing boundary; steady-state
backend samples used `native_point_to_plane`. The backend timeline records both
the source and cumulative counters, so this is not inferred from trajectory
quality alone. The native validator independently reports zero geometry,
Jacobian, normal-equation, symmetry, or PSD errors for every received packet.

## Interpretation and limits

This is the first runtime native point-to-plane coupling milestone. It is not
yet a complete Ultra-Fusion implementation:

1. The window is still a dense/sparse linear tangent-space solver, not a full
   SE(3) manifold fixed-lag optimizer with principled marginalization.
2. FAST-LIO remains the source of scan correspondence and its map. The backend
   does not yet own an independent static map, so statistical correlation with
   the frontend map is not eliminated.
3. Raw IMU is preintegrated independently in the project backend. The exported
   LiDAR packet contains LiDAR rows only, so the same timestamp's FAST-LIO pose
   covariance is not double-counted; a future frontend/backend split must still
   account for map and linearization correlations.
4. The route was nominal. A follow-up must inject low-match LiDAR, GNSS jump,
   GNSS outage, and optical-flow degradation while checking continuous native
   factor weight, binary disable, and recovery.

The next implementation gate is an offline/online native-factor replay with an
explicit fixed-weight versus scheduler-weighted comparison, followed by a
proper manifold backend decision (GTSAM or Ceres) before adding RGB-D vSLAM.
