# Stage 7 LiDAR Factor-Score Correction

Date: 2026-07-26

## Scope

This milestone corrects the LiDAR reliability boundary in the online bounded
window. It does not claim a complete Ultra-Fusion point-to-plane tightly
coupled backend.

The prior score mixed three separate decisions: LiDAR pose-factor trust,
static-map admission, and approximate external geometry. The replacement
publishes them independently:

- `/reliability/lidar_score`: equation-(19) geometry plus current-LIO versus
  LiDAR-free-prediction innovation. External geometry is soft-only and emits
  `hard_gate_allowed=0`.
- `/reliability/lidar_map_score`: residual, coverage, dynamic/uncertain ratio,
  and repeatability for map admission only.
- Scheduler: an enabled LiDAR factor can cross a binary disable threshold only
  when `hard_gate_allowed=true`; stale or invalid data remains protected by the
  existing minimum anchor.

The backend creates the innovation before inserting the current LIO factor and
publishes its position and yaw components in `/fusion/unified/diagnostics`.

## Clean Fixed Route

Run: `logs/uf_stage2_uf_stage7_lidar_score_v2`

| Evidence | Result |
| --- | ---: |
| LiDAR factor risk, median / maximum | 0.137 / 0.234 |
| LiDAR factors | 903 |
| GNSS factors | 902 |
| IMU residual updates | 900 |
| Optical-flow factors | 636 |
| Optical-flow valid samples | 1890 / 2624 |
| Backend optimization errors | 0 |
| Backend `lidar_disabled` maximum | 0 |

Optical flow was valid after takeoff, when the downward range became available.
The withheld samples are mainly the stationary startup and landing periods,
where range is invalid or image tracking has insufficient features. This proves
simultaneous four-source factor participation in the bounded window; it does
not convert the LIO pose factor into a native LiDAR residual.

The independent FAST-LIO evaluator reported position RMSE `0.0361 m`, yaw RMSE
`0.116 deg`, and no timestamp regressions for this clean run. These are
front-end checks, not a claim that the bounded backend improves FAST-LIO.

## 95 Percent LiDAR Point Dropout

Run: `logs/uf_stage2_uf_stage7_lidar_dropout_v1`

| Evidence | Result |
| --- | ---: |
| LiDAR factor risk, median / maximum | 0.247 / 0.754 |
| Matched points, median | 301 |
| LiDAR factors | 792 |
| GNSS factors | 791 |
| IMU residual updates | 788 |
| Optical-flow factors | 629 |
| Backend `lidar_disabled` maximum | 0 |
| LIO anchor overrides, maximum | 148 |
| Backend optimization errors | 0 |

The fault was observed by 274 active fault-state samples. The score increases
and scheduler enters degraded/risk states, but approximate geometry does not
authorize a false hard LiDAR shutdown. The existing anchor only covers periods
where the front end stops producing usable input.

The raw FAST-LIO point-cloud evaluator failed this extreme run because map
overlap fell and centroid motion became inconsistent. Its trajectory ATE RMSE
was `0.717 m`. That is expected front-end observation degradation and must not
be presented as a successful unified-backend recovery.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

RUN_ID=uf_stage7_lidar_score_v2 \
ENABLE_UNIFIED_BACKEND=1 ENABLE_RELIABILITY=1 \
ENABLE_RELIABILITY_TIMELINE=1 PRESERVE_LIO_ANCHOR=true \
ENABLE_D435_BRIDGE=0 ENABLE_MID360_BRIDGE=1 \
FLOW_USE_PHYSICS=false HEADLESS=1 RVIZ=0 \
bash src/ultra_fusion_nav/scripts/run_lio_baseline_experiment.sh

RUN_ID=uf_stage7_lidar_dropout_v1 ANALYSIS_DURATION_S=100 \
ENABLE_UNIFIED_BACKEND=1 ENABLE_RELIABILITY=1 \
ENABLE_RELIABILITY_TIMELINE=1 ALLOW_MISSING_RELIABILITY=1 \
PRESERVE_LIO_ANCHOR=true ENABLE_D435_BRIDGE=0 ENABLE_MID360_BRIDGE=1 \
FLOW_USE_PHYSICS=false HEADLESS=1 RVIZ=0 \
FAULT_MODALITY=lidar FAULT_TYPE=point_dropout \
FAULT_TRIGGER_DELAY_S=0 FAULT_DURATION_S=75 FAULT_MAGNITUDE=0.95 \
FAULT_DELIVERY_MODE=startup \
bash src/ultra_fusion_nav/scripts/run_lio_baseline_experiment.sh
```

The fault command exits nonzero because the independent front-end overlap gate
is intentionally violated. The scheduler and backend timeline remain valid
fault artifacts.

## Remaining Boundary

The implementation is an online tangent-space, bounded-window prototype. It
still lacks native FAST-LIO point-to-plane residuals, manifold SE(3)
relinearization, proper marginal covariance propagation, and an explicit
LIO/IMU correlation model. These are required before claiming the paper's
complete tightly coupled estimator.
