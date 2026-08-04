# Ultra-Fusion Formula Audit

Audit basis: local Chinese paper `2606.21223_zh_CN.pdf`, visually checked on pages 6-7, plus the official repository README at commit `439c8385dbcd78174a2b98ab454b53ec64c9e7ca`.

## Reliability and scheduling boundary

| Paper item | Paper role | Current implementation | Status |
| --- | --- | --- | --- |
| Eq. (15) | Binary modality activity from degradation threshold and minimum observation count | Scheduler applies minimum counts, continuous weights, binary gates, covariance inflation, and hysteresis | implemented; thresholds remain scenario parameters |
| Eq. (16) | Unified sliding-window objective with optional factors | Bounded online tangent-space window with scheduler-controlled LIO, IMU, GNSS, and optical-flow factors | prototype only; manifold SE(3) relinearization remains |
| Eq. (17) | Compact per-modality score definitions | Implemented through equations (19)-(23) mappings | audited with limitations below |

Stage 3 must not claim that `1-D` is the final factor weight. Incomplete paper evidence forces `valid=false` and `reliability_weight=0`; Stage 6 will own thresholds, minimum counts, hysteresis, covariance inflation, and state transitions.

## Modality crosswalk

| Formula | Required evidence | Project mapping | Audit verdict |
| --- | --- | --- | --- |
| Eq. (18) | Point-to-plane Jacobians and accumulated Hessian | External consecutive registered-scan voxel planes | correct algebra, approximate data source |
| Eq. (19) | Hessian degeneracy, normal diversity, weak-axis penalty, match support | `D_L` geometry component plus a LiDAR-free pose innovation for the factor score | structurally matched geometry, but external source is soft-only |
| Eq. (20) | Feature count, 8x8 spatial uniformity, reprojection residual | Feature-support heuristic, occupancy, unavailable reprojection; depth ratio extension | incomplete, coverage `0.75`, invalid for scheduling |
| Eq. (21) | Motion excitation, IMU preintegration residual, saturation | Excitation and saturation plus backend `r_imu` Mahalanobis residual using nominal preintegration covariance | complete evidence; online scheduling validated with bias injection |
| Eq. (22) | Wheel versus inertial translation-vector and yaw-increment consistency | UAV optical-flow adaptation plus quality/range | paper has no optical-flow formula; vector prediction is disabled until frame/scale calibration, coverage `0.40` |
| Eq. (23) | GNSS fix quality, covariance trace, local innovation Mahalanobis norm | Fix and covariance direct; LIO/GNSS displacement-magnitude innovation proxy | complete topic can be valid, but innovation remains approximate |

## Corrections retained in this iteration

1. Missing paper terms no longer renormalize the remaining weights. The paper defines normalized modality weights, not per-message reweighting around absent evidence.
2. Each score publishes `evidence_weight_coverage` and `score_complete`.
3. Incomplete scores remain diagnostic but cannot produce a nonzero temporary factor weight.
4. IMU low excitation is labeled as observability risk rather than sensor hardware failure.
5. Optical flow no longer compares its displacement against FAST-LIO's unset zero twist. The equation (22) adaptation stays incomplete until a calibrated vector prediction exists.
6. IMU residual evidence uses the nominal preintegration covariance. The separate `imu_covariance_scale` remains an optimization tuning term and is not allowed to weaken the health score.
7. A missing, stale, negative, or non-finite backend residual makes the IMU score incomplete instead of silently substituting zero.
8. LiDAR pose-factor risk and map-admission risk are published separately. Residual, coverage, dynamics, uncertainty, and repeatability cannot hard-disable a LiDAR pose factor through the external geometry adapter.
9. The current LiDAR score uses a prediction excluding the current LiDAR factor. Approximate geometry or missing innovation sets `hard_gate_allowed=0`; the scheduler may still apply continuous covariance inflation.

## Validation evidence

- Eight unit tests cover direction, missing-evidence weights, and unavailable optical-flow prediction.
- The ROS 2 endpoint probe checks score direction, validity, guarded weights, and evidence coverage.
- An 11-level complete-evidence sweep is monotonic for all five modalities.
- A full simple-map fixed-route run passed after the scoring changes without feeding any score back into FAST-LIO.
- The retained fixed-route run produced 884 complete IMU scores, residual median `0.0360`, residual P95 `0.6396`, and zero backend/residual errors.
- With a 15 s IMU bias (`gyro_z +0.5 rad/s`, `accel_x +2.0 m/s^2`), median residual rose from `0.0227` to `2.4603` and median `D_I` rose from `0.1696` to `0.3833`; the scheduler used continuous degradation/RISK transitions without a false binary shutdown.
- In `uf_stage7_lidar_score_v2`, the online bounded window accepted 903 LiDAR, 902 GNSS, 900 IMU, and 636 optical-flow factors with zero optimizer errors. Optical flow had 1890 valid samples out of 2624; invalid startup and landing samples had no valid range and were correctly withheld.
- In `uf_stage7_lidar_dropout_v1`, a scheduled 95% LiDAR point dropout raised median LiDAR factor risk from `0.137` to `0.247` (maximum `0.754`). The backend reported `lidar_disabled=0` and zero optimization errors. The independent FAST-LIO map-overlap evaluator failed under this extreme front-end fault, so this run is evidence of correct gating behavior, not a recovered-front-end accuracy claim.

## Non-claims

- The external LiDAR Hessian is not FAST-LIO's internal scan-to-map Hessian.
- The online window uses LIO pose factors, not native LiDAR point-to-plane residual factors. Its four-source co-window factor fusion is an initial integration milestone, not the paper's full tightly coupled estimator.
- Generic Gazebo GNSS is not a physical BDS receiver or constellation simulation.
- Optical flow is not yet equation (22)-equivalent because camera-frame direction and scale are not calibrated.
- The visual score is not scheduler-ready until its reprojection residual is supplied.
- The current IMU residual is evaluated in a linear tangent-space backend and is not a substitute for full manifold relinearization or Schur-complement marginalization.
- The official Ultra-Fusion ROS2 binary is available, but the estimator source is not public; binary behavior cannot resolve source-level formula ambiguities.
