# Ultra-Fusion Formula Audit

Audit basis: local Chinese paper `2606.21223_zh_CN.pdf`, visually checked on pages 6-7, plus the official repository README at commit `439c8385dbcd78174a2b98ab454b53ec64c9e7ca`.

## Reliability and scheduling boundary

| Paper item | Paper role | Current implementation | Status |
| --- | --- | --- | --- |
| Eq. (15) | Binary modality activity from degradation threshold and minimum observation count | Stage 3 publishes `D`, evidence coverage, and guarded validity only | deferred to Stage 6 scheduler |
| Eq. (16) | Unified sliding-window objective with optional factors | No unified backend yet | deferred to Stage 7 |
| Eq. (17) | Compact per-modality score definitions | Implemented through equations (19)-(23) mappings | audited with limitations below |

Stage 3 must not claim that `1-D` is the final factor weight. Incomplete paper evidence forces `valid=false` and `reliability_weight=0`; Stage 6 will own thresholds, minimum counts, hysteresis, covariance inflation, and state transitions.

## Modality crosswalk

| Formula | Required evidence | Project mapping | Audit verdict |
| --- | --- | --- | --- |
| Eq. (18) | Point-to-plane Jacobians and accumulated Hessian | External consecutive registered-scan voxel planes | correct algebra, approximate data source |
| Eq. (19) | Hessian degeneracy, normal diversity, weak-axis penalty, match support | `D_L` four-term structure | structurally matched; thresholds and `M_ref` remain dataset parameters |
| Eq. (20) | Feature count, 8x8 spatial uniformity, reprojection residual | Feature-support heuristic, occupancy, unavailable reprojection; depth ratio extension | incomplete, coverage `0.75`, invalid for scheduling |
| Eq. (21) | Motion excitation, IMU preintegration residual, saturation | Excitation and saturation present; backend residual unavailable | incomplete, coverage `0.55`, invalid for scheduling |
| Eq. (22) | Wheel versus inertial translation-vector and yaw-increment consistency | UAV optical-flow adaptation plus quality/range | paper has no optical-flow formula; vector prediction is disabled until frame/scale calibration, coverage `0.40` |
| Eq. (23) | GNSS fix quality, covariance trace, local innovation Mahalanobis norm | Fix and covariance direct; LIO/GNSS displacement-magnitude innovation proxy | complete topic can be valid, but innovation remains approximate |

## Corrections retained in this iteration

1. Missing paper terms no longer renormalize the remaining weights. The paper defines normalized modality weights, not per-message reweighting around absent evidence.
2. Each score publishes `evidence_weight_coverage` and `score_complete`.
3. Incomplete scores remain diagnostic but cannot produce a nonzero temporary factor weight.
4. IMU low excitation is labeled as observability risk rather than sensor hardware failure.
5. Optical flow no longer compares its displacement against FAST-LIO's unset zero twist. The equation (22) adaptation stays incomplete until a calibrated vector prediction exists.

## Validation evidence

- Eight unit tests cover direction, missing-evidence weights, and unavailable optical-flow prediction.
- The ROS 2 endpoint probe checks score direction, validity, guarded weights, and evidence coverage.
- An 11-level complete-evidence sweep is monotonic for all five modalities.
- A full simple-map fixed-route run passed after the scoring changes without feeding any score back into FAST-LIO.

## Non-claims

- The external LiDAR Hessian is not FAST-LIO's internal scan-to-map Hessian.
- Generic Gazebo GNSS is not a physical BDS receiver or constellation simulation.
- Optical flow is not yet equation (22)-equivalent because camera-frame direction and scale are not calibrated.
- IMU and visual scores are not scheduler-ready until their residual terms are supplied.
- The official Ultra-Fusion ROS2 binary is available, but the estimator source is not public; binary behavior cannot resolve source-level formula ambiguities.
