# Stage 3 Reliability Score Validation

Status: formula implementation and conservative evidence policy complete; calibrated optical-flow prediction, visual reprojection, and IMU preintegration evidence remain future-stage gates. Runtime results are generated under `logs/uf_stage3_*` and are intentionally not committed.

## Scope

- `/reliability/lidar_score`: Ultra-Fusion equations (18)-(19), using an external consecutive-scan point-to-plane proxy.
- `/reliability/gnss_score`: equation (23), used for BDS/GNSS integrity.
- `/reliability/imu_score`: equation (21); preintegration residual remains unavailable until the unified backend.
- `/reliability/optical_flow_score`: equation (22) translational-increment adaptation plus flow quality/distance.
- `/reliability/vision_score`: equation (20) plus RGB-D valid-depth extension; reprojection residual remains unavailable until calibrated feature tracking.

## Acceptance

`validate_reliability_runtime.py` publishes healthy and degraded messages through the real ROS 2 node. It requires all five degraded medians to exceed healthy medians by at least `0.15`. It also checks evidence coverage, `valid`, and guarded weights: complete LiDAR/GNSS evidence must be valid, while incomplete IMU/optical-flow/vision evidence must be invalid with zero temporary weight.

`validate_reliability_sweeps.py` evaluates 11 severity levels with all paper terms supplied. Every curve must be monotonic, complete, and span at least `0.15`. The synthetic tests verify formula direction and topic wiring; they do not replace rosbag2 fault campaigns or complete flight evaluation.

The conservative-evidence runtime probe `20260717_fixed_evidence_policy3` passed:

| Modality | Healthy | Degraded | Delta |
| --- | ---: | ---: | ---: |
| LiDAR | 0.051 | 0.940 | +0.889 |
| BDS/GNSS | 0.002 | 0.552 | +0.550 |
| IMU | 0.097 | 0.550 | +0.453 |
| Optical flow | 0.034 | 0.245 | +0.211 |
| RGB-D vision | 0.000 | 0.750 | +0.750 |

| Modality | Evidence coverage | Valid for scheduling | Reason |
| --- | ---: | --- | --- |
| LiDAR | 1.00 | yes, as marked external proxy | all equation (19) terms present |
| BDS/GNSS | 1.00 | yes when local innovation is present | all equation (23) terms present |
| IMU | 0.55 | no | preintegration residual unavailable |
| Optical flow | 0.40 | no | calibrated vector increment prediction unavailable |
| RGB-D vision | 0.75 | no | reprojection residual unavailable |

The 11-level formula sweep passed for all modalities. Score spans were LiDAR `0.929`, BDS/GNSS `0.974`, IMU `0.991`, optical flow `0.811`, and vision `0.918`. The rendered sweep plot was visually inspected.

The full fixed-route run `20260717_evidence_semantics_flight7` also passed the unchanged LIO gates. Real score distributions were:

| Modality | P50 | P95 | Valid rate | Coverage P50 |
| --- | ---: | ---: | ---: | ---: |
| LiDAR | 0.325 | 0.455 | 1.00 | 1.00 |
| BDS/GNSS | 0.002 | 0.003 | 1.00 | 1.00 |
| IMU | 0.153 | 0.350 | 0.00 | 0.55 |
| Optical flow | 0.038 | 0.250 | 0.00 | 0.40 |
| RGB-D vision | 0.477 | 0.557 | 0.00 | 0.75 |

Unit coverage includes eight score tests and five LiDAR geometry tests. The tests are discovered by `colcon test`.

## Remaining Evidence Gaps

- The true FAST-LIO internal Hessian is not exported; `D_L` uses the marked external equation (18) proxy.
- `NavSatFix` has no satellite count or DOP; both remain unavailable evidence.
- The Stage 3 IMU front end has no backend preintegration residual; equation (21) reserves its weight and marks the score incomplete.
- The RGB-D front end has no calibrated feature reprojection residual; equation (20) reserves its weight. Depth-valid ratio is a documented extension.
- Ultra-Fusion has no optical-flow score formula. `D_OF` remains an equation (22) adaptation, and its vector consistency term is disabled until the camera transform and scale are validated.
- The GNSS innovation is a displacement-magnitude proxy, not yet the full local-frame innovation vector from equation (23).
- Stage 3 does not select scheduler thresholds. Normal-flight P50/P95 values are evidence for later calibration, not automatic factor gates.
