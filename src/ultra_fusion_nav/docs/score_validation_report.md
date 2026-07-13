# Stage 3 Reliability Score Validation

Status: implementation complete; runtime probe results are generated under `logs/uf_stage3_*` and are intentionally not committed.

## Scope

- `/reliability/lidar_score`: Ultra-Fusion equations (18)-(19), using an external consecutive-scan point-to-plane proxy.
- `/reliability/gnss_score`: equation (23), used for BDS/GNSS integrity.
- `/reliability/imu_score`: equation (21); preintegration residual remains unavailable until the unified backend.
- `/reliability/optical_flow_score`: equation (22) translational-increment adaptation plus flow quality/distance.
- `/reliability/vision_score`: equation (20) plus RGB-D valid-depth extension; reprojection residual remains unavailable until calibrated feature tracking.

## Acceptance

`validate_reliability_runtime.py` publishes healthy and degraded messages through the real ROS 2 node. The test passes only if all five degraded medians exceed their healthy medians by at least 0.15. `plot_reliability_scores.py` produces the comparison chart.

The synthetic runtime probe verifies score direction and topic wiring. It does not replace rosbag2 fault campaigns or a complete flight evaluation.

The `20260713_111645` probe passed with these median scores:

| Modality | Healthy | Degraded | Delta |
| --- | ---: | ---: | ---: |
| LiDAR | 0.051 | 0.940 | +0.889 |
| BDS/GNSS | 0.005 | 0.552 | +0.547 |
| IMU | 0.177 | 1.000 | +0.823 |
| Optical flow | 0.043 | 0.845 | +0.802 |
| RGB-D vision | 0.000 | 1.000 | +1.000 |

Unit coverage includes five score-direction tests and five LiDAR geometry tests. The score plot was visually inspected after rendering.

## Remaining Evidence Gaps

- The true FAST-LIO internal Hessian is not exported; `D_L` uses the marked external equation (18) proxy.
- `NavSatFix` has no satellite count or DOP; both remain unavailable evidence.
- The Stage 3 IMU front end has no backend preintegration residual; equation (21) excludes that unavailable term and renormalizes available weights.
- The RGB-D front end has no calibrated feature reprojection residual yet; equation (20) excludes that term. Depth-valid ratio is a documented extension.
- Ultra-Fusion has no optical-flow score formula; `D_OF` is a documented equation (22) translational adaptation.
