# LiDAR-IMU Baseline Report

## Scope

The baseline remains the pinned ROS2 FAST-LIO commit driven by FCU `/mavros/imu/data_raw`. D435i IMU and evaluator ground truth are not estimator inputs.

`uf_lio_adapter` provides stable project-owned outputs:

- `/lio/odom`
- `/lio/path`
- `/lio/local_map`
- `/lidar/points_deskewed`
- `/lio/diagnostics`

## Diagnostic Boundary

The pinned FAST-LIO package does not publish accepted point-to-plane matches, residuals, plane normals, or its optimization Hessian. The adapter therefore computes an external consecutive-scan point-to-plane proxy and sets `LioDiagnostics.approximate=true` with source `external_voxel_point_to_plane_paper_eq18_proxy`.

Local voxel neighborhoods fit planes, then form `J_i = [n_i^T, -n_i^T [p_i]_x]` and `H = sum(J_i^T J_i) + 1e-8 I_6` following Ultra-Fusion equation (18). The proxy is useful for regression testing, gross geometry loss, point support, and Stage 3 equation (19) scoring. It is still not FAST-LIO's internal scan-to-map Hessian. A later pinned instrumentation patch is required before the unified backend implements `LidarPlaneFactor`.

## Evaluation

`scripts/record_lio_trajectory.py` is an evaluator-only node that records `/lio/odom` and `/sim/mid360/ground_truth_odom` into separate TUM files. `scripts/evaluate_lio_trajectory.py` performs timestamp association, rigid SE(3) position alignment, ATE, and adjacent-pose translational/rotational RPE.

## Measured Runs

Earlier runs from the extracted `$HOME/mid360_flight_ws` dependency were not repeatable. Two unchanged runs on 2026-07-17 failed with online position RMSE `2.881 m` and `1.787 m`; a post-WSL-restart run improved to `0.412 m` but still failed the final-position and yaw/gyro-correlation gates. These failures are retained as evidence and were not hidden by selecting the best run.

The manifest-pinned workspace at `$HOME/multi-slam-deps/mid360_ws` was then clean-imported from FAST-LIO commit `a4743b095409588842a5b30ddfa27e29d2f99164` and Livox driver commit `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`. The repository configuration and route were unchanged.

| Run | Online position RMSE | Yaw RMSE | Yaw/gyro corr. | TUM ATE | Voxel overlap median | Centroid jump P95 | Stamp regressions | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `pinned_dep_baseline4` | 0.043 m | 0.161 deg | 0.762 | 0.079 m | 0.508 | 3.413 m | 0 | pass |
| `pinned_dep_repeat5` | 0.046 m | 0.145 deg | 0.668 | 0.062 m | 0.505 | 3.388 m | 0 | pass |
| `pinned_dep_repeat6` | 0.049 m | 0.176 deg | 0.785 | 0.053 m | 0.505 | 3.305 m | 0 | pass |
| `evidence_semantics_flight7` | 0.056 m | 0.170 deg | 0.745 | 0.061 m | 0.493 | 2.962 m | 0 | pass |

Across the four accepted runs, online position RMSE is `0.043-0.056 m`, yaw RMSE is `0.145-0.176 deg`, TUM ATE is `0.053-0.079 m`, and final position error is `0.026-0.035 m`. `scripts/summarize_stage23_runs.py` generated the aggregate evidence in `logs/uf_stage23_summary_20260717.json`.

Stage 2 is quantitatively accepted for the simple fixed route. Acceptance is conditional on the manifest-pinned dependency workspace and does not yet claim tunnel robustness, true MID360 firing-time fidelity, or access to FAST-LIO's internal point-to-plane Hessian.
