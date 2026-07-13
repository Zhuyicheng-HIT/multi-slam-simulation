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

The pre-adapter timestamp-aligned baseline passed a complete rectangle flight with position RMSE `0.046 m`, yaw RMSE `0.125 deg`, FAST-LIO/FCU-gyro correlation `0.874`, estimated lag `20 ms`, and zero timestamp regressions.

The first adapter flight produced position RMSE `0.225 m`, yaw RMSE `5.344 deg`, voxel-overlap P05 `0.378`, centroid-jump P95 `3.153 m`, and zero timestamp regressions. Independent TUM evaluation reported ATE RMSE `0.159 m`, translational RPE RMSE `0.021 m`, and rotational RPE RMSE `0.553 deg`.

A throttled adapter rerun produced position RMSE `0.355 m` and yaw RMSE `5.311 deg`; the following adapter-disabled control run initialized IMU and the map but did not publish `/Odometry`. This proves the current regression is not attributable solely to the adapter. The run also showed bursts of minimally repaired non-monotonic FCU IMU timestamps.

Stage 2 is functionally complete and its interfaces are runtime-tested, but repeatable quantitative acceptance remains open. No estimator parameter change was retained from these degraded runs.
