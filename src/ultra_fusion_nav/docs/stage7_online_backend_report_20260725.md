# Stage 7 Online Backend Report

Date: 2026-07-25

## Runtime chain

The simple-map experiment now has an online companion-computer path:

```text
/lio/odom ----------------------\
/sensors/gnss/fix ---------------> unified_backend_fusion
/sensors/optical_flow/rad -------/          |
/sensors/imu --------------------/          v
                              /fusion/unified/odom
                                         |
                                  external_nav_gate
                                         |
                                /mavros/odometry/out
```

The node does not subscribe to `/uav/local_odom`, MAVROS fused local position,
or Gazebo truth. Truth is consumed only by the evaluator. LIO is the local pose
anchor; GNSS and optical flow are optional factors, and the scheduler supplies
their factor weight and covariance inflation. IMU uses midpoint preintegration
with first-order bias-aware Δp/Δv/Δθ and bias random-walk rows.

## Online configuration

`online_backend.yaml` uses an 8-state online window for CPU budget, while the
offline replay remains a 20-state comparison. `preserve_lio_anchor=true` keeps
the LIO pose factor enabled when its reliability diagnostic is stale; once a
fresh valid degradation score exists, continuous scheduler down-weighting is
still applied. A future SE(3) backend can safely disable LIO only after adding
an explicit prior/relocalization path.

## Simple-map experiments

All runs used the simple `simple_apm_rgbd_mid360` world, `FLOW_USE_PHYSICS=false`,
D435 bridge disabled, and the existing rectangle flight. The v1-v4 runs were
debugging iterations; v5 was the sparse-solver baseline, v6 used finite-difference
bias probes, and v7 is the retained analytic-recursion configuration.

| Run | Unified samples | Unified ATE RMSE | Unified yaw RPE | RTF median | Result |
|---|---:|---:|---:|---:|---|
| v1, 20-state | 60 | 0.704 m | 46.5 deg | 0.269 | rejected: solver backlog and yaw unobservable |
| v2, 8-state + yaw unwrap | 366 | 0.059 m | 31.4 deg | 0.636 | rejected: intermittent yaw reset |
| v3, anchor diagnosis | 404 | 0.056 m | 23.6 deg | 0.819 | rejected: occasional LIO factor disable |
| v4, anchor preserved | 406 | 0.071 m | 0.313 deg | 0.791 | retained for next iteration |
| v5, sparse solve + anchor | 451 | 0.0479 m | 0.364 deg | 0.999 | retained |
| v6, bias-aware local IMU | 408 | 0.0528 m | 0.360 deg | 0.892 | retained |
| v7, analytic bias recursion | 456 | 0.0582 m | 0.363 deg | 0.999 | retained |

For v7, `/fusion/unified/odom` reached 7.75 Hz through the ExternalNav gate,
with source-to-arrival rate ratio `1.0001`. The same run's raw FAST-LIO
evaluator reported position RMSE `0.0354 m` and yaw RMSE `0.1079 deg`; unified
position ATE is therefore not yet an accuracy improvement claim. Replacing
the six extra finite-difference integrations with SO(3) analytic recursion
restored the RTF median from `0.892` to `0.999` while preserving the bias-aware
factor contract.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
RUN_ID=online_unified_test \
ENABLE_UNIFIED_BACKEND=1 ENABLE_RELIABILITY=0 \
FLOW_USE_PHYSICS=false ENABLE_D435_BRIDGE=0 \
HEADLESS=1 SHOW_FLOW_WINDOW=0 FLOW_DEBUG=false RVIZ=0 \
bash src/ultra_fusion_nav/scripts/run_lio_baseline_experiment.sh
```

The experiment writes `trajectory_metrics.json`, `report.json`,
`simulation_performance.json`, and the unified backend logs below `logs/`.

## Next gate

The dense Python normal-equation solve has been replaced by an optional SciPy
sparse solve with a dense fallback. The local factor now consumes analytic
first-order bias Jacobians, but the remaining gate is manifold SE(3)
relinearization and proper bias covariance propagation. Only after that should
GNSS outage, optical-flow low texture, LiDAR degeneration, and dynamic-map
protection be used for a final fixed-vs-dynamic accuracy claim.
