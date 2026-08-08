# Estimator hot-path breakdown

## Final opt-in profile (joint-map r65)

| Requested boundary | Measured boundary | P50 (ms) | P90 (ms) | P95 (ms) | Max (ms) |
|---|---|---:|---:|---:|---:|
| LiDAR callback preparation | `callback_prepare` | 9.011 | 13.878 | 15.463 | 23.572 |
| state creation | `callback_add_state` | 6.715 | 11.366 | 12.563 | 21.314 |
| IMU preintegration residual/Jacobian | `factor_imu_preintegrated` | 0.260 | 1.113 | 1.550 | 4.585 |
| NativeLidarFactor admission | `callback_lidar_factor` | 0.399 | 0.727 | 1.033 | 2.868 |
| Native point-plane linearization | `factor_lidar_point_plane` | 0.244 | 1.510 | 2.015 | 5.438 |
| GNSS factor | `factor_gnss` | 0.006 | 0.027 | 0.315 | 1.908 |
| optical-flow factor | `factor_optical_flow_body` | 0.067 | 0.147 | 0.216 | 4.822 |
| pending association + auxiliary construction | `callback_aux_factors` | 0.318 | 1.888 | 2.462 | 9.709 |
| visual residual/Jacobian | `factor_visual_reprojection` | 0.496 | 1.268 | 2.411 | 8.551 |
| factor graph assembly/linearization | `factor_graph_linearization` | 5.578 | 17.048 | 21.058 | 35.702 |
| linear solve | `linear_solve` | 0.316 | 0.489 | 0.535 | 2.073 |
| state update | `state_update` | 0.362 | 1.049 | 1.634 | 4.529 |
| integrity/residual/covariance checks | `callback_post_optimize` | 2.131 | 5.988 | 7.218 | 253.043 |
| transaction snapshot | `callback_snapshot` | 0.274 | 0.795 | 1.026 | 2.192 |
| commit + odom/diagnostic publish | `callback_publish` | 1.458 | 4.934 | 6.011 | 252.504 |
| complete optimize | `solver_optimize_total` | 40.765 | 64.633 | 71.980 | 116.799 |
| marginalization | `solver_marginalization` | 6.662 | 11.304 | 12.490 | 21.208 |

The few 250 ms post/publish maxima are scheduler/preemption outliers; P50/P95
and the zero-error/zero-rollback counters are the decision metrics.

FRS is computed in the separate reliability process and consumed inside the
bounded auxiliary-factor phase.  This V2 instrumentation measures its process
cost and the backend consumption boundary, but does not split D_V scoring from
scheduler state-machine work.  Likewise, Native message deserialization,
correspondence conversion, scan prediction and trajectory-front-end transport
remain aggregated in callback preparation/state creation.  No unsupported
per-stage number is invented for those sub-boundaries.

## Process and queue observations

In r65 the estimator process used 2.164% of total WSL capacity at the median,
the visual frontend 1.266%, and shared mapping 1.747%.  Total WSL CPU/RAM were
40.494% and 3.314 GiB with profiling and joint mapping active.  Native worker
queue overflow, pending overflow, dropped/invalid Native factors and duplicate
visual submissions were all zero.  The profiler keeps at most 4096 samples per
stage and emits only aggregate diagnostics.
