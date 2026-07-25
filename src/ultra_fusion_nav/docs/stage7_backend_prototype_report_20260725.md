# Stage 7 Backend Prototype Report

Date: 2026-07-25

## Implemented boundary

`uf_backend_fusion` is the first offline backend increment. Each window state
uses 15 tangent-space variables:

`{p(3), theta(3), v(3), ba(3), bg(3)}`.

The current factors are linearized blocks for IMU delta, LiDAR pose, GNSS
position, optical-flow displacement, and RGB-D pose. The backend solves a
bounded weighted normal equation and records each factor's enabled flag,
reliability weight, covariance inflation, and effective weight.

This is intentionally not yet a full SE(3) nonlinear optimizer. IMU
preintegration, bias Jacobians, rotation manifold updates, robust losses, and
true marginalization priors remain open. The prototype exists to verify the
factor contract and scheduler coupling before adding those nonlinear pieces.

## Synthetic ablation

`ros2 run uf_backend_fusion run_backend_ablation --output <path>/ablation_table.csv`
injects one GNSS jump and compares fixed versus scheduler-weighted factors:

| Variant | Position RMSE |
|---|---:|
| fixed_weight | 0.035849 m |
| scheduler_weighted | 0.028934 m |

The 19% reduction is a deterministic unit-level result, not a rosbag or
end-to-end flight claim. The next gate is to convert the recorded LIO/GNSS/flow
streams into timestamped factor measurements and repeat this comparison on a
real replay with the scheduler timeline.

## Verification

The package builds in the Humble workspace and its four tests cover dynamic
GNSS down-weighting, factor disable/inflation bookkeeping, IMU relative
constraints, and bounded state retention.
