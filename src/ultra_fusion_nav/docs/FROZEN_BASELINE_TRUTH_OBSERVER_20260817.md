# Frozen Baseline Truth-Observer Validation (2026-08-17)

## Scope

This run freezes the estimator at commit `00efe986f29b94c00e080ec2b64e12d5862bf476`
(`checkpoint/core-cleanup-20260817`). The only source changes are test-harness
support for Gazebo-truth route feedback and offline analysis tools.

- Gazebo truth controls the route and is read by evaluators.
- Gazebo truth is not subscribed to by the unified estimator.
- ExternalNav consumption by EKF3 is disabled.
- LiDAR, FCU IMU, GNSS, optical flow and RGB-D visual reprojection remain enabled.
- RGB-D raw depth geometry, barometer factors and dynamic Z gauge remain disabled.
- The simulated barometer and MAVROS static-pressure streams are recorded.
- Every sensor source is admitted to the sliding window at most once.

The purpose is to observe estimator error without the estimator error feeding back
into vehicle motion.

## Runs

| Metric | Rectangle | Large figure-eight |
| --- | ---: | ---: |
| Route completed and disarmed | yes | yes |
| Route feedback | Gazebo truth | Gazebo truth |
| Route-active 3D RMSE | 0.0848 m | 2.5208 m |
| Route-active horizontal RMSE | 0.0191 m | 0.0811 m |
| Route-active vertical RMSE | 0.0826 m | 2.5195 m |
| Route-active final Z error | +0.1507 m | -4.3609 m |
| Full-run 3D RMSE | 0.0607 m | 3.2201 m |
| Full-run P95 | 0.0827 m | 4.4222 m |
| Full-run maximum | 0.9566 m | 5.2488 m |
| Endpoint error | 0.0273 m | 4.0187 m |
| Unified odometry source rate | about 9.2 Hz | about 9.1 Hz |
| Unified odometry maximum gap | 0.618 s | 0.819 s |
| Native worker discarded/overflow | 99 / 99 | 251 / 251 |
| Optimization rollbacks | 27 | 50 |

The rectangle is locally accurate but fails the strict 20 cm gate because of one
isolated 0.9566 m sample and backend timing/queue violations. The large
figure-eight has a real sustained vertical failure; its horizontal estimate remains
below 0.1 m RMSE while Z diverges by more than 4 m.

## Barometer Evidence

| Stream | Rate | Relative-height RMSE | Status |
| --- | ---: | ---: | --- |
| `/sim/barometer/pressure` | 20.0 Hz | 0.179-0.181 m | recorded, not fused |
| `/mavros/imu/static_pressure` | 100.0 Hz | 0.012-0.013 m | recorded, not fused |

The MAVROS topic carries values near 945 and therefore behaves as hPa despite the
`sensor_msgs/FluidPressure` Pa contract. The analysis tool detects this and applies
the x100 conversion; this unit mismatch must be corrected before a barometer factor
is enabled. Backend diagnostics confirm `barometer_factors=0` in both runs.

## Vertical Failure Timeline

During the large figure-eight, absolute Z error first crosses:

- 0.2 m at simulation time 73.389 s;
- 0.5 m at 75.108 s;
- 1.0 m at 141.801 s;
- 2.0 m at 142.723 s;
- 4.0 m at 145.234 s.

At the rapid 1-4 m collapse:

- LiDAR reports a weak Z direction (`D_L,z` about 0.96-0.98 and relative support
  falling to about 0.02-0.06), but frozen-baseline axis handoff is disabled, so its
  information scale remains `[1, 1, 1]`.
- RGB-D factor candidates become geometrically weak. The number of accepted visual
  factors remains frozen at 262 while state-consistency rejects increase.
- GNSS XY remains healthy, but GNSS Z NIS grows from 8.1 to 76.1. The backend then
  robustly downweights Z information from about 0.91 to 0.30, treating estimator
  disagreement as a reason to weaken the only absolute Z source.
- Optical flow is a true horizontal 2D factor and provides no Z observation.
- IMU supplies propagation but no absolute Z anchor.
- No barometer factor is active.

This is an estimator admission/axis-handoff failure, not a Gazebo route-control,
SITL, sensor-stream or map-collision failure. No optimizer exception occurred.

## Performance

For the figure-eight, backend solve time is 22.64 ms median, 100.23 ms P95 and
185.17 ms maximum. End-to-end backend callback time is 70.05 ms median, 438.43 ms
P95 and 619.55 ms maximum. The worker queue discards old LiDAR-triggered work, so
the current implementation is not yet reliably real-time despite a roughly 9-10 Hz
source-state rate.

## Next Single-Variable Changes

1. Enable LiDAR per-axis information handoff: keep strong axes unchanged and reduce
   only the observed weak axis. Do not add a second LiDAR pose factor.
2. Make GNSS Z recovery nonzero and hysteretic when the GNSS stream/fix is healthy;
   estimator disagreement must not permanently lock out the absolute axis.
3. Repair visual admission so valid textured-wall RGB-D geometry can contribute
   continuously, with batch information caps and no duplicate depth factor.
4. Fix the static-pressure unit contract, then test a short-lived relative barometer
   Z segment only when all regular Z sources are weak. It must not be a global
   duplicate altitude factor.
5. Remove callback and native-worker queue long tails before another long route.

After each change, replay the frozen bags first, then repeat the short rectangle.
Only after route-active Z RMSE and P95 are below 0.15 m should the large
figure-eight be repeated.

## Evidence

- `logs/frozen_baseline_20260817/rectangle_truth_observer/`
- `logs/frozen_baseline_20260817/figure8_truth_observer/`
- `phase_accuracy.json`: route-phase error summaries
- `barometer_accuracy.json`: pressure/height consistency
- `z_failure_evidence.json`: factor state at Z-error crossings
- `validation_acceptance.json`: strict acceptance gates
