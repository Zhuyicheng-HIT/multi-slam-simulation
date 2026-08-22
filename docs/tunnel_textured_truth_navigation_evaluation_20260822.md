# Textured tunnel truth-navigation evaluation - 2026-08-22

## Boundary

This run uses the textured tunnel world and current B3 directional handoff.
`VALIDATION_ENABLE_EXTERNALNAV_EKF3=0` was used, and route feedback was
`gazebo_truth`.  The unified backend was observer-only; Gazebo truth was not
used inside the estimator.  The route completed 146.1 m and landed/disarmed.

Evidence: `logs/tunnel_textured_truthnav_20260822_132134`.

## Fused trajectory accuracy

| Metric | Unified backend |
| --- | ---: |
| 3-D RMSE | 1.467 m |
| 3-D P95 | 2.141 m |
| 3-D maximum | 9.961 m |
| Endpoint error | 0.016 m |
| XY RMSE | 1.465 m |
| Z RMSE | 0.069 m |
| Yaw RMSE | 60.42 deg |

The route and runtime gates passed, but accuracy acceptance failed because
3-D RMSE/P95 and XY RMSE exceed 0.20 m.  The first sustained 20 cm error was
at 48.609 s.  `truth_used_by_estimator=false` was confirmed.

## Per-sensor evidence

| Source | Received / attempted | Accepted factors | Rejected or disabled | Diagnostic result |
| --- | ---: | ---: | ---: | --- |
| LiDAR | 2978 received | 1119 accepted; 1120 relinearized | 935 prediction-gate rejects | final prediction gate saw 31.18 m position and 118.2 deg yaw innovation; rank/condition was not the main limiting statistic |
| IMU | 29941 received | 2952 factors | 0 invalid, 0 timeouts | continuous bridge and all window intervals covered |
| GNSS | 1492 received/consumed | 1474 accepted | 2 scheduler-disabled; 396 XY robust downweights; 0 whole-factor NIS rejects | final prefit residual norm 0.046 m; Z factors retained |
| Optical flow | 3132 received; 2981 attempts | 9 factors | 2779 scheduler-disabled, 10 coverage-disabled, 7 invalid | LOS residual mean/p95 0.198/0.470 rad/s; most data was rejected by reliability policy |
| RGB-D direct | 1008 received/attempted | 884 factors | no time/track rejection; remaining factors not admitted by scheduler/transaction state | final factor reason `accepted_rgbd_direct`; PnP inlier ratio 1.0, rank 6 |

The FAST-LIO-local diagnostic trajectory, used only as a sensor/front-end
reference, had causal 3-D RMSE 2497.8 m and endpoint error 11633.4 m.  This
shows the local LiDAR trajectory was already unusable under the long run;
the unified backend bounded the final position error much better, but not to
the required accuracy.

## Runtime and completion

- route checkpoints: 73, planned distance 146.44 m;
- simulation duration: 294.98 s; normal landing and disarm confirmed;
- committed states: 2982; rollbacks: 0;
- aiding transactions rejected/recovered: 2/2;
- solver mean/max: 7.066/62.278 ms;
- unified odometry: 10.001 Hz, no duplicate/regressed/stale stamps;
- backend CPU P50/P95: 32.5/40.0%; backend RSS P50/P95: 119.5/140.1 MiB;
- Gazebo CPU P50/P95: 191.8/203.6%; GPU utilization P50/P95: 7/9%.

## Conclusion

Disabling EKF3 and using Gazebo truth successfully isolated control from
estimator evaluation.  The vehicle completed the route, so the remaining
1.467 m fused RMSE is an estimator problem rather than a flight-control
feedback failure.  Texture increased visual admission substantially, but
optical flow remained mostly disabled and LiDAR prediction gating rejected
935 frames.  The next A/B should therefore target LiDAR prediction-gate
handling and directional reliability allocation separately, while retaining
truth navigation and EKF3 disabled.
