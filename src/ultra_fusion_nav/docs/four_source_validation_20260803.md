# Four-source unified-window validation (2026-08-03)

## Scope

This milestone validates the ordinary-map simulation with FCU IMU, MID360,
GNSS and MTF01-style optical flow. Gazebo truth is used only by evaluators.
ExternalNav output is produced for continuity metrics but is not consumed by
ArduPilot EKF3 in this run.

The LiDAR path uses the direct C++ Gazebo-to-Livox adapter. FAST-LIO runs in
downstream-backend mode: it exports deskewed point-to-plane correspondences and
diagnostics, while the unified backend consumes native LiDAR factors, performs
the single IMU preintegration, and owns `/fusion/unified/odom`.

## Reproduction

```bash
cd "$HOME/multi-slam-github-staging"
LOG_DIR="$PWD/logs/uf_four_source_p05_20260803" \
FASTLIO_BACKEND_TRAJECTORY_FRONTEND=1 \
METRICS_DURATION=125 DRIFT_DURATION=115 \
VALIDATION_ENABLE_EXTERNALNAV_EKF3=0 \
ENABLE_RELIABILITY_RECORD=1 \
bash tools/run_unified_rectangle_validation.sh
```

The run completed the conservative rectangle and landed. The first accepted
SITL takeoff command did not start the motors; the bounded second attempt in
the route state machine recovered the launch.

## Accuracy

| Output | ATE RMSE | Motion ATE RMSE | ATE P95 | Yaw RMSE | Turning yaw RMSE | RPE 1 s |
|---|---:|---:|---:|---:|---:|---:|
| Unified window | 0.0617 m | 0.0617 m | 0.1371 m | 0.564 deg | 0.737 deg | 0.0290 m |
| FAST-LIO diagnostic | 0.0656 m | 0.0628 m | 0.1410 m | 0.703 deg | 1.155 deg | 0.0385 m |

The evaluator associated both outputs to truth by source header stamp and did
not apply its diagnostic scale estimate. Truth was not consumed by either
estimator.

## Factor and ownership evidence

- Native point-to-plane LiDAR factors: 1111 relinearized, 0 pose fallbacks,
  0 invalid factors, 400 matches in the latest diagnostic frame.
- IMU factors: 1110, with 0 invalid intervals and 0 pair timeouts.
- GNSS factors: 1109; duplicate observations were not reused as factors.
- Optical-flow factors: 276 accepted from 457 attempts. Rotation gating
  disabled 69 attempts and quality/range gating disabled 19.
- Backend states/factors remained bounded at 8/24; optimization reported no
  solver errors and four integrity rollbacks.
- Frontend state seeds remained disabled and unused.

`preserve_lio_anchor` is still enabled and produced 89 anchor overrides. This
protects the current milestone from unobservable gaps but prevents claiming a
final, completely FAST-LIO-independent state owner.

## Timing and throughput

- Observed simulation RTF: 0.620. Algorithm metrics use ROS simulation time;
  CPU/runtime metrics use wall monotonic time.
- Unified odometry source-stamp rate: 9.85 Hz, with two gaps above 0.25 s and
  no duplicate or regressing stamps.
- Backend solve time: median 75.0 ms, P95 95.8 ms.
- Full callback time: median 94.4 ms, P95 168.1 ms.
- LiDAR and FCU IMU input rates at startup were 10.1 Hz and 99.5 Hz.

## Optical-flow finding

MAVLink `OPTICAL_FLOW` decipixel quantization is active, and the simulated FCU
IMU clock offset now uses a low rolling percentile so callback latency is not
fully mistaken for clock offset. The bridge reported p05/p50/p95 offsets of
0.592/0.605/0.631 s in this loaded run.

The standalone flow gate still failed: expected axes were recovered, but scale
was 0.517, normalized RMSE 0.840, and correlation 0.482. Dual-IMU diagnostics
showed that the camera/Gazebo-IMU roll integral and MAVROS FCU roll integral do
not agree, while their pitch axis agrees. Therefore the remaining lateral
flow error must not be hidden by looser reliability thresholds. The unified
window remains stable because the flow factor is covariance-bounded and
gated, not because the raw flow model is already correct.

## Next gates

1. Record instantaneous Gazebo flow-IMU and FCU HIGHRES_IMU rates on one clock
   to separate roll-axis filtering from residual time mapping.
2. Correct the simulation IMU/flow roll relationship, then require the
   standalone flow accuracy gate to pass before retuning its scheduler score.
3. Add body-frame anisotropic flow covariance if real MTF01P data confirms
   different forward/lateral noise; rotate that covariance into the residual
   frame instead of applying a scalar weight.
4. Disable `preserve_lio_anchor` only after an ownership A/B run remains
   bounded with native LiDAR + IMU + GNSS/flow.
5. Validate ExternalNav continuity and reset semantics in a separate EKF3
   closed-loop run; this milestone does not claim FCU consumption.

## Evidence paths

- `logs/uf_four_source_p05_20260803/unified_accuracy.json`
- `logs/uf_four_source_p05_20260803/fastlio_accuracy.json`
- `logs/uf_four_source_p05_20260803/unified_runtime_metrics.json`
- `logs/uf_four_source_p05_20260803/flow_gazebo_accuracy.log`
- `logs/uf_four_source_p05_20260803/reliability_scores.csv`
