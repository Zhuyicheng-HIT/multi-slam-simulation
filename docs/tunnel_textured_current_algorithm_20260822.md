# Textured tunnel current-algorithm run - 2026-08-22

## Environment change

The tunnel world now contains visual-only, collision-free features:

- low-contrast asymmetric floor blocks around the existing transverse bands;
- alternating wall panels on both sides at approximately 14 m spacing;
- no new collision geometry, truth input, route feedback, or LiDAR surface.

The online launch path now exposes
`subspace_information_handoff_enabled`; its default remains `false`.  This
run explicitly enabled it to exercise the current B3 candidate.

World and launch changes were built with `colcon build --symlink-install` for
`multi_slam_uav_sim` and `uf_backend_fusion`.

## Run outcome

Evidence: `logs/tunnel_textured_current_20260822_130557`.

The simulation was headless and reached takeoff and the route phase.  It
advanced to the 14 m checkpoint, then the unified-backend position error
prevented convergence.  The controller entered its safety hold and refused a
command below the altitude safety envelope.  The run was stopped after the
hold window with Ctrl-C; it did not complete the route or land normally.

The route therefore cannot be treated as a completed navigation benchmark.
It is still useful as a sensor-admission and failure-mode experiment.

## Total fused accuracy

The final causal metrics cover 1611 samples and 160.971 s of simulation time,
including the protected hold:

| Metric | Unified backend |
| --- | ---: |
| 3-D RMSE | 1.646 m |
| 3-D P95 | 4.491 m |
| 3-D maximum | 5.365 m |
| Endpoint error | 0.341 m |
| XY RMSE | 1.646 m |
| Z RMSE | 0.0206 m |
| Yaw RMSE | 12.885 deg |

The acceptance threshold was not met.  The first sustained 20 cm error
occurred at 58.405 s.  The route stopped at 14 m, so these numbers are not
comparable to the completed fixed-bag B3 replay without restricting both
runs to the same time interval.

## Per-sensor evidence

| Source | Received / attempted | Selected or recorded | Accepted solver factors | Rejected or disabled | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| LiDAR | 1747 | 1747 | 643 relinearized | 1104 prediction-gate rejects | rank 3, condition 69.52; no structural-degenerate frames reported |
| IMU | 17643 | 1751 intervals | 1751 | 0 invalid, 0 timeouts | continuous 100 Hz bridge; supplies all window links |
| GNSS | 875 / 875 consumed | 873 records | 872 | 2 scheduler-disabled; 118 XY robust downweights | no whole-factor NIS rejection; Z retained |
| Optical flow | 1845 / 1751 attempts | 1845 packets committed | 23 | 1684 scheduler-disabled, 14 coverage-disabled, 33 invalid | features reached the flow front end, but reliability policy admitted few factors |
| RGB-D direct | 285 / 285 attempts | 285 score matches | 115 | remaining batches prefit-rejected or not admitted | final depth RMSE 0.201 m, photometric RMSE 0.952, valid ratio 1.0 |

For comparison, the FAST-LIO-local trajectory in the same partial run had
causal 3-D RMSE 4.101 m, P95 5.005 m, and endpoint error 4.321 m.  This is a
diagnostic sensor/front-end reference, not the final fused result.

## Runtime and resource evidence

- committed states: 1752;
- transaction rollbacks: 0; aiding transactions rejected/recovered: 2/2;
- solver mean/max: 4.671/29.633 ms;
- backend CPU P50/P95: 26.0/32.05%; backend RSS P50/P95: 108.9/120.7 MiB;
- Gazebo CPU P50/P95: 201.3/208.5%; Gazebo RSS P50/P95: 742.7/834.2 MiB;
- GPU utilization P50/P95: 4/4%; GPU memory P50/P95: 1281/1302 MiB.

## Interpretation

The added texture did what it was intended to do at the input level: RGB-D
formed 115 factors and optical flow formed 23, whereas the earlier tunnel
replay had zero flow factors and far fewer visual factors.  It did not by
itself solve navigation because the LiDAR prediction gate rejected 1104
frames and the route controller stopped early.  The next experiment must
separate visual admission from LiDAR prediction-gate behavior.

## Next directional-degradation iteration

1. Freeze this textured world and run a short 20-30 m rectangle, with the
   current algorithm and no code changes, to measure sensor admission without
   route-controller accumulation.
2. Make the paper's existing directional degradation output the single source
   of truth.  Do not add a second LiDAR eigendecomposition or detector.
3. Export per-direction information/support for each source from existing
   factor builders: LiDAR, GNSS, optical flow XY, and RGB-D geometry.  IMU is
   represented by propagated temporal covariance, not artificial position
   amplification.
4. Apply bounded directional handoff at factor assembly.  A healthy source
   may recover at most its nominal information in a LiDAR-weak direction;
   strong LiDAR directions receive no extra source weight.
5. Keep source-local health, age, jump, and residual gates authoritative.
   Directional handoff must never resurrect a stale or intrinsically bad
   factor.
6. For the fixed-lag window, update only a bounded recent history of factors
   during a confirmed continuous degradation episode, with hysteresis and
   causal timestamps.  Do not rewrite the full historical prior.
7. Use one-variable A/B tests in this order: GNSS directional handoff,
   optical-flow XY handoff, RGB-D directional handoff, then combined
   allocation.  Report total and per-sensor received/selected/accepted/
   rejected counts and precision for every run.
8. Before any freeze decision, fix the independent LiDAR prediction-gate
   failure exposed here.  Texture and directional factor allocation must not
   be tuned simultaneously with that gate.
