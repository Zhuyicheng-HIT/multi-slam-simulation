# HXY-PRIOR-007: marginal-prior historical LiDAR audit

Both replays used the HXY-INTERACTION-006 frozen bag and settings, including
the current weak-subspace cap. The only variable was the opt-in
`marginal_prior_suppress_historical_lidar_weak` diagnostic switch.

| replay | XY RMSE | 3D RMSE | endpoint norm | commits | input loss |
|---|---:|---:|---:|---:|---:|
| A baseline | 0.79014 m | 0.79023 m | 2.050 m | 672 | 0 |
| B suppress historical LiDAR weak mode at marginalization | 0.78373 m | 0.78396 m | 2.054 m | 672 | 0 |

The small improvement is within the replay's executor/interleaving variation;
it is not evidence of a dominant prior failure. Both runs processed native
sequence 131 through 799 with no queue overflow, supersede, latest-only skip,
or rollback.

## Attribution evidence

The first marginal prior containing a historical LiDAR source is transaction 9
(`t=31.515 s`). At that point the source bookkeeping reports LiDAR pre-Schur
translation trace `383331` and attenuated weak trace `961.84`. The latter is a
formation-time source diagnostic, not an independently identifiable component
of the nonlinear Schur prior.

Across 664 prior samples, the projected current weak-direction position
information fraction has median about `0.327` and P95 `1.0`; this fraction is
the prior's total position block projection, including IMU, previous priors and
cross-state Schur effects. It cannot be assigned to LiDAR alone. Recursive
source counts confirm that the prior accumulates LiDAR history, but do not make
that history separable after Schur elimination.

The suppression replay reconstructs each new marginalization input with the
historical LiDAR weak mode set to zero while keeping strong LiDAR, GNSS, IMU,
current-window factors, one-observation-one-factor semantics, and prior use
otherwise unchanged. It changes the accumulated attenuated weak trace only
slightly (about `106026` to `105192` late in the run) and changes XY RMSE by
about `6 mm`. This is far smaller than the original weak-direction competition
effect and does not materially change the endpoint.

## Conclusion

`marginal_prior` is not the primary source of the remaining ~0.79 m horizontal
error. No marginalization mathematics is promoted or changed. The prior does
retain historical LiDAR and can become a later limitation, but the measured
impact of removing its weak contribution is minor.

The next principal suspect is the absolute/GNSS plus IMU replay residual and
its timing/trajectory alignment after the current-window cap. The next replay
should isolate GNSS+IMU residual evolution and prior-independent state error,
without changing HXY thresholds or increasing GNSS weight.
