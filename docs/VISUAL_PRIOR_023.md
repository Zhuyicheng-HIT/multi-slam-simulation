# VISUAL-PRIOR-023: RGB-D Direct Prior Causal Audit

## Scope

This audit keeps the VISUAL-AB-022 frozen moving-tunnel bag, `rgbd_direct`,
the same HXY/Flow/GNSS/IMU/Dynamic settings, and the existing 1.0 m integrity
translation limit. Truth is used only by the offline scorer. No formal
marginalization equation or integrity threshold was changed.

Bag SHA256 remains
`180d4f0e8985ae7253dfa8edac242f0dd9f814c2b4d1c7b93c0088351c0375a9`.

## Causal replay evidence

The valid no-extra-hot-path-cost replay from VISUAL-AB-022 reproduced:

| Replay | First rollback | Time | Rollbacks | Visual factors | XY RMSE |
|---|---:|---:|---:|---:|---:|
| `rgbd_direct`, LiDAR+GNSS disabled | tx 313 | 62.007 s | 159 | 11 | 0.310 m (truncated) |
| Same, visual prior excluded | tx 461 | 76.923 s | 147 | 11 | 20.850 m (later truncated) |

The exclusion changes only whether historical `rgbd_direct` factors are put
into the next Schur prior. Current-window RGB-D factors remain enabled. Thus
the disappearance of the tx-313 failure is causal evidence that historical
visual information is required for the early good trajectory and is also part
of the early failure mechanism. It is not evidence that visual factors alone
are numerically wrong: without them, the IMU-only trajectory diverges badly.

The first baseline failure had no active visual, LiDAR, or GNSS factor in the
window. Its proposed correction was:

```
translation = [+0.100, -1.283, -0.046] m  (norm 1.288 m)
velocity    = [+0.120, -1.289, +0.005] m/s
rotation norm = 0.0658 rad
```

The correction is overwhelmingly negative Y, not a random 3-D jump. The
integrity gate only rejects this already-produced proposal; it does not create
the direction.

## Prior information and gradients

The new read-only diagnostics expose the stored Schur prior, its relinearized
current Hessian/gradient, position eigenspectrum, per-state position gradients,
and per-source normal equations. Representative direct-dual diagnostic
snapshots near the failure region show:

- prior full-state condition number about `3.66e2` and position condition about
  `8.6`, with positive rank and no indefinite prior;
- prior position trace about `81.0` and largest position eigenvalue `39.8`;
- at the first failed proposal in the earlier valid replay, the prior gradient
  was small at the newest state, while the accumulated prior gradient was
  dominated by the historical state block and Y position;
- source attribution contained 11 `rgbd_direct` factors in the historical
  prior at the valid tx-313 baseline failure. The corresponding exclusion
  replay explicitly recorded the omitted visual source factors.

The prior is therefore not globally ill-conditioned or indefinite. Its main
effect is a historical linearization anchor distributed through the fixed-lag
state blocks. As the active state moves, that anchor and the fresh IMU
preintegration disagree in the Y-like weak direction. The resulting correction
is amplified by weak absolute observability, then exceeds the integrity limit.

The available source attribution counts are exact; a scalar “visual percent of
prior information” is not mathematically meaningful for the dense Schur prior
without retaining per-source cross terms. The new diagnostics retain the
source gradients and position blocks needed for that next offline decomposition.

## Integrity and self-lock

`validate_optimized_state` computes the correction from the transaction's
initial state to the proposed state, checks cost/information validity, and only
then restores the transaction snapshot. The first failure is therefore an
integrity exposure of a prior/IMU inconsistency, not a gate-originated motion.

After restore, the same committed state/factor snapshot is reused while sensor
time advances. Subsequent attempts repeatedly produce excessive corrections,
so the transaction index and active prior remain effectively frozen while
`optimization_rollbacks` increases. This is a real prediction/state-progress
self-lock. It is distinct from estimator divergence: the visual-off control
continues committing states but reaches `53.33 m` XY RMSE over the full bag.

## Diagnostic implementation caveat

The initial implementation recomputed every source normal equation on every
transaction. That changed callback timing and could change asynchronous visual
association/admission. It was corrected so the normal path only copies state;
full source/prior diagnostics run after a proposed translation correction of at
least `0.10 m`. Even so, the final diagnostic-enabled replay showed that any
extra work can alter this timing-sensitive bag. Therefore the two replays used
for causal claims above are the earlier fixed-condition no-diagnostic runs; the
new fields are for targeted failure capture and offline follow-up, not for
claiming a new benchmark score.

## Root-cause conclusion

The strongest supported explanation is:

1. `rgbd_direct` supplies real early XY information and is marginalized into a
   dense prior.
2. In the LiDAR+GNSS-degraded interval, the fresh IMU propagation drifts in a
   weak Y-like direction while the historical visual prior retains a different
   linearization anchor.
3. The next solve proposes a large negative-Y translation/velocity correction.
4. The integrity gate correctly rejects it and restores the snapshot.
5. Because no state is committed, the same prior/propagation conflict repeats,
   producing a self-lock.

This rules out “integrity threshold is the root cause” and does not support
blindly weakening the gate. It also does not yet prove that the visual prior is
over-strong by itself; the no-visual-prior control loses the useful constraint
and fails later for ordinary IMU drift.

## Minimum next repair

Do not change thresholds or formal Schur mathematics first. Add an offline
per-source Schur decomposition using the captured source Jacobian/gradient
blocks, then test a narrowly scoped prior consistency policy:

- retain current-window `rgbd_direct` factors;
- on marginalization, monitor the visual prior's weak-direction residual and
  gradient against fresh IMU propagation;
- attenuate or relinearize only the inconsistent historical visual component,
  preserving strong metric depth information;
- require a full dual-degradation replay with no rollback self-lock before any
  production change.

Current status: **DO_NOT_PROMOTE** the present direct-prior behavior as a
production default until that consistency policy is validated.
