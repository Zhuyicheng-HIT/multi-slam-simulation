# HXY-KERNEL-004: rotated LiDAR subspace C++ kernel

Date: 2026-08-25

## Result

The arbitrary rotated translation-subspace transform now runs in the C++
point-plane normal kernel.  It is algebraically equivalent to the Python
prototype, including the combined axis-information-scale path.  The active
window still updates every raw historical LiDAR factor and deliberately leaves
the marginal prior unchanged.

The C++ migration removes the Python solver penalty and the replay-only
latest-only loss, but it does not improve C's estimation accuracy.  C remains
materially better than A and materially worse than B.  The new prior diagnostics
do not support changing marginalization next: during the one-weak-mode drift,
only a small fraction of the live prior position information lies in the current
weak direction, while current-window LiDAR recovery factors continue to enter
the solver.

## Frozen input and replay contract

All runs use the HXY-DIAG-002 frozen bag:

`/home/ld666/projects/hxy-diag-002/frozen_bag`

| File | SHA256 |
|---|---|
| `metadata.yaml` | `d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b` |
| `replay_bag_0.db3.zstd` | `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56` |

Truth is used only by the offline accuracy recorder.  A and C disable the
estimator's latest-only shortcut for this replay; all runs retain queue and QoS
depth 1024, rate 0.5, regenerated LiDAR scheduler input, two executor threads,
and one numeric thread.  C retains the HXY-SUBSPACE-003 settings without tuning:
enter threshold `0.15`, exit threshold `0.25`, and weak information scale
`0.001`.

| Run | Mode | Output |
|---|---|---|
| A | stable `hybrid/adaptive` | `/home/ld666/projects/hxy-diag-002/replay_A_kernel_fair` |
| B | PR17 `paper_eq19/paper_eq15` | `/home/ld666/projects/hxy-diag-002/replay_B_kernel_004` |
| C | C++ rotated subspace `hybrid/adaptive` | `/home/ld666/projects/hxy-diag-002/replay_C_kernel_004_fair` |

## Kernel math and equivalence

The kernel first accumulates the robust point-plane 6-DoF normal.  For blocks
`H_tt`, `H_tr`, and `H_rr`, it computes the same conditional quantities as the
Python prototype:

`M = H_tr pinv(H_rr)`, `S = H_tt - M H_tr^T`, and
`g_c = g_t - M g_r`.

For information-scale matrix `D = U diag(s_i) U^T` and its PSD root `P`, the
kernel writes:

`H'_tt = P S P + M H_rr M^T`

`g'_t = D g_c + M g_r`.

The rotation block and translation-rotation coupling remain unchanged.  The
existing diagonal axis information scale, if enabled, is applied to the raw
translation Jacobian before this transform, exactly as in Python.  This supports
one to three arbitrarily rotated weak modes and preserves one observation as one
factor.

Unit tests compare the C++ and Python Hessian, gradient, and cost both with and
without simultaneous axis scaling at absolute/relative tolerance `1e-10`.
All 310 backend tests pass.  Across the replay trace, normalized information
retention is `0.001000000000000005` mean for 461 weak-mode observations and
`0.9999999999999999` mean for 847 strong-mode observations; extrema differ from
the requested values only at approximately `3e-15`.

## A/B/C accuracy

The fair common scoring interval is `30.723 <= t <= 76.993 s`.

| Metric | A stable | B PR17 | C C++ subspace |
|---|---:|---:|---:|
| Matched samples | 462 | 462 | 454 |
| 3D RMSE (m) | 17.218 | 0.451 | 6.565 |
| XY RMSE (m) | 17.218 | 0.449 | 6.565 |
| Z RMSE (m) | 0.112 | 0.043 | 0.024 |
| 3D P95 (m) | 46.819 | 1.086 | 14.705 |
| 3D max (m) | 60.068 | 1.239 | 26.090 |
| Last scored error (m) | 60.068 | 1.221 | 26.090 |

Error projected onto each run's instantaneous LiDAR eigenbasis, using exact
source-stamp matches, is:

| Projection RMSE (m) | A | B | C |
|---|---:|---:|---:|
| Weakest direction | 12.515 | 0.376 | 4.971 |
| Strongest direction | 5.787 | 0.126 | 1.621 |

C therefore remains clearly better than A, but the C++ migration itself does
not improve accuracy over Python C (`6.457 m` XY RMSE in HXY-SUBSPACE-003 versus
`6.565 m` here).  B is still better because it admits no LiDAR factor, not
because it preserves useful strong-direction LiDAR information.

## Admission, timing, and key events

| Quantity | A | B | C |
|---|---:|---:|---:|
| Native received | 678 | 678 | 678 |
| Trace records | 669 | 672 | 669 |
| LiDAR solver admitted | 452 | 0 | 436 |
| Optimized states committed | 466 | 672 | 447 |
| Integrity rollbacks | 203 | 0 | 222 |
| Prediction recovery factors | 0 | 0 | 107 |
| Queue overflow / supersede / latest skip | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

Nine received factors in A/C are startup or non-transaction inputs; every
factor that reached the transaction worker was processed in source order.
Latest-only loss is therefore eliminated.  A and C still suffer many integrity
rejects, which are estimator decisions rather than dropped input.

Actual sensor accounting is:

| Source | A received / attempted / solver factors | B | C |
|---|---:|---:|---:|
| LiDAR | 678 / 669 / 452 | 678 / 672 / 0 | 678 / 669 / 436 |
| IMU | 7513 / 668 / 668 | 7513 / 671 / 671 | 7513 / 668 / 668 |
| GNSS | 375 / 363 / 265 | 375 / 364 / 364 | 375 / 363 / 257 |
| Optical flow | 951 / 668 / 0 | 951 / 671 / 0 | 951 / 668 / 0 |
| Visual/RGB-D | 57 / 57 / 0 | 57 / 57 / 0 | 57 / 56 / 0 |

Optical flow was scheduler-disabled on nearly all attempts.  Visual/RGB-D was
received and scored but failed the visual quality/FRS gate, so GNSS was the only
active external horizontal absolute constraint.  The factor counts, not the
topic receipt counts, are the relevant estimator contributions.

For C, first degeneracy detection, first projector attenuation, and first actual
solver use of an attenuated LiDAR factor all occur at transaction 1, native scan
131, `t=30.723 s`.  First prediction hard reject/non-admission is transaction
331, scan 461, `t=63.723 s`; the first recovery-floor factor is transaction 333,
scan 463, `t=63.921 s`.

| Timing from transaction trace | Python C | C++ C |
|---|---:|---:|
| Solver mean (ms) | 50.651 | 15.049 |
| Solver median (ms) | 39.848 | 12.486 |
| Solver P95 (ms) | 104.412 | 36.704 |
| Solver max (ms) | 181.800 | 56.870 |

The C++ kernel cuts mean solver time by about 70%.  End-to-end callback timing
is not real-time-normal: C's `pre_state` mean/P95 is `189.9/797.7 ms`, close to
A's `171.5/724.2 ms`.  This work is outside the projector and explains the
remaining wall-time backlog.  It no longer changes which input frames are
processed because latest-only is disabled for the frozen replay.

## Marginal-prior diagnosis

The first marginal prior is formed and participates in optimization at
transaction 9, scan 139, `t=31.515 s`.  Diagnostics recursively retain source
factor counts and pre-Schur LiDAR translation trace, and project the live prior's
position diagonal blocks into the current weak projector.  They do not alter
the Schur complement or prior Hessian.

At transaction 9, the prior contains one historical LiDAR factor.  By the last
one-weak-mode transaction it contains 436 historical LiDAR factors.  During the
413 one-weak-mode prior samples, the current weak-direction fraction has median
`0.00296` and P95 `0.00555`.  At C horizontal-error crossings of 0.05, 0.2, 1,
5, and 10 m, that fraction is respectively `0.00349`, `0.00287`, `0.00234`,
`0.00150`, and `0.00170`.

This is not an exact per-source decomposition of the nonlinear Schur prior:
cross-state blocks and source interactions cannot be assigned uniquely after
marginalization.  It is nevertheless strong evidence against the prior being
the main remaining weak-direction information source during the observed drift.
The prior does contain accumulated LiDAR history, so marginalization may become
the next limit after current-window behavior is corrected, but it is not the
next intervention justified by this replay.

## Decision

Continue with current-window behavior before modifying marginalization.  The
next experiment should isolate why attenuated LiDAR plus prediction recovery and
GNSS scheduler interaction still produces the weak-direction feedback and 222
integrity rollbacks.  Marginal-prior math remains frozen.

`DO_NOT_PROMOTE`

No Z-axis, state-machine, relocalization, marginalization math, or sensor weight
was changed.  No Gazebo run and no push were performed.
