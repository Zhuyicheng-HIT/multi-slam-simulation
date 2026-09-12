# HXY-SUBSPACE-003: arbitrary weak-subspace LiDAR prototype

## Implementation

C is enabled with `lidar_subspace_enabled=true` and uses the current native
translation Schur eigendecomposition.  A mode is weak when its normalized
eigenvalue is below `0.15`; an active episode uses the hysteresis exit threshold
`0.25`.  Weak modes receive information scale `0.001`; all strong modes remain
at `1.0`.  The projector is

`P = U diag(sqrt(s_i)) U^T`,

where `U` is the current 3-D translation eigenbasis.  For each raw point-plane
factor, the translation-conditioned Schur block is reweighted as
`S' = P S P`; its conditional translation gradient is scaled by `P^2`.  The
rotation block and translation-rotation coupling are retained.  This supports
arbitrary rotated weak directions, not only XYZ axes.

Every active raw `lidar_point_plane` factor in the current window receives the
same episode projector, including historical factors that have not yet been
marginalized.  `marginal_prior` is never modified.  Factors remain one
observation per factor.  No GNSS, RGB-D, optical-flow, Z-axis, state-machine, or
relocalization weight was changed.

Because the existing C++ kernel only supports diagonal XYZ scaling, a nontrivial
rotated projector uses the existing Python per-factor normal path.  This keeps
the prototype mathematically explicit but is not real-time efficient; a C++
rotated-normal kernel is a follow-up optimization, not part of this experiment.

## Replay and references

The frozen input and SHA256 are unchanged from HXY-DIAG-002.  A and B are the
formal complete-QoS replays in that report.  C used the same bag, QoS depth 1024,
worker queue 1024, regenerated LiDAR scheduler, rate 0.5, and:

- algorithm base: stable `c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981`
- score/admission: `hybrid` / `adaptive`
- subspace: threshold `0.15`, exit `0.25`, weak scale `0.001`
- output: `/home/ld666/projects/hxy-diag-002/replay_C_subspace_final`

## A/B/C accuracy

The common source-stamp interval is `30.723 <= t <= 78.804 s`; C produced only
453 scoreable samples because its Python path increased callback latency.

| Metric | A stable | B PR17 | C subspace |
|---|---:|---:|---:|
| 3D RMSE (m) | 19.204 | 0.487 | 6.457 |
| XY RMSE (m) | 19.204 | 0.485 | 6.457 |
| Z RMSE (m) | 0.135 | 0.042 | 0.024 |
| 3D P95 (m) | 51.760 | 1.115 | 14.079 |
| 3D max (m) | 66.671 | 1.239 | 25.635 |
| Endpoint/last scored error (m) | 66.671 | 1.144 | 25.635 |

Full C output ended at 46.071 s with 453 matched samples.  C is materially
better than A but does not approach B's accuracy.

Offline error projections onto the instantaneous LiDAR basis over the same
interval:

| Projection RMSE | A | B | C |
|---|---:|---:|---:|
| Weakest translation direction (m) | 12.655 | 0.372 | 5.069 |
| Strongest translation eigenvector (m) | 10.431 | 0.231 | 1.765 |

C preserves strong-direction information substantially better than A, but the
weak-direction residual remains dominant.  B is better because it disables all
LiDAR factors and avoids the unstable feedback loop; that is not evidence that
B preserves strong LiDAR information.

## Admission and timing

| Quantity | A | B | C |
|---|---:|---:|---:|
| Trace transactions | 498 | 672 | 484 |
| LiDAR solver admitted | 452 | 0 | 436 |
| Native received | 678 | 678 | 678 |
| Internal latest-only skipped | 171 | 0 | 185 |
| Prediction hard rejects | 12 | 165 | 128 |
| Recovery factors | 0 | 0 | 107 |
| Optimized states | 466 | 672 | 447 |
| Rollbacks | 32 | 0 | 37 |

C's first hard reject and first non-admission are transaction 331, scan 461,
`t=63.723 s`.  Its first marginalization remains transaction 9 at `31.515 s`,
with `marginal_prior` first used at transaction 10 / `31.614 s`.  Thus the prior
precedes the eventual C divergence by about 32 s, but C does not alter the prior;
the current evidence cannot prove that the prior is the dominant remaining error
source.  The much larger C callback time also changes latest-only admission and
must be removed before a definitive causal claim.

## Decision

1. **C vs A:** clearly better on this bag (`6.46 m` vs `19.20 m` 3D RMSE), with
   much lower strong-direction error and a preserved LiDAR admission stream.
2. **C vs B:** not close to B (`6.46 m` vs `0.49 m`); C retains LiDAR strong
   information, while B retains none.  C is not a replacement for B yet.
3. **Marginal prior:** it is temporally upstream of the residual error, but this
   run cannot isolate it from the Python-path timing/latest-only effect.  Do not
   modify marginalization yet; first add prior source/subspace accounting and a
   rotated C++ normal kernel, then replay with matched transaction horizons.

**DO_NOT_PROMOTE**

C is a valid diagnostic/projection prototype and should not be promoted to the
next algorithm baseline until timing is made fair, marginal-prior attribution is
logged, and C is compared on a replay where RGB-D or optical flow actually forms
solver factors.

