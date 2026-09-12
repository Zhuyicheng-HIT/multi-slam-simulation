# HXY-DIAG-002: LiDAR horizontal-degeneracy diagnostics and replay

Date: 2026-08-25

## Result

The diagnostic trace is sufficient to locate the first causal difference, but
it is not sufficient to claim that PR17 is the desired solution.  On this bag,
PR17's `paper_eq19`/`paper_eq15` path disables every LiDAR factor.  It avoids the
stable branch's catastrophic Y drift by falling back primarily to GNSS and IMU;
it does not preserve strong LiDAR subspaces.  This is evidence for implementing
and testing subspace C, not evidence that C is already validated.

No estimator decision, state machine, Z-axis policy, relocalization behavior, or
factor math was changed.  The patch only extends the existing performance trace
after each transaction and allows the existing replay wrapper to regenerate the
LiDAR score/scheduler path for A/B.

## Frozen input

Frozen copy:
`/home/ld666/projects/hxy-diag-002/frozen_bag`

Source capture:

- Source directory: `/home/ld666/multi-slam-simulation/logs/large_scene_tunnel_static_paper_backend_truth_fixed_20260821/replay_bag`
- Capture commit: `a37539aa34222578f3d1a186f9e616c3da7b7cb0`
- Profile: `tunnel_static`
- World profile: `large_indoor_tunnel_apm_rgbd_mid360`
- Gazebo world: `large_indoor_tunnel`
- Route: span 70.0 m, lateral amplitude 1.0 m, vertical amplitude 0.35 m, speed 0.80 m/s
- Dynamic agents: disabled
- Duration: 75.159 s; messages: 101954

Checksums:

| File | Bytes | SHA256 |
|---|---:|---|
| `metadata.yaml` | 18335 | `d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b` |
| `replay_bag_0.db3.zstd` | 126180529 | `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56` |

The verified input contract is in
`/home/ld666/projects/hxy-diag-002/bag_contract.json`.  Important topic counts
are native LiDAR 678, IMU 7513, GNSS fix 375, raw GNSS 325, optical flow 951,
RGB-D direct/geometry 57 each, visual tracks 57, and truth 752.  Truth was
subscribed only by `external_nav_accuracy` and was not an estimator input.

## Replay contract

Both final runs used the bag from offset zero, rate 0.5, CycloneDDS, one numeric
thread, two executor threads, axis handoff off, Z reanchor off, barometer
fallback off, range facet off, and identical QoS/worker depths of 1024.  Recorded
LiDAR score and scheduler messages were excluded and regenerated from the same
recorded `/lio/diagnostics` and `/lio/odom`; all non-LiDAR score inputs were the
same recorded messages.

| Run | Algorithm reference | Diagnostic commit | LiDAR score/admission | Output directory |
|---|---|---|---|---|
| A | `c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981` | `cb05eb0` | `hybrid` / `adaptive` | `/home/ld666/projects/hxy-diag-002/replay_A_complete_c7c1adc` |
| B | `4587e479d5f02dbaaaff048c266fd873d124d109` | cherry-pick `8d6afa0` | `paper_eq19` / `paper_eq15` | `/home/ld666/projects/hxy-diag-002/replay_B_complete_pr17` |

Both received all 678 native factors with zero DDS/worker queue supersession.
A nevertheless invoked its estimator-internal latest-only check and skipped 171
already queued old frames after long callbacks; B skipped zero.  This is a real
closed-loop timing consequence of the current implementation, not an input-bag
difference.  Consequently final full-run metrics have different output horizons;
the common-horizon comparison below is the defensible A/B score.

## Added trace evidence

Each existing `backend_cycle_trace.jsonl` record now includes transaction and
native sequence IDs, Schur translation information, normalized eigenvalues,
all eigenvectors, canonical weak direction, actual effective translation
information/eigenpairs, prediction innovation/gate/recovery, effective factor
weight, explicit solver admission, active LiDAR factor indices/count/ages,
marginalization, optimized state, and the already-existing GNSS, visual, flow,
factor-count, and solver profile diagnostics.

The effective matrix is diagnostic-only:
`H_eff = w_eff * sqrt(D_axis) * H_schur * sqrt(D_axis)`.  Axis handoff was off in
these runs, so `D_axis=I`.  This is computed after the solve from the actual
active factor record and cannot affect the transaction.

## Core metrics

Offline causal ATE, common source-stamp interval `30.723 <= t <= 78.804 s`
(the last samples inside that interval are A `78.693 s`, B `78.804 s`):

| Metric | A stable | B PR17 |
|---|---:|---:|
| Matched samples | 471 | 478 |
| 3D RMSE (m) | 19.204 | 0.487 |
| XY RMSE (m) | 19.204 | 0.485 |
| Z RMSE (m) | 0.135 | 0.042 |
| 3D P95 (m) | 51.760 | 1.115 |
| 3D max (m) | 66.671 | 1.239 |
| Last-sample 3D error (m) | 66.671 | 1.144 |

For completeness, B's full 74.085 s output has 3D/XY/Z RMSE
`0.788/0.787/0.040 m`, 3D P95/max `1.361/2.440 m`, and endpoint `2.065 m`.
A stops producing scoreable output after 48.171 s, so comparing that A value to
B's full horizon would not be fair.

Factor and admission accounting:

| Quantity | A | B |
|---|---:|---:|
| Native received / traced | 678 / 498 | 678 / 672 |
| Internal latest-only skipped | 171 | 0 |
| LiDAR solver admitted / trace rejected | 452 / 46 | 0 / 672 |
| Prediction hard rejects / recoveries | 12 / 0 | 165 / 0 |
| Optimized states / rollbacks | 466 / 32 | 672 / 0 |
| IMU received / factors | 7513 / 497 | 7513 / 671 |
| GNSS received / consumed / factors | 375 / 260 / 258 | 375 / 364 / 364 |
| Flow received / attempts / factors | 951 / 497 / 0 | 951 / 671 / 0 |
| RGB-D direct received / direct factors | 57 / 0 | 57 / 0 |
| Visual batches attempted / solver factors | 57 / 0 | 57 / 0 |

Flow was scheduler-disabled on the attempted transactions.  RGB-D/visual data
was received and scored, but `vision_frs_gate_disabled`/quality gating prevented
factor formation.  Thus neither flow nor RGB-D supplies horizontal correction in
this replay.  GNSS is the only active external horizontal absolute constraint;
A later downweights/rejects 95 XY GNSS prefits as its estimate departs, while B
keeps all 364 GNSS factors mutually consistent.

## First divergence and timing

There are two useful definitions:

1. First decision divergence: transaction 1, scan sequence 131, `t=30.723 s`.
   Both runs have exactly the same Schur spectrum
   `[0.09029, 0.60937, 1.0]` and weak direction
   `[-0.01638, 0.99986, -0.00182]`.  A admits the LiDAR factor at effective
   weight 1.0; B sets weight 0/inflation 20 and does not admit it.
2. First clear state divergence: transaction 83, scan 213, `t=38.907 s`, where
   A/B optimized XY positions differ by 0.084 m (first crossing of 0.05 m).
   The same timestamp is also A's first offline horizontal error over 0.05 m.

A's offline horizontal error first exceeds 0.1 m at `43.725 s`, 0.2 m at
`49.731 s`, and 1.0 m at `60.225 s`; the error is predominantly Y.  The first A
prediction hard reject is much later, transaction 460 / scan 591 /
`t=76.824 s`.  A's first factor-disabled transaction is transaction 444 at
`75.009 s`.  Prediction gating therefore reacts after the horizontal trajectory
has already escaped; recovery never activates.

## Weak direction over time

The weak eigenvector is not fixed in the world frame:

| Time (s) | Scan | Weak direction | Normalized eigenvalues | A admitted |
|---:|---:|---|---|---|
| 30.723 | 131 | `[-0.016, 1.000, -0.002]` | `[0.090, 0.609, 1]` | yes |
| 43.131 | 255 | `[0.012, 1.000, -0.009]` | `[0.047, 0.819, 1]` | yes |
| 55.506 | 379 | `[0.007, 1.000, 0.010]` | `[0.041, 0.696, 1]` | yes |
| 68.013 | 504 | `[-0.127, 0.992, 0.007]` | `[0.036, 0.306, 1]` | yes |
| 75.934 | 583 | `[0.994, -0.109, -0.004]` | `[0.031, 0.716, 1]` | no |

The vector close to the baseline observation `[0.992,-0.107,-0.072]` occurs at
`75.934 s`, after A has already crossed 1 m horizontal error.  Earlier, while Y
drift is accumulating, the weak vector is almost world Y.  A final Y error and a
later X-like instantaneous weak direction are therefore not contradictory.
Eigenvector sign is arbitrary, and the weak subspace rotates with pose and scene
geometry; causal analysis must use the time series, not one terminal vector.

## Marginal prior

Both runs first fill the 8-state window and marginalize at transaction 9,
scan 139, `t=31.515 s`.  The generated `marginal_prior` first participates in the
next optimizer call at transaction 10, scan 140, `t=31.614 s`.  This precedes the
first 5 cm trajectory divergence by 7.293 s and A's 0.2 m truth-error crossing by
18.117 s.

In A, admitted full-rank LiDAR factors enter the prior continuously: the window
holds up to 8 active LiDAR factors, with median oldest age 0.693 s and maximum
0.890 s.  Marginalization therefore preserves past LiDAR information after the
corresponding raw factors leave the window.  In B there are no active LiDAR
factors to preserve.  The trace establishes temporal precedence and the
information path; it does not by itself decompose the marginal prior back into
per-source Schur blocks, which remains a useful C-era diagnostic.

## Readiness for C

There is sufficient evidence to implement a narrowly scoped C prototype:

- the per-scan Schur matrix/eigenbasis is available before admission;
- the historical LiDAR factor records retain their correspondence Jacobians and
  effective weights, so arbitrary subspace reweighting can be applied when the
  window is relinearized;
- the replay shows the unstable behavior while the weak direction is consistently
  horizontal Y, before prediction gate/rejection intervenes;
- disabling the entire factor removes the catastrophic drift but also discards
  all strong-direction LiDAR information.

There is not yet sufficient evidence to declare C correct.  C must preserve the
same full-input replay contract, log per-eigenmode applied scales including what
enters marginalization, and be compared against A and B on the common horizon.
It should also add at least one bag where RGB-D or flow actually forms solver
factors, because this bag cannot test cross-modal compensation beyond GNSS.

## Verification

- 177 focused Python tests passed (`native_lidar`, `online_backend`, and
  `manifold_window`).
- `uf_backend_fusion` rebuilt successfully on A; PR17 and dependencies rebuilt
  successfully in its isolated worktree.
- Both final bag plays exited zero and passed the input-contract check.
- No Gazebo run and no push were performed.
