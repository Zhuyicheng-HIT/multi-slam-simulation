# HXY-BASELINE-001: LiDAR Horizontal Degeneracy Audit

## Scope and evidence

This audit compares the stable baseline `c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981`
with PR17 reference `4587e479d5f02dbaaaff048c266fd873d124d109`.
It is a read-only algorithm audit: no fusion behavior, Z handling, state machine,
or relocalization logic was changed.

Primary evidence:

- stable and PR17 source at the two exact commits;
- existing PR17 report `docs/experiments/lidar_degeneracy_20260825.md`;
- `logs/lidar_deg_visual_anchor_direct_straight_1ms_final_20260825`;
- `logs/paper_eq19_long_tunnel_8m_20260825_rerun`;
- existing replay and runtime instrumentation. No new simulation was run.

The logs live in the original repository worktree and are not committed data.
Consequently, conclusions based on them should be reproduced from a frozen bag
before becoming a new baseline claim.

## Executive conclusion

The stable backend detects directional weakness but does not generally apply the
detected arbitrary weak eigenspace to the LiDAR information delivered to the
optimizer. The normal path relinearizes raw point-to-plane correspondences and
multiplies the complete LiDAR factor by one scheduler scalar. The existing axis
handoff can scale map X/Y/Z translation columns, but only when an alternative
sensor supplies enough same-axis information. In the PR17 tunnel run it did not:
the reported scale stayed `[1,1,1]` and the handoff count stayed zero despite a
clearly weak Y axis.

The large failure is therefore not a failure to compute an eigenvector. It is a
control-path gap between directional evidence and the factor graph, compounded
by prediction-gate configuration and fixed-lag history. Once a bad LiDAR update
is admitted, its information can be absorbed into a marginal prior together
with IMU and aiding factors. That prior no longer preserves source ownership, so
old LiDAR information cannot later be reweighted exactly by modality or arbitrary
subspace.

PR17 adds a paper Eq. 19 score, binary Eq. 15 admission, reproducible tunnel
geometry/mission controls, and better experiment wiring. It does not implement
arbitrary-subspace information scaling, historical LiDAR reweighting, or a fair
one-variable replay comparison. Its strict binary path can also remove the whole
LiDAR factor rather than retain strong directions.

## 1. Hessian-to-optimizer data flow

The complete stable data path is:

1. Patched FAST-LIO publishes `NativeLidarFactor` on
   `/fast_lio/native_lidar_factor`. The packet contains the full normal equation,
   linearization pose, residual energy, variance, correspondence count, raw
   matched points/planes, extrinsics, scan bounds, sequence, and reset epoch.
2. `native_factor_from_message()` validates shape, finiteness, frames, state
   order, correspondence payload, and positive-semidefiniteness. It extracts the
   6-DoF pose normal and converts FAST-LIO right-SO(3) rotation coordinates into
   backend pose coordinates. Both the transformed normal and the native-right
   normal are retained.
3. For each accepted scan, `lidar_pose_observability()` range-normalizes the 6x6
   Hessian and reports rank, condition and weakest 6-DoF eigenvector.
   `lidar_vertical_observability()` forms the translation Schur complement
   `S_t = H_tt - H_tr pinv(H_rr) H_rt`, then reports its three eigenvalues,
   weakest 3-D translation eigenvector, and axis projections. Despite its legacy
   name, this function analyzes all XYZ translation directions.
4. `lidar_reliability_layers()` derives health, prediction consistency and XYZ
   support diagnostics. `axis_observability_latch()` adds axis hysteresis.
5. The external reliability monitor independently produces one scalar LiDAR
   degradation score. The scheduler turns it into `factor_enabled`, one
   `reliability_weight`, and one `covariance_inflation`. Stable anchor protection
   may retain LiDAR when score evidence is stale or weak.
6. The backend first inserts same-time GNSS, then evaluates optional axis handoff.
   Handoff only produces three map-axis scales and only if a configured alternative
   information source can take an axis. It is not an eigenspace projector.
7. On the normal manifold path, raw point-plane correspondences are relinearized
   at every nonlinear iteration. The decision scalar weights the entire factor.
   If raw correspondences are unavailable, the 6x6 condensed normal is inserted;
   it is still weighted by one scalar. Linear mode uses the coordinate-converted
   condensed normal in the same way.
8. The solver combines LiDAR, IMU, GNSS, optical-flow, RGB-D/visual and priors in
   the active fixed-lag window. When an old state leaves the window, all factors
   touching it are Schur-complemented into a source-agnostic `marginal_prior`.

Thus the system has the necessary current-frame geometry, but the default
optimizer contract remains whole-factor scalar weighting plus optional XYZ-axis
column scaling.

## 2. Why detection does not become directional downweighting

There are four separate reasons.

First, the rank detector uses a relative `1e-3` cutoff. A direction can be much
weaker than its peers while the 6-DoF and translation ranks still remain full.
The PR17 direct straight run ended with normalized translation eigenvalues
`[0.0972,0.8465,1]`: weak in relative terms, but not rank-deficient by that
threshold. Its `native_lidar_directionally_degenerate` counter therefore stayed
zero even though the axis diagnostic marked Y weak.

Second, scheduler output is scalar. A high degradation either inflates/disables
the entire factor or, under stable anchor protection, may retain it. Neither
operation applies `V diag(s_i) V^T` in the measured weak basis.

Third, the only stable directional actuator is axis handoff. It is restricted to
map axes and depends on alternative information. In the direct straight log:

- weakest direction: `[0.0076,0.9997,-0.0229]`;
- axis relative support: `[0.915,0.102,1.000]`;
- axis information scale: `[1,1,1]`;
- handoff frames: `0`;
- alternative information at the final diagnostic: `[0,0,0]`.

The weak direction was measured, but no optimizer information was removed.

Fourth, the condensed and marginal forms lose the raw residual-row identity
needed for later selective reweighting. The condensed current factor retains a
6x6 normal and can be spectrally modified before insertion, but an already formed
marginal prior mixes modalities and cannot be exactly decomposed afterward.

## 3. Prediction rejection/recovery and aiding interaction

### Prediction gate

The prediction innovation compares the LiDAR pose with the IMU/window motion
reference. If enabled and position or yaw exceeds its gate, the current LiDAR
factor is rejected. After the configured number of consecutive rejects, usable
full-rank raw geometry may re-enter at a recovery floor (default weight `0.2`,
inflation `5`) rather than at full strength. Recovery is additionally refused if
the scheduler has disabled LiDAR.

This mechanism can stop a bad current factor, but it has two feedback hazards:

- the prediction comes from a state already influenced by historical LiDAR;
- repeated rejection leaves propagation/aiding to carry the estimate, while a
  weak or absent aiding set can increase prediction error and prevent recovery.

The critical PR17 logs did not exercise this protection. Both the successful 2 m
direct run and the divergent 8 m rerun report the prediction gate disabled, zero
gate rejects, zero recoveries and zero recovery-floor factors. The 8 m rerun
ended with position innovation about `8246 m`, yet continued to process LiDAR
until transaction integrity rejected/rolled back 180 updates. The smaller direct
run also ended with `3.42 m` innovation while its configured 1 m gate was disabled.

### GNSS

GNSS is the strongest absolute horizontal drift suppressor in these runs. It is
inserted before LiDAR, uses predicted-state covariance in NIS, and is split into
horizontal and vertical admission. Valid continuous fixes retain a nonzero robust
floor even after large innovation. In the 2 m direct run, 154/157 attempts formed
GNSS factors with near-unit effective information and very small XY NIS; this is
consistent with the small XY RMSE (`0.0402 m`) despite weak LiDAR Y geometry.

GNSS can nevertheless fail to arrest drift if timestamp selection, scheduler
validity, robust scaling, or the solver transaction rejects its correction, or if
the absolute factor is overwhelmed by accumulated/marginalized relative
information. Counts alone are insufficient; projected information and accepted
state correction are required.

### RGB-D / visual direct factors

RGB-D direct factors can constrain pose in visually textured regions and are
associated with adjacent LiDAR-keyed window states. PR17 deliberately adds
camera-only texture without changing collision/LiDAR geometry, which is a useful
separation of modalities. The 2 m run formed and solver-accepted all 52 attempted
direct factors and stayed accurate. This is evidence that vision plus GNSS can
suppress the weak LiDAR direction, not proof that LiDAR degeneracy was corrected.
The 8 m rerun accepted 142 visual factors but still diverged, showing that factor
presence alone does not establish sufficient information in the drifting
subspace or successful correction after graph conditioning deteriorates.

### Optical flow

Flow contributes horizontal displacement/velocity, not an absolute global
position anchor. It can slow short-term drift when quality, rotation, speed,
range and scheduler gates admit it, but its own integration accumulates error.
The 2 m direct run formed 177 of 316 attempts; the divergent 8 m rerun formed
zero. This difference is a major confounder and must be controlled in replay.

### Historical window

Within the active window, individual LiDAR factors still retain correspondence
or normal-equation identity and can in principle be rebuilt before solving.
After marginalization, their contribution is inseparably mixed with IMU, GNSS,
flow, vision and old priors. A later detector cannot retroactively downweight
only historical LiDAR without retaining source-separated marginal components or
rebuilding from a longer factor history.

## 4. Weak direction versus final Y drift

The vector `[0.992,-0.107,-0.072]` and a final Y drift are not inherently
contradictory, but that vector alone does not prove the cause of Y drift.

If the vector and final error are expressed in the same fixed map frame at the
same time, it is mostly X-directed; a pure Y error projects onto it with magnitude
only about `0.107 |e_y|`. In that narrow interpretation, calling the final Y
error a direct consequence of this one snapshot would be unsupported.

The broader trajectory can still produce Y drift because:

- eigenvectors are instantaneous and can rotate/sign-flip with scan geometry;
- map, body, route and truth-aligned frames are different, and yaw error rotates
  horizontal error between X and Y;
- drift is the time integral of biased increments, not the final Hessian alone;
- old weak directions have already entered the marginal prior;
- GNSS/vision/flow information and admission change over time.

The existing PR17 direct-run final snapshot actually reports a nearly pure Y
weak direction `[0.0076,0.9997,-0.0229]`, while the divergent 8 m rerun ends at
`[-0.0020,0.99998,-0.0063]`. The cited X-dominant vector is therefore likely a
different scan, phase, run or frame. A time series with explicit frame labels is
required before reconciling it with final Y error.

## 5. What PR17 solved and what remains

PR17 solved or materially improved:

- an opt-in Eq. 19-only LiDAR score;
- opt-in binary Eq. 15 admission with stable anchor/stale-score overrides removed;
- explicit experimental configuration plumbing and tests;
- a camera-textured but LiDAR-geometry-preserving tunnel;
- a straight mission and landing-aware observers;
- useful successful 2 m and ordinary-indoor feasibility evidence;
- clearer separation of received packets from solver admission in its report.

PR17 did not solve:

- arbitrary weak-eigenspace scaling of LiDAR information;
- temporal smoothing/tracking of the weak basis;
- replay-time reweighting of historical LiDAR already in marginal priors;
- source-projected information accounting for GNSS, RGB-D and flow;
- enabled prediction gating in the reported critical runs;
- a one-variable A/B comparison on identical sensor messages and timestamps;
- robust long-distance behavior: the 8 m rerun diverged catastrophically, and
  the selected 2 m result still failed its sustained-error-duration gate;
- proof that visual/flow factors constrain the same weak subspace rather than
  merely being present in the graph.

Binary Eq. 15 admission is useful as a diagnostic extreme, but it is not the
desired subspace solution: it discards strong LiDAR directions together with the
weak one and can reduce graph observability.

## 6. Is the current history sufficient for arbitrary-subspace reweighting?

The answer is conditional:

- **Current active raw-correspondence factors: yes.** They retain points, plane
  normals/points, extrinsics and scan linearization, so residual Jacobians can be
  relinearized and transformed by a 3-D projector.
- **Current active condensed factors: partly.** The 6x6 Hessian and gradient are
  enough for a mathematically consistent local spectral/Schur modification, but
  not for changing individual robust residual weights or correspondences.
- **Factors absorbed into `marginal_prior`: no.** Source and subspace ownership
  are lost. Exact post-hoc LiDAR-only reweighting is impossible.
- **Existing saved runtime JSON: no.** It records mostly latest values and
  summaries, not every scan's full Hessian/gradient, basis, factor decision and
  marginalization lineage.
- **A frozen rosbag containing complete `NativeLidarFactor`: potentially yes for
  replay from the beginning.** Rebuild every window under A/B/C; do not attempt
  to mutate a graph midway and call it fair.

## 7. Fair A/B/C replay contract

Use one immutable bag, identical start state, deterministic ordering, solver
settings, window size, iteration budget and all non-LiDAR parameters. Disable
truth feedback into the estimator. Change exactly one LiDAR policy:

- **A:** stable `hybrid/adaptive`, current scalar/axis-handoff behavior;
- **B:** PR17 `paper_eq19/paper_eq15`, binary whole-factor control;
- **C:** proposed arbitrary-subspace scaling, preserving strong eigen-directions.

For every LiDAR scan record:

- scan/epoch/sequence and begin/end/factor timestamps;
- received, parsed, selected, correspondence-valid, attempted, graph-added,
  enabled, solver-accepted, rejected and rollback outcome, with reason;
- matched/candidate points, variance and residual energy;
- full 6x6 Hessian and gradient in a named coordinate convention;
- 3x3 translation Schur information, ordered eigenvalues/eigenvectors, rank,
  condition, basis frame, sign convention and inter-frame basis angle;
- scalar scheduler score, weight, inflation, anchor override, prediction-gate
  decision, consecutive rejects, recovery-floor decision and effective weight;
- actual subspace projector/scales applied to H and g, plus information trace
  before/after;
- LiDAR prediction position vector (not only norm), yaw innovation and thresholds;
- per-factor residual/NIS before solve, after solve and robust loss scale.

For each solver transaction record:

- state before prediction, predicted state, initial graph state, optimized state,
  and correction vectors in map and weak-basis coordinates;
- factor counts and effective information matrices from LiDAR, GNSS, flow,
  RGB-D/visual and IMU, projected onto the same LiDAR eigenbasis;
- GNSS XYZ residual, XY/Z NIS, robust scales, admission and factor formation;
- flow displacement/covariance, quality/range/rotation gates and formation reason;
- RGB-D/visual track count, rank/condition, residual, information projection,
  association stamps and solver acceptance;
- total information rank/condition, cost before/after, LM rejects, integrity
  result, rollback, marginal covariance and output covariance;
- factors entering marginalization and source-separated information contributed
  to the new prior (diagnostic accounting even if the production prior remains
  combined).

For each run report:

- 3D, XY and Z RMSE; P95; maximum and endpoint error; per-axis RMSE and endpoint;
- error projected onto instantaneous and route-aligned weak/strong directions;
- received/selected/attempted/formed/enabled/solver-accepted/rejected counts for
  LiDAR, GNSS, flow, RGB-D/visual and IMU;
- prediction reject/recovery streak distributions and time spent per LiDAR mode;
- first divergence time, weak-direction history, aiding availability at that
  time, solver condition, rollback intervals and marginalization lag;
- timing/RTF only as a secondary check that A/B/C consumed the same event stream.

Do not compare the existing successful 2 m direct run with the divergent 8 m run
as A/B: route length, flow formation, sensor timing and graph history differ.

## Recommended next step

Freeze one long-tunnel bag that contains complete Native LiDAR factors and all
four aiding streams, then first add diagnostic-only per-scan JSONL capture of the
quantities above. Replay A and B unchanged to prove deterministic equivalence and
locate the first divergence transaction. Only after that evidence is complete,
implement C as a current-window LiDAR subspace transform from the full 3x3 Schur
eigenbasis. Restart each replay from the beginning so marginal priors are formed
under the selected policy; do not change Z, state-machine, or relocalization
behavior in this experiment.
