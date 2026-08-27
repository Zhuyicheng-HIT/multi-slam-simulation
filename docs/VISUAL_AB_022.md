# VISUAL-AB-022: Moving Tunnel Visual Factor Comparison

## Scope and frozen input

This experiment compares `paper_reprojection` and `rgbd_direct` without
changing Flow, HXY, GNSS/IMU, Dynamic, or any visual admission threshold.
Truth is consumed only by the offline accuracy recorder.

- Code baseline: `1ff485cc0d50dfc542a82616f3c6920d3cb44959`
- Bag: `/home/ld666/projects/hxy-diag-002/frozen_bag/replay_bag_0.db3`
- Bag SHA256: `180d4f0e8985ae7253dfa8edac242f0dd9f814c2b4d1c7b93c0088351c0375a9`
- Inputs: 678 native LiDAR factors, 375 GNSS fixes, 7513 IMU samples,
  57 feature-track batches, 57 RGB-D geometry batches, and 57 RGB-D direct
  batches
- Replay: 0.4x, one numerical thread, two executor threads, latest-only off,
  30 s wall-time drain, dynamic recorded reliability scores
- HXY remained enabled with weak/exit/scale values `0.15/0.25/0.001`.
- The frozen bag predates FLOW-CONTRACT-021. Its recorded Flow score disables
  all flow solver factors. This condition is identical in every run.

The degradation cases use a replay-only scheduler mask. It copies the recorded
scheduler state and changes only the selected modality's factor admission,
weight, and covariance inflation. It does not change estimator factor math.
LiDAR anchor protection admits 27 startup factors before both LiDAR and IMU
scores are fresh; subsequent constructed LiDAR factors are disabled. GNSS has
no corresponding anchor override.

## Visual admission

Adoption is reported both per attempted association and per received batch.
The latter exposes candidates that cannot be associated after the optimized
trajectory stops advancing.

| Condition | Mode | Received | Attempts | Solver factors | Factors / attempts | Factors / received |
|---|---|---:|---:|---:|---:|---:|
| Normal | paper | 57 | 57 | 2 | 3.51% | 3.51% |
| Normal | direct | 57 | 57 | 11 | 19.30% | 19.30% |
| LiDAR degraded | paper | 57 | 57 | 2 | 3.51% | 3.51% |
| LiDAR degraded | direct | 57 | 57 | 10 | 17.54% | 17.54% |
| GNSS degraded | paper | 57 | 57 | 0 | 0.00% | 0.00% |
| GNSS degraded | direct | 57 | 57 | 24 | 42.11% | 42.11% |
| Dual degraded | paper | 57 | 20 | 2 | 10.00% | 3.51% |
| Dual degraded | direct | 57 | 18 | 11 | 61.11% | 19.30% |

Paper rejection accounting is exact. Normal and LiDAR-degraded runs each have
40 PnP-observability rejects, 8 insufficient-track rejects, 5 state-consistency
rejects, 2 initialization waits, and 2 accepted factors. With GNSS disabled,
the counts are 40, 8, 8, 1, and 0. In dual degradation only 20 associations are
attempted: 9 PnP, 3 track, 4 consistency, 2 initialization, and 2 accepted;
37 batches expire because no compatible state remains.

Direct mode has no track-count or state-consistency rejection on attempted
batches. The prefit rejects are 46/57 (normal), 47/57 (LiDAR degraded), 33/57
(GNSS degraded), and 7/18 (dual). Accepted direct factors retain metric depth,
while their inconsistent photometric rows are downweighted: 11/11, 9/10,
24/24, and 11/11 respectively. In dual degradation 39 received batches cannot
be attempted after the state trajectory stops.

## Accuracy and stability

All values use frozen initial alignment and source-header timestamp association.
`Coverage` is the scored simulation duration, not wall time.

| Condition | Mode | XY RMSE m | XY P95 m | XY max m | 3D RMSE m | Endpoint 3D m | RPE 1 s m | Coverage s | Rollbacks |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Normal | paper | 0.844 | 1.371 | 2.692 | 0.844 | 2.123 | 0.243 | 74.096 | 0 |
| Normal | direct | 0.846 | 1.371 | 2.695 | 0.846 | 2.126 | 0.235 | 74.096 | 0 |
| LiDAR degraded | paper | 0.835 | 1.359 | 2.682 | 0.835 | 2.113 | 0.238 | 74.096 | 0 |
| LiDAR degraded | direct | 0.833 | 1.372 | 2.684 | 0.834 | 2.114 | 0.246 | 74.096 | 0 |
| GNSS degraded | paper | 22.973 | 60.792 | 76.257 | 22.975 | 76.263 | 2.860 | 48.327 | 141 |
| GNSS degraded | direct | 21.199 | 57.119 | 71.603 | 21.200 | 71.606 | 2.677 | 47.747 | 146 |
| Dual degraded | paper | 0.823 | 2.212 | 3.157 | 0.823 | 3.157 | 0.374 | 33.162 | 165 |
| Dual degraded | direct | 0.310 | 0.667 | 1.516 | 0.320 | 1.601 | 0.383 | 32.812 | 159 |

Normal and LiDAR-degraded differences are below 2 mm XY RMSE and are not a
meaningful accuracy separation. With GNSS disabled, direct improves XY RMSE by
7.72%, but both trajectories become unusable and stop accepting transactions.
The low full-run dual RMSE values are truncated-run values and must not be read
as successful completion.

An additional dual-degraded `visual_factor_mode=disabled` causal control runs
the complete 74.096 s but diverges to 53.329 m XY RMSE and 119.867 m maximum
XY error. On the common interval ending at 63.609 s:

| Dual-degraded mode | XY RMSE m | XY P95 m | XY max m | Endpoint XY m |
|---|---:|---:|---:|---:|
| Visual disabled | 0.868 | 2.391 | 4.945 | 4.945 |
| paper | 0.763 | 2.026 | 3.136 | 3.136 |
| direct | 0.310 | 0.667 | 1.516 | 1.516 |

Thus paper reduces common-interval XY RMSE by 12.1% and direct by 64.3%
relative to visual-off. Direct supplies materially stronger XY information.
Neither visual mode is end-to-end stable in dual degradation.

## First failure and prior interaction

- Paper first admits a visual factor at transaction 292, 59.829 s. Its first
  failed transaction is 319, 62.601 s, with a 1.113 m proposed translation
  correction.
- Direct first admits a visual factor at transaction 268, 57.420 s. Its first
  failed transaction is 313, 62.007 s, with a 1.276 m proposed translation
  correction.
- Both fail with `excessive_translation_correction` and then repeatedly roll
  back. At first failure there is no active visual factor in either window.
  Source attribution in the marginal prior contains 2 `visual_reprojection`
  factors for paper and 11 `rgbd_direct` factors for direct.
- The visual-off dual control has zero rollback and continues publishing, but
  its unconstrained horizontal trajectory diverges badly.

The evidence supports a visual-to-marginal-prior/integrity interaction, not a
case for relaxing admission thresholds. The visual factors improve the state
before failure, but their retained information becomes inconsistent with the
later IMU-dominated state strongly enough to trip the transaction integrity
gate.

## Timing

| Condition | Paper solver P50/P95 ms | Direct solver P50/P95 ms | Paper visual construction P50/P95 ms | Direct construction P50/P95 ms |
|---|---:|---:|---:|---:|
| Normal | 5.657 / 35.770 | 5.164 / 28.716 | 1.230 / 5.732 | 3.403 / 11.209 |
| LiDAR degraded | 4.484 / 11.757 | 4.821 / 17.828 | 1.247 / 5.423 | 3.902 / 14.211 |
| GNSS degraded | 12.039 / 35.171 | 12.546 / 38.976 | 1.159 / 3.647 | 4.182 / 11.783 |
| Dual degraded | 3.166 / 13.069 | 2.211 / 13.132 | 1.751 / 5.942 | 3.938 / 11.378 |

Direct construction is approximately 2--3 times more expensive, but its
overall solver P95 remains in the same range as paper in the matched runs.
Solver profile call counts are repeated linearizations, not unique observation
counts; the solver-factor counts in the admission table remain authoritative.

## Recommendation

Use `rgbd_direct` as the mainline candidate for the next visual experiment: it
has much higher admission, preserves normal/LiDAR-degraded accuracy, and gives
the strongest measured XY constraint when absolute sources are absent. Do not
promote it to the production default yet. First isolate the first visual-bearing
marginal prior that causes `excessive_translation_correction`, and require a
full-capture dual-degradation replay with no sustained rollback.

Keep `paper_reprojection` as the fallback/reference. It is cheaper and stable
when GNSS is present, but its 0--3.5% received-batch adoption and weak dual-case
XY contribution make it unsuitable as the primary degeneracy constraint on
this capture.

Raw artifacts are under `logs/visual_ab_022/` and are intentionally ignored by
Git.
