# Dynamic Localization on the latest relocalization line

## Scope and immutable inputs

- Upstream: `origin/exp/passive-relocalization-reinit-five-source` at
  `35e234dd063b16e47c1f995fce9a1758349be581`.
- Frozen Dynamic V1: `29e580448b33cd8bd1a5815808435c2ac4a9342f`,
  tag `dyn-loc-007-dynamic-localization-20260820`.
- Integration branch: `integration/dynamic-latest-relocalization-v1`.
- Common ancestor: PR #14 frozen commit
  `50e96f63d19e8d9292b15a684f0cc8a76f55e5bd`.

The upstream mission state machine, checkpoint timing, relocalization core,
reinitialization transaction, scorer, scenes, ExternalNav, factor ownership,
and Z-axis behavior remain authoritative and unchanged. Dynamic remains
opt-in; production `/livox/lidar` remains raw.

## Coordinate and epoch contract

An accepted upstream relocalization changes `map_from_lio` and creates a new
unified-backend `FusionEpoch`. It does not reset FAST-LIO's `camera_init`
frame. Observer, Clean Gateway, long-term map, `/lio/local_map`, and the
previous FAST-LIO posterior all remain mutually consistent in `camera_init`.
Therefore a backend-only epoch is recorded diagnostically and does not clear
valid Dynamic history.

An increase of `PreviousFastLioState.reset_counter` is different: it denotes
a real LIO-local epoch change. Pending scans then pass through exactly raw,
short-term voxel/free-space evidence is cleared, transient long-term output is
quarantined, and six healthy causal scans rebuild evidence. Filtering resumes
on the following scan. Failed, duplicate, stale, or unapplied backend epochs
and stale LIO reset counters cannot mutate state.

Eight unit tests cover initial state, genuine LIO reset, stale LIO state,
small backend correction, large translation/yaw correction, repeated and
failed reinitialization, reinitialization during dynamic occlusion, and invalid
configuration. The ROS smoke additionally verifies five reseed scans, the
sixth completion scan, and the seventh normal clean scan.

## Observer and active-motion behavior

The standard deterministic matrix contains 18 scenarios, three seeds and two
repeats per seed. Dynamic metrics are micro-aggregated over pooled TP/FP/FN;
static-only scenes report dynamic P/R/F1 as N/A and remain in static-negative
metrics.

| Method | Precision | Recall | F1 | Static preservation | Contamination | P50/P95/P99 latency |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| TemporalVoxelFilter | 80.18% | 3.28% | 6.29% | 99.925% | 89.07% | 0.292/0.443/0.551 ms |
| Observer v1 | 100.00% | 95.08% | 97.48% | 100.000% | 3.63% | 0.705/0.922/1.061 ms |
| Observer v2 | 99.84% | 97.09% | 98.45% | 99.986% | 1.81% | 1.204/1.614/1.796 ms |

The active-motion benchmark covers structural hold/checkpoint 4, fast
figure-eight plus rotate/checkpoint 4, window-opening passive hold, and fast
checkpoint 8 pressure. Aggregate P/R/F1 is 98.25%/76.16%/85.81%, static
preservation is 99.946%, and contamination changes from 3.82% raw to 0.94%
clean. P50/P95/P99 is 4.175/5.027/6.729 ms. Native information ratios are
X/Y/Z = 97.96%/99.19%/98.78%; the weakest result is bounded and no estimator
input is removed because the candidate remains opt-in/fail-open.

This synthetic benchmark uses solid, sufficiently sampled walls and ground.
Sparse mathematical surfaces were rejected as a benchmark artifact because
rays could pass through nonphysical sampling holes; no production threshold
was loosened.

## Raw/Clean current-localization A/B

Fourteen independent frozen replay runs use identical MID360/IMU input and
independent Raw/Clean FAST-LIO state. The current-position aggregate below is
over ten primary dynamic scenarios.

| Phase | Metric | Raw | Clean |
| --- | --- | ---: | ---: |
| DURING | ATE RMSE | 5.645 mm | 4.818 mm |
| DURING | translation RPE | 2.466 mm | 1.892 mm |
| DURING | yaw RPE | 0.01469 deg | 0.01074 deg |
| DURING | pose-jump P95 | 3.978 mm | 2.973 mm |
| DURING | Native residual median | 22.971 mm | 20.290 mm |
| DURING | Z RMSE | 1.426 mm | 1.304 mm |
| AFTER | ATE RMSE | 5.408 mm | 5.306 mm |
| AFTER | translation RPE | 3.990 mm | 3.958 mm |

Clean improves DURING ATE by 14.65%, RPE by 23.30%, pose-jump P95 by
25.27%, and Native residual by 11.67%. Native information changes DURING by
X/Y/Z = -0.99%/-2.84%/+2.33% and AFTER by +0.12%/-1.64%/+2.96%. There are
no lost runs, resets, missing clean scans, or queue overflows.

Large dynamic occlusion is the strongest localization result: DURING ATE is
34.770 mm raw versus 6.065 mm clean, RPE is 13.145 versus 2.305 mm, and
pose-jump P95 is 25.849 versus 2.738 mm. Residual persistent-map contamination
under never-vacated occlusion is therefore a physical map-observability issue,
but the observed dynamic geometry does affect raw current localization and is
substantially isolated by Clean.

## Relocalization map ownership A/B/C/D

The offline GICP comparison covers eight conditions, three seeds, and three
query frames: 72 attempts per map. Truth is evaluator-only. Registration uses
previous causal state and cannot feed a current Raw posterior into Clean.

| Ownership | Success | False | Stable error | Inliers | Dynamic reference | Candidate contamination | Candidate completeness |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Raw candidate + Raw registration | 72/72 | 0 | 1.775 mm | 1283.9 | 1.80% | 14.08% | 100.00% |
| Clean candidate + Clean registration | 72/72 | 0 | 2.224 mm | 1221.3 | 0.62% | 5.50% | 99.99% |
| Refined candidate + Refined registration | 54/72 | 0 | 10.617 mm | 1119.1 | 0.00% | 0.39% | 59.18% |
| Refined candidate + Clean registration | 72/72 | 0 | 2.224 mm | 1221.3 | 0.62% | 0.39% | 59.18% |

Refined-only fails the structural-hold and window-opening passive groups when
single-view confirmed completeness is only about 3.72%. The hybrid strategy
recovers 100% success and Clean-level precision while keeping candidate-map
contamination at 0.39%; it is the recommended data ownership. Registration
P95 is 4.55--4.68 s in this offline, unoptimized GICP benchmark and is not an
online estimator callback timing claim.

No tested map produces a false relocalization. Fast figure-eight raw uses
9.40% dynamic reference geometry and has 29.29% candidate contamination;
hybrid reduces dynamic reference to 0.23% and candidate contamination to zero.
Checkpoint-8 pressure retains 100% hybrid success but is still an upstream
policy negative control, not a recommended trigger.

## Long-term refinement and occlusion boundary

Across 11 long-duration scenarios and three seeds, Raw/Clean/Refined
contamination is 41.62%/8.44%/1.80%. Refined static completeness is 96.82%,
ghost ratio is 1.80%, and mean removed ghost voxels is 26. Update
P50/P95/P99 is 0.864/1.024/1.157 ms; state memory is 7.74 MiB. The benchmark
reports 97.99% CPU while executing updates continuously; at the intended 10 Hz
cadence the measured update wall time corresponds to about 0.86% of one core.

Persistent occlusion without a measured clearing ray remains physically
unobservable: refined contamination 11.26%, completeness 88.31%. Same-view
reobservation reduces contamination to 5.18%; natural multi-view
reobservation reduces it to 4.96%. Temporarily invisible confirmed static
geometry is not deleted merely because it is not returned.

## Fail-open, integrity, and runtime

Clean Gateway ROS smoke confirms exact raw pass-through for missing previous
state, IMU coverage failure, queue overflow, timestamp regression, and real
LIO reseed. Livox timestamp, `offset_time`, line, tag, coordinates, and point
semantics are preserved. Observer and long-term-map smokes confirm no future
pose, no production LiDAR mutation, STATIC_CONFIRMED_ONLY output, map hold on
missing state, and optional semantic shadow evidence.

In the 14-run release replay, gateway P50/P95/P99 is
4.468/4.873/5.879 ms, median CPU is 6.52%, and RSS is 53.54 MiB. Clean FAST-LIO
median CPU is 28.86% and RSS is 176.71 MiB. The 47 fail-open events are bounded
startup previous-state waits across separate processes, not steady-state
drops. Missing scans, overflow, lost/reset, optimization errors, integrity
rejects, and rollbacks are all zero in the evaluated Dynamic path.

The upstream scorer and integrity definitions remain unchanged. Existing
upstream flight evidence remains screening evidence: structural hold/checkpoint
4 and fast figure8/rotate/checkpoint 4 each have two runs; window-opening hold
has one; structural active motion is not stable; checkpoint 8 remains poor.
This integration does not relabel those results as production flight evidence.

## Build and regression result

- Release build: 18 packages passed.
- `colcon test`: all package runners passed; 132 indexed xUnit tests have zero
  errors/failures/skips. The unified backend's own runner reports 286 passed.
- Dynamic package: 52 indexed tests passed, including 23 observer/deskew/admission,
  15 long-term map, 8 epoch guard, and evaluator/replay tests.
- ROS smokes: observer, Clean Gateway, and long-term static map passed.
- YAML, XML, Python bytecode, shell syntax, and `git diff --check`: passed.
- Dynamic ROS processes were stopped after validation.

The external FAST-LIO checkout remains pinned at `a4743b...`. Existing project
patches 0003, 0004, and 0006 were applied additively to the already patched
local dependency so that epoch gating, Native factor ownership, and previous
posterior export are all present. No dependency source is vendored or pushed.

## Decision and remaining production boundary

`PROMOTE_DYNAMIC_ON_LATEST_RELOCALIZATION`

The software integration gate passes: upstream logic is preserved, Dynamic ON
does not reduce relocalization success or increase false matches, current
localization benefits remain, Z information is preserved, real LIO resets are
fail-open and bounded, active motion maintains high static preservation,
integrity counters do not regress, and full build/test/smoke pass.

Remaining production blockers are real MID360/D435i hardware timing and
extrinsic verification, repeated real-flight confidence intervals for the
upstream screening policies, and real-world dynamic/structural sessions for
the hybrid candidate/registration ownership. Dynamic stays default-off and
Raw remains the immediate fallback until those are closed.
