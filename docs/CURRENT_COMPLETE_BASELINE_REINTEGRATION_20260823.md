# Current complete baseline reintegration (2026-08-23)

## Baseline decision

The old group-leader baseline `35e234dd063b16e47c1f995fce9a1758349be581`
and the current complete algorithm line are divergent descendants of
`7441e4f49b19ccc1d1443958a48a0edf76da606a`.  The selected baseline is:

- branch: `feat/core-algorithm-cleanup-20260817`
- commit: `e934132ffdd991b0dd59a752eead93d2e0313b40`
- current PR #14 head: the same commit
- old/new ancestry: divergent, with 3 old-side and 45 complete-line commits

`baseline/stable-microlink-20260822` at
`08c6a7c615a63c8607e5002b2f31a4b77675887c` is newer by commit timestamp but is
not the complete algorithm line.  Its own baseline document says that it is a
stable MicoLink/simulation snapshot based on `5b3e36a` and deliberately omits
the later relocalization, directional-handoff, RGB-D and dynamic-scene work.
It remains a useful runtime-contract reference, not the integration base.

The HTTPS smart transport timed out during the audit.  GitHub's authenticated
API was used to enumerate every branch, tag and pull request and to import the
exact Git trees and commits locally; reconstructed object IDs were verified
against the server SHAs before local remote-tracking refs were updated.  No
remote ref was written.

## Material changes after the old baseline

Relative to `35e234d`, the complete line changes 106 files with 9,606 additions
and 699 deletions.  The important semantic changes are:

- the unified backend implements the paper reliability model, restores atomic
  optimization transactions, expands integrity diagnostics and keeps
  one-observation-one-factor ownership;
- relocalization now uses continuous unified odometry for source-time
  association, correlates epoch commits by transaction, and applies explicit
  safe correction bounds to manual/fault as well as automatic searches;
- FAST-LIO gains body-envelope return rejection and strengthened native-factor
  epoch/QoS contracts;
- optical-flow velocity and lever-arm compensation, MicoLink transport and
  fault-injection/validation tooling are expanded;
- a group-leader historical-voxel map cleanup and an opt-in pre-FAST-LIO
  temporal filter are present; the latter is default-off and its own A/B found
  worse dynamic-city accuracy;
- a directional information handoff and tunnel-validation line are present as
  experiments, not as a promoted Z covariance solution.

The old PR #15 head and this complete line also diverge at `7441e4f`: PR #15
has 9 side commits while the complete line has 45.  Therefore PR #15's old
backend, relocalization and flight metrics cannot be carried forward as proof.

## Integration ownership

Work continues only in the independent worktree and branch
`integration/dynamic-current-complete-v2`.  Six Dynamic commits were replayed
in dependency order rather than merging PR #15 wholesale.  The retained
project-owned components are:

- class-agnostic observer v2 with causal IMU deskew;
- disabled-by-default fail-open Clean Scan Gateway;
- read-only previous FAST-LIO posterior interface;
- long-term `STATIC_CONFIRMED_ONLY` map refinement and admission;
- Raw/Clean localization and hybrid relocalization evaluators;
- epoch reseed guard for true FAST-LIO-local resets.

The only textual conflict was the downstream FAST-LIO patch list.  The
resolution preserves both group-leader body-envelope rejection and the
Dynamic previous-posterior export.  The combined patch series was applied to a
clean pinned FAST-LIO checkout in order and rebuilt successfully.

The group-leader temporal filter is not chained with Clean Gateway: it remains
default-off, while Clean Gateway uses its own topic and is also default-off.
Unknown points remain admitted and every unhealthy scan passes through raw.
This prevents double point deletion and preserves the production
`/livox/lidar` publisher contract.

## Epoch, causality and relocalization contract

The new backend's `FusionEpoch` changes `map_from_lio` while the FAST-LIO local
frame remains continuous.  Dynamic history is therefore retained for a valid
backend epoch.  Only a monotonic change in the previous-posterior
`reset_counter` clears local visibility history; the next six healthy scans are
passed raw while reseeding.  Stale/duplicate backend transactions and stale
FAST-LIO states cannot mutate Dynamic state.

Clean processing uses only a completed previous FAST-LIO posterior plus
current/past IMU.  It does not subscribe to current/future unified pose or use
future IMU.  There is no backend-to-observer-to-FAST-LIO instantaneous cycle.

The hybrid policy remains `STATIC_CONFIRMED_ONLY`/Refined for candidate search
and Clean geometry for final registration.  The complete line's new epoch
correction gate is an additional safety owner after registration; Dynamic does
not bypass or widen it.

## Validation

Validation results are populated from the new worktree only.  Build artifacts
and replay outputs remain ignored under `build/`, `install/`, `log/` and
`logs/tmp/`.

- build: 18 packages passed;
- colcon xUnit: 133 tests, zero errors/failures/skips;
- unified backend runner: 317 tests passed;
- FAST-LIO combined downstream patch build: passed;
- deterministic observer/map/localization/relocalization metrics: see final
  run summary below;
- ROS smoke, syntax and repository hygiene: see final run summary below.

## Z-line audit

The independent Z worktrees are clean and remain separate:

- `feat/z-axis-observability-v1` at `4a42f275...`;
- `feat/z-covariance-calibration-v1` at `ac6ab73...`, `DO_NOT_PROMOTE`;
- `feat/z-covariance-model-v1` at `b033e862...`, `DO_NOT_PROMOTE`.

The latest complete line changes the same backend transaction, reliability,
axis-information and directional-handoff surfaces that the Z candidates were
measured against.  Their numerical coverage/NEES and runtime results are thus
not transferable to this baseline.  The core finding still stands: the naive
fresh marginal is overconfident, while the correlated shadow model became
over-conservative on held-out data and failed the zero-rollback matrix.  No Z
candidate, covariance publication path or Z-axis algorithm is included here.

## Final run summary

All figures below were regenerated in this worktree.  Outputs are kept under
ignored `logs/tmp/current_complete_v2/`; no old PR #15 result is used as the
sole proof for the new baseline.

### Observer and long-term-map matrices

The Release-build 18-scenario, three-seed/two-repeat observer matrix reports:

| Method | Precision | Recall | F1 | Static preservation | Contamination | P95 / P99 latency |
|---|---:|---:|---:|---:|---:|---:|
| TemporalVoxelFilter | 80.18% | 3.28% | 6.29% | 99.925% | 89.07% | 0.467 / 0.599 ms |
| Observer v1 | 100.00% | 95.08% | 97.48% | 100.000% | 3.63% | 0.982 / 1.146 ms |
| Observer v2 | 99.84% | 97.09% | 98.45% | 99.986% | 1.81% | 1.793 / 2.111 ms |

Truth remains evaluator-only.  Static-only scenarios have null dynamic P/R/F1
and are excluded from macro dynamic aggregation.  Production FAST-LIO input
mutations are zero.

The 11-scenario, three-seed long-term matrix reports Raw/Clean/Refined
contamination of 41.62%/8.44%/1.80%.  Refined static completeness and
relocalization overlap are both 96.82%, with 3.18% false removal and 26 mean
ghost voxels removed.  Refinement P50/P95/P99 is
0.888/1.075/1.227 ms and retained state is 7.78 MiB.  Persistent large
occlusion remains the dominant physical-observability boundary; refinement
does not reinterpret missing returns as free space.

### Localization, Native information and relocalization

The deterministic active-motion evaluator retains class-agnostic dynamic
P/R/F1 of 98.25%/76.16%/85.81% and reduces map contamination from 3.82% to
0.94%.  Native translation-information ratios remain close to unity and do
not weaken Z: X/Y/Z are 97.96%/99.19%/98.78% in that matrix.

Across 8 relocalization conditions, 3 seeds and 3 query frames per trial:

- Raw: 72/72 success, zero false, 14.08% candidate-map contamination;
- Clean: 72/72 success, zero false, 5.50% contamination;
- Refined-only registration: 54/72 success, zero false, 0.394% contamination;
- Hybrid Refined search plus Clean final registration: 72/72 success, zero
  false, with 0.394% candidate contamination.

This confirms that the hybrid ownership remains preferable on the current
baseline: Refined geometry is the clean search database, while Clean geometry
retains enough support for final registration.  A full online loop closure is
not claimed by this deterministic evaluator.

### Runtime and contract checks

- observer ROS smoke: passed, with zero queue overflow and no production input
  mutation;
- Clean Gateway smoke: exact-raw fail-open passed for missing previous state,
  IMU timeout, timestamp regression, queue overflow and LIO epoch reseed;
  Livox timestamp/offset/line/tag metadata was preserved;
- backend epochs retain local history, while a true previous-posterior reset
  triggers six raw reseed scans and then resumes clean processing;
- long-term-map smoke: `STATIC_CONFIRMED_ONLY`, map-hold fail-open, no future
  pose and opt-in shadow semantic auxiliary all passed;
- Python compile, YAML/XML parsing, shell syntax and `git diff --check` passed;
- Release build: 18 packages passed; colcon: 133/133 xUnit passed; backend
  runner: 317 tests passed.

The frozen-input, real-time-rate Raw/Clean dual FAST-LIO replay covered 10
primary dynamic scenes plus three occlusion-observability scenes.  During the
dynamic phase, Raw/Clean ATE is 5.645/4.770 mm and translation RPE is
2.466/1.832 mm.  Large-occlusion full-run ATE is 27.002/7.289 mm.  There are
zero lost runs, resets, missing clean scans and queue overflows; maximum odom
gap remains 100.001 ms.  Gateway P50/P95/P99 is
4.407/5.039/5.687 ms, median gateway CPU is 6.67%, and RSS is 53.57 MiB.

During dynamic motion, Clean changes Native translation information by
-0.99%/-2.84%/+2.33% in X/Y/Z, respectively, so Z is not weakened.  Native
effective-factor counts are identical (300) and median residual improves from
22.971 to 20.161 mm.  The Clean branch uses strictly previous posterior state,
the detector has no truth access, and all 13 runs finish without lost/reset.

A diagnostic 5x replay was also retained separately: it induced 74 bounded
queue overflows and one lost Clean run.  It is intentionally excluded from the
promotion result because it violates the frozen online playback contract; the
same data at 1x has zero overflow/loss.  This documents the throughput boundary
rather than hiding it.

The new-baseline Dynamic stack therefore remains **PROMOTE** for continued
integration and production/hardware validation.  Old PR #15 numbers remain
historical only; the comparable current result is the regenerated 1x replay
above.  Z-COV remains explicitly excluded.
