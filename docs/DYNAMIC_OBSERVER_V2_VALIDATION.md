# DYN-EVAL-003 observer v2 validation

Date: 2026-08-19

## Scope and isolation

This report covers the 18-scenario deterministic A/B/C matrix. Every method saw
the same evaluator-generated MID360-like returns for seeds 101, 202, and 303,
with two repeats per seed and 40 frames per run. Ground truth was used only by
the evaluator. The detector received only geometry and timestamps.

The observer remains disabled by default and side-channel only. It did not
publish or remap `/livox/lidar`, odometry, unified state, ExternalNav, or TF.
ATE, RPE, and NativeLidarFactor residual deltas are zero by wiring, not claims
of improved localization. The frozen five-source baseline and one-observation-
one-factor contract were not changed.

The earlier ten-scenario v1 frozen result (100.00% precision, 74.13% recall,
85.14% F1, 100.00% static preservation) remains in
`DYNAMIC_OBSERVER_VALIDATION.md`. It is not numerically interchangeable with
this expanded matrix because the latter adds eight cases, physical occlusion,
non-repetitive coverage holes, three seeds, and a low-altitude sensor origin.

## Aggregate A/B/C

Classification counts were identical across three independent complete matrix
runs. Timing ranges below contain all three runs.

| Metric | TemporalVoxelFilter | Observer v1 | Observer v2 |
|---|---:|---:|---:|
| Dynamic precision | 80.1815% | 100.0000% | 99.8439% |
| Dynamic recall | 3.2760% | 95.0779% | 97.0854% |
| Dynamic F1 | 6.2948% | 97.4769% | 98.4454% |
| Static preservation | 99.9249% | 100.0000% | 99.9859% |
| False dynamic ratio | 0.0751% | 0.0000% | 0.0141% |
| Static-map contamination | 89.0657% | 3.6305% | 1.8083% |
| Map completeness | 94.7928% | 93.6594% | 93.6475% |
| Unknown ratio | 5.3465% | 5.9123% | 5.8945% |
| P50 latency | 0.298-0.318 ms | 0.712-0.757 ms | 1.142-1.219 ms |
| P95 latency | 0.444-0.479 ms | 0.909-0.991 ms | 1.544-1.669 ms |
| P99 latency | 0.597-0.647 ms | 1.032-1.097 ms | 1.736-1.873 ms |
| P50 thread CPU | 0.293-0.298 ms | 0.699-0.712 ms | 1.122-1.142 ms |
| P95 thread CPU | 0.436-0.446 ms | 0.892-0.921 ms | 1.515-1.550 ms |
| Peak estimated filter state | 0.108 MiB | 0.314 MiB | 0.600 MiB |

The benchmark process peak RSS was 7.25-7.34 MiB. Relative to v1, v2 gains
2.0075 percentage points of recall and 0.9685 points of F1 while halving static-
map contamination. The 0.0141-point static-preservation cost is localized to
moving-surface/occlusion boundaries and remains below the 0.5% scenario failure
threshold. No static fast-turn or new-FoV point was marked dynamic.

## Per-scenario matrix

P/R/F1 are dynamic precision/recall/F1. SPR/FDR are static preservation and
false-dynamic ratio. C/U are static-map contamination and unknown ratio.

| Scenario | Temporal P/R/F1 | v1 P/R/F1 | v2 P/R/F1 | v2 SPR/FDR | v2 C/U | v2 P95/P99 ms |
|---|---:|---:|---:|---:|---:|---:|
| static fast turn | N/A | N/A | N/A | 100.00/0.00% | 0.00/6.87% | 1.620/1.785 |
| new area in FoV | N/A | N/A | N/A | 100.00/0.00% | 0.00/7.23% | 1.433/1.512 |
| person crossing | 94.64/3.48/6.71% | 100.00/100.00/100.00% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/5.93% | 1.312/1.605 |
| stationary then moving | 0.00/0.00/100.00% | 100.00/88.64/93.98% | 100.00/88.64/93.98% | 100.00/0.00% | 9.69/6.16% | 1.277/1.401 |
| multiple people crossing | 97.66/3.97/7.63% | 100.00/100.00/100.00% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/5.78% | 1.354/1.641 |
| small fast target | 98.28/26.47/41.71% | 100.00/63.62/77.77% | 99.77/68.42/81.18% | 100.00/0.00% | 0.93/6.41% | 1.321/1.542 |
| slow target | 96.30/4.43/8.47% | 100.00/100.00/100.00% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/5.91% | 1.338/1.439 |
| moving box or vehicle | 94.02/3.25/6.29% | 100.00/99.91/99.96% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/5.76% | 1.355/1.481 |
| opening/closing door | 50.00/0.08/0.15% | 100.00/62.31/76.78% | 97.90/67.16/79.67% | 99.89/0.11% | 32.35/6.00% | 1.348/1.731 |
| large dynamic occlusion | 99.14/2.98/5.78% | 100.00/97.64/98.81% | 99.90/99.98/99.94% | 99.86/0.14% | 0.00/2.91% | 1.832/2.082 |
| radial approach/departure | 96.74/7.22/13.43% | 100.00/100.00/100.00% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/5.97% | 1.379/1.592 |
| moving then stops | 94.55/3.89/7.47% | 100.00/100.00/100.00% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/5.96% | 1.386/1.652 |
| co-moving target | 95.56/3.11/6.02% | 100.00/100.00/100.00% | 99.14/100.00/99.57% | 99.97/0.03% | 0.00/6.26% | 1.292/1.602 |
| appear/disappear behind occluder | 70.00/4.69/8.79% | 100.00/47.99/64.86% | 100.00/54.02/70.14% | 100.00/0.00% | 43.08/6.39% | 1.531/1.739 |
| near-wall motion | 96.74/5.69/10.76% | 100.00/92.07/95.87% | 100.00/94.18/97.00% | 100.00/0.00% | 0.96/6.08% | 1.368/1.682 |
| vertical target motion | 90.62/7.07/13.12% | 100.00/100.00/100.00% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/6.04% | 1.360/1.515 |
| non-rigid motion | 96.30/3.90/7.49% | 100.00/100.00/100.00% | 100.00/100.00/100.00% | 100.00/0.00% | 0.00/5.89% | 1.305/1.563 |
| far sparse target | 92.68/6.14/11.52% | 100.00/0.00/0.00% | 100.00/0.00/0.00% | 100.00/0.00% | 0.00/7.23% | 1.394/1.514 |

The pure-static scenarios contain no positive class, so dynamic P/R/F1 are N/A
and excluded from macro dynamic averages. Their false-dynamic ratio and static
preservation remain part of the pooled static metrics. This presentation
correction does not alter any classification or hide a regression.

Representative v2 per-scenario CPU/state figures from final run 1:

| Scenario | CPU P50/P95 ms | Peak state MiB | Declared failure mode |
|---|---:|---:|---|
| static fast turn | 1.369/1.586 | 0.600 | none |
| new area in FoV | 1.225/1.403 | 0.481 | none |
| person crossing | 1.099/1.284 | 0.459 | none |
| stationary then moving | 1.123/1.255 | 0.459 | none |
| multiple people crossing | 1.109/1.330 | 0.461 | none |
| small fast target | 1.123/1.298 | 0.472 | none |
| slow target | 1.116/1.314 | 0.459 | none |
| moving box or vehicle | 1.137/1.321 | 0.460 | none |
| opening/closing door | 1.127/1.325 | 0.456 | dynamic static-map contamination |
| large dynamic occlusion | 1.530/1.800 | 0.482 | none |
| radial approach/departure | 1.091/1.356 | 0.460 | none |
| moving then stops | 1.103/1.361 | 0.459 | none |
| co-moving target | 0.928/1.270 | 0.466 | none |
| appear/disappear | 1.249/1.505 | 0.450 | low dynamic recall |
| near-wall motion | 1.106/1.344 | 0.465 | none |
| vertical target motion | 1.105/1.336 | 0.459 | none |
| non-rigid motion | 1.109/1.282 | 0.460 | none |
| far sparse target | 1.143/1.370 | 0.539 | low dynamic recall |

## Weak-case interpretation

- Opening/closing door: v2 recall rises from 62.31% to 67.16%, F1 from 76.78%
  to 79.67%, and contamination falls from 33.76% to 32.35%. Confirmed static
  hinge/frame points cannot be relabeled solely by adjacent vacated evidence.
- Large occlusion: recall rises from 97.64% to 99.98%; background cells hidden
  by the occluder are not deleted. The remaining 0.14% static-boundary false
  dynamic ratio is below the declared failure threshold.
- Radial, slow, and moving-then-stops cases retain 100% recall/F1. Dynamic hold
  prevents a recently stopped target from immediately entering static.
- Near-wall F1 rises from 95.87% to 97.00% with 100% static preservation.
- Appear/disappear remains visibility-limited: F1 improves from 64.86% to
  70.14%, but hidden intervals cannot create evidence.
- Far sparse target has no previously observed free background and therefore
  cannot be safely called dynamic. V2 reports it UNKNOWN rather than static:
  contamination falls from the initial 45.23% to 0.00% through a configurable
  15 m / 12-observation far-range static dwell. Recall remains honestly 0%.

## Causal deskew and ROS2 smoke

The v2 node accepts only `causal_fastlio_imu` deskew. It selects the latest
FAST-LIO posterior with timestamp no later than scan start, estimates velocity
from earlier posterior states, and propagates raw `/livox/imu` to each Livox
nanosecond `offset_time`. A terminal zero-order hold is allowed only when the
last IMU-to-point gap is no greater than the configured 25 ms. Future pose
anchors, future IMU samples, regressed timestamps, missing coverage, and larger
gaps are rejected by unit tests.

Installed ROS2/Humble smoke results with real `livox_ros_driver2/CustomMsg`:

- statistics messages 8; scored points 1,617; max dynamic points 70;
- final static/dynamic/unknown 221/70/0;
- processing 0.289 ms; queue residence 1.525 ms;
- terminal IMU gap 19.696 ms within the 25 ms contract;
- deskew reason `ok`; deskew rejects/pose timeouts/queue overflow 0/0/0;
- exactly one `/livox/lidar` publisher, the smoke source;
- `fastlio_input_modified=false`.

No unified pose is subscribed. Because the anchor predates the current scan and
only IMU samples at or before each point are consumed, a future clean-scan
gateway has no unified-backend-to-observer-to-FAST-LIO cycle.

## Final quality gate

- All 17 repository packages built successfully in RelWithDebInfo mode.
- `colcon test` reported 94 xUnit tests, 0 errors, 0 failures, 0 skipped; the
  backend runner separately passed 283 tests and visual frontend passed 13.
- Observer core passed 17 gtests, including five visibility/state tests and
  seven causal deskew contract tests.
- Python bytecode, all repository YAML/package XML, shell syntax, and
  `git diff --check` passed.
- Disabled-mode launch created no LiDAR subscription; enabled Livox smoke had
  zero deskew rejection, queue overflow, or pose timeout and left no process.

## Decision

**PROMOTE_TO_DYN_INTEGRATION** means approve a separately gated, fail-open
integration experiment; it does not authorize replacing `/livox/lidar` in this
commit. Production cutover still requires captured/team MID360 truth evaluation
and raw-vs-clean dual FAST-LIO replay.
