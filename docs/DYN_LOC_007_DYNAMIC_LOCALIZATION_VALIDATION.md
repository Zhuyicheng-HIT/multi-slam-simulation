# DYN-LOC-007: Dynamic localization and cross-session validation

## Scope and safety contract

DYN-LOC-007 starts at commit
`64dd899cb5ef0f11a8e3324626971f000a8a5e8c`, tagged
`dyn-map-006-long-term-static-refinement-20260820`. It evaluates whether the
already promoted observer v2, fail-open Clean Gateway, and long-term
`STATIC_CONFIRMED_ONLY` map improve localization. It does not change the PR #14
frozen tag, production `/livox/lidar`, FAST-LIO factor definitions, the
five-source backend, ExternalNav, EKF3, or Z-axis fusion.

All Raw/Clean runs use identical frozen MID360 and IMU messages. Clean FAST-LIO
uses its own previous posterior plus current/past IMU only. Gazebo/synthetic
truth is stored in an evaluator-only sidecar and is never presented to the
observer, gateway, map lifecycle, descriptor database, or registration core.
No current/future Raw FAST-LIO or unified pose is used by Clean. There is no
instantaneous estimator loop.

## Frozen current-localization replay

The replay contains 90 scans per scenario. BEFORE is scans 0-24, DURING is
25-54, and AFTER is 55-89. Ten primary low-altitude scenarios are evaluated:
person crossing, multiple targets, small fast target, slow target, opening or
closing door, large dynamic occlusion, radial motion, moving then stopping,
near-wall motion, and occlusion/reappearance.

The following values are macro means over the ten scenarios. Distances are in
metres unless noted.

| Phase | Branch | ATE RMSE | translation RPE | yaw RPE (deg) | Z RMSE | pose jump P95 | Native residual | effective factors |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| BEFORE | Raw | 0.009081 | 0.001333 | 0.008529 | 0.001216 | 0.001969 | 0.014843 | 210 |
| BEFORE | Clean | 0.009057 | 0.001334 | 0.008529 | 0.001113 | 0.001969 | 0.014843 | 210 |
| DURING | Raw | 0.005645 | 0.002756 | 0.016390 | 0.001426 | 0.004156 | 0.022971 | 300 |
| DURING | Clean | 0.004908 | 0.001892 | 0.010745 | 0.001304 | 0.002972 | 0.020359 | 300 |
| AFTER | Raw | 0.005408 | 0.003986 | 0.015104 | 0.000943 | 0.003546 | 0.020005 | 360 |
| AFTER | Clean | 0.005267 | 0.003940 | 0.014544 | 0.001102 | 0.003523 | 0.019519 | 360 |

During dynamic events, Clean improves macro ATE by 13.1%, translation RPE by
31.4%, pose-jump P95 by 28.5%, and Native residual by 11.4%. Clean absolute yaw
RMSE is 0.314 degrees versus Raw 0.212 degrees, while the more relevant
frame-to-frame yaw RPE improves from 0.0164 to 0.0107 degrees. There are no
resets in either branch.

### Per-scenario dynamic interval

| Scenario | Raw/Clean ATE (mm) | Raw/Clean RPE (mm) | Raw/Clean jump P95 (mm) | Raw/Clean residual (mm) |
|---|---:|---:|---:|---:|
| Person crossing | 7.03 / 4.54 | 2.80 / 1.66 | 3.86 / 2.81 | 22.95 / 19.35 |
| Multiple targets | 6.43 / 5.32 | 3.08 / 2.02 | 4.32 / 3.25 | 26.70 / 19.65 |
| Small fast target | 4.42 / 4.93 | 2.13 / 2.00 | 4.10 / 3.71 | 19.54 / 19.76 |
| Slow target | 4.90 / 4.54 | 1.92 / 1.88 | 2.73 / 3.05 | 20.97 / 20.54 |
| Opening/closing door | 5.48 / 5.03 | 2.07 / 1.90 | 3.30 / 2.96 | 21.72 / 19.99 |
| Large dynamic occlusion | 34.77 / 6.07 | 13.15 / 2.31 | 25.85 / 2.74 | 41.73 / 21.32 |
| Radial motion | 5.81 / 4.47 | 3.29 / 1.67 | 5.92 / 2.87 | 23.90 / 20.23 |
| Moving then stops | 5.86 / 4.95 | 2.12 / 2.13 | 3.63 / 3.45 | 22.99 / 20.60 |
| Near-wall motion | 5.26 / 4.89 | 2.81 / 1.68 | 4.21 / 2.98 | 22.72 / 20.49 |
| Occlusion/reappearance | 4.34 / 3.82 | 2.72 / 1.78 | 5.74 / 2.67 | 26.42 / 26.12 |

Small-fast ATE rises by 0.51 mm and slow-target jump P95 rises by 0.32 mm, but
their RPE/residual remain equal or better and neither produces a reset, factor
loss, or Z-information collapse. They are recorded rather than hidden.

## NativeLidarFactor XYZ information

Median translational information diagonals over the primary scenarios are:

| Phase | Branch | X | Y | Z | condition number |
|---|---|---:|---:|---:|---:|
| BEFORE | Raw/Clean | 100059 | 151274 | 148667 | 1.515 |
| DURING | Raw | 98171 | 155846 | 145092 | 1.584 |
| DURING | Clean | 97392 | 151856 | 151643 | 1.577 |
| AFTER | Raw | 99198 | 154769 | 146383 | 1.573 |
| AFTER | Clean | 99369 | 149350 | 151740 | 1.538 |

Clean changes DURING X/Y/Z information by -0.59%/-2.57%/+2.16%. AFTER the
changes are +0.26%/-2.66%/+2.15%. Effective factor counts are identical. Z is
not weakened; the conditioning remains essentially unchanged.

## Large-occlusion constraint impact

The primary large-moving-occluder run proves that Raw dynamic geometry reaches
the current point-to-plane constraints: DURING Raw ATE, jump P95, and residual
are 34.77 mm, 25.85 mm, and 41.73 mm. Clean reduces them to 6.07 mm, 2.74 mm,
and 21.32 mm. Raw/Clean Z RMSE is 3.91/4.13 mm, while Z information increases
from 128530 to 152370. Thus the improvement is not obtained by discarding the
vertical constraint.

The `occlusion_appear` Raw comparator emitted source stamp 5.889958755 before
5.289957916, although the frozen input is strictly monotonic. This created one
evaluator `lost` flag (0.7 s apparent source-stamp gap) without a reset or lost
factor count. The Clean branch emits all 87 posterior/factor messages in order
with no gap. This is a measured Raw callback-order anomaly, not a hidden Clean
drop.

## Cross-session relocalization

Session A is saved with a person leaving, P1 moving to P2, an empty-A/occupied-B
case, and multiple targets changing position. Session B restarts from a new
process-equivalent map/database instance. Each map has four ESF candidates;
the existing keyframe database ranks them and the existing GICP core performs
geometric verification. The same 0.198 m coarse initial error is used for all
maps. Three seeds and three consecutive query frames produce 36 registrations
per map.

| Reference map | Success | false relocalization | stable error | yaw error | inliers | overlap | dynamic reference | contamination | completeness |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Raw | 36/36 | 0/36 | 1.45 mm | 0.0017 deg | 1190 | 47.19% | 0.143% | 2.56% | 100% |
| Clean | 36/36 | 0/36 | 1.37 mm | 0.0018 deg | 1190 | 47.19% | 0.131% | 2.34% | 100% |
| STATIC_CONFIRMED_ONLY | 36/36 | 0/36 | 11.65 mm | 0.0029 deg | 1058 | 41.97% | 0% | 0.31% | 87.85% |

Every three-frame trial reaches a stable pose in 0.2 s. Geometric verification
successfully rejects wrong descriptor candidates; the selected descriptor rank
averages 3.31/3.50/3.08 for Raw/Clean/Refined, yet the false-localization rate
is zero. Refined geometry trades some short-session completeness for purity,
but retains enough geometry and does not reduce success. The longer DYN-MAP-006
matrix remains the representative steady-state completeness result (96.82%).

Condition-level Refined stable errors are 6.55 mm (person left), 7.00 mm (P1 to
P2), 7.81 mm (A empty/B occupied), and 25.23 mm (multiple targets moved); all
are accepted correctly with zero dynamic-reference contribution. This is an
offline candidate/database/registration validation, not a claim that a full
online loop closure was executed.

## Occlusion observability split

Current Raw/Clean FAST-LIO remains stable in all three frozen cases:

| Case | Raw/Clean full ATE | Raw/Clean DURING residual | Raw/Clean Z information | Raw/Clean map contamination |
|---|---:|---:|---:|---:|
| C1 persistent | 8.35 / 6.87 mm | 25.61 / 20.84 mm | 121156 / 144549 | 13.96% / 15.84% |
| C2 vacated, same view | 7.61 / 7.44 mm | 25.80 / 22.27 mm | 121605 / 145065 | 15.93% / 15.50% |
| C3 vacated, natural multi-view | 8.13 / 6.66 mm | 25.61 / 20.84 mm | 121156 / 144595 | 15.39% / 14.63% |

FAST-LIO maps are append-oriented, so they do not retroactively remove all old
geometry. The actual long-term lifecycle gives the following independent
three-seed map result when the occluder is present from the first scan (no
historical free evidence):

| Case | Raw contamination | Clean contamination | Refined contamination | Refined completeness |
|---|---:|---:|---:|---:|
| C1 persistent | 10.47% | 10.47% | 11.26% | 88.31% |
| C2 same-view reobservation | 10.47% | 10.47% | 5.18% | 89.76% |
| C3 natural multi-view | 10.47% | 10.47% | 4.96% | 88.00% |

C1 is a `PHYSICAL_OBSERVABILITY_LIMITATION`: without a measured ray through
the occupied region, the system cannot causally prove free space, and must not
aggressively delete UNKNOWN. C2/C3 prove that real reobservation activates
refinement and removes about half the contamination. Their residual is an
`ALGORITHM_AND_COVERAGE_LIMITATION` caused by finite MID360 ray coverage and
the conservative evidence count, not by lack of a future pose. It does not
cause a current-localization failure in these runs.

## Runtime and fail-open behavior

- Raw FAST-LIO: median 29.99% CPU and 176.63 MiB RSS.
- Clean FAST-LIO: median 29.25% CPU and 176.76 MiB RSS.
- Gateway: median 6.61% CPU, 52.81 MiB RSS, latency P50/P95/P99
  4.464/5.081/5.781 ms.
- Gateway: 0 missing clean scans, 0 queue overflow, 0 Clean lost runs, 0 resets.
- The 50 fail-open events are explicit startup/scheduling health events; every
  affected raw scan is passed exactly and no input interruption occurs.
- Cross-session ESF plus four-candidate GICP verification has P50/P95 latency
  128.4/170.3 ms for Refined and a 34.1 MiB process high-water mark. It is an
  offline relocalization workload, not a 10 Hz estimator callback.

ROS smoke verifies observer v2 Livox input, exact-raw fail-open for missing
state/IMU/overflow/timestamp regression, preservation of Livox metadata,
`STATIC_CONFIRMED_ONLY` relocalization/loop outputs, semantic shadow-only mode,
and `future_pose_used=false`. Production `/livox/lidar` still has exactly one
publisher and is not modified.

## Build, tests, and promotion

- 18 project packages build successfully.
- `colcon test`: 122 tests, 0 errors, 0 failures, 0 skipped.
- Frozen Raw/Clean replay: 14 scenarios x 2 branches completed; all players
  exited normally.
- Cross-session registration benchmark: 108 map registrations, all accepted
  correctly, no false relocalization.
- Observer, Clean Gateway, and long-term/semantic ROS2 smoke tests pass.
- `git diff --check` and Python/XML/YAML syntax checks pass.
- No future pose, future IMU, truth leak, production LiDAR remap, backend
  ownership change, ExternalNav/EKF3 change, or Z-axis change is present.

The software gate result is `PROMOTE_DYNAMIC_LOCALIZATION_STACK`. Production
activation remains default-off and reversible. Remaining blockers are real
MID360/D435i timing and ray-coverage validation, long-duration hardware CPU and
memory measurement, and team-data cross-session testing. Persistent complete
occlusion remains physically unobservable until the region is reobserved. A
full online loop-closure control-path success is deliberately not claimed.
