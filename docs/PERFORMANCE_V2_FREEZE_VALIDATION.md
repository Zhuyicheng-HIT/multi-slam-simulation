# Performance V2 freeze validation

## Decision

Performance V2 remains a candidate and is **not frozen** by this validation.
The accuracy, visual-use, safety, mapping, build and test gates pass, but the
required `<= 45 ms` live solver median did not reproduce in the final six-run
set.  No algorithm threshold was relaxed and no result was discarded to force
a freeze decision.

## Rectangle V1/V2 A/B

The matched five-run rectangle comparison used the same world, route, sensor
rates, balanced visual cadence, 0.065 s association tolerance, D_V/FRS,
ExternalNav and FAST-LIO configuration.  Optical-flow noise used fixed seed
29.  Gazebo does not expose a global seed through this launch contract.

| Statistic | Frozen V1 | V2 candidate |
|---|---:|---:|
| translation RPE median | 0.039753 m | 0.030634 m |
| translation RPE mean | 0.040304 m | 0.031494 m |
| translation RPE std | 0.008858 m | 0.004174 m |
| translation RPE min / max | 0.025359 / 0.052759 m | 0.026573 / 0.038008 m |
| ATE median | 0.533603 m | 0.427050 m |
| solver median of run medians | 63.378 ms | 46.541 ms |
| visual accepted / quality-valid | 294 / 386 | 367 / 476 |
| time rejection | 9.84% | 10.29% |

The earlier rectangle warning is not systematic.  V2 translation RPE median
is 22.9% lower than V1 and its ATE median is 20.0% lower.  Therefore none of
the marginal-prior block transform, transaction snapshot optimization or
visual vectorization was reverted.

## Remaining visual timing rejection

Across the final three rectangle and three S-curve runs, 782 of 1089
quality-valid candidates were solver accepted (71.81%).  There were 112 time
rejects (10.28%):

- 105 `state_tolerance_mismatch` rejects (93.75% of time rejects): 58 missing
  the left-side state and 47 missing the right-side state;
- 7 candidates expired outside the active window (6.25%);
- zero queue overflow, duplicate submission or track rejection.

Rejected observations commonly fell 0.066--0.099 s from the nearest real
state, with occasional 0.198 s LiDAR-state gaps.  Queue residence and ROS wall
scheduling jitter were present, but the pending buffer cannot synthesize a
missing state.  The implementation still uses real causal states only: the
0.065 s tolerance, source timestamps and D_V threshold are unchanged; no
future state, retimestamping or interpolation was introduced.

## Final accuracy and runtime

| Scenario | ATE (m) | translation RPE (m) | rotation RPE (deg) | solver median / P95 (ms) | RTF |
|---|---:|---:|---:|---:|---:|
| rectangle r151 | 0.154630 | 0.027468 | 0.120747 | 42.509 / 91.823 | 0.4566 |
| rectangle r153 | 0.636268 | 0.033044 | 0.126866 | 54.807 / 70.805 | 0.4731 |
| rectangle r154 | 0.459032 | 0.028227 | 0.115221 | 58.348 / 83.676 | 0.4167 |
| S-curve r151 | 0.687640 | 0.046254 | 0.112758 | 50.028 / 76.515 | 0.4419 |
| S-curve r152 | 0.538685 | 0.046189 | 0.112401 | 53.943 / 78.356 | 0.5294 |
| S-curve r153 | 0.525499 | 0.047406 | 0.115260 | 56.019 / 74.311 | 0.4432 |

The six-run solver median of run medians is 54.375 ms and the corresponding
P95 median is 77.435 ms.  This fails the freeze threshold despite the original
V2 candidate's six-run 42.399 ms result.  All six runs had zero optimization
errors, integrity rejects and transaction rollbacks.  The interrupted r152
rectangle directory is retained as an external task-interruption sample and
is not represented as a completed runtime run.

## Replay and simulation separation

The same deterministic 180-frame factor stream produced identical V1/V2 costs
and final states.  Median pure replay throughput was 111.84 frame/s for V1 and
112.97 frame/s for V2, a 1.01% increase.  This replay path exercises the base
window solver but not all online transaction, callback and visual hot paths,
so it does not reproduce the 27.95% original live-solver improvement.

For the final six complete simulations, median classified process CPU was:

- REAL_TRANSFERABLE (backend, visual frontend, FAST-LIO, shared mapping):
  4.296% of whole-WSL capacity;
- SIM_ONLY (Gazebo, simulation bridges and SITL): 7.330%;
- SIM_ONLY share of classified pipeline CPU: 63.05%.

Only 29.86% of whole-WSL CPU was attributable to those named groups, so this
63.05% is a classified-pipeline share, not a claim about all host work.  Gazebo
used `kms_swrast`: `/dev/dri/renderD128` is owned by group `render`, while the
runtime user is not a member.  `/dev/dxg` and the RTX 4070 are visible, but
OpenCV reports no CUDA/OpenCL path.  Fixing group/driver/WSLg configuration
requires host or sudo-level system changes and is recorded as
`SIM_ENV_BLOCKED`.

## Joint map and verification

The final joint-map run completed LAND/disarm with 108191 total voxels:
97990 LiDAR voxels, 22449 RGB-D voxels and 10201 supplementary RGB-D voxels.
Occupied-volume growth was 10.41%, color coverage 12.50%, conflict ratio zero
and evictions zero.  LiDAR remained the geometry authority.

- 15 packages built in `RelWithDebInfo`.
- Full colcon result: 57 test-result entries, 0 errors, 0 failures, 0 skipped.
- Backend 158/158 and visual 4/4 direct tests passed.
- D435i active-run lifecycle short test passed.
- Python 198, YAML 29, XML 15 and shell 53 syntax/static checks passed.
- `git diff --check` passed.

The candidate can be frozen only after the live solver median is shown to be
stably at or below 45 ms on a controlled runtime host.  The RTF shortfall is
separately classified as `SIM_ENV_BLOCKED`; it is not a reason to alter fusion
semantics.
