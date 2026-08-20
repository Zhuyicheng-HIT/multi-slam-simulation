# DYN-INTEGRATION-005 Clean Scan Gateway

Date: 2026-08-20

Baseline: `dyn-eval-003-core-004-20260819` / `40d35ff5b399aa1767c0e78ea820020f77ebf12c`

This stage creates a default-off integration candidate. It does not remap or
replace production `/livox/lidar`, and it does not modify the frozen PR #14
tag, unified five-source backend, ExternalNav, EKF3, or one-observation-one-
factor contract.

## Evaluator contract audit

The frozen ten-scenario v1 result (74.13% recall) and the expanded 18-scenario
v1 result (95.08% recall) are both valid for their own inputs but are not an
algorithm A/B:

- the old matrix used ten hand-built scenarios, 28 frames, and three repeats;
- the new matrix uses 18 scenarios, 40 frames, seeds 101/202/303, two repeats
  per seed, low-altitude sensor origin, nearest-return occlusion, and
  deterministic MID360-like non-repetitive coverage holes;
- geometry, visibility, dynamic timing, and the opening-door case changed;
- the v1 class and its core thresholds did not change in that comparison.

The 20.95-point recall difference is therefore dataset composition and
visibility, not a claimed v1 improvement. The locked reporting contract is:

- **micro dynamic**: pool TP/FP/FN across all inputs; false positives from
  pure-static scenes remain in FP;
- **macro dynamic**: unweighted mean across dynamic-bearing scenarios only;
- pure-static Dynamic P/R/F1: **N/A**, never 100/100/100 and never included in
  macro dynamic scores;
- pure-static scenes remain fully represented by Static Preservation and False
  Dynamic Ratio.

Unit tests cover pure-static N/A, a positive class with no detections reporting
zero rather than perfect, and static false positives reducing precision. This
changes presentation only; no prior classification was relabelled.

## Gateway and state handoff

```text
frozen/raw MID360 -----------------------------> Raw FAST-LIO
       |
       +-> Clean Gateway -> namespaced clean --> Clean FAST-LIO
                              ^                       |
                              +-- previous posterior-+
       +------------------------- current/past IMU
```

- Raw and Clean FAST-LIO use independent namespaces, maps, states, parameters,
  and initial conditions.
- The raw topic is unchanged. Clean output is a separate Livox `CustomMsg`.
- STATIC is retained; DYNAMIC_CONFIRMED is removed; UNKNOWN is retained.
- Every retained point keeps x/y/z, reflectivity, tag, line, and `offset_time`.
  Message timestamp, timebase, lidar id, reserved fields, and point order are
  retained; remaining points are not retimed or renumbered.
- The previous-state message is exported only after Clean FAST-LIO completes a
  scan and carries pose, velocity, calibrated accel/gyro bias, timestamp,
  sequence, reset epoch, and frame contract.
- Scan i may use only a posterior strictly preceding scan i plus IMU samples no
  later than each point. It never subscribes to Raw FAST-LIO state, unified
  pose, current posterior, future IMU, or evaluator truth.
- An older causal posterior is not consumed immediately when a nearer previous
  posterior may still arrive. It waits inside the existing 250 ms bounded
  queue, removing callback-order nondeterminism without widening the 200 ms
  prediction horizon or accepting future data.

There is no instantaneous cycle: scan i produces posterior i only after the
gateway has already published scan i using posterior i-1.

## Fail-open contract

State/IMU timeout, stale posterior, timestamp regression, epoch change, queue
overflow, malformed classification, excessive observer latency, and internal
exception all publish the exact raw scan and an explicit degraded reason. No
gateway failure may drop a scan or synthesize an empty one.

The ROS/Livox smoke test verified exact raw passthrough for previous-state
timeout, IMU-coverage timeout, queue overflow, and input timestamp regression;
24/24 output scans and status messages arrived. It also verified retained point
metadata, five dynamic-removal scans, and a single original raw publisher.
C++ tests cover malformed/empty admission, bounded terminal IMU hold, excessive
terminal gap, future IMU/pose rejection, timestamp regression, and calibrated
bias propagation.

## Frozen Raw/Clean replay

The evaluator generated 12 low-altitude, near-constant-height bags: static,
person crossing, multiple targets, small/fast, slow, door, large occlusion,
radial, moving-then-stops, appear-after-occlusion, near-wall, and far/sparse.
Every branch replayed the same hashed 70-scan, 10 Hz MID360 and 100 Hz IMU
input. Truth sidecars were opened only by the analyzer.

### Aggregate result

| Metric | Raw | Clean |
|---|---:|---:|
| Dynamic micro P/R/F1 | 0/0/0% | 94.962/82.193/88.117% |
| Dynamic macro P/R/F1 (11 dynamic scenes) | 0/0/0% | 81.003/69.975/73.777% |
| Static preservation | 100.000% | 99.862% |
| False dynamic ratio | 0.000% | 0.138% |
| Admission contamination | 100.000% | 17.807% |
| Unknown ratio | 0.000% | 14.747% |
| Final-map contamination, scenario mean | 6.538% | 0.768% |
| Dynamic trace retained in final map, mean | 81.894% | 29.129% |
| Static map completeness, mean | 98.987% | 99.969% |
| ATE RMSE, scenario median | 0.006659 m | 0.006722 m |
| Translation RPE RMSE, median | 0.003310 m | 0.003094 m |
| Yaw RMSE, median | 0.247 deg | 0.225 deg |
| Lost / reset / queue overflow | 0 / 0 / N/A | 0 / 0 / 0 |
| Effective NativeLidarFactor packets | 803 | 804 |

Clean ATE median changes by +0.94% (0.063 mm), while translation RPE improves
6.53%. Mean ATE improves because large-occlusion ATE drops from 2.41 cm to
0.77 cm. Final-map contamination falls 88.25% by the evaluator-only spatial
voxel audit. Static completeness does not regress.

### Scenario result

| Scenario | Clean P/R/F1 | Actual map contamination Raw -> Clean | ATE Raw -> Clean (m) | Native Z info change |
|---|---:|---:|---:|---:|
| Static | N/A | 0.00 -> 0.00% | .00663 -> .00663 | -0.08% |
| Person crossing | 91.71/93.99/92.84% | 6.35 -> 0.00% | .00803 -> .00663 | +3.17% |
| Multiple targets | 97.77/92.99/95.32% | 10.57 -> 0.00% | .00691 -> .00612 | +3.65% |
| Small fast | 85.55/55.11/67.04% | 0.56 -> 0.63% | .00640 -> .00695 | +0.43% |
| Slow | 97.67/95.32/96.48% | 5.12 -> 0.00% | .00669 -> .00681 | +1.90% |
| Opening/closing door | 96.23/56.94/71.54% | 0.94 -> 0.00% | .00610 -> .00644 | +0.67% |
| Large occlusion | 98.68/92.19/95.33% | 35.73 -> 2.73% | .02411 -> .00774 | +9.88% |
| Radial motion | 95.90/91.62/93.71% | 5.24 -> 0.14% | .00579 -> .00664 | +1.58% |
| Moving then stops | 90.26/80.29/84.98% | 4.11 -> 2.01% | .00822 -> .00686 | +2.39% |
| Appear after occlusion | 53.13/16.49/25.17% | 1.04 -> 0.00% | .00488 -> .00483 | -0.60% |
| Near-wall motion | 84.12/94.78/89.13% | 4.66 -> 0.00% | .00632 -> .00702 | +3.04% |
| Far sparse | 0/0/0% | 4.14 -> 3.70% | .00687 -> .00687 | -0.11% |

The small-fast case has a 0.07-point absolute map-contamination increase and
the far-sparse case remains undetected. Both are reported, not tuned away. Far
sparse returns lack confirmed historical free/ray evidence and therefore stay
UNKNOWN/raw by design. Appear-after-occlusion is similarly visibility-limited.

## Native factor observability

Across scenarios, Clean-vs-Raw translation information changes were:

- X: median -0.30%, worst -4.98%;
- Y: median -0.82%, worst -7.85% in the large-occlusion scene;
- Z: median **+1.74%**, worst **-0.60%**, best +9.88%;
- residual RMS: median 5.71% lower;
- matched-point median: unchanged at 400;
- information condition number: median 0.68% lower.

The clean scan does not materially weaken Z. The weakest-direction and
condition audits show no observability collapse, and all Clean runs retained
67 effective factor packets.

## Runtime

- gateway compute P50/P95/P99, median across scenarios:
  4.527/5.009/5.912 ms; worst P99 9.027 ms;
- gateway CPU: median 6.50% of one core; peak RSS 53.11 MiB;
- FAST-LIO CPU: Raw 29.37%, Clean 28.76% scenario median; RSS about 176 MiB;
- healthy queue residence P50/P95/P99: 70.996/72.137/162.896 ms. The normal
  wait is causal IMU coverage, not observer compute;
- Clean-input-to-odom callback P50/P95/P99 scenario medians:
  18.64/21.98/22.24 ms. Combined normal raw-arrival-to-odom remains about one
  10 Hz scan period, comparable to Raw;
- 840/840 clean scans and status messages, zero missing, zero overflow, maximum
  odom gap 100.01 ms, zero lost/reset.

There were 86 explicit fail-open scans: 48 initialization state timeouts and 38
bounded stale-state timeouts during WSL scheduling spikes. This is safe
degradation, not dropped data. The large-occlusion/static/far-sparse runs show
occasional 0.6-0.9 s FAST-LIO callback spikes; exact raw passthrough maintained
continuity. Hardware scheduling must be remeasured before production cutover.

The external pinned FAST-LIO process completes every output but its upstream
signal-handler/destructor exits 245 after SIGINT in the replay harness. This
occurs after all 67 odometry/factor messages and is not a runtime lost/reset;
it remains an external shutdown-hygiene issue.

## Gate

**PROMOTE_CLEAN_GATEWAY** as a reversible, default-off integration candidate.

This decision does not authorize remapping production `/livox/lidar`. A team
MID360 capture with hardware timing and evaluator-only annotations is still
required before production input cutover, especially for far-sparse,
appear-after-occlusion, small-fast targets, and scheduling-spike frequency.
