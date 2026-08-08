# Performance V2 stability convergence

## Freeze decision

Performance V2 is frozen.  The earlier 42.399 ms versus 54.375 ms live-run
spread is a simulation-host scheduling effect, not a repeatable estimator
regression.  The frozen full-online input replay meets the `<= 45 ms` gate in
all five V2 runs, with a 15.766 ms median of run medians and 2.55% run-to-run
CV.  No estimator threshold, sensor model, D_V/FRS setting, 0.065 s visual
association tolerance, integrity check or rollback behavior was changed.

The complete Gazebo wall-time metric is classified `SIM_ENV_CONTENDED`.
Performance profiling remains opt-in and CPU affinity remains a diagnostic;
neither changes the default deployment configuration.

## Cause of the live-run spread

The exact transaction trace separates estimator work from time spent off CPU.
With the same V2 implementation and frozen online inputs, solver P50 was
17.535 ms.  Under Gazebo, RGB-D rendering, bridges, FAST-LIO, mapping and SITL,
it was 33.081 ms.  The 15.546 ms difference is 47.0% of the full-Gazebo P50
and is larger than the previously observed 11.976 ms 42/54 spread.

The strongest full-Gazebo correlations with solver duration were voluntary
context switches (`r=0.821`) and involuntary context switches (`r=0.535`).
The graph-linearization P50 changed from 17.092 ms in replay to 35.066 ms in
Gazebo while the graph shape stayed at eight states and normally 400 LiDAR
correspondences.  This broad inflation of the same numeric work is consistent
with preemption, not an added factor or changed estimator path.  Minor faults
had only `r=0.304`; major faults were zero.

The profiled Gazebo process groups used 6.906% of whole-WSL CPU capacity for
SIM_ONLY work (Gazebo, simulation sensor bridges and SITL) and 5.919% for the
named REAL_TRANSFERABLE pipeline (backend, FAST-LIO, visual frontend and
shared mapping).  SIM_ONLY was therefore 53.85% of this non-overlapping named
CPU set.  This is not a percentage of all host CPU work.

Gazebo could not open `/dev/dri/renderD128` because the device is owned by the
`render` group and the runtime user is not a member.  EGL consequently used
`kms_swrast`.  `/dev/dxg` and the NVIDIA device were visible, but the installed
OpenCV reported no CUDA or OpenCL device.  No group, driver or sudo-level
system setting was changed.

## Transaction timing decomposition

The following values are P50 / P95 milliseconds over 2,874 V2 full-online
replay transactions from the five accepted runs.  Timings nested inside graph
linearization overlap and therefore must not be summed.

| Stage | P50 | P95 |
|---|---:|---:|
| snapshot | 0.178 | 0.411 |
| state staging | 2.507 | 3.524 |
| IMU factor construction | 0.060 | 0.086 |
| NativeLidarFactor construction | 0.204 | 0.449 |
| GNSS factor construction | 0.045 | 0.062 |
| optical-flow factor construction | 0.007 | 0.566 |
| visual association | 0.034 | 0.056 |
| visual factor construction | 0.770 | 1.188 |
| graph assembly | 0.590 | 1.401 |
| graph linearization | 17.092 | 32.042 |
| LiDAR point-plane linearization subset | 5.991 | 10.807 |
| IMU preintegration linearization subset | 5.537 | 10.477 |
| marginal-prior linearization subset | 4.084 | 7.043 |
| visual reprojection linearization subset | 2.627 | 4.562 |
| linear solve | 0.657 | 1.403 |
| marginalization | 2.495 | 3.498 |
| integrity check | 0.145 | 0.256 |
| transaction commit | 0.807 | 2.422 |
| callback total | 22.450 | 39.876 |

The solver itself was 17.535 / 34.386 ms P50/P95.  Non-marginalizing startup
cycles were 11.520 / 21.721 ms (40 cycles); marginalizing cycles were
17.599 / 34.432 ms (2,834 cycles).  Marginalization is a normal, visible cost,
but its stable cadence does not explain the 42/54 run-to-run spread.

## Factor scale and runtime noise

The active window had a median of eight states, 400 LiDAR correspondences and
seven IMU factors.  Solver correlations were small for state count (`r=0.124`),
LiDAR correspondences (`r=-0.144`), IMU factors (`r=0.090`) and visual factors
(`r=0.172`).  Flow-factor presence had the largest factor-shape correlation
(`r=0.354`) but was much smaller than voluntary context switching
(`r=0.650`) in replay.

Scale-normalized P50 values were 0.0466 ms per LiDAR correspondence and
2.202 ms per window state.  Transactions with an active visual factor had a
21.438 ms solver P50 versus 17.295 ms without one; the factor is real work but
its low, fixed cadence does not match the whole-run variance.  The trace stores
both active factor counts and per-cycle newly added factor counts.

GC was not disabled.  A callback profiler recorded generation collections,
allocation counts and duration.  Replay GC duration correlated only `r=0.058`
with solver duration; in full Gazebo it was `r=0.086`.  Median and P95 GC time
per cycle were both zero, so no GC-specific production change was justified.
CPU frequency files were unavailable inside this WSL instance and are reported
as unavailable rather than inferred.

## Full-online V1/V2 replay

The frozen 172.236 s input contains 83,125 messages and exercises the scan
prediction/native-factor handshake, IMU, GNSS, flow, visual observations,
D_V/reliability scheduling, pending association, the eight-state window,
transactional integrity and marginalization.  It deliberately excludes
Gazebo, ArduPilot, camera rendering, maps and ground truth and remains under
the ignored `logs/tmp` tree.

| Metric | Frozen V1 | V2 |
|---|---:|---:|
| per-run medians (ms) | 19.546, 18.855, 18.844, 18.991, 18.821 | 15.766, 15.146, 15.465, 16.334, 15.899 |
| median of run medians (ms) | 18.855 | 15.766 |
| run-median CV | 1.44% | 2.55% |
| pooled P50 (ms) | 19.667 | 16.916 |
| pooled P90 (ms) | 28.954 | 23.641 |
| pooled P95 (ms) | 31.303 | 25.969 |
| odometry messages per run | 574--576 | 574--576 |

V2 improves the matched run-median by 16.38% and pooled P95 by 17.04%.  The
transaction trace has zero optimization errors, integrity rejects and
rollbacks.  Each run accepted 11 visual factors.  A few duplicate native
worker inputs were coalesced by the existing bounded queue; they were not
integrity failures and did not change the final estimator state.

## CPU-affinity diagnostic

On the same replay, normal scheduling produced 17.839 / 35.528 ms solver
P50/P95.  Pinning only the backend to CPU 30 produced 13.711 / 29.795 ms,
improvements of 23.1% and 16.1%.  Median voluntary context switches per cycle
fell from 171.5 to 11.0.  Involuntary switches rose from 0 to 8 because one
isolated logical CPU can still be preempted.  Correctness remained zero-error,
zero-integrity-reject and zero-rollback.

The full-stack affinity probe reduced Gazebo solver P50 from 33.081 to
31.477 ms and P95 from 73.364 to 54.679 ms.  That probe is not correctness
evidence: its flight encountered eight early bias-integrity rollbacks and its
post-flight vision topic wait stalled until bounded cleanup.  The diagnostic
script now continuously assigns late-starting descendants, rather than taking
one early process snapshot.  Affinity is retained only as a reproducible
diagnostic and is not a deployment requirement.

## Correctness gates retained

The accepted final three rectangles and three S-curves remain the correctness
set: candidate-to-solver acceptance is 71.81%, time rejection is 10.28%, and
optimization errors, integrity rejects and rollbacks are all zero.  The prior
matched five-run V1/V2 rectangle comparison found no systematic accuracy
regression.  The accepted joint-map run has 108,191 voxels, 10.41% occupied
volume growth, 12.50% color coverage, zero geometry conflicts and zero
evictions; LiDAR remains geometry authority.

The new performance-only instrumentation is disabled by default.  It writes a
bounded JSONL trace only when explicitly requested, records process faults,
context switches, CPU/RSS, GC and process-group load, and does not alter factor
selection or solver math.

Final verification:

- 15 packages built in `RelWithDebInfo`;
- 57 colcon result files, zero errors, failures or skips;
- backend 160/160 and visual frontend 4/4 direct tests passed;
- final trace-schema smoke replay: 576 cycles, 576 odometry messages, 11/11
  visual factors, 17.108 / 34.370 ms solver P50/P95 and zero correctness
  failures;
- D435i active-run lifecycle short test passed;
- Python 198, YAML 29, XML 15 and shell 53 checks passed;
- `git diff --check` passed;
- all owned runtime processes were cleaned up;
- frozen V1 remained at `d76543e9c8f80dcaecbcbe4d898811a420978094`
  with no source modification.
