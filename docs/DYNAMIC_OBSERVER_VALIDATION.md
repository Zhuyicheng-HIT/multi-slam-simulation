# Dynamic observer phase-1 validation

> This is the frozen v1/ten-scenario record. DYN-EVAL-003 v2 results are in
> `DYNAMIC_OBSERVER_V2_VALIDATION.md`; the values use different matrices and
> must not be compared as if they came from one input set.

Date: 2026-08-19

## Safety and ownership

`uf_dynamic_observer` is disabled by default. When enabled it subscribes to the
existing `/livox/lidar` `livox_ros_driver2/CustomMsg` and
`/fusion/unified/odom` in parallel. It publishes only namespaced side-channel
clouds/statistics and never publishes `/livox/lidar`, `/Odometry`, unified odom,
or TF. FAST-LIO therefore continues to receive the original input.

The observer never subscribes to truth. Scenario truth is used only by the
standalone benchmark/evaluator.

## Implemented first layer

- C++17 conservative free-space core with a three-state output:
  static/dynamic/unknown.
- Classification precedes current-scan map updates, preventing self-evidence.
- Repeated free-space confirmation, endpoint guard, bounded dynamic growth, and
  slow occupied recovery.
- Livox per-point `offset_time` deskew using bounded interpolation between
  committed poses.
- Bounded pending queue, pose timeout, queue overflow, processing latency, and
  voxel-memory diagnostics.
- PointCloud2 compatibility mode for the legacy simulator path.
- No TF publication and no original-map modification.
- Existing `TemporalVoxelFilter` behavior reproduced as the required A/B
  baseline inside the deterministic harness.

The current per-point pose source is deliberately a delayed committed unified
pose. That makes observer A/B causal, but it would form a loop if used before
FAST-LIO. A production gateway must instead share FAST-LIO's causal IMU
prediction/deskew trajectory.

## Deterministic scenario matrix

Ten scenarios are generated without detector access to their labels:

1. static environment with a fast turn/new view;
2. new static region entering FoV;
3. person crossing;
4. stationary person beginning to move;
5. multiple people crossing;
6. small fast target;
7. slow target;
8. moving box/vehicle;
9. opening door;
10. large dynamic occlusion.

Every independent benchmark invocation runs every scenario three times. Three
independent invocations produced byte-identical classification metrics; latency
varied normally with scheduling.

### Aggregate A/B

| Metric | Conservative free-space observer | TemporalVoxelFilter baseline |
|---|---:|---:|
| Dynamic precision | 100.000% | 83.265% |
| Dynamic recall | 74.130% | 4.837% |
| Dynamic F1 | 85.144% | 9.142% |
| Static preservation | 100.000% | 99.493% |
| False dynamic ratio | 0.000% | 0.507% |
| Static-map contamination | 21.051% | 85.094% |
| Map completeness | 92.108% | 92.366% |
| P50 latency, three runs | 2.901-2.984 ms | 1.436-1.554 ms |
| P95 latency, three runs | 7.657-8.076 ms | 4.045-4.183 ms |

The new observer deliberately trades 0.258 percentage points of confirmed map
completeness and about 1.4 ms median CPU time for much higher dynamic recall and
zero false-dynamic labels in this deterministic geometry. It does not claim
these numbers will transfer to real MID360 data.

### Scenario findings

- Person crossing F1: 98.40%; multiple crossing: 98.08%.
- Small fast target F1: 92.20%; slow target: 100.00%.
- Moving box/vehicle F1: 99.98%.
- Stationary-then-moving F1: 88.66%; evidence is delayed until the object enters
  previously confirmed free space.
- Opening door is the weakest case at 54.66% F1; the articulated surface was
  previously valid static structure, so a conservative method initially keeps
  or marks much of it unknown.
- Large occlusion F1: 80.47%; background rays are unavailable while the large
  surface blocks them.
- Static fast-turn and new-FoV cases produced zero false dynamic points; novel
  structure remains unknown until confirmed.

These two weak cases justify retaining a separate long-term map-refinement
backend and prohibit immediate FAST-LIO cutover.

## ROS 2 protocol smoke test

A live ROS 2 smoke test used the installed package, the actual
`livox_ros_driver2/CustomMsg` type, per-point offsets, and a 20 Hz odometry
stream. The synthetic packet generator was the only `/livox/lidar` publisher;
the observer did not take topic ownership.

Observed final sample:

- statistics messages: 8;
- scored points received: 1,617;
- maximum dynamic candidates in one scan: 70;
- final static/dynamic/unknown: 221 / 70 / 0;
- processing latency: 2.999 ms;
- pose-queue residence: 2.075 ms;
- scan duration represented by offsets: 9.667 ms;
- queue overflow: 0;
- pose timeout: 0;
- `/livox/lidar` publishers: 1 (the smoke source only);
- `fastlio_input_modified`: false.

This validates the ROS2/Humble/Livox message and output contract, not sensor
accuracy. The packets are deterministic synthetic geometry rather than captured
MID360 data.

## Localization/map metrics in phase 1

Because the observer is not connected to FAST-LIO, ATE, RPE, and
NativeLidarFactor residual are identical between observer-on and observer-off by
wiring, not by an estimator experiment. The deterministic report records zero
input mutations and zero deltas only to prove isolation. It must not be quoted as
an accuracy improvement.

The frozen flight launcher could not start LiDAR in this desktop environment due
to its hard-coded GPU requirement, as recorded in
`DYNAMIC_STATIC_MAP_BASELINE_FREEZE.md`. Consequently, current evidence does not
include real/Gazebo static-map contamination, flight ATE/RPE, closed-loop map
completeness, or NativeLidarFactor residual changes.

## Entry decision

Status: **observer prototype PASS; FAST-LIO frontend cutover NOT YET APPROVED**.

Before a formal clean-scan gateway is enabled:

1. record or obtain synchronized team MID360 `CustomMsg`, IMU, calibration,
   committed pose, and evaluator-only truth/annotations;
2. run all ten cases against that frozen input, including aggressive UAV 6DoF;
3. implement the DUFOMap-style fully-observed 3-D neighborhood test;
4. validate any MID360 raycast enhancement using a measured FoV/scanning mask;
5. replace delayed unified-pose deskew with the same causal IMU trajectory used
   by FAST-LIO;
6. replay raw and clean branches into identical FAST-LIO instances and compare
   ATE/RPE, residuals, map contamination/completeness, CPU/RAM, and P95 latency;
7. require fail-open behavior and static preservation before changing topic
   ownership.

## Final local quality gate

- All 17 ROS packages built successfully with `--symlink-install`.
- `colcon test` completed for all 17 packages; the generated xUnit/CTest records
  contain 81 tests with 0 errors, 0 failures, and 0 skipped. The backend's
  aggregated runner separately reported 283 tests OK and the visual frontend
  reported 13 tests OK.
- The observer's five C++ unit tests pass, including unknown-to-static
  confirmation, confirmed-free contradiction, occupied recovery, invalid range,
  and the existing TemporalVoxelFilter contract.
- Python style/bytecode, YAML, XML, and Git whitespace checks pass. Generated
  Python cache files are excluded from the package install.
- Launching with no overrides logs that the observer is disabled and creates no
  LiDAR subscription; the bounded launch check left no observer process behind.
