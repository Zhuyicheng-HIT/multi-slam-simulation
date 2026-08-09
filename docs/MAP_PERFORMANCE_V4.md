# Map Performance V4 audit and validation

## Scope and inherited guarantees

This branch starts at frozen Robustness V3.1 commit `4e31dc9`. It changes map
ingestion, diagnostics and simulated sensor mounting only. The five-source
unified estimator, visual reprojection factor, FRS, transaction integrity and
rollback thresholds are unchanged.

## Stage3 map audit

Stage3 commit `21e656d` is primarily a LiDAR map stability and visualization
upgrade, not a second probabilistic map backend. The reusable changes are:

- one-owner enforcement for `/livox/lidar` and `/livox/imu`, including a
  stable startup observation and runtime watchdog;
- one consistent simulated MID360 mount (`x=0.05 m`, `z=0.10 m`, pitch
  `+10 deg`) across SDF, bridge and FAST-LIO settings;
- one consistent D435i mount (`x=0.20 m`) across simulation and fusion;
- a close S-curve pole course which exposes low/high coverage and yaw drift;
- packet timing, publisher ownership, body-filter, registered-map bounds,
  overlap and centroid-jump diagnostics.

Those changes are adopted with two compatibility protections. Existing
Robustness V3.1 ownership checks were retained where stronger: two consecutive
clean samples are still required before FAST-LIO starts. The new pole course
keeps collision geometry for LiDAR but not camera-visible materials, preserving
the frozen RTAB scene and cross-session database contract. No Stage3
localization backend replaced the frozen five-source backend.

## Height-aware occlusion policy

The previous map only compared RGB-D with LiDAR geometry in the exact same
voxel. With a 0.10 m voxel, the voxel diagonal is about 0.173 m, smaller than
the default 0.18 m conflict threshold, so a behind-surface RGB-D sample would
normally land in a different voxel and escape the conflict check.

V4 builds a short-lived depth buffer from the latest registered LiDAR scan.
Both azimuth and elevation identify a ray. An RGB-D sample is rejected only if
a LiDAR return in the same angular cell is at least 0.40 m closer. This retains
the LiDAR-primary geometry rule and prevents a high surface from hiding an
independent low obstacle. The scan must have a compatible frame and be within
0.12 s of the RGB-D source timestamp; otherwise insertion falls back to the
existing conservative voxel rules and records a stale-scan diagnostic.

All thresholds are parameters. The defaults came from a fixed visible-surface
sweep over 0.25/0.5/1.0 degree bins and 0/1 neighboring cells. The chosen
0.5-degree/no-neighbor setting rejected 99.84% of injected behind-surface
ghosts while rejecting 0% of visible consistent points and 0% of legitimate
non-overlap supplements. Neighbor dilation was not selected because a dense
multi-surface stress case showed it could over-filter valid geometry.

## Python profile and implementation decision

`cProfile` identified per-point Python loops, repeated key construction and
individual voxel eviction as the dominant costs. NumPy grouping now batches
finite filtering, voxel keys, centroids, colors and exact-voxel conflict
checks. Eviction selects the oldest excess set once. The data model and map
ownership semantics did not move to C++ because the profiled Python hotspot was
removed without a new ABI or ROS boundary.

On the deterministic 28,800 LiDAR + 24,000 RGB-D input (five unselected runs):

| implementation | LiDAR P50 | RGB-D P50 | total P50 |
|---|---:|---:|---:|
| frozen V3.1 | 85.88 ms | 134.06 ms | 229.74 ms |
| V4 batching, filter off | 26.93 ms | 101.10 ms | 136.14 ms |

The total update is 40.74% faster. Frozen and V4 control summaries are equal
for accepted points, conflicts, supplements, consistent observations and every
source voxel count. This is the semantic A/B gate for retaining the batching.

With the filter enabled on the same visible-surface scene, the final five-run
median wall time was 104.82 ms versus 130.16 ms for the V4 control invocation;
4,992/5,000 ghosts were rejected, joint voxel count and color coverage were
unchanged, and all labeled legitimate points remained accepted. Supplementary
volume dropped from 7,977 to 4,422 voxels because behind-surface ghosts no
longer inflate it. Peak traced Python memory was 9.24 MB; process maximum RSS
was 57.7 MiB in that benchmark.

Reproduce with:

```bash
python3 tools/benchmark_shared_mapping_v4.py --runs 5 --output report.json
```

These deterministic proxies validate causality and semantics, not final field
accuracy. A real D435i/MID360 run must still inspect glass, moving objects,
calibration error and sparse LiDAR returns before changing the defaults.

## Headless runtime A/B

One matched small-rectangle run was made with mapping off and one with the
joint map enabled. Both brought up NativeLidarFactor, IMU, GNSS, optical flow,
paper visual reprojection, RTAB, D_V, unified odometry and ExternalNav. Both
failed at the first 90-degree turn with ArduPilot's independent simulation
check `Crash: Disarming: AngErr=55>30` while the FCU was using GPS. Mapping is
therefore not a necessary cause of this known WSL/Gazebo dynamics failure; the
runs are retained and not reported as completed missions.

| metric | map off | joint map |
|---|---:|---:|
| estimator CPU, median (whole-system capacity) | 2.315% | 2.345% |
| estimator RSS, median | 0.093 GiB | 0.095 GiB |
| mapping CPU, median | 0.009% | 1.419% |
| mapping CPU, P95 | 0.012% | 2.142% |
| mapping RSS, median | 0.041 GiB | 0.231 GiB |
| system CPU, median | 43.21% | 45.41% |
| system used RAM, median | 3.834 GiB | 3.892 GiB |
| Gazebo RTF, median | 0.473 | 0.423 |
| backend optimization errors | 0 | 0 |
| safety rollbacks near the simulated crash | 4 | 5 |

Before cleanup, the joint map contained 57,732 voxels from 587,216 LiDAR and
280,651 RGB-D input points. It had 8,381 jointly observed voxels, 17.92% LiDAR
color coverage, 10,954 supplementary RGB-D voxels, 454 occlusion rejections,
zero evictions, and no stale/frame-mismatched occlusion frames. Integration
latency was 10.69/19.03 ms LiDAR P50/P95 and 0.27/63.22 ms RGB-D P50/P95. The
estimator CPU was effectively unchanged; the remaining map cost mainly reduces
software-rendered simulation RTF.

The partial-route aligned trajectory was not used to claim a mapping accuracy
gain: joint-map ATE/RPE-translation/RPE-rotation RMSE were
0.073 m/0.024 m/0.898 deg, while the independent control run measured
0.195 m/0.030 m/0.670 deg. One stochastic, crash-truncated pair is evidence of
no obvious localization regression, not a statistically valid superiority
claim.
