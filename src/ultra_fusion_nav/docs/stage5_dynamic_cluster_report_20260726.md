# Stage 5 Moving-Cluster Validation Report

Date: 2026-07-26

## Scope

This iteration adds a reproducible moving LiDAR cluster before FAST-LIO and
uses it to test the temporal map evidence, LiDAR reliability score, scheduler,
and online backend on the fixed route in `simple_apm_rgbd_mid360`.

The injected cuboid preserves every original `PointCloud2` record and appends
1200 points. Its center moves at 0.6 m/s using source-message time. The fault
is injected on `/sensors/lidar/points`, so FAST-LIO receives the degraded
cloud. Gazebo truth remains evaluation-only.

## Retained implementation

- `dynamic_cluster` fault model with deterministic point count and motion.
- Startup fault scheduling, avoiding unreliable mid-flight ROS parameter
  service calls on the busy point-cloud node.
- Timeline expectations that fail the run unless the requested modality and
  fault type are observed as active.
- Paper equation (19) remains available as `paper_score_eq19`.
- The project `D_L` adds a separately named map-protection extension using
  residual P95, spatial coverage, dynamic and uncertain ratios, feature
  repeatability, and map quality. The retained conservative combination is
  `0.70 * paper_score_eq19 + 0.30 * extension_score_normalized`.

## Experiment matrix

| Run | Fault validity | Window | ATE RMSE | Translation RPE | Result |
|---|---|---:|---:|---:|---|
| `uf_stage5_dynamic_cluster_v1` | valid, runtime parameter service | 19.34 s | 0.0818 m | 0.0580 m | Established the scoring gap |
| `uf_stage5_dynamic_cluster_v2` | invalid, zero active samples | none | 0.2565 m | 0.1152 m | Excluded from algorithm comparison |
| `uf_stage5_dynamic_cluster_v3` | valid, startup scheduled | 14.93 s | 0.0782 m | 0.0442 m | Fault occurred too early for v1 alignment |
| `uf_stage5_dynamic_cluster_v4` | valid, startup scheduled | 14.92 s | 0.1039 m | 0.0726 m | Route-aligned retained evidence run |

Run v2 timed out while calling `ros2 param set`; its timeline contained
`active_fault_samples=0`. The new expectation gate would reject this case
through `timeline_status`, even if the trajectory recorder completes.

## Route-aligned v4 evidence

The fault was active from timeline 51.62 s to 66.54 s with 116 active fault
status samples. Simulation median real-time factor was 0.9987 and LiDAR rate
was 7.58 Hz.

| Metric | Before fault | During fault | After fault |
|---|---:|---:|---:|
| dynamic ratio, median | 0.0056 | 0.0729 | 0.0145 |
| uncertain ratio, median | 0.0213 | 0.0470 | 0.0155 |
| feature repeatability, median | 0.9724 | 0.7985 | 0.9099 |
| map quality, median | 0.7094 | 0.4993 | 0.6739 |
| residual P95, median | 0.0467 m | 0.0604 m | 0.0510 m |
| `paper_score_eq19`, median | 0.6182 | 0.5760 | 0.6033 |
| extension score, median | 0.5389 | 0.9365 | 0.6949 |
| final `D_L`, median | 0.6278 | 0.6614 | 0.6046 |
| scheduler LiDAR weight, median | 0.4197 | 0.3345 | 0.3748 |
| LiDAR disabled samples | 308/507 | 95/149 | 180/335 |

The extension corrects the direction of the response: the paper-only score
decreases during this fault because the artificial cluster can improve the
Hessian while corrupting temporal consistency. The final score rises instead.
The increase is modest, and trajectory accuracy is not improved in the
route-aligned run.

## Rejected iteration

A second classifier treated paired appeared/disappeared voxels as local
occupancy migration. The clean-route run
`uf_stage5_motion_pair_baseline_v2` produced ATE 0.0520 m, but its reliability
behavior regressed:

- static-scene dynamic-ratio median increased to about 0.077;
- 43 of 97 LiDAR scores exceeded 0.8;
- the scheduler disabled LiDAR for 783 of 998 samples.

View-boundary motion creates the same appeared/disappeared pattern without
ray-based visibility evidence. The classifier and an associated hard score
gate were therefore reverted. The run remains in local logs as negative
evidence.

## Current boundary

This milestone validates fault injection and measurable score response, not
complete dynamic-object removal. The temporal classifier can still label new
view geometry as dynamic. `/lidar/static_cloud` protects only the project-owned
`/lio/local_map`; it does not remove dynamic points from FAST-LIO's internal
map. The online backend also keeps a 0.05 LIO anchor floor when the scheduler
disables LiDAR, so this experiment cannot yet prove that disabling the factor
protects the fused trajectory.

## Next gate

1. Add ray/range-image visibility reasoning or a tracked-cluster motion model
   before strengthening the dynamic extension.
2. Require a clean-route false-positive gate and a moving-cluster true-positive
   gate in the same evaluation script.
3. Make scheduler LiDAR disablement bypass the external LIO pose factor while
   retaining IMU/GNSS/flow propagation.
4. Admit only validated static keyframes to the relocalization map.
