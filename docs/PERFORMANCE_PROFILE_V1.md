# Ultra-Fusion V1 performance profile

## Scope and controls

The frozen reference is `feat/ultrafusion-visual-tight-coupling-v1` at
`d76543e9c8f80dcaecbcbe4d898811a420978094`.  No commit on that branch was
changed.  The six production baseline runs used the same world, sensor rates,
0.065 s association gate, balanced visual cadence, D_V/FRS, integrity checks,
rollback policy and map parameters.  Raw run values are in
`PERFORMANCE_BASELINE_V1.csv`.

The profiler is opt-in (`performance_profiling_enabled:=false` by default),
uses bounded deques, wall-monotonic nanosecond clocks, and reports P50/P90/P95/
maximum.  The production baseline was collected with profiling off.  Run r31
then enabled profiling to attribute cost; its solver median was 52.741 ms and
RTF 0.436, close to the representative V1 operating point.

## Three primary estimator bottlenecks

| Rank | V1 stage | P50 (ms) | P95 (ms) | Maximum (ms) | Evidence |
|---:|---|---:|---:|---:|---|
| 1 | complete nonlinear optimize | 52.867 | 77.503 | 164.566 | r31 |
| 1a | graph linearization/assembly | 7.764 | 23.195 | 67.507 | r31 |
| 1b | marginalization | 7.537 | 14.358 | 26.082 | r31 |
| 2 | transactional snapshot | 3.979 | 7.817 | 17.356 | r31 |
| 3 | visual reprojection factor | 1.821 | 7.914 | 22.174 | r31 |

The dense block-diagonal Jacobian transform inside the marginal prior, full
`deepcopy` of immutable factor payloads, and the per-feature Python
reprojection loop were the actionable causes.  Linear solve itself was not the
bottleneck (0.310 ms P50, 0.533 ms P95).

## Source-aware map baseline

The r36 joint-map control showed a second, asynchronous hot path: LiDAR voxel
integration 12.511/21.663 ms P50/P95, RGB-D voxel integration
70.757/274.713 ms, and full-map publication 16.009/305.979 ms.  These costs do
not run in the estimator callback, but they compete for the same CPU and can
lower simulation RTF.

## Baseline outcome

Across the three rectangle runs the median solver time was 51.973 ms and median
RTF 0.428570.  Across the three S-curve runs the values were 59.794 ms and
0.463435.  Across all six runs the median solver time was 58.847 ms and RTF
0.460813.  All six had zero optimization errors and zero rollbacks.

The historical S-curve wrapper stopped the recorder after the route and did not
retain raw/tracked/window counters.  Accepted/quality/time-rejection totals
were retained and are included in the CSV; missing window values are left
blank rather than reconstructed.
