# Stage 5 Temporal Map Baseline Report

Date: 2026-07-25

## Scope

This run validates the project-owned temporal voxel classifier and its runtime
evidence on the simple fixed-route world. It is a static-scene baseline, not a
dynamic-object removal acceptance result. FAST-LIO consumed the filtered sensor
chain:

```text
/sim/mid360/points_raw -> body filter -> fault injector
    -> /sensors/lidar/points -> FAST-LIO -> /cloud_registered
    -> temporal voxel classifier
```

The classifier publishes `/lidar/static_cloud`, `/lidar/dynamic_cloud`, and
`/lidar/uncertain_cloud`. Only static points enter the project-owned
`/lio/local_map`; this does not modify FAST-LIO's internal map.

## Fixed-route result

Run directory:
`logs/uf_stage2_uf_stage5_temporal_map_baseline_v1/`

| Metric | Result |
|---|---:|
| LIO diagnostic samples | 96 |
| matched points, median | 349.5 |
| point-to-plane residual P95, median | 0.0479 m |
| dynamic ratio, median | 0.0114 |
| uncertain ratio, median | 0.0318 |
| feature repeatability, median | 0.8698 |
| map quality, median | 0.6694 |
| unified ATE RMSE | 0.0635 m |
| unified maximum error | 0.1762 m |
| unified rotation RPE | 0.3776 deg |
| ExternalNav rate | 7.70 Hz |
| real-time factor, median | 0.9993 |

The raw FAST-LIO evaluator passed with position RMSE `0.0424 m`, yaw RMSE
`0.0975 deg`, zero timestamp regressions, median voxel overlap `0.4796`, and
centroid-jump P95 `3.553 m`.

## Interpretation

The runtime chain and outputs are healthy, but the classifier is not yet a
validated moving-object detector. In the static route, dynamic-ratio P90 was
`0.259` and repeatability P10 was `0.654`. These excursions occur while the
vehicle changes view or enters newly observed geometry, so the current
support-only classifier can label view novelty as dynamic.

The LiDAR reliability score remains the paper Eq. 19 combination of Hessian,
normal covariance, axial observability, and matched-point support. Residual,
spatial coverage, dynamic ratio, uncertainty, repeatability, and map quality
are exported as clearly named extension evidence; they do not silently change
the paper weights.

## Next gate

1. Add a reproducible moving-cluster or moving-object sequence.
2. Compare classifier enabled/disabled using repeatability, false dynamic
   ratio in static regions, and static-map contamination.
3. Add motion compensation or visibility reasoning if view novelty remains a
   dominant false-positive source.
4. Admit only validated static keyframes to the future relocalization map.

