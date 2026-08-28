# DYNAMIC-V3-VALIDATE-014

## HXY frozen replay

The current V3 Dynamic branch was replayed against the decompressed HXY-DIAG-002 frozen bag with the existing C++ HXY kernel and native-factor contract. The replay processed all `678` native factors, `672` committed states, zero native queue overflow/discard, zero IMU pair timeout, zero worker error, and zero optimization error. Causal XY RMSE was `0.79306 m`, compared with the established approximately `0.790 m` result. This is not a meaningful regression and remains close to the GNSS+MID360 IMU floor (`0.787 m`).

## Static replay status

Two V3 static attempts were made. A full Dynamic + visual run reached FAST-LIO/backend and then timed out waiting for `/vision/feature_tracks`; a second run with visual frontend/RTAB disabled reached the estimator chain but did not satisfy the existing PR6 visual readiness contract before manual termination. No position drift result from a complete 60 s V3 run is claimed here. The prior complete Dynamic static evidence remains maximum 3D deviation `0.028 m`; a clean 60 s run with the V3 defaults is still required before promotion.

## Feature repeatability zero frame

The unique zero repeatability sample is at LiDAR stamp `17.2 s` (recording elapsed `1.36 s`, wall arrival `29.2 s`). It has `2828` input points and `400` matched points, but `uncertain_ratio=1.0`, `map_quality=0.0`, and `dynamic_ratio=0.0`. Therefore it is startup/map-warmup uncertainty, not Dynamic deletion or a Dynamic false positive. Formal static evidence remains median/P5 `100%`, minimum `0%`, and `<95%` ratio `1/60 = 1.67%`.

## Difficult scenes

- `far_sparse_target`: P/R/F1 `0/0/0`, dynamic-unknown ratio `54.8%`. The target is at 18 m, sparse, and moves below useful voxel/neighborhood evidence. This is a LiDAR geometry/visibility observability limit; broad deletion would trade directly against static preservation.
- `occlusion_appear_disappear`: Recall `59.8%`, precision `100%`, contamination `40.2%`. Reappearance is often unknown rather than falsely dynamic; the observer remains conservative around occlusion and cannot infer motion when the target is not observed.
- `small_fast_target`: Recall `68.4%`, precision `99.8%`, dynamic-unknown ratio `30.7%`. Small targets cross voxels quickly and leave insufficient temporal occupancy history; precision remains high.
- `opening_closing_door`: Recall `67.6%`, precision `97.9%`, contamination `32.3%`. Articulated surface motion is partially protected as static hinge/frame structure, so conservative admission leaves some dynamic returns in the map.

## Decision

V3 is not yet a drop-in replacement for V2: it improves benchmark macro recall (`85.77% -> 86.53%`) without materially changing precision or static preservation, but the far-sparse case remains unobservable and the required complete V3 60 s static replay is still outstanding. Keep V2 as the release baseline; next run the full V3 static replay with the visual readiness contract fixed, then evaluate a scene-specific far-sparse observability metric rather than expanding global deletion.

