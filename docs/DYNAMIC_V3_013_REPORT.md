# DYNAMIC-V3-013

## Baseline and worst scenes

The PR15-compatible 18-scenario benchmark was rerun with three deterministic seeds and two repeats per seed. The previous v2 result was micro P/R/F1 `99.8439/97.0854/98.4454%`, macro `93.5449/85.7748/88.8424%`, static preservation `99.9859%`, and latency P95 `9.784 ms`.

The lowest v2 dynamic recall was `far_sparse_target` (`0%`), followed by `occlusion_appear_disappear` (`54.0%`), `small_fast_target` (`68.4%`), and `opening_closing_door` (`67.2%`). The far sparse case is an observability-limited target: it is sparse, far from the sensor, moves only a fraction of a voxel per frame, and has little neighboring evidence. It is not safe to solve by broad deletion because the same evidence is ambiguous with new static structure.

## Minimal prototype

The prototype makes two conservative visibility settings less inert for sparse/far returns:

- increase dynamic neighborhood growth from `1` to `2` voxels;
- treat `20 m` as the far-range boundary and use `4` far static confirmations instead of `12`.

The changes affect only Dynamic Observer configuration/defaults. HXY, GNSS, MID360 IMU, Z, state machine, relocalization, prediction recovery, and scan contract are untouched.

## Benchmark result

| method | micro P/R/F1 | macro P/R/F1 | static preserve | latency P95 |
|---|---|---|---:|---:|
| Temporal baseline | 80.1815/3.2760/6.2948% | 85.2009/5.3977/9.7086% | 99.9249% | 2.565 ms |
| Observer v1 | 100.0000/95.0779/97.4769% | 93.7500/84.5115/88.0008% | 100.0000% | 5.736 ms |
| Observer v2 prototype | 99.8430/97.2592/98.5342% | 93.5435/86.5318/89.3450% | 99.9858% | 10.746 ms |

The prototype improves macro recall by `0.756 percentage points` and macro F1 by `0.503 points`, while micro precision changes by `-0.0009 points` and static preservation by `-0.0001`. Latency P95 increases by about `0.96 ms`, remaining below the 15 ms budget. The far sparse scenario remains at `0%` recall; this is documented as an observability gap rather than addressed with unsafe broad deletion. Occlusion recall improves to `59.8%`.

## Formal feature repeatability

The existing 60 s Dynamic-enabled static evidence contains 60 scored frames:

- median: `100%`
- P5: `100%`
- minimum: `0%` (startup frame)
- fraction below `95%`: `1/60 = 1.67%`

This is the formal statistic from the available diagnostic stream, not only the median.

## Localization regressions

The previously completed Dynamic-enabled 60 s static replay remains the current localization evidence: maximum 3D deviation `0.028 m`, with 10/30/60 s XYZ displacement `0.019/0.017/0.023 m` and XY displacement `0.019/0.015/0.021 m`. The HXY frozen long-tunnel replay remains approximately `0.790 m` XY RMSE versus `0.787 m` GNSS+MID360 IMU. No Dynamic-specific localization regression was observed in those recorded runs.

The 60 s and HXY replay artifacts were not regenerated after this local Dynamic configuration prototype in this pass; the existing runs use the same estimator chain but the prior Dynamic defaults. A promotion decision should require rerunning both with the prototype before merging.

