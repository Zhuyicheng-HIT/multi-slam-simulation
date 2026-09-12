# DYNAMIC-V3-013

## 基线与困难场景

PR15-compatible 的 18-scenario benchmark 以 3 个 deterministic seed、每个 2 次 repeat 重跑。旧 v2 的 micro P/R/F1 为 `99.8439/97.0854/98.4454%`，macro 为 `93.5449/85.7748/88.8424%`，static preservation `99.9859%`，latency P95 `9.784 ms`。

v2 最低 dynamic recall 是 `far_sparse_target`（`0%`），其次为 `occlusion_appear_disappear`（`54.0%`）、`small_fast_target`（`68.4%`）和 `opening_closing_door`（`67.2%`）。Far sparse 是 observability-limited target：稀疏、距离远、每帧移动不足一个 voxel，邻域证据很少，不能用 broad deletion 安全解决。

## 最小 prototype

针对 sparse/far return 仅放宽两项 visibility 设置：dynamic neighborhood growth 从 `1` 增至 `2` voxel；将 `20 m` 作为 far-range boundary，并把 far static confirmation 从 `12` 改为 `4`。只影响 Dynamic Observer configuration/default；HXY、GNSS、MID360 IMU、Z、state machine、relocalization、prediction recovery 与 scan contract 均不变。

## Benchmark 结果

| method | micro P/R/F1 | macro P/R/F1 | static preserve | latency P95 |
|---|---|---|---:|---:|
| Temporal baseline | 80.1815/3.2760/6.2948% | 85.2009/5.3977/9.7086% | 99.9249% | 2.565 ms |
| Observer v1 | 100.0000/95.0779/97.4769% | 93.7500/84.5115/88.0008% | 100.0000% | 5.736 ms |
| Observer v2 prototype | 99.8430/97.2592/98.5342% | 93.5435/86.5318/89.3450% | 99.9858% | 10.746 ms |

Prototype 将 macro recall 提高 `0.756` 个百分点、macro F1 提高 `0.503`，micro precision 变化 `-0.0009`，static preservation 变化 `-0.0001`；latency P95 增加约 `0.96 ms`，仍低于 15 ms budget。Far sparse 仍为 `0%` recall，作为 observability gap 记录；occlusion recall 提高至 `59.8%`。

## Feature repeatability

现有 60 s Dynamic-enabled static evidence 有 60 个 scored frame：median `100%`、P5 `100%`、minimum `0%`（startup frame）、低于 95% 的比例 `1/60 = 1.67%`。

## Localization 回归

已完成的 Dynamic-enabled 60 s static replay 仍是当前证据：最大 3D deviation `0.028 m`，10/30/60 s XYZ displacement `0.019/0.017/0.023 m`，XY displacement `0.019/0.015/0.021 m`。HXY frozen long-tunnel replay 约 `0.790 m` XY RMSE（GNSS+MID360 IMU 为 `0.787 m`）。未观察到 Dynamic-specific localization regression。Prototype 后尚未重新生成这些 artifacts，promotion 前应重跑。
