# HXY-PRIOR-007：marginal-prior 历史 LiDAR 审计

两次 replay 使用 HXY-INTERACTION-006 的冻结 bag 与设置，包括当前 weak-subspace cap。唯一变量是 opt-in 的 `marginal_prior_suppress_historical_lidar_weak` diagnostic switch。

| replay | XY RMSE | 3D RMSE | endpoint norm | commits | input loss |
|---|---:|---:|---:|---:|---:|
| A baseline | 0.79014 m | 0.79023 m | 2.050 m | 672 | 0 |
| B 在 marginalization 时抑制历史 LiDAR weak mode | 0.78373 m | 0.78396 m | 2.054 m | 672 | 0 |

微小改善处于 replay executor/interleaving 变化范围内，不能证明存在主导性的 prior failure。两次运行均处理 native sequence 131 至 799，没有 queue overflow、supersede、latest-only skip 或 rollback。

## 归因证据

首个包含历史 LiDAR source 的 marginal prior 是 transaction 9（`t=31.515 s`）。此时 source bookkeeping 报告 LiDAR pre-Schur translation trace `383331` 和衰减后的 weak trace `961.84`。后者是 formation-time source diagnostic，并非 nonlinear Schur prior 中可独立识别的组成部分。

在 664 个 prior sample 中，投影到当前 weak-direction 的 position information fraction 中位数约为 `0.327`，P95 为 `1.0`；该 fraction 是 prior 总 position block projection，包含 IMU、previous prior 及 cross-state Schur effect，无法单独归因于 LiDAR。Recursive source count 证实 prior 累积了 LiDAR history，但 Schur elimination 后这些 history 不可分离。

Suppression replay 在每次新 marginalization 输入中将 historical LiDAR weak mode 设为零，同时保持 strong LiDAR、GNSS、IMU、current-window factor、one-observation-one-factor 语义和其他 prior 使用不变。它仅略微改变累积的衰减 weak trace（运行后段约从 `106026` 变为 `105192`），XY RMSE 变化约 `6 mm`，远小于原始 weak-direction competition effect，对 endpoint 没有实质影响。

## 结论

`marginal_prior` 不是剩余约 0.79 m 水平误差的主要来源。不会推广或修改任何 marginalization mathematics。Prior 确实保留历史 LiDAR，未来可能成为限制因素，但移除其 weak contribution 的实测影响很小。

下一个主要嫌疑是 absolute/GNSS 与 IMU replay residual，以及当前窗口 cap 之后的 timing/trajectory alignment。下一次 replay 应在不改变 HXY threshold 或增加 GNSS weight 的前提下，隔离 GNSS+IMU residual 演化和不依赖 prior 的 state error。
