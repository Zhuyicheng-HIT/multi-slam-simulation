# HXY-KERNEL-004：旋转 LiDAR 子空间 C++ kernel

日期：2026-08-25

## 结果

任意旋转的 translation-subspace transform 现在已在 C++ point-plane normal kernel 中运行。其代数结果与 Python prototype 等价，包括组合 axis-information-scale 路径。active window 仍会更新每个原始历史 LiDAR factor，并有意保持 marginal prior 不变。

C++ migration 消除了 Python solver penalty 和仅 replay 的 latest-only 丢帧，但没有提升 C 的估计精度。C 明显优于 A，且明显劣于 B。新的 prior diagnostics 不支持下一步修改 marginalization：在单一 weak-mode drift 期间，live prior position information 只有很小一部分位于当前 weak direction，而 current-window LiDAR recovery factors 仍持续进入 solver。

## 冻结输入与 replay contract

所有运行均使用 HXY-DIAG-002 frozen bag：

`/home/ld666/projects/hxy-diag-002/frozen_bag`

| 文件 | SHA256 |
|---|---|
| `metadata.yaml` | `d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b` |
| `replay_bag_0.db3.zstd` | `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56` |

Truth 仅由 offline accuracy recorder 使用。A 和 C 对本次 replay 禁用 estimator 的 latest-only shortcut；所有运行保留 queue 和 QoS depth 1024、rate 0.5、regenerated LiDAR scheduler input、两个 executor threads 与一个 numeric thread。C 不调参地保留 HXY-SUBSPACE-003 settings：enter threshold `0.15`、exit threshold `0.25`、weak information scale `0.001`。

| 运行 | 模式 | 输出 |
|---|---|---|
| A | stable `hybrid/adaptive` | `/home/ld666/projects/hxy-diag-002/replay_A_kernel_fair` |
| B | PR17 `paper_eq19/paper_eq15` | `/home/ld666/projects/hxy-diag-002/replay_B_kernel_004` |
| C | C++ rotated subspace `hybrid/adaptive` | `/home/ld666/projects/hxy-diag-002/replay_C_kernel_004_fair` |

## Kernel math 与等价性

kernel 首先累加 robust point-plane 6-DoF normal。对 blocks `H_tt`、`H_tr` 和 `H_rr`，计算与 Python prototype 相同的 conditional quantities：

`M = H_tr pinv(H_rr)`、`S = H_tt - M H_tr^T`，以及 `g_c = g_t - M g_r`。

对于 information-scale matrix `D = U diag(s_i) U^T` 及其 PSD root `P`，kernel 写入：

`H'_tt = P S P + M H_rr M^T`

`g'_t = D g_c + M g_r`。

rotation block 和 translation-rotation coupling 保持不变。若启用现有 diagonal axis information scale，则会在该 transform 之前应用于 raw translation Jacobian，与 Python 完全一致。这样支持 1 至 3 个任意旋转的 weak modes，并将一次观测保持为一个 factor。

Unit tests 在同时启用和不启用 axis scaling 时，对比 C++ 与 Python 的 Hessian、gradient 和 cost，absolute/relative tolerance 为 `1e-10`。全部 310 个 backend tests 通过。在 replay trace 中，461 个 weak-mode observations 的 normalized information retention 平均为 `0.001000000000000005`，847 个 strong-mode observations 平均为 `0.9999999999999999`；极值与要求值的差异仅约 `3e-15`。

## A/B/C 精度

公平的 common scoring interval 为 `30.723 <= t <= 76.993 s`。

| 指标 | A stable | B PR17 | C C++ subspace |
|---|---:|---:|---:|
| Matched samples | 462 | 462 | 454 |
| 3D RMSE (m) | 17.218 | 0.451 | 6.565 |
| XY RMSE (m) | 17.218 | 0.449 | 6.565 |
| Z RMSE (m) | 0.112 | 0.043 | 0.024 |
| 3D P95 (m) | 46.819 | 1.086 | 14.705 |
| 3D max (m) | 60.068 | 1.239 | 26.090 |
| Last scored error (m) | 60.068 | 1.221 | 26.090 |

使用精确 source-stamp matches，将误差投影到各运行瞬时 LiDAR eigenbasis：

| Projection RMSE (m) | A | B | C |
|---|---:|---:|---:|
| Weakest direction | 12.515 | 0.376 | 4.971 |
| Strongest direction | 5.787 | 0.126 | 1.621 |

因此 C 仍明显优于 A，但 C++ migration 本身没有比 Python C 提升精度（HXY-SUBSPACE-003 中 XY RMSE 为 `6.457 m`，此处为 `6.565 m`）。B 更好是因为它不接纳任何 LiDAR factor，而不是因为保留了有用的 strong-direction LiDAR information。

## 接纳、时序与关键事件

| 数量 | A | B | C |
|---|---:|---:|---:|
| Native received | 678 | 678 | 678 |
| Trace records | 669 | 672 | 669 |
| LiDAR solver admitted | 452 | 0 | 436 |
| Optimized states committed | 466 | 672 | 447 |
| Integrity rollbacks | 203 | 0 | 222 |
| Prediction recovery factors | 0 | 0 | 107 |
| Queue overflow / supersede / latest skip | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

A/C 中有 9 个 received factors 属于 startup 或 non-transaction inputs；所有到达 transaction worker 的 factor 都按 source order 处理。因此 latest-only loss 已消除。A 与 C 仍有大量 integrity rejects，这是 estimator decisions，而非丢失输入。

实际 sensor accounting：

| Source | A received / attempted / solver factors | B | C |
|---|---:|---:|---:|
| LiDAR | 678 / 669 / 452 | 678 / 672 / 0 | 678 / 669 / 436 |
| IMU | 7513 / 668 / 668 | 7513 / 671 / 671 | 7513 / 668 / 668 |
| GNSS | 375 / 363 / 265 | 375 / 364 / 364 | 375 / 363 / 257 |
| Optical flow | 951 / 668 / 0 | 951 / 671 / 0 | 951 / 668 / 0 |
| Visual/RGB-D | 57 / 57 / 0 | 57 / 57 / 0 | 57 / 56 / 0 |

Optical flow 几乎在所有 attempts 中都被 scheduler-disabled。Visual/RGB-D 已接收并评分，但未通过 visual quality/FRS gate，因此 GNSS 是唯一 active external horizontal constraint。对于 estimator contribution，应看 factor counts，而不是 topic receipt counts。

C 的 first degeneracy detection、first projector attenuation 以及首次由 solver 使用 attenuated LiDAR factor，均发生在 transaction 1、native scan 131、`t=30.723 s`。first prediction hard reject/non-admission 发生在 transaction 331、scan 461、`t=63.723 s`；first recovery-floor factor 发生在 transaction 333、scan 463、`t=63.921 s`。

| Timing from transaction trace | Python C | C++ C |
|---|---:|---:|
| Solver mean (ms) | 50.651 | 15.049 |
| Solver median (ms) | 39.848 | 12.486 |
| Solver P95 (ms) | 104.412 | 36.704 |
| Solver max (ms) | 181.800 | 56.870 |

C++ kernel 将 mean solver time 降低约 70%。End-to-end callback timing 仍非 real-time-normal：C 的 `pre_state` mean/P95 为 `189.9/797.7 ms`，接近 A 的 `171.5/724.2 ms`。这部分工作位于 projector 之外，解释了剩余 wall-time backlog。由于 frozen replay 禁用 latest-only，它已不再改变所处理的 input frames。

## Marginal-prior diagnosis

首个 marginal prior 在 transaction 9、scan 139、`t=31.515 s` 形成并参与 optimization。Diagnostics 递归保留 source factor counts 和 pre-Schur LiDAR translation trace，并将 live prior 的 position diagonal blocks 投影到当前 weak projector；它们不会改变 Schur complement 或 prior Hessian。

在 transaction 9，prior 包含 1 个历史 LiDAR factor。到最后一个 one-weak-mode transaction 时，包含 436 个历史 LiDAR factors。在 413 个 one-weak-mode prior samples 中，当前 weak-direction fraction 的 median 为 `0.00296`、P95 为 `0.00555`。C 的 horizontal-error crossings 为 0.05、0.2、1、5、10 m 时，该 fraction 分别为 `0.00349`、`0.00287`、`0.00234`、`0.00150` 和 `0.00170`。

这不是 nonlinear Schur prior 的精确 per-source decomposition：marginalization 后，cross-state blocks 和 source interactions 无法唯一分配。但证据充分表明，在观测到的 drift 期间，prior 并非剩余 weak-direction information 的主要来源。prior 确实包含累积的 LiDAR history；因此在 current-window behavior 修正后，marginalization 可能成为下一瓶颈，但这次 replay 并不支持立即干预它。

## 决定

在修改 marginalization 之前继续研究 current-window behavior。下一项 experiment 应隔离 attenuated LiDAR、prediction recovery 与 GNSS scheduler interaction 为何仍产生 weak-direction feedback 和 222 次 integrity rollbacks。Marginal-prior math 保持冻结。

`DO_NOT_PROMOTE`

未修改 Z-axis、state-machine、relocalization、marginalization math 或 sensor weight。未运行 Gazebo，也未执行 push。
