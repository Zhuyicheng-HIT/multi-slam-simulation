# HXY-DIAG-002：LiDAR 水平退化诊断与 replay

日期：2026-08-25

## 结果

诊断 trace 足以定位首个因果差异，但不足以宣称 PR17 是期望的最终方案。在此 bag 上，PR17 的 `paper_eq19`/`paper_eq15` 路径禁用了全部 LiDAR factor，主要依靠 GNSS 和 IMU，因而避开 stable branch 的灾难性 Y 漂移，却没有保留 LiDAR 的 strong subspaces。这些结果支持实现并测试 subspace C，不能证明 C 已经验证。

未改变 estimator decision、state machine、Z-axis policy、relocalization、factor math。patch 仅在每次 transaction 后扩展 performance trace，并让现有 replay wrapper 重新生成 A/B 的 LiDAR score/scheduler 路径。

## 冻结输入

冻结副本：`/home/ld666/projects/hxy-diag-002/frozen_bag`

来源：`/home/ld666/multi-slam-simulation/logs/large_scene_tunnel_static_paper_backend_truth_fixed_20260821/replay_bag`，capture commit `a37539aa34222578f3d1a186f9e616c3da7b7cb0`，profile `tunnel_static`，world `large_indoor_tunnel_apm_rgbd_mid360`，Gazebo world `large_indoor_tunnel`。路线 span 70.0 m、lateral amplitude 1.0 m、vertical amplitude 0.35 m、speed 0.80 m/s；dynamic agents disabled；duration 75.159 s，共 101954 messages。

| 文件 | Bytes | SHA256 |
|---|---:|---|
| `metadata.yaml` | 18335 | `d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b` |
| `replay_bag_0.db3.zstd` | 126180529 | `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56` |

输入 contract 位于 `/home/ld666/projects/hxy-diag-002/bag_contract.json`。topic counts：native LiDAR 678、IMU 7513、GNSS fix 375、raw GNSS 325、optical flow 951、RGB-D direct/geometry 各 57、visual tracks 57、truth 752。Truth 只被 `external_nav_accuracy` 订阅，不是 estimator input。

## Replay contract

两次最终运行都从 offset zero 使用该 bag，rate 0.5、CycloneDDS、一个 numeric thread、两个 executor threads、axis handoff off、Z reanchor off、barometer fallback off、range facet off，QoS/worker depth 均为 1024。Recorded LiDAR score 和 scheduler messages 被排除，并依据 `/lio/diagnostics` 与 `/lio/odom` 重新生成；其他 score inputs 完全相同。

| Run | Algorithm reference | Diagnostic commit | LiDAR score/admission | Output |
|---|---|---|---|---|
| A | `c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981` | `cb05eb0` | `hybrid` / `adaptive` | `/home/ld666/projects/hxy-diag-002/replay_A_complete_c7c1adc` |
| B | `4587e479d5f02dbaaaff048c266fd873d124d109` | cherry-pick `8d6afa0` | `paper_eq19` / `paper_eq15` | `/home/ld666/projects/hxy-diag-002/replay_B_complete_pr17` |

两次均收到全部 678 个 native factors，无 DDS/worker queue supersession。A 的 estimator-internal latest-only check 因长 callback 跳过了 171 个已排队旧帧，B 跳过 0 个。这是当前实现的 closed-loop timing consequence，并非输入 bag 差异，因此以下 common-horizon 比较才是可辩护的 A/B score。

## Trace 证据

每条 `backend_cycle_trace.jsonl` 记录现在包含 transaction/native sequence IDs、Schur translation information、normalized eigenvalues、全部 eigenvectors、canonical weak direction、effective translation information/eigenpairs、prediction innovation/gate/recovery、effective factor weight、solver admission、active LiDAR factor indices/count/ages、marginalization、optimized state，以及已有的 GNSS、visual、flow、factor-count 和 solver profile diagnostics。

diagnostic-only effective matrix 为 `H_eff = w_eff * sqrt(D_axis) * H_schur * sqrt(D_axis)`。这些运行中 axis handoff 关闭，所以 `D_axis=I`。它在 solve 后由实际 active factor record 计算，不能影响 transaction。

## 核心指标

共同 source-stamp interval 为 `30.723 <= t <= 78.804 s`：

| 指标 | A stable | B PR17 |
|---|---:|---:|
| Matched samples | 471 | 478 |
| 3D RMSE (m) | 19.204 | 0.487 |
| XY RMSE (m) | 19.204 | 0.485 |
| Z RMSE (m) | 0.135 | 0.042 |
| 3D P95 (m) | 51.760 | 1.115 |
| 3D max (m) | 66.671 | 1.239 |
| Last-sample 3D error (m) | 66.671 | 1.144 |

B 的完整 74.085 s 输出为 3D/XY/Z RMSE `0.788/0.787/0.040 m`，3D P95/max `1.361/2.440 m`，endpoint `2.065 m`。A 在 48.171 s 后不再产生可评分输出，不能与 B 的完整 horizon 直接比较。

| 数量 | A | B |
|---|---:|---:|
| Native received / traced | 678 / 498 | 678 / 672 |
| Internal latest-only skipped | 171 | 0 |
| LiDAR solver admitted / trace rejected | 452 / 46 | 0 / 672 |
| Prediction hard rejects / recoveries | 12 / 0 | 165 / 0 |
| Optimized states / rollbacks | 466 / 32 | 672 / 0 |
| IMU received / factors | 7513 / 497 | 7513 / 671 |
| GNSS received / consumed / factors | 375 / 260 / 258 | 375 / 364 / 364 |
| Flow received / attempts / factors | 951 / 497 / 0 | 951 / 671 / 0 |
| RGB-D direct received / factors | 57 / 0 | 57 / 0 |
| Visual batches attempted / solver factors | 57 / 0 | 57 / 0 |

Flow 在尝试的 transaction 中被 scheduler-disabled；RGB-D/visual 虽接收并评分，却被 `vision_frs_gate_disabled`/quality gating 阻止形成 factor。因此本 replay 只有 GNSS 提供 active external horizontal constraint。A 随估计偏离而 downweight/reject 95 个 XY GNSS prefits，B 的 364 个 GNSS factors 均保持一致。

## 首次 divergence 与 weak direction

首个 decision divergence 是 transaction 1、scan 131、`t=30.723 s`：两次运行 Schur spectrum 均为 `[0.09029, 0.60937, 1.0]`，weak direction 为 `[-0.01638, 0.99986, -0.00182]`。A 以 weight 1.0 接纳 LiDAR factor；B 设置 weight 0、inflation 20，不接纳它。

首个清晰 state divergence 是 transaction 83、scan 213、`t=38.907 s`，A/B optimized XY 相差 0.084 m；这也是 A 的 offline horizontal error 首次超过 0.05 m。A 的误差在 `43.725 s` 超过 0.1 m、`49.731 s` 超过 0.2 m、`60.225 s` 超过 1.0 m，主要沿 Y。首次 prediction hard reject 直到 transaction 460 / scan 591 / `t=76.824 s`；首次 factor-disabled transaction 为 444（`75.009 s`），说明 prediction gating 反应过晚，recovery 从未激活。

weak eigenvector 会随时间旋转，并非固定 world frame：

| Time (s) | Scan | Weak direction | Normalized eigenvalues | A admitted |
|---:|---:|---|---|---|
| 30.723 | 131 | `[-0.016, 1.000, -0.002]` | `[0.090, 0.609, 1]` | yes |
| 43.131 | 255 | `[0.012, 1.000, -0.009]` | `[0.047, 0.819, 1]` | yes |
| 55.506 | 379 | `[0.007, 1.000, 0.010]` | `[0.041, 0.696, 1]` | yes |
| 68.013 | 504 | `[-0.127, 0.992, 0.007]` | `[0.036, 0.306, 1]` | yes |
| 75.934 | 583 | `[0.994, -0.109, -0.004]` | `[0.031, 0.716, 1]` | no |

接近 baseline observation `[0.992,-0.107,-0.072]` 的向量出现在 `75.934 s`，此时 A 已越过 1 m horizontal error；此前 Y drift 累积阶段 weak vector 几乎沿 world Y。eigenvector 符号任意，因果分析必须使用 time series，而非单个终端向量。

## Marginal prior 与 C 的准备度

两次运行在 transaction 9、scan 139、`t=31.515 s` 首次填满 8-state window 并形成 marginal prior；它在 transaction 10、scan 140、`t=31.614 s` 首次参与 optimizer，早于首个 5 cm trajectory divergence 7.293 s、早于 A 的 0.2 m truth-error crossing 18.117 s。A 的 admitted full-rank LiDAR factors 持续进入 prior；window 最多保留 8 个 active LiDAR factors，oldest age median 0.693 s、maximum 0.890 s。B 没有 active LiDAR factors。trace 证明了时间先后和信息路径，但不能把 marginal prior 精确分解为 per-source Schur blocks。

已有足够证据实现范围受限的 C prototype：每 scan 的 Schur matrix/eigenbasis 在 admission 前可用，历史 factor 保留 correspondence Jacobians/effective weights，且 replay 显示 weak direction 持续为 horizontal Y。尚不足以宣称 C 正确；C 必须保持相同 full-input replay contract，记录每个 eigenmode 的 applied scales（包括进入 marginalization 的部分），并与 A/B 在 common horizon 比较。还应增加 RGB-D 或 flow 实际形成 solver factors 的 bag，以检验超越 GNSS 的 cross-modal compensation。

## 验证

- 177 个 focused Python tests 通过（`native_lidar`、`online_backend`、`manifold_window`）。
- `uf_backend_fusion` 在 A 上成功重建；PR17 及依赖在独立 worktree 成功重建。
- 两次 final bag play 均以 zero exit code 结束，并通过 input-contract check。
- 未运行 Gazebo，也未执行 push。
