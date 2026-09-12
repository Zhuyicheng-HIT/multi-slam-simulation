# HXY-SUBSPACE-003：任意 weak-subspace LiDAR prototype

## 实现

C 通过 `lidar_subspace_enabled=true` 启用，并使用当前 native translation Schur eigendecomposition。normalized eigenvalue 低于 `0.15` 的 mode 视为 weak，active episode 的 hysteresis exit threshold 为 `0.25`。Weak mode 的 information scale 为 `0.001`，所有 strong mode 保持 `1.0`。投影器为 `P = U diag(sqrt(s_i)) U^T`，其中 `U` 是当前 3-D translation eigenbasis。每个 raw point-plane factor 的 translation-conditioned Schur block 重加权为 `S' = P S P`，conditional translation gradient 乘以 `P^2`；rotation block 与 translation-rotation coupling 保留。该方法支持任意旋转的 weak direction，而不局限于 XYZ 轴。

当前窗口中每个 active raw `lidar_point_plane` factor 都接收同一 episode projector，包括尚未 marginalize 的历史 factor。`marginal_prior` 从不修改。每条 observation 仍对应一个 factor；GNSS、RGB-D、optical-flow、Z-axis、state-machine 和 relocalization weight 均未改变。

现有 C++ kernel 仅支持对角 XYZ scaling，因此非平凡旋转 projector 使用现有 Python per-factor normal path。该 prototype 数学定义清晰但不具备 real-time 效率；C++ rotated-normal kernel 属于后续优化，不在本实验范围。

## Replay 与参考

冻结输入和 SHA256 与 HXY-DIAG-002 相同。A、B 是该报告中的 formal complete-QoS replay。C 使用相同 bag、QoS depth 1024、worker queue 1024、重新生成的 LiDAR scheduler、rate 0.5，以及：

- algorithm base：stable `c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981`；
- score/admission：`hybrid` / `adaptive`；
- subspace：threshold `0.15`、exit `0.25`、weak scale `0.001`；
- output：`/home/ld666/projects/hxy-diag-002/replay_C_subspace_final`。

## A/B/C 精度

共同 source-stamp 区间为 `30.723 <= t <= 78.804 s`；由于 Python path 增加 callback latency，C 仅产生 453 个可评分 sample。

| 指标 | A stable | B PR17 | C subspace |
|---|---:|---:|---:|
| 3D RMSE (m) | 19.204 | 0.487 | 6.457 |
| XY RMSE (m) | 19.204 | 0.485 | 6.457 |
| Z RMSE (m) | 0.135 | 0.042 | 0.024 |
| 3D P95 (m) | 51.760 | 1.115 | 14.079 |
| 3D max (m) | 66.671 | 1.239 | 25.635 |
| Endpoint/last scored error (m) | 66.671 | 1.144 | 25.635 |

C 的完整输出在 46.071 s 结束，共 453 个匹配 sample。C 明显优于 A，但未接近 B 的精度。

同一区间内将误差投影到瞬时 LiDAR basis：

| Projection RMSE | A | B | C |
|---|---:|---:|---:|
| 最弱 translation direction (m) | 12.655 | 0.372 | 5.069 |
| 最强 translation eigenvector (m) | 10.431 | 0.231 | 1.765 |

C 比 A 更好地保留 strong-direction information，但 weak-direction residual 仍占主导。B 禁用所有 LiDAR factor 并避开不稳定 feedback loop，因而更好；这不证明 B 保留了 strong LiDAR information。

## Admission 与 timing

| 数量 | A | B | C |
|---|---:|---:|---:|
| Trace transaction | 498 | 672 | 484 |
| LiDAR solver admitted | 452 | 0 | 436 |
| Native received | 678 | 678 | 678 |
| Internal latest-only skipped | 171 | 0 | 185 |
| Prediction hard reject | 12 | 165 | 128 |
| Recovery factor | 0 | 0 | 107 |
| Optimized state | 466 | 672 | 447 |
| Rollback | 32 | 0 | 37 |

C 的首次 hard reject 和首次 non-admission 为 transaction 331、scan 461、`t=63.723 s`。首次 marginalization 仍是 transaction 9（`31.515 s`），`marginal_prior` 首次使用为 transaction 10（`31.614 s`）。因此 prior 比 C 的最终 divergence 早约 32 s，但 C 未修改 prior，现有证据不能证明 prior 是主要剩余误差来源。C 更长的 callback time 还改变了 latest-only admission，必须先消除该影响才能提出确定因果结论。

## 决策

1. **C 对 A：** 在此 bag 上明显更好（3D RMSE 为 `6.46 m` 对 `19.20 m`），strong-direction error 更低且保留 LiDAR admission stream。
2. **C 对 B：** 与 B（`6.46 m` 对 `0.49 m`）仍有明显差距；C 保留 LiDAR strong information，而 B 完全不保留。C 目前不能替代 B。
3. **Marginal prior：** 在时间上先于 residual error，但本次运行无法将其与 Python-path timing/latest-only effect 分离。暂不修改 marginalization；应先增加 prior source/subspace accounting 和 rotated C++ normal kernel，再以匹配的 transaction horizon replay。

**DO_NOT_PROMOTE**

C 是有效的 diagnostic/projection prototype。在 timing 公平、记录 marginal-prior attribution，并在 RGB-D 或 optical flow 实际形成 solver factor 的 replay 上完成比较前，不应将其提升为下一算法 baseline。
