# Performance V2 冻结验证

> 历史 candidate-gate 报告。`PERFORMANCE_V2_STABILITY_CONVERGENCE.md` 中的 controlled full-online replay 与 per-cycle profiler 已取代以下 performance decision，并将 V2 冻结状态标记为 `SIM_ENV_CONTENDED`。

## 决策

本验证下 Performance V2 仍是 candidate，**未冻结**。Accuracy、visual-use、safety、mapping、build 和 test gate 通过，但最终 6-run set 未重现要求的 live solver median ≤45 ms。没有放宽 algorithm threshold，也没有丢弃结果来强行 freeze。

## Rectangle V1/V2 A/B

匹配的 5-run rectangle comparison 使用同一 world、route、sensor rate、balanced visual cadence、0.065 s association tolerance、D_V/FRS、ExternalNav 与 FAST-LIO configuration；optical-flow noise 固定 seed 29。

| Statistic | Frozen V1 | V2 candidate |
|---|---:|---:|
| translation RPE median | 0.039753 m | 0.030634 m |
| translation RPE mean | 0.040304 m | 0.031494 m |
| translation RPE std | 0.008858 m | 0.004174 m |
| translation RPE min / max | 0.025359 / 0.052759 m | 0.026573 / 0.038008 m |
| ATE median | 0.533603 m | 0.427050 m |
| solver median of run medians | 63.378 ms | 46.541 ms |
| visual accepted / quality-valid | 294 / 386 | 367 / 476 |
| time rejection | 9.84% | 10.29% |

此前的 rectangle warning 并非系统性；V2 translation RPE median 比 V1 低 22.9%，ATE median 低 20.0%。因此没有回滚 marginal-prior block transform、transaction snapshot optimization 或 visual vectorization。

## 剩余 visual timing rejection

最终 3 次 rectangle 和 3 次 S-curve run 中，1089 个 quality-valid candidate 有 782 个被 solver 接受（71.81%），112 个 time reject（10.28%）：105 个是 `state_tolerance_mismatch`（93.75%，其中 58 个缺左 state、47 个缺右 state），7 个超出 active window；queue overflow、duplicate submission、track rejection 均为 0。Rejected observation 常距最近真实 state 0.066–0.099 s，偶有 0.198 s LiDAR-state gap。实现继续只使用真实 causal state：0.065 s tolerance、source timestamp、D_V threshold 均不变，没有 future state、retimestamping 或 interpolation。

## 最终 accuracy 与 runtime

| Scenario | ATE (m) | translation RPE (m) | rotation RPE (deg) | solver median / P95 (ms) | RTF |
|---|---:|---:|---:|---:|---:|
| rectangle r151 | 0.154630 | 0.027468 | 0.120747 | 42.509 / 91.823 | 0.4566 |
| rectangle r153 | 0.636268 | 0.033044 | 0.126866 | 54.807 / 70.805 | 0.4731 |
| rectangle r154 | 0.459032 | 0.028227 | 0.115221 | 58.348 / 83.676 | 0.4167 |
| S-curve r151 | 0.687640 | 0.046254 | 0.112758 | 50.028 / 76.515 | 0.4419 |
| S-curve r152 | 0.538685 | 0.046189 | 0.112401 | 53.943 / 78.356 | 0.5294 |
| S-curve r153 | 0.525499 | 0.047406 | 0.115260 | 56.019 / 74.311 | 0.4432 |

6-run solver median 为 54.375 ms，P95 median 为 77.435 ms，未达 freeze threshold；6 次均为 zero optimization error、integrity reject、transaction rollback。

## Replay 与 simulation 分离

同一 deterministic 180-frame factor stream 产生完全相同的 V1/V2 cost 与 final state。Pure replay throughput median：V1 111.84 frame/s，V2 112.97 frame/s，仅提升 1.01%；该路径未覆盖完整 online transaction、callback 与 visual hot path，不能复现原 live-solver 27.95% 提升。最终 6 次 simulation 的 classified process CPU：REAL_TRANSFERABLE（backend、visual frontend、FAST-LIO、shared mapping）占 whole-WSL 4.296%，SIM_ONLY（Gazebo、bridge、SITL）占 7.330%；SIM_ONLY 占 classified pipeline 63.05%，不是全部 host work。Gazebo 使用 `kms_swrast`，WSL/driver 修复记录为 `SIM_ENV_BLOCKED`。

## Joint map 与验证

最终 joint-map run 完成 LAND/disarm，共 108191 voxel：97990 LiDAR、22449 RGB-D、10201 supplementary RGB-D；occupied-volume growth 10.41%、color coverage 12.50%、conflict ratio 与 eviction 均为 0，LiDAR 仍是 geometry authority。15 个 package 以 `RelWithDebInfo` 构建；colcon 57 项全通过；backend 158/158、visual 4/4、D435i lifecycle short test 与 Python/YAML/XML/shell 检查均通过。

只有在 controlled runtime host 上稳定证明 live solver median ≤45 ms 后，candidate 才可 freeze。RTF shortfall 单独归类为 `SIM_ENV_BLOCKED`，不是修改 fusion semantics 的理由。
