# LiDAR 时间契约分析

## 范围与证据

本分析区分 FAST-LIO/backend 边界上的一致延迟与故意破坏的接口契约。冻结 replay 为 `logs/tmp/robustness_v3_frozen_clock_replay_c`，完整 26-profile 结果位于 `logs/tmp/robustness_v3_1_lidar_temporal_deterministic/temporal_summary.json`。

冻结 bag 含 `FrontendScanRequest` 和 `NativeLidarFactor`，不含原始 LiDAR packet 或逐点 timestamp。因此 A1 只证明 FAST-LIO frontend 边界之后的一致时间契约，不能代表真实 MID360 的 packet/deskew 物理容差。

## 旧 injector 的变化

旧 `native_lidar/time_offset` 只移动 `NativeLidarFactor` header、scan-begin 和 scan-end，没有移动配对的 `FrontendScanRequest`，也无法访问 packet、逐点、FAST-LIO frontend 或 deskew timestamp；旧的 `+2 ms` 是 A2 interface-mismatch 实验，不是物理传感器延迟结果。

V3.1 injector 提供两个明确范围：`coherent_frontend_contract` 同时移动 request、factor header、scan begin/end；`factor_only` 只移动 factor timestamp，故意违反 request/factor cache contract。Scan-request publisher 使用与 backend subscription 匹配的 reliable、transient-local QoS，仅移动 timestamp，不用到达时间重新盖章。

## 确定性矩阵

每个值使用相同冻结消息和顺序；所有行 optimization error、integrity reject、transaction rollback 均为零。

| Offset | A1 一致边界 | A2 仅 factor 不匹配 |
|---:|---|---|
| 0 ms | PASS，completeness 1.000 | PASS，completeness 1.000 |
| ±0.5 ms | PASS，completeness 1.000 | PASS，completeness 1.000 |
| ±1 ms | PASS，completeness 1.000 | PASS，completeness 1.000 |
| ±2 ms | PASS，completeness 1.000 | FAIL：+2 ms gap 1.023 s；-2 ms completeness 0.0186 |
| ±5 ms | PASS，completeness 1.000 | FAIL，几乎无可用轨迹 |
| +10 ms | PASS，completeness 1.000，max gap 0.627 s | FAIL |
| -10 ms | PASS，completeness 1.000，max gap 0.232 s | FAIL |
| +20 ms | FAIL，completeness 0.6637，first gap 1.716 s | FAIL |
| -20 ms | FAIL，completeness 0.7696 | FAIL |

实测一致边界为 **[-10 ms, +10 ms]**；factor/request mismatch 仅为 **[-1 ms, +1 ms]**。这些是测试边界，不是插值或通用硬件上限。

## Scan-prediction cache 机制

名义一致 timing 产生 575 个 Native factor，574 次 cache hit、0 miss。+10 ms 保留 542 次 hit、0 miss；backend 拒绝 43 次 reuse 和 34 个 IMU/window coverage 不足的 scan。+20 ms 仍无 cache miss，但 reuse 与 scan reject 升至 160 和 202，导致 completeness 真正下降。

Factor-only +2 ms 中 request 保持原 timestamp 而 factor key 移动，产生 184 次 reuse reject 和 1.023 s 轨迹 gap；-2 ms 仅 7 个 factor 存活，说明 request/factor pairing contract 崩溃，即 `INTERFACE_CONTRACT_SENSITIVITY`。

## 在线标定与硬件证据

现有 LiDAR-IMU time calibration 在该 runtime 中仅 shadow-only，冻结 bag 没有独立平移的原始 LiDAR motion stream，无法合法恢复或验证 packet/point/deskew delay。以下项目仍标记为 `HARDWARE_DATA_REQUIRED`：MID360 packet/逐点 timestamp 传播、生产 driver 的 scan begin/end 推导、真实 deskew trajectory 与 FAST-LIO frontend timing，以及生产 LiDAR-IMU online time-calibration path 的恢复能力。

## 结论

报告中的 `+2 ms` 失败不是物理时间敏感性，而是 `NativeLidarFactor` 与配对 trajectory/cache request 的故意不匹配。接口现已显式、可测试并可诊断，未修改任何 association tolerance。
