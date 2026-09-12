# Ultra-Fusion V1 性能剖析

## 范围与控制项

冻结参考为 `feat/ultrafusion-visual-tight-coupling-v1`，提交
`d76543e9c8f80dcaecbcbe4d898811a420978094`。该分支没有修改。六次 production
baseline runs 使用相同场景、sensor rates、0.065 s association gate、balanced
visual cadence、D_V/FRS、integrity checks、rollback policy 和 map parameters。
原始运行值在 `PERFORMANCE_BASELINE_V1.csv`。

Profiler 为可选功能（默认 `performance_profiling_enabled:=false`），使用有界 deques、
wall-monotonic nanosecond clocks，并报告 P50/P90/P95/maximum。Production baseline
在 profiling 关闭时采集。随后运行 r31 开启 profiling 以归因开销；其 solver median
为 52.741 ms，RTF 为 0.436，接近代表性 V1 operating point。

## 三个主要 estimator 瓶颈

| 排名 | V1 阶段 | P50 (ms) | P95 (ms) | 最大值 (ms) | 证据 |
|---:|---|---:|---:|---:|---|
| 1 | complete nonlinear optimize | 52.867 | 77.503 | 164.566 | r31 |
| 1a | graph linearization/assembly | 7.764 | 23.195 | 67.507 | r31 |
| 1b | marginalization | 7.537 | 14.358 | 26.082 | r31 |
| 2 | transactional snapshot | 3.979 | 7.817 | 17.356 | r31 |
| 3 | visual reprojection factor | 1.821 | 7.914 | 22.174 | r31 |

marginal prior 中的 dense block-diagonal Jacobian transform、不可变 factor payload 的
完整 `deepcopy` 以及逐 feature 的 Python reprojection loop 是可行动的原因。Linear solve
本身不是瓶颈（P50 0.310 ms，P95 0.533 ms）。

## 感知 source 的地图基线

r36 joint-map control 显示第二条异步 hot path：LiDAR voxel integration 的 P50/P95
为 12.511/21.663 ms，RGB-D voxel integration 为 70.757/274.713 ms，full-map
publication 为 16.009/305.979 ms。这些开销不在 estimator callback 中运行，但会争用
同一 CPU 并降低仿真 RTF。

## 基线结果

三次 rectangle runs 的 median solver time 为 51.973 ms，median RTF 为 0.428570。三次
S-curve runs 的对应值为 59.794 ms 和 0.463435。六次运行合计 median solver time 为
58.847 ms，RTF 为 0.460813。六次运行均无 optimization errors 和 rollbacks。

历史 S-curve wrapper 在路线结束后停止 recorder，没有保留 raw/tracked/window counters。
Accepted/quality/time-rejection totals 已保留并写入 CSV；缺失的 window 值留空，不进行重建。
