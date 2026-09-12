# Joint-map 长时运行因果分析

## 实验

所有规定运行使用同一 10 m × 6 m rectangle、0.5 m/s command、balanced visual cadence、未改变的 FRS/integrity threshold 和 software rendering fallback。首个矩阵每种 mode 连续运行 3 次；FCU StatusText 增加 direct wall monotonic timestamp 后，每种 mode 保留第 4 次作为 supplementary evidence。

| Mode | 规定运行 | 补充 r4 | 规定 rollback 总数 |
|---|---|---|---|
| map OFF | 0/3 LAND；3 FCU Crash | FCU Crash | 0, 6, 1 |
| LiDAR-only | 0/3 LAND；3 FCU Crash | LAND/disarm PASS | 8, 0, 0 |
| RGB-D + LiDAR joint | 0/3 LAND；3 FCU Crash | FCU Crash | 48, 15, 6（cycle trace） |

每次失败都报告 ArduPilot `Crash: Disarming`，姿态误差超过 30° 且加速度很低，通常发生在第二次转弯或第二条边。Flight controller 报告 `navigation_source=gps`；这些实验中 unified estimator 不是 FCU control source。

## 事件顺序

最初 9 次运行没有 direct monotonic FCU logging，但关键样本仍完整：map-off r1 与 LiDAR-only r2/r3 在整个记录中分别以 zero rollback 崩溃，说明 estimator rollback 和 mapping 不是 FCU failure 的必要原因。

Direct-clock supplementary run：joint r4 在 monotonic 17474.580566 s 发生 FCU Crash，1.736 s 后第一次 rollback，故 crash-before-rollback 明确，Crash 前 rollback 为 0。map-off r4 在 Crash 前有 13 次、之后 21 次 rollback，但未启用 mapping 且 FCU 仍使用 GPS navigation，这符合两个 estimator 同时响应 simulated vehicle dynamic failure。LiDAR-only r4 完成全部 4 条边并 LAND/disarm，884 Native LiDAR factor、0 optimization error、0 integrity reject、0 rollback。

## Mapping 负载与 backend timing

精确时钟运行没有稳定的“map spike → solver spike → state gap → excessive correction → FCU Crash”序列。

| Crash/end 前指标 | map OFF r4 | LiDAR-only r4 | joint r4 |
|---|---:|---:|---:|
| solver P50 / P95 | 37.13 / 76.23 ms | 41.18 / 77.08 ms | 39.73 / 76.23 ms |
| max backend ROS-state gap | 0.297 s | 0.594 s | 0.331 s |
| map publish P95 | n/a | 139.86 ms | 49.90 ms |
| LiDAR insertion P95 | n/a | 9.54 ms | 9.19 ms |
| RGB-D insertion P95 | n/a | n/a | 109.50 ms |
| rollback before Crash/end | 13 | 0 | 0 |
| flight result | FCU Crash | LAND/disarm | FCU Crash |

成功的 LiDAR-only 运行承受了比失败 joint run 更大的 full-map publish spike；joint run 在 Crash 前没有 rollback，也没有超过 1 s 的 state gap。因此没有依据进行 map performance isolation change。现有 sensor-data QoS 与 RGB/depth cache 有界，诊断数据不显示无界 application-level growth。

## Map 结果

4 次 joint run 的 voxel count 为 54,856–97,544（median 88,473），RGB coverage 16.97–26.61%（median 21.31%），补充 occupied volume 28.72–57.12%（median 38.45%）。Conflict ratio、ghosting proxy 和 eviction 在 4 次运行中均为 0。LiDAR 仍是 primary geometry source，RGB-D 仅增加颜色与不冲突的 occupancy。

LiDAR-only r4 在完整航线达到 91,664 voxel；较短的失败运行达到 47,030–47,623 voxel，解释了 r4 更大的 map，而非语义变化。

## FCU 结论

旧的 27 次 rollback 观察混合了两个独立效应：

1. ArduPilot/Gazebo 在转弯时会间歇性丢失 vehicle attitude/altitude 并声明 Crash，即使 mapping disabled 且 rollback 为 0；
2. Crash 之后，invalid vehicle motion 与 sensor data 可能触发 unified backend 合法的 integrity rejection 和 transaction rollback。

相同设置下唯一保留的成功长航线证明故障不是 deterministic，但总体 1/12 route success 不足以授权 tethered flight。WSL software rendering 与 simulated FCU/dynamics path 仍是限制环境。
