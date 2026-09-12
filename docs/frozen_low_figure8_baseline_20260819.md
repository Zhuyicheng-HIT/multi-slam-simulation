# 冻结低空八字基线

这是 2026-08-19 低空八字飞行后的稳定演示和回归入口。使用：

```bash
cd /home/zyc/multi-slam-pr12-audit
source /opt/ros/humble/setup.bash
source /home/zyc/multi-slam-deps/mid360_ws/install/setup.bash
source install/setup.bash
bash tools/run_frozen_low_figure8_validation.sh
```

冻结配置使用低空室内场景、名义 2.2 m 起飞高度、RGB-D direct factors、10 m
仿真深度上限和 directional LiDAR axis handoff，并将 online time calibration
保持在 shadow/diagnostic mode。低空路线契约是冻结的 29 m 航迹，至少包含 14 个
route checkpoints。FAST-LIO drift 仍作为 diagnostic 记录，但不是本配置的致命
acceptance gate，因为 FAST-LIO 不拥有最终 estimator pose。

Mission observers（包括 runtime、drift 和 reliability recorders）默认在
`/mission/phase=landed` 时停止；该信号仅在路线节点确认着陆和 FCU disarm 后发布。
随后在验证 teardown 期间中断 replay bag recording，避免基线在着陆后空闲数据上
消耗 wall time 或磁盘。只有明确需要固定时长的飞行后记录时才设置
`VALIDATION_STOP_OBSERVERS_ON_LANDING=false`。

以下实验路径保留但不会由此入口调用：Range-Facet、barometer fallback 和 active
relocalization triggers。此入口启用 EKF3 ExternalNav control。单变量实验必须使用
新的 `LOG_DIR`，不得覆盖冻结运行。

参考运行：`logs/low_indoor_figure8_rangefacet_20260819`。

| 指标 | 参考值 |
| --- | ---: |
| 仿真时长 | 279.807 s |
| 匹配样本数 | 2617 |
| Causal 3D RMSE | 3.232 cm |
| Causal 3D P95 | 4.973 cm |
| 3D 最大误差 | 11.460 cm |
| 垂直 RMSE | 2.685 cm |
| Solver P95 | 约 33 ms |
| Backend callback P95 | 约 81 ms |

记录的运行包含 Range-Facet 开关，但没有接受任何 Range-Facet factor。冻结配置
明确禁用它，防止实验路径消耗 runtime budget。因此参考指标是归档的比较点，
不声称未来每次运行都完全相同。
