# RELEASE-HXY-DYNAMIC-010 阶段总结

## 基线

- Branch：`feat/lidar-horizontal-degeneracy-v1`
- Summary 前 release commit：`88203bf22e12887e1aacecebe99219d550e89b9e`
- 目标 base：当前 `main`（`c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981`）
- Repository：`Zhuyicheng-HIT/multi-slam-simulation`

## 已交付

- MID360 native IMU 是 FAST-LIO 与 Ultra-Fusion backend 的公共 IMU 源；FCU IMU 仍在 ArduPilot 内部使用。
- HXY horizontal LiDAR degeneracy 处理支持任意旋转的 weak-subspace attenuation。
- 增加 prediction recovery 与 scan-prediction contract fail-closed 保护。
- 迁移 PR15 Dynamic Observer v2 和 Clean Scan Gateway，同时保留 safety/avoidance 使用的 raw cloud 路径。
- 增加 Gazebo clock startup/ownership guard，并修复 FAST-LIO clean-topic runtime 配置。

## 证据

- Dynamic benchmark（PR15-compatible synthetic benchmark）：micro precision `99.8439%`、recall `97.0854%`、F1 `98.4454%`；macro precision `93.5449%`、recall `85.7748%`、F1 `88.8424%`。
- Static-point preservation `99.9859%`；dynamic contamination `1.8083%`；observer latency p50 `7.186 ms`、p95 `9.784 ms`。
- HXY long-tunnel replay 将冻结 comparison bag 的水平漂移从约 `6.5 m` 降至 `0.79 m`。
- Dynamic-enabled static replay 完成 60 s ROS time：最大 3D deviation `0.028 m`；10/30/60 s XYZ displacement `0.019/0.017/0.023 m`，XY displacement `0.019/0.015/0.021 m`。
- Static replay 记录 600 个 FAST-LIO 与 600 个 truth sample，无 invalid timestamp。Native LiDAR factor：`598`；GNSS factor：`41`；IMU factor：`597`；optimization rollback：`0`。
- Feature repeatability median：`100%`。

## 已知限制与后续

- Feature repeatability 目前只有 median diagnostic，仍需正式 P5/minimum 及低于 95% 的 frame fraction 评分。
- Dynamic benchmark 的 macro recall 低于 micro 结果，宣称通用 recall 前需进行多场景验证。
- Marginal-prior weak-direction attribution 与更多 replay coverage 仍待完成；本阶段未调整 marginalization。
- 本 release PR 不请求 merge。
