# Final Ultra-Fusion visual 执行报告

## 结果

分支 `feat/ultrafusion-visual-tight-coupling-v1` 仅基于 commit `57930c86d7d96468b3416f84f8e6f504f527df8a`。它实现了真实的 RGB-D feature frontend、现有 manifold window 中与 paper 对齐的 reprojection residual、现有 FRS 中的 vision evidence，以及可选择启用的 source-aware RGB-D/LiDAR online map。Paper mode 不使用 RTAB local odometry；RTAB 继续用于 persistence、loop closure、cross-session localization 和 legacy A/B factor。

## 验证矩阵

| 项目 | 结果 | 证据 |
|---|---|---|
| Stable-tag identity | PASS | 已记录 annotated tag/tree/commit |
| 15-package build | PASS | 完整 colcon build |
| Colcon tests | PASS | 57 项测试，0 errors/failures |
| New direct tests | PASS | 14 项测试 |
| Jacobian finite difference | PASS | visual factor test |
| Reliability logic | PASS | tracker/reliability tests 和 live topics |
| Launch defaults | PASS | 两个新模块默认 disabled |
| Lifecycle cleanup | PASS | 短生命周期测试，无残留 process/port |
| Deterministic ablation, 3 seeds | PASS | `ABLATION_RESULTS.csv` |
| Online shared-map deterministic run | PASS | 三次稳定运行 |
| Live RGB-D feature front end | PASS | 观察到 RGB/depth/CameraInfo/tracks topic |
| Live paper visual factor accepted | PARTIAL | unified state 未提交 |
| LiDAR/IMU/GNSS/flow live inputs | PASS at transport level | 观察到所有必需 topic |
| Full small_rectangle/S-curve | PARTIAL | route 开始前 preflight timeout |
| Degraded-scene matrix | BLOCKED | 依赖健康的 live unified state |
| Cross-session representative rerun | PARTIAL | tooling 已迁移；runtime gate 后未重跑 |

## Runtime 根因追踪

Round 1 发现 camera-rotation 参数有八个值且无效，并且 CameraInfo topic 错误。Round 2 证明本地 FAST-LIO checkout 只有第一份 downstream patch：四种 stable-tag message type 无法整体导入，导致 native mode 被静默禁用。随后将固定的第二份 patch 应用到外部 dependency，并成功重建 FAST-LIO。Round 3 证明所有 sensor、RGB-D、feature-track、D_V、scheduler 和 NativeLidarFactor topic 均处于 live 状态，backend 也明确报告 native-factor mode，但在 120 秒内未发布 `/fusion/unified/odom`。没有使用 `/Odometry` fallback、放宽阈值或绕过 integrity check。

## 客观结论

新 frontend 确实存在，并产生经过测量的 KLT/depth/PnP 证据。该 factor 确实接入同一个 manifold window 并在那里测试，但由于 native backend 未提交首个 state，不能宣称 live factor 已接受。D_V 在 unit 和 transport 层面与 paper 对齐。新的 online shared map 已实现且具确定性，但真实飞行地图质量尚未测量。本分支适合团队在受控 hardware/simulator 环境中调试，尚不足以创建声称 runtime 已完整完成的 Draft PR。
