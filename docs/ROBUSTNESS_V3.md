# Ultra-Fusion Robustness V3

Robustness V3 冻结 Performance V2 estimator，并从 fusion core 外部测试其故障行为。它不增加 estimator factor、不修改 integrity threshold，也绝不使用 `/Odometry` 替代 `NativeLidarFactor`。

## 架构

冻结的 rosbag 重放到 `/robustness/raw/*`。Profile 驱动的 injector 重新发布规范化 sensor/factor topic 及匹配的 reliability evidence；新的 reliability scheduler 生成实时 scheduler state。Backend 在 dynamic（`FRS ON`）或固定单位权重（`FRS OFF`，仅用于 A/B 对照）模式运行。记录的 scheduler state 被隔离，不能泄漏到实验中。

故障 timing 始终使用 source header clock。`time_offset` fault 按配置的物理 offset 移动 source timestamp；不会重写输出 timestamp 来强行通过 association。所有随机选择使用 profile seed。每次运行均保存 input bag、profile、FRS mode、output directory、metrics 和 resource record。

原始 capture 的 rosbag storage timestamp 包含 WSL/Gazebo software-rendering slowdown。`tools/normalize_robustness_replay_clock.py` 可按 bag 记录的 `/clock` 创建 derived functional-test bag；它逐字节复制 CDR payload，仅修改 storage receive time，不改写 sensor timestamp。Performance/RTF 结论仍须使用原始 bag；fault-boundary test 可使用 normalized bag 以避免 simulator 瓶颈。

## Profile 与工程级别

`uf_sensor_pipeline/config/robustness_v3_profiles.yaml` 包含 light、medium、heavy 的 visual、Native LiDAR、GNSS denial/jump、optical-flow 与 IMU fault；camera/LiDAR time offset；D435i/MID360 rotation/translation error；double fault 和 endurance cycle。这些是明确的 V3 engineering test level，并非论文给出的 threshold。

Native LiDAR correspondence dropout 减少送入 backend 的 raw point-plane row；LiDAR outage 丢弃 NativeLidarFactor message。没有运行启用 pose fallback。D435i calibration case 使用 backend 文档中的 `visual_rotation_body_camera` 与 `visual_translation_body_camera_m` 参数；MID360 case 扰动每个 native factor 携带的 `T_body_sensor`。

冻结 replay 包含记录的 reliability evidence，而不是所有 frontend diagnostic input。注入 fault 期间，adapter 将对应 degradation evidence 提高到 profile 声明的 floor；scheduler 本身保持 live 且不变。Sensor-specific monitor 的 detection behavior 继续由 unit test 覆盖。原始 diagnostic evidence 可用时，完整 Gazebo/hardware run 应在 live monitor 前使用同一 injector。

## 命令

构建 validation package 并运行确定性测试：

```bash
colcon build --packages-select uf_sensor_pipeline
colcon test --packages-select uf_sensor_pipeline --event-handlers console_direct+
colcon test-result --verbose
```

运行一组冻结输入的 A/B：

```bash
PROFILE=visual_medium FRS_MODE=on tools/run_robustness_v3_replay.sh
PROFILE=visual_medium FRS_MODE=off tools/run_robustness_v3_replay.sh
```

运行矩阵（不会挑选最佳 trial）：

```bash
RUN_SET=singles tools/run_robustness_v3_matrix.sh
RUN_SET=calibration tools/run_robustness_v3_matrix.sh
RUN_SET=doubles tools/run_robustness_v3_matrix.sh
RUN_SET=endurance tools/run_robustness_v3_matrix.sh
```

`RUN_SET=all` 执行以上全部集合；`RUN_SET=smoke` 为有界 entry check。输出仅写入 `logs/tmp`，并被 Git 忽略。设置 `RESUME_EXISTING=1` 可保留已完成报告，仅运行缺失的 matrix entry；runner 仍会在使用前校验每份保留报告。

## 记录的证据

每次运行记录 aligned ATE/RPE、trajectory span completeness、各 modality factor count/rejection、scheduler weight/score、FRS switch 与 recovery time、solver/callback latency、replay RTF proxy、backend CPU/RSS/page-fault/context-switch counter、optimization error、integrity reject 和 rollback。若 capture 没有同一运行的 ground-truth stream，matrix 使用冻结的 nominal backend trajectory 作为 reference，并将结果标为 delta-ATE/RPE；不会把跨运行 simulator truth 当作绝对精度结果。

Matrix summary 使用声明的 V3 continuity criterion：route span 至少 90%、unified-odom gap 不超过 1.0 s、transaction/integrity failure 为零，且 aligned ATE 不超过 `max(2 * nominal, nominal + 0.10 m)`。该 criterion 是工程决策，不作为论文结果。

Joint-map stress 仍是 full-stack test，因为冻结 backend bag 不包含 RGB-D image 或 map update。Map evidence 必须包括 voxel count、RGB coverage、supplementary volume、conflict ratio 和现有 source-aware mapper 的 ghosting proxy；不得用 replay metric 声称 map stability。

运行受管控的长路线 joint-map 检查：

```bash
tools/run_robustness_v3_joint_map_stress.sh
```

该命令复用现有 lifecycle-safe headless harness、20 m × 12 m route、balanced paper-reprojection frontend 和 source-aware joint mapper。其 ghosting proxy 明确定义为被分类为 geometry conflict 的 RGB-D update fraction，不得误标为绝对 surface ground truth。

包含失败边界和 full-stack joint-map failure 的 measured campaign outcome 记录在 `docs/ROBUSTNESS_V3_RESULTS.md`。
