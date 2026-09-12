# 五源 ExternalNav 稳定候选版本

版本范围：`2026-08-16-five-source-externalnav-candidate`

本里程碑在一个 sliding window 中集成 FCU IMU、MID360 point-to-plane factors、
GNSS/BDS、MTF-01P 风格 optical flow 和 D435i RGB-D feature reprojection，并向
ArduPilot EKF3 发布一条 fused odometry stream。Gazebo truth 仅由 evaluator 使用。

这是经过仿真验证的 research candidate，不是硬件飞行 release，也不声称 automatic
online time-offset application 已达到 production-ready。

## 稳定的一键验证

使用 `cuf_ws` 进入工作空间后运行：

```bash
bash tools/run_stable_five_source_validation.sh
```

命令负责完整的 headless graph，执行一次短矩形飞行，让 EKF3 消费
`/mavros/odometry/out`，记录 replay bag，将 unified state 与 Gazebo truth 比较，
检查着陆/disarm，并停止子进程。严格门限失败时返回非零状态。

无需修改脚本即可选择隔离的 ROS domain：

```bash
VALIDATION_ROS_DOMAIN_ID=42 bash tools/run_stable_five_source_validation.sh
```

## 已验证的现场结果

证据目录：

```text
logs/externalnav_rectangle_20260816_v20_five_source
```

v20 闭环运行通过所有严格门限：

| 指标 | 结果 |
|---|---:|
| causal 3D RMSE | 0.0482 m |
| causal 3D P95 | 0.0785 m |
| causal 3D 最大值 | 0.0944 m |
| 终点误差 | 0.0235 m |
| horizontal RMSE | 0.0160 m |
| vertical RMSE | 0.0455 m |
| unified odometry source rate | 10.0 Hz |
| ExternalNav output rate | 20.0 Hz |
| visual factors | 412 |
| visual solver acceptance | 412 / 412 |
| optimizer errors / rollbacks | 0 / 0 |

Evaluator 冻结 10 s 的初始 yaw/translation 对齐，不使用未来轨迹。共匹配 1528 个
样本并观测 576 个运动样本。EKF3 ExternalNav consumption、MAVROS NED/FRD TF contract、
全部五类 factor、路线完成、着陆和 disarm 均独立通过 gate。

## ExternalNav covariance 与连续性策略

Backend 根据 active factor graph 计算 15-state marginal covariance。不可观测特征方向
获得较大的有限 variance。在 optimization states 之间，IMU propagation 使用
`F P F^T + Q`。随后将 state covariance 映射为 ROS pose 与 body-velocity covariance，
再发布到 `/fusion/unified/odom`。

ExternalNav gate 以 20 Hz 重新发布，并在 optimized state 变旧时加入有限 process
uncertainty；同时按 estimator capability support 对 covariance 缩放，直到配置的有限上限。
因此，ArduPilot 在有界退化期间收到的是连续但置信度较低的 measurement，而非静默声称
名义精度的 stream。

在 v20 中，最终 pose-position diagonal 最大值为 `0.00667 m^2`，orientation diagonal
最大值为 `0.00635 rad^2`，linear-velocity diagonal 最大值为 `0.189 m^2/s^2`。运行期间
观测到的最大 position/orientation diagonal 为 `1.053 m^2` 和 `0.0309 rad^2`。Covariance
来自 window marginal 或其 IMU-propagated anchor，未使用 fallback covariance。这些是
estimator outputs，不是手工填写的常数或精度结果。

输出准入对冗余传感器使用 OR 语义：

- 一个新鲜且可用的 modality 即足以让 estimator 保持在 `FAILSAFE` 之外；
- 缺少必需 IMU 会将 scheduler 改为 `RISK`，但不会单独终止 state stream；
- 缺少 horizontal/yaw/propagation capability 会报告给 safety state machine，它可保持或请求 relocalization；
- `DEGRADED`、`RISK` 和 `RELOCALIZING` 仍可继续发布 ExternalNav；
- 没有可用 source、stale state、非有限值、无效 quaternion 或 covariance、时间戳回退以及物理上不合理的跳变，仍是硬性 output failure。

这样将“估计已退化”与“没有可安全发布的 state”分开。Safety controller 负责飞行行为，
estimator 负责保持连续性。

## ArduPilot 接口边界

稳定 ROS 输出为：

```text
/mavros/odometry/out
```

MAVROS 通过显式的 `camera_init_ned` 和 `body_frd` transforms 转换 unified
`camera_init -> body` odometry。当前 SITL EKF3 profile 使用 ExternalNav 作为水平位置
和水平速度来源；在首个闭环 profile 中，barometer 仍提供垂直来源，compass 仍提供 yaw
来源。硬件部署飞行前必须读取实际 EKF3 source parameters、frames、message rate、
covariance、quality 和 reset counter。

不允许 FAST-LIO odometry、FCU local position 或 Gazebo truth 成为 unified backend 的第二权威 state input。

## 时间标定策略

LiDAR-IMU 与 visual-IMU offset estimators 均在线运行，用于 observability 和诊断。稳定默认值为：

```text
calibration_apply_locked_values: false
calibration_apply_locked_time_offset: false
calibration_apply_locked_rotation: false
visual_time_calibration_apply_locked: false
visual_initialization_require_time_lock: false
```

v20 LiDAR-IMU estimator 生成了候选值但未获得可重复 lock。Visual estimator 在部分运动
期间 lock，但最终 correlation 较低。早期受控运行得到约 `-5 ms` 至 `-15 ms` 的 offset，
这是有用证据，但不足以启用自动时间戳修改。

因此 production policy 为：

1. 测量并配置固定 sensor offsets；
2. 将 online estimators 保持 shadow mode，监测 correlation、peak margin、independent agreement 和 lock revocation；
3. 仅在注入 offset 且具备 repeatability gate 的独立 A/B 实验中启用自动应用。

### Online calibration 后续

后续一次仅标定仿真将 visual-to-IMU offset lock 在约 `+16 ms`。应用该 locked visual offset
后，causal 3D RMSE 为 `0.0396 m`、P95 为 `0.0778 m`、最大误差为 `0.0911 m`。同次运行
还记录到两次 transactional optimizer rollback、一次 latest-only native queue discard 以及
`0.29 s` unified-odometry gap。因此 automatic visual offset path 已实现且与精度兼容，但仍属实验功能。

LiDAR-IMU spatiotemporal calibrator 和 visual-IMU calibrator 是独立机制。报告与 promotion
gate 必须分别注明；锁定或应用其中一个，不能证明另一个已应用。

## 已知限制

- 现场五源运行在短矩形上达到 0.20 m 目标；大型八字路线和真实硬件仍是独立 acceptance milestone；
- D435i 仿真深度理想化，实验量程为 10 m。硬件 profile 必须恢复到经过标定的量程，初始为 0.3–6.0 m，并加入逐点深度质量及真实孔洞/噪声；
- D435i/MID360 online shared mapping 是独立 map consumer，不是此稳定 sliding window 中的额外 dense factor；
- 当前 EKF3 闭环证据覆盖 ExternalNav 水平位置和速度，不覆盖 ExternalNav 接管气压高度或 compass yaw；
- Solver 保持所需 10 Hz source output，但完整仿真 stack 运行慢于 wall time，仍需硬件 CPU profiling。

## 硬件推广门限

在移除 simulation-only 标签前：

1. 使用带 source timestamps 和 measured extrinsics 的真实 FCU IMU、MID360、MTF-01P、GNSS/BDS 与 D435i bags 进行 replay；
2. 验证固定 time offsets，并保持 automatic calibration 为 shadow-only；
3. 确认没有 factor 重复计算 FAST-LIO 或 FCU fused local position；
4. 在无螺旋桨条件下，结合 EKF3 parameter readback 对 `/mavros/odometry/out` 进行 bench test；
5. 测试 source outage，同时确认 covariance inflation、continuous output、HOLD behavior 和 reset-counter handling；
6. 在自由飞行前，于 motion-capture 或测量参考轨迹上重复 0.20 m accuracy 与 continuity gate。
