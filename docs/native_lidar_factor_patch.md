# FAST-LIO native LiDAR factor 导出

本目录提供针对固定版本 FAST-LIO2 ROS 2 source checkout 的可复现 patch。Patch 采用 additive 方式，FAST-LIO 原有 iterated EKF update、map update、odometry output 与 IMU input chain 均保留。

## 版本边界

- Upstream：`https://github.com/hku-mars/FAST_LIO.git`
- 期望 commit：`a4743b095409588842a5b30ddfa27e29d2f99164`
- External checkout：`$HOME/multi-slam-deps/mid360_ws/src/FAST_LIO_ROS2`

执行：

```bash
bash tools/apply_fast_lio_native_factor_patch.sh
```

脚本会拒绝不同 commit 或 dirty checkout，且不会把 external source commit 到本仓库。

Patch series 还包含 backend-owned deskew trajectory frontend，并刻意分离两个 state input：`downstream_backend.activation_state_topic` 只用于激活 scan request 和 backend trajectory prediction；`downstream_backend.state_topic` 是 health-gated optimized pose，只用于确认不可逆的 static-map insertion。

FAST-LIO launch 层 activation topic 默认空字符串；为空时复用 `downstream_backend.state_topic`，保持旧的 single-topic 行为。仿真 wrapper 显式使用 `/fusion/unified/odom` 激活、`/fusion/unified/map_pose` 确认地图，避免 map-health gate 关闭时 frontend deadlock。

## Packet 契约

启用后 patched node 发布 `fast_lio/msg/NativeLidarFactor`。每个 packet 含一次 FAST-LIO update 使用的冻结 point-to-plane correspondence、signed residual、native 12-column measurement Jacobian 和未缩放 normal equation：

```text
H = J^T J
g = J^T r
```

Linearization pose 为 `map_T_body`，matched point 仍在 `sensor_frame`，因此 packet 同时携带 `T_body_sensor` 及 12 个 Jacobian column 的有序名称。Consumer 不能跳过 extrinsic 直接用 body pose 变换 LiDAR point。

Backend 形成 information 时必须应用 `measurement_variance`。`pose_covariance` 是 FAST-LIO posterior covariance，仅用于 logging/reliability diagnostics，不得作为独立 LiDAR covariance 再加入。

Packet 是 scan-local linearization。当前 Stage 7 backend 将固定 extrinsic 的 6DoF pose block 作为 condensed tangent-space normal equation，并按 header timestamp 与 `/lio/odom` 配对。Native packet 存在时不再加入同 state 的 LIO pose proxy，也不消费 `pose_covariance` 作为另一 factor。Relinearization、独立 map ownership 及 FAST-LIO state variable 的消元/保留留待后续 manifold backend。

## Runtime 开关

默认关闭。MID360 仿真 launcher 使用：

```bash
FASTLIO_NATIVE_FACTOR_EXPORT=1 \
FASTLIO_NATIVE_FACTOR_TOPIC=/fast_lio/native_lidar_factor \
FASTLIO_NATIVE_FACTOR_SENSOR_FRAME=mid360_link \
bash src/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
```

Launch wrapper 暴露同样三个参数，`RVIZ` 与既有 FAST-LIO input mode 不变。

## Validator

加载 ROS 2 与 external FAST-LIO install overlay 后运行：

```bash
ros2 run uf_lio_adapter native_factor_validator
```

Validator 检查 array dimension 和 finite value，从导出 geometry 重建所有 point-to-plane residual，独立重建 Ultra-Fusion Eq. (18) 的 6-column pose Jacobian，并验证 `H = J^T J`、`g = J^T r`、Hessian symmetry/PSD 与 posterior covariance symmetry。配置 `output_path`、`summary_path` 时写出 JSONL packet metrics 和 dynamic-range summary。它不修改 estimator，也不向 FCU 回灌数据。

Validator 强制 BLAS 单线程，避免小矩阵检查启动大量 worker 干扰被测 simulation。

## 仿真证据

2026-07-26 固定 rectangle run（`/tmp/multi_slam_gpu_residual_route_20260726_201853`）产生：1051 个 packet 全部 valid，无 malformed 或 sequence gap；matched point median 302（范围 149–1544）；point-to-plane residual RMS median `0.0271 m`、P95 `0.0468 m`；geometry 最大误差 `1.91e-12 m`；相对 `J^T J` 误差 `9.54e-16`、`J^T r` 误差 `1.37e-14`；pose-Hessian 最小 eigenvalue 0.699–336.0；FAST-LIO position RMSE `0.0586 m`、yaw RMSE `0.0978 degrees`；point-cloud/IMU timestamp regression 为 0。

LiDAR observability/degeneration 使用左上角 6×6 position/rotation block。Online extrinsic estimation 关闭时，完整 12-column Jacobian 的后 6 列按设计为零，因此 full 12×12 Hessian 的最小 eigenvalue 为零，不能作为 LiDAR degeneration 信号。

Raw eigenvalue 还会随 accepted correspondence 数量缩放，故 validator 输出 `pose_hessian_min_eigenvalue_per_match` 和 `pose_hessian_condition_number`；reliability scoring 应结合 match count、residual statistics 使用，而不要单独阈值化 raw minimum eigenvalue。

Adapter 现在优先使用 native packet 填充 `/lio/diagnostics`，发布 `approximate=false` 及 native residual、pose-Hessian、normal distribution、spatial-coverage evidence。Temporal static/dynamic map statistics 仍来自 registered-cloud persistence；native packet 超时时，旧 voxel point-to-plane proxy 作为显式 `approximate=true` fallback。

2026-07-26 nominal、75% dropout、90% dropout 矩阵见 `docs/native_lidar_factor_test_report.md`。最强运行启用独立 pose-Jacobian geometry check，验证 1404/1404 packet，并显示 scheduler 的 continuous down-weighting、binary disable 与 recovery。

首个在线 consumer 结果记录于 `src/ultra_fusion_nav/docs/stage7_native_lidar_backend_report_20260726.md`：固定航线插入 692 个 native packet，invalid packet 为 0，仅有 3 个 startup-only pose fallback。
