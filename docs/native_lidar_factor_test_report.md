# Native FAST-LIO 残差运行报告

日期：2026-07-26

## 范围

本报告验证 patched FAST-LIO2 的 scan-to-map measurement export，不宣称 unified sliding-window backend 已经消费 native point-to-plane factor。

实现遵循论文 LiDAR model：

```text
r_i = n_i^T (p_i^w - s_i)                         Eq. (6)
J_i = [n_i^T, -n_i^T [p_i]_x]                    Eq. (18)
H_k = sum_i J_i^T J_i + 1e-8 I_6                 Eq. (18)
D_L = w_h phi_h + w_n normal_term + w_a phi_a
      + w_c (1 - min(1, M_k / M_ref))            Eq. (19)
```

FAST-LIO 使用 right SO(3) perturbation；validator 检查的等价 rotation block 为 `(p_body x R_WB^T n_world)^T`。

## 验证层

1. 从导出的 point、plane、body pose 和 LiDAR-to-body extrinsic 重建所有 signed point-to-plane residual。
2. 独立依据 geometry 重建前 6 个 Jacobian column。
3. 重新计算 `J^T J` 与 `J^T r`，检查 symmetry 和 positive semidefiniteness。
4. 检查 posterior pose covariance、packet dimensions、frame、sequence 与 finite value。
5. 将 native 6D pose Hessian 与 residual statistics 送入 `/lio/diagnostics`，再送给 reliability scheduler。

原 FAST-LIO iterated EKF 与 map update 不变；仅在 native packet 超时才回退 external voxel proxy。

## 固定航线结果

各运行使用 `simple_apm_rgbd_mid360`、相同 rectangle flight、headless Gazebo GPU rendering、FCU HIGHRES_IMU 和单线程 OpenBLAS。表中为评估区间 median。

| Run | Native packets | Matches | Residual P95 (m) | lambda_1 | lambda_1 / match | condition | D_L output | Scheduler action |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| Nominal | 1415 | 288 | 0.0512 | 29.67 | 0.1103 | 460.7 | 0.186 | enabled, continuous weight |
| 75% dropout | 421 | 89 | 0.0609 | 2.790 | 0.0335 | 1083 | 0.250 | enabled, weight 0.750, inflation 1.33 |
| 90% dropout | 387 | 38 | 0.0636 | 0.410 | 0.0108 | 3410 | 0.326 | disabled, weight 0, inflation 20 |
| 90% recovery | 540 | 1148 | 0.0510 | 248.8 | 0.2159 | 31.4 | 0.0196 | re-enabled, weight 0.932 |

90% dropout 是首个启用 independent pose-Jacobian geometry check 的 runtime 运行；接收并通过了 1404/1404 packet 检查，无 residual、Jacobian、normal-equation、covariance 或 metadata 失败。

## 轨迹与运行门控

| Run | Aligned ATE RMSE (m) | RPE translation RMSE (m) | Drift yaw RMSE (deg) | RTF median | RTF P10 | Stamp regressions |
|---|---:|---:|---:|---:|---:|---:|
| Nominal | 0.0439 | 0.00933 | 0.0988 | 0.9997 | 0.815 | 0 |
| 75% dropout | 0.0472 | 0.01104 | 0.1011 | 0.9994 | 0.781 | 0 |
| 90% dropout | 0.0600 | 0.01367 | 0.1150 | 0.9995 | 0.774 | 0 |

即使大量随机丢点，简单测试地图仍可定位。因此 scheduler 在 75% 丢点时连续降权，仅在 90% 运行的 median match count 低于 50 时启用 Eq. (15) minimum-observation gate。

## 复现

```bash
source /opt/ros/humble/setup.bash
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
source install/setup.bash

python3 src/ultra_fusion_nav/scripts/analyze_native_lidar_experiment.py \
  /tmp/native_lidar_dropout90_20260726_210614 \
  --output /tmp/native_lidar_dropout90_20260726_210614/native_factor_report.json
```

证据目录：

```text
/tmp/native_lidar_nominal_v2_20260726_205300
/tmp/native_lidar_dropout_20260726_205731
/tmp/native_lidar_dropout90_20260726_210614
```

## 剩余集成边界

Exporter 已适合下一步 backend 集成，但 backend 不得在同一 timestamp 同时加入 FAST-LIO pose anchor 与相同 native point-to-plane 信息。后续应选择：

1. 消费 native LiDAR pose block，使用独立 IMU preintegration，并移除这些 state 的 proxy LIO pose factor；
2. FAST-LIO odometry 仅作 initialization/output continuity，不作为额外 information factor；
3. 导出的 posterior covariance 只作 diagnostics，factor weight 使用 `measurement_variance` 与 native residual/Jacobian。
