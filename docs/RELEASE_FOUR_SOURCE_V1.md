# 四源融合稳定候选版

版本标识：`v0.1.0-four-source-reloc-calibration`

本版本冻结当前仿真中已经具备可重复入口和测试证据的四源导航主线，供视觉合作者在此基础上接入 D435i 前端。四源指飞控 IMU、MID360 LiDAR、GNSS/BDS 和 MTF-01P 风格光流。该版本是研究用稳定基线，不是最终 Ultra-Fusion 论文复现完成版，也不是实机放飞版本。

## 本版本包含

- 统一滑窗后端：IMU、原生 FAST-LIO 点面因子、GNSS 位置因子和光流水平位移因子在同一时间队列和窗口内处理。
- FAST-LIO 前端解耦：保留 Livox 点时间、去畸变、点面对应、平面统计和 Hessian/退化信息；默认不使用 `/lio/odom` 作为最终后端位姿，也不启用 LIO pose fallback。
- 因子级可靠性调度：LiDAR、GNSS、IMU 和光流分别评分，调度器输出连续权重、协方差膨胀和健康状态，不因单一传感器退化而关闭全部观测。
- 光流处理：采用 FCU IMU 旋转补偿、MTF-01P 质量/测距门控、水平位移残差和传感器杆臂补偿；偏航运动期间保留有效补偿观测，并按残差和可靠性降权。
- 重定位初步闭环：静态关键帧、候选检索、多帧一致性门控、事务式提交、融合 epoch/reset counter，以及重新开始当前窗口预积分和因子消费。
- 初步时空标定：独立 LiDAR 相对运动输入、时间偏移相关性搜索、旋转激励和残差门控。当前默认只做 shadow/diagnostic，不自动应用锁定外参，避免未经 Fast-Calib 验证的外参改变定位结果。
- ExternalNav 稳定输出：统一后端只产生一个融合状态，经过健康、协方差、步长和 epoch 检查后输出给 APM EKF3。
- 仿真输入适配：Gazebo MID360 直接转换为 Livox CustomMsg，机身点云剔除和 ROS 时间戳链路保持不变；公共启动命令保持兼容。

## 主要入口和话题

启动统一四源验证：

```bash
cd "$HOME/projects/multi-slam-simulation"
VALIDATION_ROUTE=s_curve S_CURVE_PASSES=3 \
  ENABLE_RELIABILITY_RECORD=1 \
  bash tools/run_unified_rectangle_validation.sh
```

后端默认输入：

| 数据 | 话题 | 说明 |
|---|---|---|
| 飞控 IMU | `/sensors/imu` | 由 MAVROS/飞控 IMU 链路提供；不使用 D435i IMU替换 |
| MID360 原生因子 | `/fast_lio/native_lidar_factor` | 后端主 LiDAR 因子；不把 `/lio/odom` 当权威状态 |
| GNSS/BDS | `/sensors/gnss/fix` | 位置因子，带年龄、跳变和协方差门控 |
| 光流 | `/sensors/optical_flow/rad` | 旋转补偿后的水平运动观测 |
| 调度器 | `/reliability/scheduler_state` | 因子权重和健康状态 |

后端输出：

- `/fusion/unified/odom`：统一窗口状态；
- `/fusion/unified/path`：轨迹可视化；
- `/fusion/unified/diagnostics`：因子计数、协方差、调度和标定诊断；
- `/fusion/unified/epoch`：重定位后的 epoch/reset 事件；
- `/mavros/odometry/out`：ExternalNav 唯一输出，经过 `external_nav_gate` 后发送给 APM EKF3。

重定位和时间标定接口：

- `/relocalization/result`：经过关键帧检索和多帧一致性检查的重定位结果；
- `/calibration/lidar_relative_motion`：独立 LiDAR 相对运动，供时间/旋转标定 shadow 计算；
- `/fusion/unified/epoch`：重定位提交后通知前端丢弃旧 epoch 因子并重新开始窗口消费。

## 给视觉合作者的接入边界

1. 视觉前端可以发布 RGB-D/RTAB-Map 相对位姿或视觉几何因子，但应作为新的可选因子进入统一窗口；不要直接发布第二个 `/mavros/odometry/out`，也不要把 `/mavros/local_position/pose` 或 Gazebo 真值回灌后端。
2. D435i 自带 IMU不能替换飞控主 IMU。视觉前端使用自己的时间戳和外参，在后端入口处与现有 IMU、LiDAR、GNSS、光流队列对齐。
3. 视觉可靠性应独立实现为 `D_V`，至少包含深度有效比例、模糊/纹理、重投影或相对位姿一致性；由调度器控制视觉因子权重和协方差膨胀。
4. 外参以 Fast-Calib 的离线测量值为主。在线旋转/外参标定在本版本仅提供诊断，不得默认修改生产状态；时间偏移候选同样必须通过相关性、激励和残差门控后才能进入后续实验。
5. 双点云地图融合与状态估计分层：D435i 点云和 MID360 点云不能未经 SE(3) 外参、时间对齐和体素置信度处理直接拼接。视觉地图融合仍是独立的 MapFusion/离线模块，不属于本稳定候选版的 ExternalNav 闭环。

## 验证记录与边界

最近一次完整校准激励长 S 运行（run49）记录：统一后端位置 ATE `0.111 m`、运动 RMSE `0.117 m`、1 s RPE `0.141 m`、偏航 RMSE `0.503 deg`；FAST-LIO 参考 ATE `0.136 m`、偏航 RMSE `0.510 deg`。仿真 RTF 为 `0.683`，ExternalNav 曾出现约 `1.6 s` 的安全断流，随后进入保持/重定位流程；这说明闭环逻辑已存在，但当前性能还不能宣称“基本不断连”。

该运行没有优化器拒绝、回滚或 LIO fallback；显式重定位事务成功提交（candidate 2，epoch 约 `325.9`，reset `1`）。在线标定收到并接受过候选，但本次航线的实际偏航激励不足以锁定标定结果，因此 `calibration_apply_locked_values=false` 必须保持不变。

已通过的仓库级验证包括 `multi_slam_uav_sim` 77 项、`uf_backend_fusion` 160 项以及时间标定/运行指标针对性测试 22 项；发布前仍需在目标机器重新执行 ROS 2 构建和测试。

## 未包含内容

- D435i/RTAB-Map 五源因子尚未合并；
- D435i 点云与 MID360 点云的统一概率体素地图尚未进入在线后端；
- 原生伪距级 GNSS、光度/重投影级视觉因子、完整协方差传播和最终 Ultra-Fusion 论文级滑窗仍未完成；
- 在线外参锁定、跨会话自动重定位和实机 ExternalNav 验收不属于本版本承诺。
