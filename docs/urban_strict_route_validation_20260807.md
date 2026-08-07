# 城市结构与统一后端严格航线验证

日期：2026-08-07

## 变更范围

- 主测试世界新增轻量静态拱门、短隧道、分段高墙走廊和非对称建筑立面；
- 删除沿航线重复排列的标识柱，保留少量不对称 LiDAR 地标；
- 长 S 航线在 4--6 m 间完成两次升降，默认 0.35 m/s、每约 2 m 停留；
- 进场和返场沿 S 曲线本身完成，不再直线穿越隧道侧墙；
- 航点误差、到达判断和任务完成只使用 `/fusion/unified/odom`；Gazebo 真值仅用于
  事后精度评估。
- 飞行中定位丢失时冻结当时的 FCU-local hold setpoint；该安全动作不进入估计器，
  不允许航线进度继续累加。

## 前端模式 A/B

实验性的 `FASTLIO_BACKEND_TRAJECTORY_FRONTEND=1` 在静止初始化后只提交约 124 个
统一状态，随后出现 FAST-LIO 等待后端去畸变轨迹、后端等待下一原生 LiDAR 因子的
循环等待。该模式不满足长航线连续性，不能作为稳定默认值。

稳定验证改用 `FASTLIO_BACKEND_TRAJECTORY_FRONTEND=0`：FAST-LIO 内部预测只服务于
去畸变和点面匹配，仍导出原生点面因子；统一后端拥有最终位姿、航线反馈和
backend-confirmed 地图插入。该边界不是论文最终形态，后端轨迹反向握手仍需修复。

## 一趟严格闭环结果

测试任务为一次完整 S 穿越，含起飞、标定八字、曲线进场、24.28 m 主 S、曲线
返场、降落和解除武装。运行期间没有切换为 Gazebo/FCU route feedback，也没有
LiDAR pose fallback。

| 指标 | 结果 |
|---|---:|
| 统一后端 ATE RMSE / P95 / max | 0.0489 / 0.0785 / 0.1048 m |
| 运动段 ATE RMSE | 0.0381 m |
| 1 s RPE RMSE / P95 | 0.0153 / 0.0289 m |
| 偏航 RMSE / P95 / max | 0.0768 / 0.1607 / 0.4024 deg |
| 统一后端输出率 / 最大源时间间隔 | 9.85 Hz / 0.280 s |
| ExternalNav 验证流输出率 / 最大间隔 | 20.00 Hz / 0.133 s |
| LiDAR / IMU / GNSS / 光流因子 | 3771 / 3770 / 3766 / 1261 |
| 光流因子接受率 | 1261 / 3770 = 33.4% |
| native LiDAR pose fallback | 0 |
| 后端平均求解耗时 | 53.4 ms |
| 图契约违规 / 优化异常 | 0 / 0 |

精度报告使用 `/sim/mid360/ground_truth_odom` 做时间关联和刚体对齐，但
`truth_used_by_estimator=false`；真值没有进入统一后端或状态机。

## 当前边界

- APM 位置内环仍由 EKF local state 执行 setpoint；本次将 ExternalNav 发送到隔离的
  `/fusion/validation/externalnav`，因此不是 EKF3 ExternalNav 控制闭环验收；
- 统一后端是外层航线唯一反馈源，若其漂移，状态机会产生错误物理航迹，而不会用
  Gazebo 真值修正；
- 仿真飞行成功不能替代实机外参、时间同步、真实噪声和无 GNSS 条件验证；
- `sensor_contract_monitor` 当前仍把未使用的 `/sensors/lidar/points` 报为缺失，实际
  LiDAR 主链为 `/fast_lio/native_lidar_factor`，该监控口径需后续收口。
