# Ultra-Fusion 统一滑窗推进报告

日期：2026-07-28
工作区：`$HOME/multi-slam-github-staging`
分支：`feature/ultra-fusion-stage3`
本轮起点：`d823dd4 feat: validate manifold multi-sensor fusion backend`

## 1. 当前结论

LiDAR、FCU IMU、GNSS/BDS 和下视光流已经作为独立观测进入同一个
`p/R/v/b_a/b_g` 固定滞后窗口。当前名义 rosbag 和普通测试地图固定矩形航线
均可完整运行，未使用 Gazebo 真值、MAVROS local position 或飞控融合位置作为
估计器反馈。

这已经是可运行的第一版紧耦合后端，但还不能称为完整复现 Ultra-Fusion：

- LiDAR 使用 FAST-LIO 原生点到面对应点，在后端当前状态处重线性化；
- FCU `/mavros/imu/data_raw` 是飞控已标定、已换算为 SI 单位的高频测量，只用于
  一次残余偏置初始化和相邻 LiDAR 关键帧间预积分；
- GNSS 和光流是独立因子，因子开关、权重和协方差膨胀由 ReliabilityScheduler 控制；
- 后端使用 SO(3) 右扰动、解析 IMU Jacobian、Huber 点面核和 LM 接受/拒绝；
- 仍缺少在线时空标定、视觉因子、重定位、可靠的输出边缘协方差和完整退化矩阵。

## 2. IMU 数据所有权

飞控已经负责驱动、温度/尺度标定、单位换算和 MAVLink/MAVROS 传输，因此项目
不是从 ADC 原始计数重新解算 IMU。`data_raw` 这个 ROS 话题名容易引起误解；在
当前链路中，它表示 HIGHRES_IMU 的瞬时加速度和角速度，而不是未标定电信号。
估计器边界是：

```text
ArduPilot HIGHRES_IMU
  -> /mavros/imu/data_raw
  -> /sensors/imu
  -> 一次静止短窗残余 b_a/b_g 初始化
  -> 相邻 LiDAR 关键帧间预积分
  -> 统一滑窗 IMUFactor
```

飞控 EKF 的 local position、GNSS/光流融合位置和 Gazebo truth 不进入窗口。
否则同一 GNSS/光流信息会先被飞控融合，再被统一后端重复计数。

飞控还提供 `/mavros/imu/data` 姿态。它可以用于首状态的重力方向和姿态初始化，
但不能未经来源审计就作为连续强姿态因子：该姿态可能已经包含磁罗盘、GNSS、
光流或 ExternalNav 的 EKF 修正，连续回灌会形成相关信息重复计算。当前版本保留
FAST-LIO 首帧姿态作为窗口先验；下一轮先验证 `/mavros/imu/data` 的 frame、时间戳
和 covariance，再只替换启动 roll/pitch 候选，yaw 仍保持 SLAM 地图的任意基准。

关键帧间预积分仍然需要保留。HIGHRES_IMU 没有直接给出两个关键帧之间的
`delta_R/delta_v/delta_p`、偏置 Jacobian 和 15 x 15 协方差；除非修改 ArduPilot
导出这些量，否则统一后端必须从飞控已处理的瞬时测量构造 IMUFactor。窗口中的
`b_a/b_g` 表示飞控标定后残留的慢变误差，不是重新做整套飞控 IMU 标定。

启动初始化只有在以下条件同时满足时才接受：样本数和时间跨度足够、平均角速度
低、角速度波动低、平均比力接近重力、比力波动低。运动中启动会记录拒绝原因，
使用更宽的零偏置先验继续运行，不强行把运动平均成偏置。

名义录包接受 58 个样本、跨度 0.914079 s，得到：

- `b_a = [-0.0045548, 0.0064357, 0.0008630] m/s^2`
- `b_g = [0.0006302, -0.0006480, -0.0006213] rad/s`

## 3. 与论文残差模型的对应

LiDAR 点到面残差保持论文式 (6)-(7) 的几何形式：

```text
r_i = n_i^T (R_wb (R_bl p_i^l + t_bl) + p_wb - q_i)
```

每个原始对应点保留 `p_i^l`、`n_i`、`q_i` 和 LiDAR-body 外参。后端在当前
窗口状态重新计算残差和 Jacobian，FAST-LIO 的完整后验 pose 不作为第二个
LiDAR 因子重复加入。

IMU 残差为标准 15 维预积分残差：

```text
r_imu = [r_p, r_v, r_R, r_ba, r_bg]
```

预积分现在传播完整 `15 x 15` 误差协方差，包含位置-速度、姿态-偏置及其后续
传播相关性。优化使用完整信息矩阵，不再只取 15 个对角元素。调度器的连续权重
作为整个因子的外部尺度，不改变协方差内部相关结构。

## 4. FAST-LIO 边界

FAST-LIO 仅保留以下前端职责：

1. 点云去畸变和局部地图管理；
2. 点到面数据关联；
3. 导出原始对应点、平面、外参、诊断 Hessian/gradient；
4. 每个处理扫描都发布触发包，无有效对应点时发布 `correspondences_valid=false`。

统一后端不订阅 `/lio/odom` 作为连续输入；原生因子包是关键帧时钟。打包补丁
基于 FAST-LIO 提交 `a4743b095409588842a5b30ddfa27e29d2f99164`，SHA-256：

```text
ba720761653fde576c36ff1ca5067d94259d8f64114d0d4696ecc81edc748429
```

`tools/apply_fast_lio_native_factor_patch.sh` 会先检查精确基线提交和干净工作区，
再执行补丁检查与应用。最终补丁已重新在干净临时克隆验证，四个目标文件哈希
与当前外部工作区完全一致。`jacobian` 仅是可选逐点调试负载；默认不发布时，
适配器根据原始点、平面、外参和导出的 Hessian/gradient 重建并校验 Jacobian。

## 5. 名义 rosbag 定量结果

输入：`/tmp/unified_flow_yaw_gate_nominal_20260727_v3/rosbag_inputs`
时长：90.6 s，1x 回放
当前输出：`/tmp/manifold_backend_nominal_full_imu_covariance_1x_20260728_v16`

| 指标 | Huber/LM 对角协方差 | 完整协方差 + FCU 启动初始化 |
|---|---:|---:|
| 输出关键帧 | 696 | 696 |
| IMU 因子 | 689 | 689 |
| 已知 IMU 间隙 | 6 | 6 |
| 优化错误 | 0 | 0 |
| ATE RMSE | 0.172594 m | 0.172503 m |
| RPE 平移 RMSE | 0.036244 m | 0.035381 m |
| RPE 旋转 RMSE | 0.474300 deg | 0.474317 deg |
| 平均求解耗时 | 17.147 ms | 16.570 ms |
| 最大求解耗时 | 28.823 ms | 25.504 ms |

小幅精度差异不足以证明统计显著提升。可确认的结论是：完整协方差没有导致
名义轨迹、关键帧覆盖和实时性回归，同时修正了因子白化模型。

后端 69 项测试通过，包含 SO(3) 更新、解析 Jacobian 数值对照、白噪声协方差
闭式解、SPD 检查、完整马氏距离、Huber 离群点、LM 回滚和 Schur 边缘化。

## 6. 动态权重当前证据边界

光流 `scale=2.0`、源时间 30-60 s 的回放中：

| 模式 | ATE RMSE | RPE 平移 | RPE 旋转 |
|---|---:|---:|---:|
| 固定权重 | 0.172625 m | 0.036764 m | 0.475121 deg |
| 动态权重 | 0.172577 m | 0.036544 m | 0.474370 deg |

动态模式将 478 次光流尝试中的启用数降到 60，固定模式启用 378，证明调度器
确实改变了因子准入；但 LiDAR/GNSS 在该序列过强，轨迹改善只有亚毫米级，不能
据此宣称“动态权重显著优于固定权重”。需要普通测试地图上的 LiDAR/GNSS 并发
退化或可控回放矩阵继续验证。

## 7. 已修复的 ExternalNav 接口问题

统一后端输出 frame 为 `camera_init/body`。ExternalNav 门控只校验并转发，不做
坐标变换，原 launch 却期待 `map/base_link`，会拒绝所有统一后端输出。当前已将
统一后端 launch 的门控契约改为 `camera_init/body`；GPS/光流独立基线仍保持
自己的 `map/base_link` 配置。

固定航线已确认 `/mavros/odometry/out` 持续收到约 7.89 Hz 的统一后端输出，说明
门控和 MAVROS 传输链路已打通。该结果仍不能证明 ArduPilot EKF 已把 ExternalNav
选为控制来源；本轮矩形飞行仍由现有 GPS 导航条件触发，后续必须单独检查 EKF3
source set、innovation 和状态标志。

## 8. 普通测试地图固定航线

世界：`simple_apm_rgbd_mid360`。采用无界面仿真，关闭本轮不使用的 D435 点云桥，
不注入故障；光流保持 100 x 100 原生输入，FAST-LIO 原生因子和统一后端同时运行。
第一次完整运行暴露并修复了“可选 debug Jacobian 被误判为必需”的接口问题，之后
两次 125 s 固定矩形航线均完成，所有主进程退出码为 0。

完整协方差基线目录：`/tmp/uf_manifold_fullcov_nominal_20260728_v3`。修正光流
坐标、旋转补偿、按速度恢复门和仿真传感器离线尺度后，最终目录为
`/tmp/uf_flow_scaled_rate_gate_nominal_20260728_v10`。

| 指标 | 完整协方差基线 v3 | 光流修正 v10 |
|---|---:|---:|
| 统一后端匹配位姿 | 994 | 1004 |
| 统一后端 ATE RMSE | 0.073166 m | 0.055385 m |
| 统一后端 ATE 最大值 | 0.253387 m | 0.199914 m |
| 统一后端 RPE 平移 RMSE | 0.028589 m | 0.027267 m |
| 统一后端 RPE 旋转 RMSE | 0.414357 deg | 0.427888 deg |
| 原生 LiDAR 因子 | 1255，有效 1255 | 1035，无效 0 |
| IMU 因子/残差错误 | 1025 / 0 | 1032 / 0 |
| 光流因子启用/尝试 | 52 / 712 | 233 / 714 |
| 仿真实时率中位数 | 0.9851 | 0.9988 |
| 统一后端/ExternalNav 频率 | 7.8877 / 7.8873 Hz | 7.9449 / 7.9452 Hz |
| 后端求解均值/最大值 | 20.160 / 62.002 ms | 19.726 / 91.561 ms |
| 优化错误/工作队列溢出 | 0 / 0 | 0 / 0 |

启动 IMU 偏置初始化接受 41 个飞控 HIGHRES_IMU 样本、跨度 0.9082 s；累计形成
1025 个 IMU 因子，残差更新错误和优化错误均为 0。原生 LiDAR 因子来源全部为
`native_point_to_plane_relinearized`，没有 pose fallback。

无故障注入时 `active_fault_samples=0`。v10 scheduler 统计为 NORMAL 51、
DEGRADED 939、RISK 165、RECOVERED 95、FAILSAFE 0。它仍主要处于 DEGRADED，
实际原因是低激励可观测性和起降/偏航阶段光流降权。旧消息中的 vision 缺失值
不参与四源健康状态，但显示为退化 1.0 容易造成误读，已在后续四源收口中修正。

v10 光流全程独立评测为 `scale=0.931`、`corr=0.859`、`NRMSE=0.494`；剔除
`|yaw_rate| > 0.08 rad/s` 的平移段为 `scale=1.000`、`corr=0.875`、
`NRMSE=0.462`，两组都通过门槛。尺度 `0.683` 只作用于这个 Gazebo 100 x 100
相机桥的平移分量，陀螺积分不缩放；它由两次未缩放固定路线离线拟合，既不在线
读取真值，也不用于真实硬件。v8/v9 在未缩放条件下分别启用 385/716 和
330/717 个因子，证明按速度恢复门的覆盖率提升可重复；v10 是尺度修正后的单次
最终验证，尚不宣称轨迹改善具有统计显著性。

完整协方差基线 v2 的统一后端 ATE 为 0.086976 m、RPE 平移 0.032597 m、
RPE 旋转 0.456012 deg，说明 v3 不是单次启动才勉强通过；样本仍只有两次，
不用于统计显著性结论。

## 9. 尚未完成

1. `/mavros/imu/data` 启动姿态候选的 frame、时间戳、covariance 和上游融合来源
   尚未在线审计；当前不作为连续姿态因子。
2. ArduPilot EKF3 已选择并实际使用统一 ExternalNav 的证据尚未取得。
3. 输出 pose/twist covariance 仍是固定值，尚未从窗口边缘协方差导出。
4. Schur 先验的 SO(3) 重线性化和秩阈值仍需加强；阻尼不应污染边缘先验。
5. 偏置变化较大时尚未触发重新预积分。
6. FAST-LIO 数据关联仍由其内部 IMU 预测间接影响，尚非完全统计独立前端。
7. 在线 LiDAR-IMU、LiDAR-body/GNSS/光流的时空标定尚未实现。
8. RGB-D/视觉因子、静态关键帧地图和重定位尚未接入统一窗口。
9. 动态权重优于固定权重的退化矩阵尚未形成有统计意义的结果。

## 10. 四源配置收口与普通地图回归

四源运行现在显式限定为 LiDAR、FCU IMU、GNSS 和光流：

- `sensor_pipeline.launch.py enable_vision:=false` 不启动 depth/color 故障注入器；
- 合同监控只订阅并检查四个活动模态；
- ReliabilityScheduler 对未激活 vision 输出 `inactive_modality`、权重 0、因子关闭，
  不再把它显示成退化 1.0，也不会误开启未激活因子；
- 实验入口把实际 profile、活动模态和故障配置写入 `experiment_profile.txt`；
- 光流模型、尺度、转弯门和现有直达机载电脑仿真链路均未修改。

修改前基线为 `/tmp/uf_four_source_preclosure_20260728_v1`。修改后第一次运行
`/tmp/uf_four_source_closed_nominal_20260728_v1` 在飞行结束前后发生约 5115 s 的
ROS 时间跳变，与 WSL 暂停/恢复一致；该运行不进入精度统计。两次连续有效四源
运行和一个重新启用空闲 depth/color 节点的控制组如下：

| 指标 | 修改前单次基线 | 四源 v2 | 四源 v3 | 全节点控制 v4 |
|---|---:|---:|---:|---:|
| 统一 ATE RMSE | 0.057379 m | 0.138869 m | 0.131500 m | 0.160383 m |
| RPE 平移 RMSE | 0.031536 m | 0.026565 m | 0.023703 m | 0.026601 m |
| RPE 旋转 RMSE | 0.400916 deg | 0.420930 deg | 0.459151 deg | 0.443246 deg |
| FAST-LIO 位置 RMSE | 0.028118 m | 0.235451 m | 0.235434 m | 0.284004 m |
| 实时率中位数 | 0.9228 | 0.9093 | 0.9196 | 0.9134 |
| 融合输出频率 | 6.9764 Hz | 7.0695 Hz | 7.1800 Hz | 7.0878 Hz |
| 光流因子启用/尝试 | 307 / 665 | 168 / 662 | 177 / 667 | 197 / 669 |
| 优化错误/队列溢出 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| 时间戳回退 | 0 | 0 | 0 | 0 |

四源 v2/v3 的 ATE 中位数为 0.135184 m，实时率中位数为 0.9145，均完成起飞、
固定矩形航线和降落；原生 LiDAR 因子分别为 937/942，IMU 因子为 933/938，
无故障注入、无 FAILSAFE。光流全程和无转弯段的独立校验均通过。

本轮只验收配置边界、四源因子覆盖和运行稳定性，不宣称精度提升。关闭视觉节点
的两次结果优于重新启用空闲视觉节点的控制组，且三次 FAST-LIO 自身误差同步
增大，因此没有证据把相对历史最佳值的回退归因于四源收口。历史最佳 0.055385 m
继续保留，后续协方差迭代不得覆盖该参考值。

## 11. 下一执行顺序

1. 从滑窗 Hessian/Schur 补空间导出在线 pose/twist covariance，并验证正定性、
   有界性和退化时膨胀方向。
2. 配置并验证 ArduPilot EKF3 ExternalNav source，检查 innovation 和 source status，
   但不把飞控融合位置反馈给统一后端。
3. `/mavros/imu/data` 启动姿态、SO(3) 边缘先验、偏置阈值重预积分和故障矩阵
   保持后置，不挤占当前四源闭环主线。
