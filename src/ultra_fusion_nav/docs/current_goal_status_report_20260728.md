# Ultra-Fusion 统一滑窗推进报告

日期：2026-07-28
工作区：`/home/zyc/multi-slam-github-staging`
分支：`feature/ultra-fusion-stage3`
当前远端基线：`3831ee3 feat: consume native FAST-LIO factors in unified backend`

## 1. 当前结论

LiDAR、FCU IMU、GNSS/BDS 和下视光流已经作为独立观测进入同一个
`p/R/v/b_a/b_g` 固定滞后窗口。当前名义 rosbag 和普通测试地图固定矩形航线
均可完整运行，未使用 Gazebo 真值、MAVROS local position 或飞控融合位置作为
估计器反馈。

这已经是可运行的第一版紧耦合后端，但还不能称为完整复现 Ultra-Fusion：

- LiDAR 使用 FAST-LIO 原生点到面对应点，在后端当前状态处重线性化；
- FCU `/mavros/imu/data_raw` 只用于一次启动偏置估计和相邻 LiDAR 关键帧间预积分；
- GNSS 和光流是独立因子，因子开关、权重和协方差膨胀由 ReliabilityScheduler 控制；
- 后端使用 SO(3) 右扰动、解析 IMU Jacobian、Huber 点面核和 LM 接受/拒绝；
- 仍缺少在线时空标定、视觉因子、重定位、可靠的输出边缘协方差和完整退化矩阵。

## 2. IMU 数据所有权

飞控已经负责驱动、标定、单位换算和 MAVLink/MAVROS 传输，因此项目不从
ADC 或传感器驱动层重新计算 IMU。估计器边界是：

```text
ArduPilot HIGHRES_IMU
  -> /mavros/imu/data_raw
  -> /sensors/imu
  -> 一次静止短窗 b_a/b_g 初始化
  -> 相邻 LiDAR 关键帧间预积分
  -> 统一滑窗 IMUFactor
```

飞控 EKF 的 local position、GNSS/光流融合位置和 Gazebo truth 不进入窗口。
否则同一 GNSS/光流信息会先被飞控融合，再被统一后端重复计数。

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

第三次运行目录：`/tmp/uf_manifold_fullcov_nominal_20260728_v3`

| 指标 | 第三次固定航线 |
|---|---:|
| 统一后端匹配位姿 | 994 |
| 统一后端 ATE RMSE | 0.073166 m |
| 统一后端 ATE 最大值 | 0.253387 m |
| 统一后端 RPE 平移 RMSE | 0.028589 m |
| 统一后端 RPE 旋转 RMSE | 0.414357 deg |
| FAST-LIO 独立位置 RMSE | 0.032462 m |
| FAST-LIO 偏航 RMSE | 0.096896 deg |
| FAST-LIO 偏航/FCU 陀螺相关系数 | 0.938056 |
| 估计 FCU IMU 链路延迟 | 20 ms |
| 原始云/注册云/IMU 时间戳回退 | 0 / 0 / 0 |
| 原生 LiDAR 因子校验 | 1255 / 1255，有效率 1.0 |
| 仿真实时率中位数 | 0.9851 |
| 统一后端输出频率 | 7.8877 Hz |
| ExternalNav 输出频率 | 7.8873 Hz |
| 后端求解耗时中位数/均值/最大值 | 19.882 / 20.160 / 62.002 ms |
| 原生因子工作队列最大深度/溢出 | 1 / 0 |

启动 IMU 偏置初始化接受 41 个飞控原始 IMU 样本、跨度 0.9082 s；累计形成
1025 个 IMU 因子，残差更新错误和优化错误均为 0。原生 LiDAR 因子来源全部为
`native_point_to_plane_relinearized`，没有 pose fallback。

无故障注入时 `active_fault_samples=0`，但 scheduler 仍主要处于 DEGRADED/RISK：
DEGRADED 956、RISK 272、RECOVERED 6、FAILSAFE 11、NORMAL 0。这不能直接解释成
LiDAR/IMU 硬件严重退化；当前状态机把低激励可观测性、光流不可用和启动/降落期
也计入全局健康状态。尤其光流独立评测仍为 `corr=0.851`、`NRMSE=0.506`、
`passed=false`，动态调度只启用了 712 次尝试中的 52 个光流因子。统一后端因此
保持稳定，但当前还不能宣称光流链路已经达到真实传感器等效精度。

第二次运行的统一后端 ATE 为 0.086976 m、RPE 平移 0.032597 m、RPE 旋转
0.456012 deg，说明第三次结果不是单次启动才勉强通过；样本仍只有两次，不用于
统计显著性结论。

## 9. 尚未完成

1. 光流独立精度门槛仍未通过；起飞/降落低质量段、比例和坐标映射需继续修正。
2. ArduPilot EKF3 已选择并实际使用统一 ExternalNav 的证据尚未取得。
3. 输出 pose/twist covariance 仍是固定值，尚未从窗口边缘协方差导出。
4. Schur 先验的 SO(3) 重线性化和秩阈值仍需加强；阻尼不应污染边缘先验。
5. 偏置变化较大时尚未触发重新预积分。
6. FAST-LIO 数据关联仍由其内部 IMU 预测间接影响，尚非完全统计独立前端。
7. 在线 LiDAR-IMU、LiDAR-body/GNSS/光流的时空标定尚未实现。
8. RGB-D/视觉因子、静态关键帧地图和重定位尚未接入统一窗口。
9. 动态权重优于固定权重的退化矩阵尚未形成有统计意义的结果。

## 10. 下一执行顺序

1. 修正光流独立精度，先把名义平移段评测通过，再增加其融合权重或覆盖率。
2. 配置并验证 ArduPilot EKF3 ExternalNav source，检查 innovation 和 source status，
   但不把飞控融合位置反馈给统一后端。
3. 从滑窗 Hessian/Schur 补空间导出在线 pose/twist covariance。
4. 加强 SO(3) 边缘先验和偏置阈值重预积分，再做多次名义路线统计。
5. 名义链路稳定后再做固定/动态权重消融；故障注入优先级保持靠后。
