# LiDAR 点云稳定候选版

版本标识：`v0.1.1-lidar-pointcloud-audit`

这是四源融合候选版之上的 LiDAR 输入、机身剔除、点云评估和可视化修订版，供
D435i/RTAB-Map 合作者继续接入。它仍是仿真研究版本，不是实机放飞承诺。

## 本轮修订

- MID360 仿真安装固定为机体前方 `0.05 m`、上方 `0.10 m`，俯视 `10 deg`；
  SDF、三个 FAST-LIO 配置、C++ 直连桥和通用传感器配置使用同一外参。
- D435i 仿真安装收近为 `base_link -> front_d435i_link = (0.20, 0, 0.02) m`，
  保持正前方；Gazebo 模型、RGB-D 桥静态 TF 和传感器 launch 默认值一致。
- 默认 `direct_livox` 只允许 C++ Gazebo LaserScan -> Livox `CustomMsg` 桥拥有
  `/livox/lidar` 和 `/livox/imu`；Python 旧桥必须显式启用，避免两套 IMU 时间历史
  交错导致 FAST-LIO 清空缓存和大范围发散。
- 输入 watchdog 要求每个话题恰好一个发布者；短暂 DDS 图发现抖动连续 5 次才判定丢失。
- `tools/analyze_slam_drift.py` 现在同时审计 Livox 包点数、有限点、点时间偏移、
  时间回退/重复、机身剔除比例、注册点坐标上界、连续体素重叠、质心跳变和输入所有权。
  Gazebo 真值只作为评估对照，不会进入估计器。
- 静止或小平移短测不再因 XY 刚体对齐的任意奇异角制造假偏航；报告记录
  `yaw_alignment_basis` 和 `position_excitation_m`。
- S 航线两侧增加少量静态标识柱，补充固定俯视 FOV 下的环境几何；不改变飞行包络，
  不发布真值或控制输入。

## 复现与审计

公共启动命令保持不变：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_sim_with_flow.sh
bash tools/run_fastlio_mapping.sh
bash tools/run_s_curve_state_machine.sh
```

建议用下面命令采集 120 s 的点云/轨迹报告：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
cd "$HOME/projects/multi-slam-simulation"
source install/setup.bash
python3 tools/analyze_slam_drift.py \
  --duration 120 --wall-timeout 900 \
  --output /tmp/multi_slam_pointcloud_audit.json
```

一次补柱后的完整 S 航线证据（2026-08-07）如下：

| 指标 | 结果 | 解释 |
|---|---:|---|
| 位置 RMSE | `0.0336 m` | FAST-LIO 与 Gazebo 真值的评估对齐结果 |
| 最大位置误差 | `0.1146 m` | 没有出现大范围位姿漂移 |
| 偏航 RMSE | `0.206 deg` | 含 36 个有效偏航/IMU 耦合样本 |
| Livox 时间回退/重复 | `0 / 0` | 包、timebase、点时间偏移均无回退 |
| `/livox/lidar`、`/livox/imu` 发布者 | `1 / 1` | 输入所有权满足唯一约束 |
| 注册坐标绝对值 P99 / max | `21.25 / 21.29 m` | 未发生坐标爆炸 |
| Livox 包点数 P05 / median | `85 / 563` | 航线边缘仍有几何覆盖告警 |
| 注册点数 P05 | `73` | 与上项一致，不能当作稠密扫描 |
| RTF | `0.9994` | 算法时间使用 ROS 仿真时钟，算力只看墙钟 |

P05 低点数和质心跳变只在同时满足“低体素重叠 + 大坐标质心跳变”时升级为失败；
单独出现时是覆盖告警。当前结果的位姿、坐标范围、时间戳和唯一输入均通过，说明
此前“整张地图向一侧漂移”的主要诱因是重复 Livox 桥/交错时间历史，而不是一个被
RMSE 自我掩盖的上万米地图。航线边缘覆盖仍应作为视觉/地图融合前的已知限制处理。

## 视觉协作者接口

- 使用 `/cloud_registered` 或 `/fastlio_denoised_map` 作为 LiDAR 算法输出，读取其
  `camera_init` frame 和 ROS header 时间；不要订阅 `/sim/mid360/cloud_registered`
  或 `/sim/mid360/ground_truth_odom` 作为估计输入。
- MID360 机体外参当前为 `T_body_lidar=[0.05, 0, 0.10] m`、`R_body_lidar=Ry(+10 deg)`；
  真实机仍以 Fast-Calib 测量值覆盖，不把仿真值硬写入实机。
- D435i 保留彩色、深度和静态 TF 接口，默认不启用派生点云；视觉前端应将 RTAB-Map
  相对位姿/几何因子按自己的时间戳和 `D_V` 送入统一后端，不发布第二个 ExternalNav。
- 双点云拼接仍属于独立 MapFusion 层：先做 SE(3) 外参/时间对齐，再做来源标记、
  置信度和冲突处理；本版本只保证 LiDAR 输出可审计，未宣称在线概率体素融合完成。

## 已知边界

- 仿真 GPU LiDAR 的射线命中分布不等同于真实 MID360 的回波统计；P05 覆盖指标要在
  rosbag2 和实机数据上重新标定。
- `truth_scan_pose_fallback` 只用于 Gazebo 评估注册点云，不能被后端或 ExternalNav 消费。
- FAST-LIO 前端和统一后端的原生点面因子接口仍需在长期视觉融合试验中继续验证；本版本
  不是 Ultra-Fusion 论文级五源紧耦合最终版。
