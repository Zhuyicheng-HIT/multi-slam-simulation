# Multi-SLAM Simulation

ROS 2 Humble / Gazebo Harmonic / ArduPilot SITL 的无人机多源融合导航项目。
当前主基线为提交 `5f15ab0`，包含统一滑窗后端、FAST-LIO/MID360、GNSS、飞控 IMU、光流/MicoLink、D435i RGB-D、可靠性调度、动态点剔除、重定位组件、Gazebo 场景、RViz 配置和数据集回放可视化。

## 基线边界

五源统一后端的最终状态由统一滑窗发布：

- 飞控 IMU：`/mavros/imu/data_raw`
- MID360：FAST-LIO 负责点云解析、去畸变、匹配和点面残差；统一后端拥有最终状态
- GNSS/BDS：统一后端位置因子
- MTF-01P 光流：MicoLink-compatible `OpticalFlowRad` + `Range`
- D435i：RGB-D 视觉前端，支持重投影和 RGB-D direct 因子模式

算法约束：同一传感器同一时间窗只进入一次滑窗；允许按方向加权；光流只约束水平二维运动，测距用于尺度，不约束世界 Z；Gazebo 真值只用于评估或隔离的诊断航线控制，不进入估计器。

当前默认冻结项：五源统一后端、LiDAR 方向信息诊断、MicoLink 光流、RGB-D 视觉前端、EKF3 ExternalNav 适配、在线时间标定 shadow 诊断、机身包络剔除。RangeFacet、气压计 fallback、主动重定位触发和在线外参自动应用仍为实验开关或冻结状态。

回环和重定位组件已纳入源码和测试，但不能据此宣称已经形成稳定的闭环重定位能力。长隧道基线当前用于暴露方向退化和因子接管问题，不是精度通过样例。

## 环境配置

目标环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Sim Harmonic、ArduPilot Copter SITL、MAVROS2、Cyclone DDS、Livox-SDK2、Livox ROS Driver 2、FAST-LIO。

首次安装：

```bash
sudo apt update
sudo apt install -y git
cd "$HOME"
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git
cd multi-slam-simulation
bash tools/setup_ubuntu.sh
```

每个终端加载工作空间：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
source "$HOME/multi-slam-simulation/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

构建和测试：

```bash
cd "$HOME/multi-slam-simulation"
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

完整依赖默认安装在 `$HOME/ardupilot`、`$HOME/ardupilot_gazebo` 和 `$HOME/multi-slam-deps`，不提交到仓库。

## 仿真启动

### 基础仿真

启动 Gazebo、ArduPilot SITL、MAVROS2、D435i、MID360 和 C++ MicoLink 光流桥：

```bash
cd "$HOME/multi-slam-simulation"
HEADLESS=1 bash tools/run_sim_with_flow.sh
```

有桌面环境时可去掉 `HEADLESS=1`。需要光流诊断窗口时：

```bash
SHOW_FLOW_WINDOW=1 bash tools/run_sim_with_flow.sh
```

基础矩形航线：

```bash
bash tools/run_rectangle_state_machine.sh
```

单段直线后直接降落：

```bash
ROUTE_MODE=straight RECTANGLE_LENGTH_Y=48 RECTANGLE_SPEED=1.0 \
  bash tools/run_rectangle_state_machine.sh
```

### 当前统一后端验证

统一验证脚本会启动隔离的仿真、FAST-LIO、传感器管线、可靠性调度器、统一后端、误差观测器和资源观测器。默认关闭 EKF3 消费，适合算法观测验证：

```bash
cd "$HOME/multi-slam-simulation"
LOG_DIR="$PWD/logs/five_source_$(date +%Y%m%d_%H%M%S)" \
  VALIDATION_ENABLE_EXTERNALNAV_EKF3=0 \
  bash tools/run_unified_rectangle_validation.sh
```

测试完成后重点查看：

```text
logs/.../unified_accuracy.json
logs/.../unified_runtime_metrics.json
logs/.../validation_acceptance.json
logs/.../unified/online_backend.log
logs/.../sim/gz_livox_bridge.log
```

报告必须同时包含总体 3D/XY/Z RMSE、P95、最大值、末端误差，各传感器误差，以及 received/selected/accepted/rejected 和真实形成的 solver 因子数。

### 长隧道退化场景

仓库内已恢复：

```text
src/multi_slam_uav_sim/worlds/large_indoor_tunnel_apm_rgbd_mid360.sdf
src/multi_slam_uav_sim/config/large_tunnel_motion_params.yaml
```

直接进入 Gazebo：

```bash
cd "$HOME/multi-slam-simulation"
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$PWD/src/multi_slam_uav_sim:$PWD/src/multi_slam_uav_sim/models:$PWD/src/multi_slam_worlds/models"
gz sim -r "$PWD/src/multi_slam_uav_sim/worlds/large_indoor_tunnel_apm_rgbd_mid360.sdf"
```

当前直线基线测试：

```bash
WORLD="$PWD/src/multi_slam_uav_sim/worlds/large_indoor_tunnel_apm_rgbd_mid360.sdf" \
WORLD_NAME=large_indoor_tunnel \
VALIDATION_WORLD_NAME=large_indoor_tunnel \
VALIDATION_ROUTE_MODE=straight \
RECTANGLE_LENGTH_Y=48 \
RECTANGLE_SPEED=1.0 \
VALIDATION_ROUTE_FEEDBACK_SOURCE=gazebo_truth \
VALIDATION_LOCALIZATION_SAFETY_ENABLED=false \
VALIDATION_ENABLE_EXTERNALNAV_EKF3=0 \
  bash tools/run_unified_rectangle_validation.sh
```

该模式中 Gazebo 真值只控制诊断航线并用于评估，统一后端仍为 observer-only。桥接器必须收到当前世界的 `/world/large_indoor_tunnel/pose/info`，否则测试会拒绝产生无效结果。

## 动态目标和重定位

动态点过滤由 LiDAR 前端和历史地图策略共同处理，机身中心附近 `0.9 m x 0.9 m x 0.5 m` 的配置包络用于剔除自身回波。时间动态过滤默认关闭，只有显式启用才会建立 `/livox/lidar_raw -> /livox/lidar` 过滤链：

```bash
TEMPORAL_DYNAMIC_FILTER_ENABLED=true \
  bash tools/run_unified_rectangle_validation.sh
```

重定位、关键帧数据库、描述子、配准、多帧一致性和主动重定位代码位于 `src/ultra_fusion_nav/uf_relocalization`。这些组件必须通过专门验证脚本和日志证明有效，不能仅凭 RViz 观感宣称闭环完成。

## 可视化

### RViz

独立启动统一后端可视化：

```bash
bash tools/run_unified_visualization.sh
```

默认配置：`src/multi_slam_uav_sim/config/ultra_fusion_demo.rviz`。

主要显示统一轨迹、FAST-LIO 诊断轨迹、在线点云、稳定/动态/不确定点云和统一位姿。Gazebo 真值默认不显示为估计输入。

### 数据集回放页面

回放页面位于 `tools/replay_viewer`，支持完整时间轴拖动、相机画面、LiDAR 局部点云、实时增长的全局点云和轨迹；全局地图不会提前显示最终结果。

```bash
cd tools/replay_viewer
npm ci
npm run dev -- --host 0.0.0.0
```

从 Windows 浏览器访问 WSL 地址和开发服务器端口。数据集导出和验证：

```bash
python3 scripts/export_replay_assets.py --help
node scripts/verify_playback.mjs
```

无桌面服务器优先使用 headless 仿真；不要在没有显示环境时启动 RViz 或 Gazebo GUI。

## 话题和接口

| 功能 | 话题 |
|---|---|
| 统一位姿 | `/fusion/unified/odom` |
| ExternalNav 输出 | `/fusion/externalnav/odom` 或 `/mavros/odometry/out` |
| MID360 原始 Livox | `/livox/lidar` |
| FAST-LIO 点云 | `/cloud_registered` |
| 飞控 IMU | `/mavros/imu/data_raw` |
| GNSS | `/mavros/global_position/raw/fix` |
| MicoLink 光流 | `/sim/optical_flow/rad` |
| 光流测距 | `/sim/optical_flow/range` |
| RGB-D | `/front/d435i/color/image_raw`、`/front/d435i/aligned_depth_to_color/image_raw` |
| Gazebo 评估真值 | `/sim/mid360/ground_truth_odom` |

## 已知限制

- 当前长隧道直线基线可以完成起飞、48 m 航段和降落，但历史有效运行显示水平误差会在退化方向显著增长；这正是下一阶段分方向信息接管实验的输入，不是已通过的精度基线。
- 光流和 RGB-D 是否形成有效因子取决于纹理、时间同步、可靠性门控和视觉模式；`received` 不等于 `accepted`。
- EKF3 ExternalNav 是否消费必须由 MAVROS/EKF 日志和话题证据确认，不能由统一后端发布存在推断。
- 回环和重定位源码已存在，但稳定闭环能力尚未宣称完成。
- 所有历史实验报告中的精度只对其对应日志和配置负责，不自动代表当前主基线。

## 版本规则

当前主基线提交：`5f15ab0`。

修改算法前先保留可回退提交；每次实验记录日志路径、配置、构建测试结果和总体/分传感器指标。稳定版本才推送到远端主分支。
