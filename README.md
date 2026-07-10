# 多传感器无人机 SLAM 仿真：快速启动版

本仓库提供 ROS 2 Humble、Gazebo Sim Harmonic 与 ArduPilot SITL 联合仿真，包含 D435i RGB-D、MID360 激光雷达、光流、MAVROS 飞控接口和 FAST-LIO 建图适配。

本页只保留最短复现步骤。首次配置、软件作用、完整终端分工、可视化与排错请阅读 [详细配置教程](README_详细配置教程.md)。

## 1. 仓库包含什么

```text
src/multi_slam_uav_sim/      无人机、传感器、桥接、飞行和场景启动
src/multi_slam_worlds/       可复用 Gazebo 场景
src/mid360_reliable_mapper/  项目自研的点云筛选与栅格地图节点
docs/                        架构、安装、运行、场景和打包说明
tools/                       仓库检查工具
dependencies.repos           FAST-LIO 与 Livox ROS 驱动的固定版本
```

ArduPilot、Gazebo、ArduPilot Gazebo 插件、FAST-LIO、Livox 驱动及其编译产物都不打包进仓库，按照 [安装说明](docs/INSTALL.md) 下载。

## 2. 环境要求

- Ubuntu 22.04 或 WSL2 Ubuntu-22.04
- ROS 2 Humble
- Gazebo Sim Harmonic
- ArduPilot Copter SITL

外部依赖的完整安装命令见 [详细配置教程](README_详细配置教程.md)。

## 3. 克隆与编译

```bash
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git
cd multi-slam-simulation
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 tools/verify_repository.py
```

## 4. 启动仿真

终端 1：启动 Gazebo、ArduPilot SITL、MAVROS、D435i、MID360 和光流诊断。

```bash
cd <仓库目录>
source /opt/ros/humble/setup.bash
source install/setup.bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh
```

终端 2：启动矩形飞行状态机。状态机在本地位姿有效，并且 GPS 或新鲜光流任一可用时即可继续起飞准备。

```bash
cd <仓库目录>
source /opt/ros/humble/setup.bash
source install/setup.bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

终端 3（可选）：启动 FAST-LIO 与 RViz。

```bash
cd <仓库目录>
source /opt/ros/humble/setup.bash
source install/setup.bash
LIDAR_WS="$HOME/multi-slam-deps/mid360_ws" RVIZ=1 \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_fastlio_mapping.sh
```

## 5. 快速确认可视化

D435i 彩色图：

```bash
ros2 run rqt_image_view rqt_image_view
# 在下拉框选择 /front/d435i/color/image_raw
```

D435i 深度图：

```bash
ros2 run rqt_image_view rqt_image_view
# 在下拉框选择 /front/d435i/depth/image_rect_raw
```

FAST-LIO：使用上面的 `RVIZ=1` 命令，确认 RViz 中出现 `/cloud_registered`、轨迹与地图。详细检查命令见 [运行说明](docs/RUNNING.md)。

## 6. 进一步阅读

- [详细配置教程](README_详细配置教程.md)：从新系统到完整仿真的逐步说明
- [安装说明](docs/INSTALL.md)：依赖版本与下载命令
- [运行说明](docs/RUNNING.md)：各终端、RGB-D、FAST-LIO 和非 GPS 模式
- [系统架构](docs/ARCHITECTURE.md)：数据流、TF 与飞控边界
- [场景说明](docs/WORLDS.md)：可用世界及启动命令
- [打包原则](docs/PACKAGING.md)：纳入与排除内容

## 7. 许可证

主仿真包采用 Apache-2.0。`src/mid360_reliable_mapper` 保留其目录内声明的许可证；外部依赖继续遵循各自上游许可证。
