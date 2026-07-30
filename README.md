# 多传感器无人机 SLAM 仿真：快速配置版

本页用于新电脑直接复现。除第一次启动 Ubuntu 时输入用户名 `zyc`、鱼香 ROS 菜单选择“ROS 2 Humble 桌面版”外，下面命令均可原样复制，不需要修改路径。

零基础解释、组件作用和排错见 [详细配置教程](README_详细配置教程.md)。

## 1. 安装内容

| 组件 | 用途 | 安装方式 |
|---|---|---|
| WSL2 Ubuntu-22.04 | Windows 上的 Linux 运行环境 | PowerShell 安装到 D 盘 |
| ROS 2 Humble | 节点、话题、TF 和工具 | 鱼香 ROS 一键安装 |
| Gazebo Sim Harmonic | 三维场景、物理和传感器 | 项目脚本从 OSRF 官方源安装 |
| MAVROS2 | ROS 2 与 ArduPilot 的通信桥 | 项目脚本安装 Humble 软件包和 GeographicLib 数据 |
| ArduPilot Copter SITL | APM 飞控仿真 | 项目脚本下载固定版本并编译 |
| ArduPilot Gazebo | 飞控与 Gazebo 的动力学接口 | 项目脚本下载固定版本并编译 |
| D435i 与 MID360 | RGB-D、IMU 与三维激光雷达仿真 | 已包含项目专用模型与桥接源码 |
| FAST-LIO 地图包 | 激光里程计、注册点云与栅格地图 | 项目脚本下载 FAST-LIO、Livox 驱动和 SDK 并编译 |
| 默认仿真地图 | 简单场景、隧道和 ArduPilot 仓库 | 已包含在本仓库 |
| 可选大型地图 | Clearpath 场景与城市地形 | 单独一条脚本按需下载，不上传 GitHub |

## 2. Windows 安装 WSL2 到 D 盘

以管理员身份打开 PowerShell，复制执行：

```powershell
wsl --update
wsl --install -d Ubuntu-22.04 --location D:\WSL\Ubuntu-22.04
```

安装完成后重启电脑。第一次打开 Ubuntu-22.04 时，Linux 用户名输入：

```text
zyc
```

以后从 PowerShell 进入 Ubuntu：

```powershell
wsl -d Ubuntu-22.04
```

下面所有 `bash` 命令都在 Ubuntu 终端执行。

## 3. 安装 ROS 2 Humble

在 Ubuntu 终端复制执行：

```bash
sudo apt update
sudo apt install -y wget
wget http://fishros.com/install -O fishros && . fishros
```

在鱼香 ROS 菜单中选择“安装 ROS” -> “ROS 2” -> “Humble” -> “桌面版”。完成后关闭当前 Ubuntu 终端，再重新进入：

```powershell
wsl -d Ubuntu-22.04
```

## 4. 一键安装全部仿真组件

回到 Ubuntu 终端，整段复制执行：

```bash
sudo apt update
sudo apt install -y git
mkdir -p "$HOME/projects"
cd "$HOME/projects"
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git
cd "$HOME/projects/multi-slam-simulation"
bash tools/setup_ubuntu.sh
```

该脚本会自动完成以下工作：

- 安装 Gazebo Harmonic、MAVROS2、RGB-D、RViz、PCL 与编译依赖；
- 下载并编译固定版本 ArduPilot Copter SITL；
- 下载并编译 ArduPilot Gazebo 插件；
- 下载并安装 Livox-SDK2；
- 下载并编译 Livox ROS Driver 2 与 FAST-LIO；
- 编译本仓库的三个 ROS 2 包并执行仓库检查。

外部大型源码默认保存在 `$HOME/ardupilot`、`$HOME/ardupilot_gazebo` 和 `$HOME/multi-slam-deps`，不会被提交到本仓库。

## 5. 启动仿真

打开第 1 个 Ubuntu 终端，启动 Gazebo、APM、MAVROS2、D435i、MID360 和光流：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_sim_with_flow.sh
```

打开第 2 个 Ubuntu 终端，启动自动起飞和矩形飞行状态机：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_rectangle_state_machine.sh
```

打开第 3 个 Ubuntu 终端，启动 FAST-LIO 建图与 RViz：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_fastlio_mapping.sh
```

## 6. 查看 RGB-D

彩色图：

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在下拉框选择 `/front/d435i/color/image_raw`。

深度图再打开一个终端执行相同命令，并选择 `/front/d435i/depth/image_rect_raw`。

FAST-LIO 的 RViz 会由第 3 个终端自动打开，正常时可以看到 `/cloud_registered`、轨迹和 `/fastlio_occupancy_grid`。

## 7. 下载可选大型地图

默认仿真和 FAST-LIO 不需要本节。需要 Clearpath 仓库、办公室、施工场景或城市地形时执行：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/fetch_optional_worlds.sh
```

下载后可直接启动示例：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh clearpath_warehouse
```

## 8. 快速检查

```bash
source /opt/ros/humble/setup.bash
source "$HOME/projects/multi-slam-simulation/install/setup.bash"
ros2 topic echo --once /mavros/state
ros2 topic hz /front/d435i/color/image_raw
ros2 topic hz /sim/mid360/points_raw
ros2 topic hz /cloud_registered
```

光流矩形飞行期间量化检查 FAST-LIO 漂移、偏航/IMU耦合和点云突变：

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/analyze_slam_drift.py --duration 120
```

## 9. D435i RGB-D 视觉 SLAM（Draft）

新增 D435i-only headless 视觉 SLAM 入口，默认使用 C++ RGB-D bridge 和
RTAB-Map `feature_aligned` 配置，不改变原完整多传感器仿真的默认入口：

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select d435i_rgbd_bridge_cpp multi_slam_uav_sim
source install/setup.bash
RTABMAP_PROFILE=feature_aligned D435I_WORLD=textured \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_headless.sh
```

详细说明、当前状态和性能口径：

- [D435i 视觉 SLAM 使用说明](docs/D435I_VISUAL_SLAM_README.md)
- [D435i 视觉 SLAM 状态](docs/D435I_VISUAL_SLAM_STATUS.md)
- [D435i 视觉 SLAM Benchmark](docs/D435I_VISUAL_SLAM_BENCHMARK.md)

## 10. 进一步阅读

- [详细配置教程](README_详细配置教程.md)
- [安装与依赖说明](docs/INSTALL.md)
- [运行与排错](docs/RUNNING.md)
- [系统架构](docs/ARCHITECTURE.md)
- [场景说明](docs/WORLDS.md)
- [GitHub 打包原则](docs/PACKAGING.md)

主仿真包采用 Apache-2.0；外部依赖遵循各自上游许可证。
