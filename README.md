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

## 5. 稳定启动入口

本节中的 `tools/*.sh` 是项目公共启动接口。后续可以调整内部节点、参数和
话题桥，但应保持这些命令可用。若关键架构调整确实无法兼容旧命令，必须在
同一个版本中更新本节和迁移说明，并在合并或发布前明确告知，不允许静默失效。

### 5.1 基础 Gazebo + FAST-LIO 可视化

打开第 1 个 Ubuntu 终端，启动 Gazebo、APM、MAVROS2、D435i、MID360 和光流：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_sim_with_flow.sh
```

该入口默认加载仅用于 SITL Iris 模型的滚转稳定参数，避免原生参数在当前
Gazebo plant 上产生约 7.5 Hz 的滚转极限环；不会修改真实飞控。需要复现
ArduPilot 原生参数进行 A/B 时使用 `WIPE_EEPROM=1
ENABLE_IRIS_ROLL_STABILITY_PROFILE=0 bash tools/run_sim_with_flow.sh`。旧工作区首次
切换到稳定 profile 时也应运行一次
`WIPE_EEPROM=1 bash tools/run_sim_with_flow.sh`，之后恢复普通启动命令；这是因为
SITL EEPROM 中已保存的参数优先于 defaults 文件。定量依据见
[`docs/sensor_payload_pid_stability_report.md`](docs/sensor_payload_pid_stability_report.md)。

需要同时打开 100x100 光流画面和跟踪矢量时，将上一条启动命令改为：

```bash
cd "$HOME/projects/multi-slam-simulation"
SHOW_FLOW_WINDOW=1 bash tools/run_sim_with_flow.sh
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

要飞保守的长 S 航线，只替换第 2 个终端的命令，其他终端不变：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_s_curve_state_machine.sh
```

### 5.2 四源统一后端长 S 自动验证

下面的单条命令自动启动无界面仿真、LiDAR 前端、统一后端、三圈长 S 航线、
定量记录和退出清理。它默认不让 APM EKF3 消费 ExternalNav，适合算法验证，
不等同于闭环飞控验收：

```bash
cd "$HOME/projects/multi-slam-simulation"
VALIDATION_ROUTE=s_curve \
S_CURVE_PASSES=3 \
ENABLE_RELIABILITY_RECORD=1 \
bash tools/run_unified_rectangle_validation.sh
```

验证运行期间，可在另一个终端打开统一轨迹和点云 RViz：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/run_unified_visualization.sh
```

若还要看光流窗口，在自动验证命令前增加 `SHOW_FLOW_WINDOW=1`：

```bash
cd "$HOME/projects/multi-slam-simulation"
SHOW_FLOW_WINDOW=1 \
VALIDATION_ROUTE=s_curve \
S_CURVE_PASSES=3 \
ENABLE_RELIABILITY_RECORD=1 \
bash tools/run_unified_rectangle_validation.sh
```

`run_unified_rectangle_validation.sh` 的文件名为兼容旧实验保留；通过
`VALIDATION_ROUTE=s_curve` 选择长 S，无需改脚本名。

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

## 9. 进一步阅读

- [四源融合稳定候选版与视觉协作接口](docs/RELEASE_FOUR_SOURCE_V1.md)
- [详细配置教程](README_详细配置教程.md)
- [安装与依赖说明](docs/INSTALL.md)
- [运行与排错](docs/RUNNING.md)
- [系统架构](docs/ARCHITECTURE.md)
- [场景说明](docs/WORLDS.md)
- [GitHub 打包原则](docs/PACKAGING.md)

主仿真包采用 Apache-2.0；外部依赖遵循各自上游许可证。
