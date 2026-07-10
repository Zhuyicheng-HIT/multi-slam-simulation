# 多传感器无人机 SLAM 仿真：详细配置教程

本文面向第一次复现本项目的组员，从环境准备开始说明 Gazebo、ArduPilot、ROS 2、D435i、MID360 与 FAST-LIO 如何协同工作。只想快速启动时请阅读 [快速启动版](README.md)。

## 1. 项目目标

项目在 Gazebo 中模拟一架搭载多种传感器的无人机，并通过 ArduPilot SITL 运行接近真实飞控的软件栈：

- D435i 提供彩色图、深度图、相机内参和 IMU 接口。
- MID360 提供三维点云，供 FAST-LIO 里程计与建图使用。
- 下视光流与距离数据可作为无 GPS 场景的飞控输入。
- MAVROS 将 ArduPilot 状态和本地位姿接入 ROS 2。
- 飞行状态机在本地位姿有效，并且 GPS 或光流任一满足条件时进入起飞流程。

Gazebo 的真值位姿只用于明确标注的仿真诊断，不替代飞控状态。系统数据流见 [架构说明](docs/ARCHITECTURE.md)。

## 2. 为什么不把所有软件放进仓库

本仓库采用“项目源码入库、公开上游依赖按版本下载、编译结果本地生成”的原则。以下内容不提交：

- ArduPilot 源码与 SITL 编译结果；
- Gazebo 安装目录；
- ArduPilot Gazebo 插件及其编译目录；
- FAST-LIO、Livox ROS Driver 2、Livox-SDK2 的上游源码副本；
- `build/`、`install/`、`log/`、rosbag、地图和运行日志。

这样可以避免仓库巨大、二进制与系统不兼容、上游许可证混杂和绝对路径失效。固定版本记录在 [安装说明](docs/INSTALL.md) 与 `dependencies.repos` 中。

## 3. 推荐环境

已验证的主要环境：

```text
Ubuntu 22.04 / WSL2 Ubuntu-22.04
ROS 2 Humble
Gazebo Sim Harmonic
ArduPilot Copter SITL
Python 3.10
```

项目源码不依赖固定安装目录，但为了让所有命令可以原样复制，并降低历史脚本或遗漏路径造成的复现风险，本教程统一使用 Linux 用户名 `zyc` 和仓库路径 `$HOME/projects/multi-slam-simulation`。脚本内部仍使用包共享目录、脚本相对目录和环境变量查找资源。

## 4. Windows 用户准备 WSL2

原生 Ubuntu 22.04 用户可跳过本节。Windows 用户建议把 WSL2 的 Ubuntu 虚拟磁盘放在 D 盘，避免长期编译 ROS、Gazebo 和 ArduPilot 占用大量 C 盘空间。

### 4.1 准备 D 盘目录

以管理员身份打开 PowerShell，创建安装目录并更新 WSL：

```powershell
New-Item -ItemType Directory -Force D:\WSL\Ubuntu-22.04
wsl --update
```

建议至少为 D 盘预留 80 GB 可用空间。不要把 WSL 虚拟磁盘目录放在移动硬盘、网络盘或开启云端按需同步的目录中。

### 4.2 把 Ubuntu-22.04 安装到 D 盘

管理员 PowerShell 中执行：

```powershell
wsl --install -d Ubuntu-22.04 --location D:\WSL\Ubuntu-22.04
```

`--location` 是 WSL 官方提供的自定义安装目录参数。若旧版 WSL 提示不认识该参数，先执行 `wsl --update`，关闭 PowerShell 后重新以管理员身份打开再试。官方命令参考：<https://learn.microsoft.com/windows/wsl/basic-commands>

安装完成后按提示重启 Windows。

### 4.3 第一次启动并创建 `zyc` 用户

重启后在 PowerShell 中执行：

```powershell
wsl -d Ubuntu-22.04
```

第一次启动会要求创建 Linux 用户名和密码。用户名建议输入：

```text
zyc
```

密码输入时屏幕不会显示字符或星号，这是 Linux 的正常行为。完成后如果看到类似下面的提示符，说明已经进入 Ubuntu：

```text
zyc@电脑名:~$
```

虽然当前仓库已经使用相对路径和环境变量，但统一使用 `zyc` 可以减少历史配置、外部脚本或后续组员手工配置中遗漏绝对路径造成的问题。

以后进入该环境仍使用：

```powershell
wsl -d Ubuntu-22.04
```

后续 `bash` 代码块都在 Ubuntu/WSL 终端运行。WSL 图形界面需要 WSLg；可用下面命令确认显示变量存在：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

建议把仓库放在 WSL 的 Linux 文件系统中，例如 `$HOME/projects`。因为整个 Ubuntu 虚拟磁盘已经位于 D 盘，这些 Linux 路径的数据也会保存在 D 盘。不要把 ROS 工作空间直接放在 `/mnt/c` 或 `/mnt/d` 下编译，以免跨文件系统访问明显变慢。

## 5. 安装 ROS 2 Humble

本项目建议使用鱼香 ROS 一键安装器配置 ROS 2 Humble。在 Ubuntu/WSL 终端执行：

```bash
sudo apt update
sudo apt install -y wget
wget http://fishros.com/install -O fishros && . fishros
```

进入交互菜单后依次选择：

```text
安装 ROS
ROS 2
Humble
桌面版（Desktop）
```

菜单文字或编号可能随安装器更新，以“ROS 2 Humble 桌面版”为最终选择目标。安装完成后执行：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

如果鱼香 ROS 安装器无法访问或安装失败，可改用 ROS 2 官方 Ubuntu deb 安装说明：

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

ROS 2 安装完成后，再安装本项目常用工具与 ROS 包：

```bash
sudo apt update
sudo apt install -y \
  git curl wget cmake build-essential python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-rqt-image-view ros-humble-rviz2 ros-humble-tf2-tools

sudo rosdep init 2>/dev/null || true
rosdep update
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

最后确认 ROS 2、MAVROS 和常用工具可用：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
ros2 pkg prefix mavros
```

## 6. 一键安装其余全部组件

ROS 2 Humble 安装完成后，推荐直接使用仓库脚本安装 Gazebo、MAVROS2、APM、插件和 FAST-LIO 地图依赖。下面命令可以整段复制：

```bash
sudo apt update
sudo apt install -y git
mkdir -p "$HOME/projects"
cd "$HOME/projects"
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git
cd "$HOME/projects/multi-slam-simulation"
bash tools/setup_ubuntu.sh
```

脚本按固定提交号下载外部项目并自动编译，默认目录如下：

```text
$HOME/ardupilot
$HOME/ardupilot_gazebo
$HOME/multi-slam-deps/Livox-SDK2
$HOME/multi-slam-deps/mid360_ws
$HOME/projects/multi-slam-simulation
```

运行成功后可直接跳到第 11 节启动仿真。第 6.1 至第 10 节用于解释脚本做了什么，以及安装失败时逐项排查。

### 6.1 Gazebo Sim Harmonic

Gazebo 通过 OSRF 官方软件源安装，不复制其他电脑的安装目录。一键脚本会配置软件源并安装 `gz-harmonic`、C++ 开发包与 Python 消息/传输绑定。安装后确认：

```bash
gz sim --versions
```

Gazebo 负责场景、物理、传感器与图形渲染；它不负责飞行控制。飞控由 ArduPilot SITL 运行。

## 7. ArduPilot 下载与编译细节

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
cd "$HOME/ardupilot"
git checkout f9d619e26002d6aaa41643ee99c0ae0ee01e2247
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. "$HOME/.profile"
./waf configure --board sitl
./waf copter
```

ArduPilot 较大且包含子模块，因此只记录提交号，不纳入本仓库。

## 8. ArduPilot Gazebo 插件编译细节

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$HOME/ardupilot_gazebo"
cd "$HOME/ardupilot_gazebo"
git checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
sudo apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgz-transport13-dev libgz-msgs10-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
```

默认位置是 `$HOME/ardupilot` 与 `$HOME/ardupilot_gazebo`，与启动脚本的默认查找位置一致，无需设置环境变量。

## 9. 主仓库编译细节

```bash
mkdir -p "$HOME/projects"
cd "$HOME/projects"
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git
cd multi-slam-simulation

source /opt/ros/humble/setup.bash
python3 -m pip install --user -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
colcon build --symlink-install
source install/setup.bash
python3 tools/verify_repository.py
```

每次新开终端都要重新执行：

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

不要把本机实际路径写回仓库文件。需要改变外部组件位置时使用环境变量。

## 10. 准备 FAST-LIO 与 Livox 依赖

FAST-LIO 是可选组件；只验证飞控或 RGB-D 时可以暂不安装。创建独立外部工作空间：

```bash
mkdir -p "$HOME/multi-slam-deps/mid360_ws/src"
cd "$HOME/multi-slam-deps/mid360_ws"
vcs import --recursive src < "$HOME/projects/multi-slam-simulation/dependencies.repos"
```

安装 Livox-SDK2：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git \
  "$HOME/multi-slam-deps/Livox-SDK2"
cd "$HOME/multi-slam-deps/Livox-SDK2"
git checkout f5d9375f84efe2b15bc0a052d3e18482ed13adf4
cmake -S . -B build
cmake --build build -j"$(nproc)"
sudo cmake --install build
```

编译 Livox ROS Driver 2 与 FAST-LIO：

```bash
cd "$HOME/multi-slam-deps/mid360_ws/src/livox_ros_driver2"
./build.sh humble
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
```

外部工作空间固定放在 `$HOME/multi-slam-deps/mid360_ws`，与下载工具和建图脚本的默认位置一致，无需修改 `LIDAR_WS`。

## 11. 终端分工与完整启动

### 11.1 终端 1：仿真、飞控与传感器

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh
```

这个终端统一管理 Gazebo、ArduPilot SITL、MAVROS、传感器桥接与光流诊断。不要再启动第二套完整仿真，否则端口和话题会冲突。

### 11.2 终端 2：飞行状态机

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

默认 `NAVIGATION_SOURCE=auto`：本地位姿有效后，只要 GPS 或新鲜光流任一可用，状态机即可继续。`PREFLIGHT_WAIT_S` 是最长等待超时，不是固定休眠。

强制 GPS：

```bash
NAVIGATION_SOURCE=gps \
  bash "$HOME/projects/multi-slam-simulation/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh"
```

强制光流：

```bash
NAVIGATION_SOURCE=optical_flow FLOW_MIN_QUALITY=0 \
  bash "$HOME/projects/multi-slam-simulation/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh"
```

真正的解锁时间还受 EKF、传感器健康、位置估计和 ArduPilot 安全检查影响。不要通过关闭全部飞控安全检查来换取表面上的快速起飞。

### 11.3 终端 3：FAST-LIO 建图

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
```

脚本会加载外部 FAST-LIO 工作空间，并启动项目自己的可靠点云与栅格处理节点。

## 12. D435i RGB-D 可视化

彩色图和深度图建议分别打开一个 `rqt_image_view`，因为工具可能记住上次话题：

```bash
ros2 run rqt_image_view rqt_image_view
# 选择 /front/d435i/color/image_raw
```

```bash
ros2 run rqt_image_view rqt_image_view
# 选择 /front/d435i/depth/image_rect_raw
```

检查发布频率与 TF：

```bash
ros2 topic hz /front/d435i/color/image_raw
ros2 topic hz /front/d435i/depth/image_rect_raw
ros2 topic echo --once /front/d435i/color/camera_info
ros2 run tf2_ros tf2_echo base_link front_d435i_color_optical_frame
```

若窗口空白，先确认终端 1 没有报桥接错误，再在下拉框中手动选择正确话题；图像使用传感器数据 QoS，不应使用可靠传输强制订阅。

## 13. MID360 与 FAST-LIO 可视化

先检查原始点云：

```bash
ros2 topic hz /sim/mid360/points_raw
ros2 topic info -v /sim/mid360/points_raw
```

再检查 FAST-LIO：

```bash
ros2 topic list | grep -E 'cloud_registered|Odometry|path|grid'
ros2 topic hz /cloud_registered
ros2 topic echo --once /Odometry
```

RViz 中固定坐标系和显示项以实际 FAST-LIO 配置为准。通常至少添加注册点云、轨迹或里程计；若原始点云正常但没有 `/cloud_registered`，重点检查 `LIDAR_WS`、Livox 消息类型和 FAST-LIO 参数文件。

## 14. 非 GPS 光流模式

终端 1 改用向 ArduPilot 注入光流与距离数据的启动脚本：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_nongps_flow.sh
```

然后在终端 2 使用 `NAVIGATION_SOURCE=optical_flow`。诊断光流模式与飞控注入模式用途不同，不能仅凭图像流存在就断定 EKF 已接受光流。

## 15. 常用诊断

```bash
ros2 topic echo --once /mavros/state
ros2 topic list | sort
ros2 topic hz /sim/mid360/points_raw
ros2 topic hz /front/d435i/color/image_raw
ros2 run tf2_ros tf2_echo base_link front_d435i_color_optical_frame
```

常见现象：

- Gazebo 有画面但无人机不能解锁：检查 SITL 控制台、MAVROS 连接、EKF 和传感器健康。
- RGB-D 话题存在但看不到图：手动选择话题，并检查发布者与订阅者 QoS。
- FAST-LIO 无地图：先确认 `/sim/mid360/points_raw` 有频率，再检查外部工作空间是否 source。
- 脚本找不到 ArduPilot：设置 `ARDUPILOT_DIR` 与 `ARDUPILOT_GAZEBO_DIR`。
- 从其他目录启动失败：先 `source install/setup.bash`，不要依赖当前工作目录碰巧正确。

更多逐项命令见 [运行说明](docs/RUNNING.md)。

## 16. 可选场景

仓库提供简单测试、隧道、仓库、办公室、施工环境及可选城市地形入口。场景名称和启动命令见 [场景说明](docs/WORLDS.md)。部分场景依赖 Clearpath Simulator 或 Gazebo Terrain Generator，这些大型上游项目只在需要时下载到 `external/` 或 `MULTI_SLAM_EXTERNAL_DIR` 指定的位置。

## 17. 发布前检查

```bash
cd "$HOME/projects/multi-slam-simulation"
python3 tools/verify_repository.py
git status --short
```

检查器会阻止常见编译目录、日志、大文件、个人绝对路径和绝对符号链接进入仓库。完整原则见 [打包说明](docs/PACKAGING.md)。
