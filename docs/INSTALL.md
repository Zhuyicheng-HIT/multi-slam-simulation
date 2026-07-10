# 安装与依赖说明

已验证平台为 Ubuntu 22.04 或 WSL2 Ubuntu-22.04。团队推荐 Linux 用户名 `zyc`，仓库统一放在 `$HOME/projects/multi-slam-simulation`。以下命令无需修改路径。

## 1. 安装 ROS 2 Humble

Ubuntu/WSL 终端执行：

```bash
sudo apt update
sudo apt install -y wget
wget http://fishros.com/install -O fishros && . fishros
```

在菜单中选择 ROS 2 Humble 桌面版。安装完成后重新打开 Ubuntu 终端，确认：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

鱼香 ROS 无法使用时，可按 [ROS 2 官方 Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)安装 Humble Desktop。

## 2. 推荐的一键安装

```bash
sudo apt update
sudo apt install -y git
mkdir -p "$HOME/projects"
cd "$HOME/projects"
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git
cd "$HOME/projects/multi-slam-simulation"
bash tools/setup_ubuntu.sh
```

脚本会自动安装和编译：

- Gazebo Sim Harmonic、Python Gazebo 消息与传输绑定；
- MAVROS2、GeographicLib 数据、RGB-D、RViz、TF 与 PCL 依赖；
- ArduPilot Copter SITL；
- ArduPilot Gazebo 插件；
- Livox-SDK2、Livox ROS Driver 2 与 FAST-LIO；
- 本仓库的 `multi_slam_uav_sim`、`multi_slam_worlds` 和 `mid360_reliable_mapper`。

## 3. 固定版本

| 外部项目 | 固定提交 |
|---|---|
| ArduPilot | `f9d619e26002d6aaa41643ee99c0ae0ee01e2247` |
| ArduPilot Gazebo | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` |
| Livox-SDK2 | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` |
| Livox ROS Driver 2 | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` |
| FAST-LIO ROS 2 | `a4743b095409588842a5b30ddfa27e29d2f99164` |

下载位置固定为：

```text
$HOME/ardupilot
$HOME/ardupilot_gazebo
$HOME/multi-slam-deps/Livox-SDK2
$HOME/multi-slam-deps/mid360_ws
```

这些大型上游源码和编译产物都在项目仓库外，不会上传到 GitHub。

## 4. 分组件检查

```bash
source /opt/ros/humble/setup.bash
gz sim --versions
ros2 pkg prefix mavros
test -x "$HOME/ardupilot/build/sitl/bin/arducopter"
test -f "$HOME/ardupilot_gazebo/build/libArduPilotPlugin.so"
test -f "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
test -f "$HOME/projects/multi-slam-simulation/install/setup.bash"
```

以上命令无报错，说明 ROS、Gazebo、MAVROS2、APM、Gazebo 插件、FAST-LIO 和主工作空间均已准备。

## 5. 只重新下载外部源码

一键脚本中断后，无需手工选择版本。回到仓库重新执行即可自动复用已有下载：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/setup_ubuntu.sh
```

仅下载、不编译时执行：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/fetch_external_sources.sh
```

## 6. 可选大型场景

默认场景和 FAST-LIO 建图不依赖大型地图仓库。需要 Clearpath 仓库、办公室、施工场景以及 Apple Park、Joshimath 地形时执行：

```bash
cd "$HOME/projects/multi-slam-simulation"
bash tools/fetch_optional_worlds.sh
```

脚本固定下载已验证版本：

| 场景来源 | 固定提交 |
|---|---|
| Clearpath Simulator | `25997cb564d65867d85de155233b95567e8724a3` |
| Gazebo Terrain Generator | `4946f4c8150633e4c1fb2ffe9a2ab4f495de9577` |

旧说明中的地形仓库地址已失效，现已改为实际包含本项目 Apple Park 与 Joshimath 示例的上游仓库。

## 7. 重新编译主仓库

源码修改后执行：

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys ament_python
colcon build --symlink-install
source install/setup.bash
python3 tools/verify_repository.py
```
