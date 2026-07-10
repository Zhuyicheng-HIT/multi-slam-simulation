# 安装说明

已验证平台为 Ubuntu 22.04 或 WSL2 Ubuntu-22.04。命令默认在 Ubuntu 终端执行，项目本身不要求固定用户名或固定安装目录。

## 1. ROS 2 Humble

先按官方说明安装 ROS 2 Humble Desktop：

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

安装项目依赖：

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

## 2. Gazebo Sim Harmonic

使用 OSRF 官方二进制软件源安装，不要复制其他机器的 Gazebo 安装目录：

```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic python3-gz-msgs10 python3-gz-transport13
```

官方说明：<https://gazebosim.org/docs/harmonic/install_ubuntu/>

## 3. 可选的一键下载脚本

安装 ROS 2 与 Gazebo 后，可在仓库根目录执行脚本下载固定版本的外部源码：

```bash
tools/fetch_external_sources.sh
```

脚本只负责下载到仓库外或被忽略的目录，外部源码和编译结果不会加入 Git。

## 4. ArduPilot SITL

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

## 5. ArduPilot Gazebo 插件

该插件提供仿真接口以及本项目引用的基础 Iris 模型：

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$HOME/ardupilot_gazebo"
cd "$HOME/ardupilot_gazebo"
git checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
sudo apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgz-transport13-dev libgz-msgs10-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
```

运行脚本默认查找 `$HOME/ardupilot` 和 `$HOME/ardupilot_gazebo`。其他位置通过环境变量传入：

```bash
export ARDUPILOT_DIR="/你的路径/ardupilot"
export ARDUPILOT_GAZEBO_DIR="/你的路径/ardupilot_gazebo"
```

## 6. 克隆并编译本仓库

```bash
mkdir -p "$HOME/projects"
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git \
  "$HOME/projects/multi-slam-simulation"
export MULTI_SLAM_REPO="$HOME/projects/multi-slam-simulation"
cd "$MULTI_SLAM_REPO"
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 tools/verify_repository.py
```

## 7. 可选 FAST-LIO 工作空间

FAST-LIO、Livox ROS Driver 2 与 Livox-SDK2 保持为外部依赖。先导入固定版本源码：

```bash
mkdir -p "$HOME/multi-slam-deps/mid360_ws/src"
cd "$HOME/multi-slam-deps/mid360_ws"
export MULTI_SLAM_REPO="${MULTI_SLAM_REPO:-$HOME/projects/multi-slam-simulation}"
vcs import --recursive src < "$MULTI_SLAM_REPO/dependencies.repos"
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

编译 ROS 2 外部工作空间：

```bash
cd "$HOME/multi-slam-deps/mid360_ws/src/livox_ros_driver2"
./build.sh humble
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
```

上游说明：

- <https://github.com/Livox-SDK/Livox-SDK2>
- <https://github.com/Livox-SDK/livox_ros_driver2>
- <https://github.com/hku-mars/FAST_LIO>

启动建图时指定工作空间：

```bash
export LIDAR_WS="$HOME/multi-slam-deps/mid360_ws"
```

`mid360_reliable_mapper` 是本项目源码，随主仓库一起编译；只有 FAST-LIO 与 Livox 消息/驱动来自外部 overlay。

## 8. 可选大型场景仓库

默认无人机场景不依赖 Clearpath Simulator 和 Gazebo Terrain Generator。只有需要对应场景时才下载到被 Git 忽略的 `external/`：

```bash
export MULTI_SLAM_REPO="${MULTI_SLAM_REPO:-$HOME/projects/multi-slam-simulation}"
mkdir -p "$MULTI_SLAM_REPO/external"
git clone https://github.com/clearpathrobotics/clearpath_simulator.git \
  "$MULTI_SLAM_REPO/external/clearpath_simulator"
git clone https://github.com/fkromer/gazebo_terrain_generator.git \
  "$MULTI_SLAM_REPO/external/gazebo_terrain_generator"
```

放在其他位置时设置：

```bash
export MULTI_SLAM_EXTERNAL_DIR="/你的路径/external"
```
