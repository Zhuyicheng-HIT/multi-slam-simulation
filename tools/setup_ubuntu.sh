#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
DEPS_ROOT="$HOME/multi-slam-deps"
ARDUPILOT_DIR="$HOME/ardupilot"
ARDUPILOT_GAZEBO_DIR="$HOME/ardupilot_gazebo"
LIDAR_WS="$DEPS_ROOT/mid360_ws"
LIVOX_SDK2_DIR="$DEPS_ROOT/Livox-SDK2"

if [[ ! -f /etc/os-release ]]; then
  printf '无法识别当前 Ubuntu 版本。\n' >&2
  exit 1
fi
source /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  printf '本脚本只支持 Ubuntu 22.04，当前版本为 %s。\n' "${VERSION_ID:-unknown}" >&2
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  cat >&2 <<'EOF'
未找到 ROS 2 Humble。请先复制执行：

  sudo apt update
  sudo apt install -y wget
  wget http://fishros.com/install -O fishros && . fishros

在菜单中选择 ROS 2 Humble 桌面版，完成后重新运行本脚本。
EOF
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

printf '\n[1/8] 配置 Gazebo Harmonic 官方软件源...\n'
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release ca-certificates
sudo mkdir -p /usr/share/keyrings
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable %s main\n' \
  "$(dpkg --print-architecture)" "$(lsb_release -cs)" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

printf '\n[2/8] 安装 Gazebo、MAVROS2、ROS 工具和建图编译依赖...\n'
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git ninja-build pkg-config rapidjson-dev \
  python3-colcon-common-extensions python3-numpy python3-opencv \
  python3-pip python3-rosdep python3-vcstool python3-yaml \
  gz-harmonic libgz-msgs10-dev libgz-sim8-dev libgz-transport13-dev \
  python3-gz-msgs10 python3-gz-transport13 \
  libapr1-dev libeigen3-dev libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev libopencv-dev libpcl-dev \
  ros-humble-ament-cmake-auto ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-launch-ros \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-rqt-image-view ros-humble-rosidl-default-generators \
  ros-humble-rviz2 ros-humble-tf2-tools

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

set +u
source /opt/ros/humble/setup.bash
set -u
python3 -m pip install --user -r "$REPO_ROOT/requirements.txt"

printf '\n[3/8] 下载固定版本的 ArduPilot、Gazebo 插件和 FAST-LIO 地图依赖...\n'
DEPS_ROOT="$DEPS_ROOT" \
ARDUPILOT_DIR="$ARDUPILOT_DIR" \
ARDUPILOT_GAZEBO_DIR="$ARDUPILOT_GAZEBO_DIR" \
LIDAR_WS="$LIDAR_WS" \
LIVOX_SDK2_DIR="$LIVOX_SDK2_DIR" \
  "$REPO_ROOT/tools/fetch_external_sources.sh"

printf '\n[4/8] 编译 ArduPilot Copter SITL...\n'
cd "$ARDUPILOT_DIR"
Tools/environment_install/install-prereqs-ubuntu.sh -y
set +u
source "$HOME/.profile"
set -u
./waf configure --board sitl
./waf copter

printf '\n[5/8] 编译 ArduPilot Gazebo 插件...\n'
cmake -S "$ARDUPILOT_GAZEBO_DIR" -B "$ARDUPILOT_GAZEBO_DIR/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$ARDUPILOT_GAZEBO_DIR/build" -j"$(nproc)"

printf '\n[6/8] 编译并安装 Livox-SDK2...\n'
cmake -S "$LIVOX_SDK2_DIR" -B "$LIVOX_SDK2_DIR/build" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$LIVOX_SDK2_DIR/build" -j"$(nproc)"
sudo cmake --install "$LIVOX_SDK2_DIR/build"
sudo ldconfig

printf '\n[7/8] 编译 Livox ROS Driver 2 与 FAST-LIO...\n'
cp -f "$LIDAR_WS/src/livox_ros_driver2/package_ROS2.xml" \
  "$LIDAR_WS/src/livox_ros_driver2/package.xml"
rm -rf "$LIDAR_WS/src/livox_ros_driver2/launch"
cp -a "$LIDAR_WS/src/livox_ros_driver2/launch_ROS2" \
  "$LIDAR_WS/src/livox_ros_driver2/launch"
cd "$LIDAR_WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

printf '\n[8/8] 编译主仿真仓库并执行检查...\n'
cd "$REPO_ROOT"
set +u
source /opt/ros/humble/setup.bash
source "$LIDAR_WS/install/setup.bash"
set -u
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys ament_python
colcon build --symlink-install
set +u
source "$REPO_ROOT/install/setup.bash"
set -u
python3 "$REPO_ROOT/tools/verify_repository.py"

cat <<EOF

全部安装完成。

主仓库：       $REPO_ROOT
ArduPilot：    $ARDUPILOT_DIR
Gazebo 插件：  $ARDUPILOT_GAZEBO_DIR
FAST-LIO：     $LIDAR_WS

终端 1 启动完整仿真：
  cd "$REPO_ROOT"
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh

终端 2 启动飞行状态机：
  cd "$REPO_ROOT"
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh

终端 3 启动 FAST-LIO 与 RViz：
  cd "$REPO_ROOT"
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
EOF
