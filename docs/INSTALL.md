# Installation

The verified platform is Ubuntu 22.04 or WSL 2 Ubuntu-22.04. Commands use
`<workspace>` as the clone directory and never depend on a specific username.

## 1. ROS 2 Humble

Install ROS 2 Humble Desktop using the official instructions:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Then install the project ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  git curl lsb-release gnupg build-essential cmake ninja-build \
  python3-pip python3-numpy python3-opencv python3-vcstool \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-cv-bridge ros-humble-rqt-image-view ros-humble-rviz2 \
  ros-humble-tf2-ros ros-humble-pcl-ros

sudo rosdep init 2>/dev/null || true
rosdep update
```

Install MAVROS GeographicLib datasets once:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

## 2. Gazebo Sim Harmonic

Use the official OSRF binary repository; do not copy a Gazebo installation
into this workspace:

```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic python3-gz-msgs10 python3-gz-transport13
gz sim --versions
```

Official documentation: https://gazebosim.org/docs/harmonic/install_ubuntu/

After installing ROS and Gazebo, the pinned Git repositories in sections 3,
4 and 6 can be downloaded automatically without compiling them:

```bash
cd <workspace>
tools/fetch_external_sources.sh
```

## 3. ArduPilot SITL

ArduPilot is not stored in this repository. Clone it with submodules and use
the tested commit:

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

## 4. ArduPilot Gazebo Plugin

Clone and build the official plugin. It supplies the base Iris model referenced
by `model://iris_with_standoffs`:

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$HOME/ardupilot_gazebo"
cd "$HOME/ardupilot_gazebo"
git checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
sudo apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
```

The runtime scripts default to `$HOME/ardupilot` and
`$HOME/ardupilot_gazebo`. Different locations are supported:

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
export ARDUPILOT_GAZEBO_DIR=/path/to/ardupilot_gazebo
```

## 5. Clone and Build This Repository

```bash
git clone https://github.com/Zhuyicheng-HIT/multi-slam-simulation.git <workspace>
cd <workspace>
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 tools/verify_repository.py
```

## 6. Optional FAST-LIO Workspace

FAST-LIO and Livox ROS Driver 2 remain external. Import their pinned source
versions into a separate workspace:

```bash
mkdir -p "$HOME/multi-slam-deps/mid360_ws/src"
cd "$HOME/multi-slam-deps/mid360_ws"
vcs import --recursive src < <workspace>/dependencies.repos
source /opt/ros/humble/setup.bash
```

Install the official Livox-SDK2 dependency:

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git \
  "$HOME/multi-slam-deps/Livox-SDK2"
cd "$HOME/multi-slam-deps/Livox-SDK2"
git checkout f5d9375f84efe2b15bc0a052d3e18482ed13adf4
cmake -S . -B build
cmake --build build -j"$(nproc)"
sudo cmake --install build
```

Prepare the Livox ROS 2 package and build the external workspace. Its official
build script selects `package_ROS2.xml` and builds FAST-LIO in the same colcon
workspace:

```bash
cd "$HOME/multi-slam-deps/mid360_ws/src/livox_ros_driver2"
./build.sh humble
source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
```

Upstream documentation:

https://github.com/Livox-SDK/Livox-SDK2

https://github.com/Livox-SDK/livox_ros_driver2

Then set the workspace used by the mapping launcher:

```bash
export LIDAR_WS="$HOME/multi-slam-deps/mid360_ws"
```

The project-owned `mid360_reliable_mapper` is built with the main repository;
only FAST-LIO and Livox message/driver packages come from the external overlay.

## 7. Optional Large World Collections

Clearpath Simulator and Gazebo Terrain Generator are not required by the
default UAV worlds. Clone them under `<workspace>/external` only when needed:

```bash
mkdir -p <workspace>/external
git clone https://github.com/clearpathrobotics/clearpath_simulator.git \
  <workspace>/external/clearpath_simulator
git clone https://github.com/fkromer/gazebo_terrain_generator.git \
  <workspace>/external/gazebo_terrain_generator
```

For a different location:

```bash
export MULTI_SLAM_EXTERNAL_DIR=/path/to/external
```
