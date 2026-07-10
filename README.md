# Multi-SLAM UAV Simulation

ROS 2 Humble + Gazebo Sim Harmonic + ArduPilot SITL multi-sensor UAV
simulation. The repository contains project-owned source, worlds, sensor
models, launch scripts and mapping adapters. Downloadable upstream projects
and compiled artifacts are intentionally not vendored.

## Features

- ArduPilot Copter SITL and MAVROS flight-state chain
- GPS/GUIDED and non-GPS optical-flow modes
- rigid front-facing D435i-style RGB-D camera, IMU, TF and point cloud
- downward optical-flow camera with optional FCU injection
- simulated MID360 point cloud
- FAST-LIO ROS 2 integration, reliable map filtering and occupancy grid
- simple indoor and city worlds with dynamic objects
- event-driven flight preflight: local pose plus GPS or fresh optical flow

## Repository Layout

```text
src/multi_slam_uav_sim/      UAV, sensors, bridges, flight and worlds
src/multi_slam_worlds/       additional reusable Gazebo worlds
src/mid360_reliable_mapper/  project-owned FAST-LIO map filtering/grid nodes
docs/                        installation, operation and packaging policy
tools/                       repository verification helpers
dependencies.repos           pinned FAST-LIO and Livox ROS driver sources
```

## Not Included

The following are downloaded and built separately:

- ArduPilot
- ArduPilot Gazebo plugin and base Iris model
- Gazebo Sim Harmonic
- ROS 2 Humble and MAVROS
- FAST-LIO and Livox ROS Driver 2
- optional Clearpath and terrain-generator world collections

No `build/`, `install/`, logs, bags, generated maps or third-party binaries
are committed. See [Packaging Policy](docs/PACKAGING.md).

## Quick Start

Install external dependencies by following [Installation](docs/INSTALL.md),
then build this repository:

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Terminal 1, complete simulator and sensors:

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh
```

Terminal 2, optional flight state machine:

```bash
install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_rectangle_state_machine.sh
```

Terminal 3, optional FAST-LIO and RViz:

```bash
LIDAR_WS=<fast-lio-workspace> \
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_mid360_fastlio_mapping.sh
```

Complete commands, visualization topics and non-GPS mode are documented in
[Running the Simulation](docs/RUNNING.md).

## Verified Environment

- Ubuntu 22.04 / WSL 2
- ROS 2 Humble
- Gazebo Sim Harmonic 8.13.0
- MAVROS 2.14.0
- ArduPilot Copter SITL commit `f9d619e26002d6aaa41643ee99c0ae0ee01e2247`
- Python 3.10

The project was verified with Gazebo, SITL, MAVROS, RGB-D, MID360, FAST-LIO,
GPS flight and optical-flow-gated flight running together.

## License

The main simulation packages are Apache-2.0. The bundled
`mid360_reliable_mapper` package declares LGPL-3.0-only and includes its own
license file. External dependencies retain their upstream licenses.

