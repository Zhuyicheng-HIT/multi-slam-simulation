# multi-slam Gazebo Sim Worlds

This workspace collects Gazebo Sim / Harmonic-compatible worlds for UAV multi-source SLAM and navigation testing.

## Build

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select multi_slam_worlds
```

## Environment

```bash
source install/setup.bash
source install/multi_slam_worlds/share/multi_slam_worlds/scripts/env.sh
```

## Launch Commands

Simple UAV test map with textured ground, walls, and basic obstacles:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh simple_test
```

LiDAR tunnel degeneration:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh tunnel
```

ArduPilot warehouse, best for later APM SITL integration:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh ardupilot_warehouse
```

Clearpath warehouse, indoor obstacle avoidance:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh clearpath_warehouse
```

Clearpath office, indoor corridors and relocalization:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh office
```

Clearpath construction, cluttered indoor obstacle avoidance:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh construction
```

Gazebo Terrain Generator Apple Park, real campus / city-scale terrain:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh city_applepark
```

Gazebo Terrain Generator Joshimath, real town / terrain environment:

```bash
install/multi_slam_worlds/share/multi_slam_worlds/scripts/run_named_world.sh city_joshimath
```

## Notes

The curated worlds were smoke-tested with `gz sim -s -r --headless-rendering` on Gazebo Sim 8.13.0.

The curated set keeps only the worlds selected for this project: tunnel, Clearpath warehouse / office / construction, ArduPilot warehouse, and terrain-based city / town worlds.
