# Packaging Policy

This repository is source-first and reproducible from public upstream
dependencies.

## Included

- project-owned ROS 2 packages and nodes
- SDF worlds and small project-specific sensor models
- flight parameters, RViz configuration and scripts
- the small project-owned MID360 reliable mapper
- dependency URLs, pinned revisions and complete run instructions

## Excluded

- `build/`, `install/`, `log/`, runtime logs and caches
- ArduPilot source or SITL binaries
- Gazebo packages, cache or compiled plugins
- full `ardupilot_gazebo`, FAST-LIO or Livox driver repositories
- downloadable Clearpath and terrain repositories
- rosbag, PCD, map, video and generated image outputs
- copied upstream Iris meshes not used by current worlds

## Paths

Project files use package-share lookup, script-relative paths or placeholders
such as `<workspace>`. External locations are supplied through:

```text
ARDUPILOT_DIR
ARDUPILOT_GAZEBO_DIR
LIDAR_WS
MULTI_SLAM_EXTERNAL_DIR
```

Run `python3 tools/verify_repository.py` before every release.

