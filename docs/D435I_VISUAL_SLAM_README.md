# D435i RGB-D Visual SLAM

This feature provides a D435i-only RGB-D visual SLAM baseline that can run alongside the original multi-sensor simulation. It uses the C++ bridge and the RTAB-Map `feature_aligned` profile by default in headless mode; the Python bridge remains available as a compatibility fallback.

## Scope

```text
Gazebo D435i RGB/depth/CameraInfo/IMU
  -> d435i_rgbd_bridge_cpp
  -> paired RGB + aligned 16UC1 depth + CameraInfo + optical TF
  -> RTAB-Map RGB-D odometry/mapping
  -> /rtabmap/odom, map, and read-only database diagnostics
```

RTAB-Map is used for evaluation only; poses are not fed back into the ArduPilot EKF or flight controller. The D435i-only profile disables MID360, FAST-LIO, optical flow, the Gazebo GUI, the RTAB-Map GUI, RViz, and PointCloud2 by default. The original full simulation keeps its existing entry point and defaults. This feature does not modify FAST-LIO or Ultra-Fusion algorithms.

## Build

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select d435i_rgbd_bridge_cpp multi_slam_uav_sim
source install/setup.bash
```

The environment requires ROS 2 Humble, Gazebo Harmonic, `ros_gz_bridge`, RTAB-Map ROS 2, MAVROS, NumPy, PyYAML, and psutil. The repository installation scripts provide the common project dependencies.

## Start and stop

```bash
cd "$HOME/projects/multi-slam-simulation"
RTABMAP_PROFILE=feature_aligned D435I_WORLD=textured \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_headless.sh
```

To stop the run, process only the PIDs recorded and verified in the run manifest:

```bash
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/stop_d435i_visual_slam_headless.sh
```

Logs are written to `logs/d435i_visual_slam/headless/<run-id>/` by default. Databases and PID/active markers are runtime artifacts and are not committed to Git.

## Main topics

| Purpose | ROS 2 topic |
|---|---|
| RGB | `/front/d435i/color/image_raw` |
| Raw depth | `/front/d435i/depth/image_rect_raw` |
| Aligned depth | `/front/d435i/aligned_depth_to_color/image_raw` |
| Color CameraInfo | `/front/d435i/color/camera_info` |
| Depth CameraInfo | `/front/d435i/depth/camera_info` |
| IMU | `/front/d435i/imu` |
| Simulation time | `/clock` (requires a single publisher) |
| RTAB odometry | `/rtabmap/odom` |
| RTAB diagnostics | `/rtabmap/odom_info`, `/rtabmap/info` |
| Evaluation ground truth | `/d435i_visual_slam/ground_truth` |

The C++ bridge publishes a pair only after both RGB and depth have been updated, and gives RGB, depth, aligned depth, and CameraInfo the same timestamp. Depth uses `16UC1` by default and messages use the D435i optical frame. PointCloud2 is disabled by default; when enabled, it is generated only when subscribers exist and the rate limit allows it.

## Profile parameters

The RTAB-Map profile is at `src/multi_slam_uav_sim/config/d435i_rtabmap_feature_aligned.yaml`. Key invariants are:

- `frame_id=base_link`, `use_sim_time=true`, and exact synchronization;
- `Kp/DetectorStrategy=6` and `Vis/FeatureType=6`;
- `Mem/UseOdomFeatures=true`;
- `Vis/MinInliers=10` and `Rtabmap/LoopThr=0.11`;
- the launch rejects lower MinInliers/LoopThr values and approximate synchronization.

Common environment switches use `0` or `1`:

| Variable | Default | Purpose |
|---|---:|---|
| `GAZEBO_GUI` | 0 | Gazebo GUI |
| `RTABMAP_GUI` | 0 | RTAB-Map GUI |
| `RVIZ` | 0 | RViz |
| `ENABLE_FLOW` | 0 | Optical-flow stack |
| `ENABLE_FLOW_VIEWER` | 0 | Optical-flow viewer |
| `ENABLE_MID360` | 0 | MID360 bridge |
| `ENABLE_D435I_POINTCLOUD` | 0 | D435i PointCloud2 |
| `D435I_START_FLIGHT_STACK` | 1 | SITL/MAVROS/flight state |
| `D435I_ENABLE_RTABMAP` | 1 | RTAB-Map |

Set `D435I_BRIDGE_IMPL=python` to use the compatibility bridge; the supported baseline uses `cpp`.

## Validation commands

```bash
# Bridge throughput, RTAB latency, ATE/RPE
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/profile_d435i_visual_pipeline.sh

# A-G visual-friendly flight route
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_flight.sh

# Feature-alignment and speed-envelope matrices
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_feature_alignment_matrix.sh
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_speed_envelope_matrix.sh

# Read-only diagnostics for an existing database
ros2 run multi_slam_uav_sim rtabmap_database_diagnostics --help
```

See [D435I_VISUAL_SLAM_BENCHMARK.md](D435I_VISUAL_SLAM_BENCHMARK.md) and [D435I_VISUAL_SLAM_STATUS.md](D435I_VISUAL_SLAM_STATUS.md) for performance results, limitations, and reproduction details.
