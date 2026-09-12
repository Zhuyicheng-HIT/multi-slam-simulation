# D435i RGB-D Visual SLAM

此功能提供仅使用 D435i 的 RGB-D visual SLAM baseline，可与原有 multi-sensor simulation 并行运行。默认在 headless mode 下使用 C++ bridge 和 RTAB-Map `feature_aligned` profile；Python bridge 仍作为 compatibility fallback 保留。

## 范围

```text
Gazebo D435i RGB/depth/CameraInfo/IMU
  -> d435i_rgbd_bridge_cpp
  -> paired RGB + aligned 16UC1 depth + CameraInfo + optical TF
  -> RTAB-Map RGB-D odometry/mapping
  -> /rtabmap/odom、map 和只读 database diagnostics
```

RTAB-Map 仅用于 evaluation；pose 不会反馈给 ArduPilot EKF 或 flight controller。D435i-only profile 默认禁用 MID360、FAST-LIO、optical flow、Gazebo GUI、RTAB-Map GUI、RViz 和 PointCloud2。原有完整 simulation 保留现有 entry point 和默认值。本功能不修改 FAST-LIO 或 Ultra-Fusion algorithm。

## 构建

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select d435i_rgbd_bridge_cpp multi_slam_uav_sim
source install/setup.bash
```

环境需要 ROS 2 Humble、Gazebo Harmonic、`ros_gz_bridge`、RTAB-Map ROS 2、MAVROS、NumPy、PyYAML 和 psutil。仓库安装脚本提供常用的项目依赖。

## 启动与停止

```bash
cd "$HOME/projects/multi-slam-simulation"
RTABMAP_PROFILE=feature_aligned D435I_WORLD=textured \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_headless.sh
```

停止运行时，只处理 run manifest 中记录并验证过的 PID：

```bash
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/stop_d435i_visual_slam_headless.sh
```

日志默认写入 `logs/d435i_visual_slam/headless/<run-id>/`。database 及 PID/active marker 是 runtime artifact，不会提交到 Git。

## 主要 topic

| 用途 | ROS 2 topic |
|---|---|
| RGB | `/front/d435i/color/image_raw` |
| Raw depth | `/front/d435i/depth/image_rect_raw` |
| Aligned depth | `/front/d435i/aligned_depth_to_color/image_raw` |
| Color CameraInfo | `/front/d435i/color/camera_info` |
| Depth CameraInfo | `/front/d435i/depth/camera_info` |
| IMU | `/front/d435i/imu` |
| Simulation time | `/clock`（需要单一 publisher） |
| RTAB odometry | `/rtabmap/odom` |
| RTAB diagnostics | `/rtabmap/odom_info`、`/rtabmap/info` |
| Evaluation ground truth | `/d435i_visual_slam/ground_truth` |

C++ bridge 仅在 RGB 和 depth 都已更新后发布一对消息，并为 RGB、depth、aligned depth 与 CameraInfo 使用相同 timestamp。Depth 默认使用 `16UC1`，消息使用 D435i optical frame。PointCloud2 默认禁用；启用后仅在存在 subscriber 且速率限制允许时生成。

## Profile 参数

RTAB-Map profile 位于 `src/multi_slam_uav_sim/config/d435i_rtabmap_feature_aligned.yaml`。关键不变量：

- `frame_id=base_link`、`use_sim_time=true` 和精确同步；
- `Kp/DetectorStrategy=6` 与 `Vis/FeatureType=6`；
- `Mem/UseOdomFeatures=true`；
- `Vis/MinInliers=10` 与 `Rtabmap/LoopThr=0.11`；
- launch 会拒绝较低的 MinInliers/LoopThr 值和 approximate synchronization。

常用 environment switch 使用 `0` 或 `1`：

| 变量 | 默认值 | 用途 |
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

设置 `D435I_BRIDGE_IMPL=python` 可使用 compatibility bridge；支持的 baseline 使用 `cpp`。

## 验证命令

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

性能结果、限制和复现细节请参阅 [D435I_VISUAL_SLAM_BENCHMARK.md](D435I_VISUAL_SLAM_BENCHMARK.md) 与 [D435I_VISUAL_SLAM_STATUS.md](D435I_VISUAL_SLAM_STATUS.md)。
