# D435i RGB-D 视觉 SLAM

本功能提供一个与原多传感器仿真并存的 D435i-only RGB-D 视觉 SLAM
基线。默认使用 C++ bridge 和 RTAB-Map `feature_aligned` profile，以
headless 方式运行；Python bridge 保留为兼容降级方案。

## 范围与边界

```text
Gazebo D435i RGB/depth/CameraInfo/IMU
  -> d435i_rgbd_bridge_cpp
  -> paired RGB + aligned 16UC1 depth + CameraInfo + optical TF
  -> RTAB-Map RGB-D odometry/mapping
  -> /rtabmap/odom, map and read-only database diagnostics
```

RTAB-Map 只用于评测，不向 ArduPilot EKF 或飞控回灌位姿。D435i-only
profile 默认关闭 MID360、FAST-LIO、光流、Gazebo GUI、RTAB-Map GUI、
RViz 和 PointCloud2。原完整仿真仍由原入口按原默认值启动；本功能没有修改
FAST-LIO 或 Ultra-Fusion 算法。

## 构建

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select d435i_rgbd_bridge_cpp multi_slam_uav_sim
source install/setup.bash
```

需要 ROS 2 Humble、Gazebo Harmonic、`ros_gz_bridge`、RTAB-Map ROS 2、
MAVROS、NumPy、PyYAML 和 psutil。仓库安装脚本负责项目的通用依赖。

## 一键启动和停止

```bash
cd "$HOME/projects/multi-slam-simulation"
RTABMAP_PROFILE=feature_aligned D435I_WORLD=textured \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_headless.sh
```

停止时只处理本次运行清单中记录并核对过 process group 的 PID：

```bash
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/stop_d435i_visual_slam_headless.sh
```

默认日志写入
`logs/d435i_visual_slam/headless/<run-id>/`，数据库和 PID/active 标记均为
运行产物，不提交到 Git。

## 主要话题

| 作用 | ROS 2 话题 |
|---|---|
| RGB | `/front/d435i/color/image_raw` |
| raw depth | `/front/d435i/depth/image_rect_raw` |
| aligned depth | `/front/d435i/aligned_depth_to_color/image_raw` |
| color CameraInfo | `/front/d435i/color/camera_info` |
| depth CameraInfo | `/front/d435i/depth/camera_info` |
| IMU | `/front/d435i/imu` |
| simulation time | `/clock`，要求唯一 publisher |
| RTAB odometry | `/rtabmap/odom` |
| RTAB diagnostics | `/rtabmap/odom_info`、`/rtabmap/info` |
| evaluation ground truth | `/d435i_visual_slam/ground_truth` |

C++ bridge 只在 RGB 和 depth 都更新后发布一对消息，并为 RGB、depth、
aligned depth 和 CameraInfo 写入同一个时间戳。深度默认编码为
`16UC1`，frame 使用 D435i optical frame。PointCloud2 默认关闭；开启时
也只在存在订阅者且满足限频条件时生成。

## Profile 参数

RTAB-Map profile 位于
`src/multi_slam_uav_sim/config/d435i_rtabmap_feature_aligned.yaml`。
关键不变量：

- `frame_id=base_link`，`use_sim_time=true`，exact sync；
- `Kp/DetectorStrategy=6`，`Vis/FeatureType=6`；
- `Mem/UseOdomFeatures=true`；
- `Vis/MinInliers=10`，`Rtabmap/LoopThr=0.11`；
- launch 拒绝降低 MinInliers/LoopThr 或开启 approximate sync。

常用环境开关均为 `0` 或 `1`：

| 变量 | 默认 | 作用 |
|---|---:|---|
| `GAZEBO_GUI` | 0 | Gazebo GUI |
| `RTABMAP_GUI` | 0 | RTAB-Map GUI |
| `RVIZ` | 0 | RViz |
| `ENABLE_FLOW` | 0 | optical-flow stack |
| `ENABLE_FLOW_VIEWER` | 0 | optical-flow viewer |
| `ENABLE_MID360` | 0 | MID360 bridge |
| `ENABLE_D435I_POINTCLOUD` | 0 | D435i PointCloud2 |
| `D435I_START_FLIGHT_STACK` | 1 | SITL/MAVROS/flight-state |
| `D435I_ENABLE_RTABMAP` | 1 | RTAB-Map |

`D435I_BRIDGE_IMPL=python` 可切换到兼容 bridge；正式基线使用 `cpp`。

## 验证入口

```bash
# bridge 吞吐、RTAB 延迟、ATE/RPE
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/profile_d435i_visual_pipeline.sh

# A-G 视觉友好航线
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_flight.sh

# feature alignment 和速度包线矩阵
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_feature_alignment_matrix.sh
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_speed_envelope_matrix.sh

# 对已有数据库执行只读诊断
ros2 run multi_slam_uav_sim rtabmap_database_diagnostics --help
```

性能结果、限制和复现口径分别见
[D435I_VISUAL_SLAM_BENCHMARK.md](D435I_VISUAL_SLAM_BENCHMARK.md) 与
[D435I_VISUAL_SLAM_STATUS.md](D435I_VISUAL_SLAM_STATUS.md)。
