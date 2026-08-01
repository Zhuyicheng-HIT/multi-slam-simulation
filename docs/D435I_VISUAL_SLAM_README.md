
# D435i RGB-D è§†è§‰ SLAM

æœ¬åŠŸèƒ½æä¾›ä¸€ä¸ªä¸ŽåŽŸå¤šä¼ æ„Ÿå™¨ä»¿çœŸå¹¶å­˜çš„ D435i-only RGB-D è§†è§‰ SLAM
åŸºçº¿ã€‚é»˜è®¤ä½¿ç”¨ C++ bridge å’Œ RTAB-Map `feature_aligned` profileï¼Œä»¥
headless æ–¹å¼è¿è¡Œï¼›Python bridge ä¿ç•™ä¸ºå…¼å®¹é™çº§æ–¹æ¡ˆã€‚

## èŒƒå›´ä¸Žè¾¹ç•Œ

```text
Gazebo D435i RGB/depth/CameraInfo/IMU
  -> d435i_rgbd_bridge_cpp
  -> paired RGB + aligned 16UC1 depth + CameraInfo + optical TF
  -> RTAB-Map RGB-D odometry/mapping
  -> /rtabmap/odom, map and read-only database diagnostics
```

RTAB-Map åªç”¨äºŽè¯„æµ‹ï¼Œä¸å‘ ArduPilot EKF æˆ–é£žæŽ§å›žçŒä½å§¿ã€‚D435i-only
profile é»˜è®¤å…³é—­ MID360ã€FAST-LIOã€å…‰æµã€Gazebo GUIã€RTAB-Map GUIã€
RViz å’Œ PointCloud2ã€‚åŽŸå®Œæ•´ä»¿çœŸä»ç”±åŽŸå…¥å£æŒ‰åŽŸé»˜è®¤å€¼å¯åŠ¨ï¼›æœ¬åŠŸèƒ½æ²¡æœ‰ä¿®æ”¹
FAST-LIO æˆ– Ultra-Fusion ç®—æ³•ã€‚

## æž„å»º

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select d435i_rgbd_bridge_cpp multi_slam_uav_sim
source install/setup.bash
```

éœ€è¦ ROS 2 Humbleã€Gazebo Harmonicã€`ros_gz_bridge`ã€RTAB-Map ROS 2ã€
MAVROSã€NumPyã€PyYAML å’Œ psutilã€‚ä»“åº“å®‰è£…è„šæœ¬è´Ÿè´£é¡¹ç›®çš„é€šç”¨ä¾èµ–ã€‚

## ä¸€é”®å¯åŠ¨å’Œåœæ­¢

```bash
cd "$HOME/projects/multi-slam-simulation"
RTABMAP_PROFILE=feature_aligned D435I_WORLD=textured \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_headless.sh
```

åœæ­¢æ—¶åªå¤„ç†æœ¬æ¬¡è¿è¡Œæ¸…å•ä¸­è®°å½•å¹¶æ ¸å¯¹è¿‡ process group çš„ PIDï¼š

```bash
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/stop_d435i_visual_slam_headless.sh
```

é»˜è®¤æ—¥å¿—å†™å…¥
`logs/d435i_visual_slam/headless/<run-id>/`ï¼Œæ•°æ®åº“å’Œ PID/active æ ‡è®°å‡ä¸º
è¿è¡Œäº§ç‰©ï¼Œä¸æäº¤åˆ° Gitã€‚

## ä¸»è¦è¯é¢˜

| ä½œç”¨ | ROS 2 è¯é¢˜ |
|---|---|
| RGB | `/front/d435i/color/image_raw` |
| raw depth | `/front/d435i/depth/image_rect_raw` |
| aligned depth | `/front/d435i/aligned_depth_to_color/image_raw` |
| color CameraInfo | `/front/d435i/color/camera_info` |
| depth CameraInfo | `/front/d435i/depth/camera_info` |
| IMU | `/front/d435i/imu` |
| simulation time | `/clock`ï¼Œè¦æ±‚å”¯ä¸€ publisher |
| RTAB odometry | `/rtabmap/odom` |
| RTAB diagnostics | `/rtabmap/odom_info`ã€`/rtabmap/info` |
| evaluation ground truth | `/d435i_visual_slam/ground_truth` |

C++ bridge åªåœ¨ RGB å’Œ depth éƒ½æ›´æ–°åŽå‘å¸ƒä¸€å¯¹æ¶ˆæ¯ï¼Œå¹¶ä¸º RGBã€depthã€
aligned depth å’Œ CameraInfo å†™å…¥åŒä¸€ä¸ªæ—¶é—´æˆ³ã€‚æ·±åº¦é»˜è®¤ç¼–ç ä¸º
`16UC1`ï¼Œframe ä½¿ç”¨ D435i optical frameã€‚PointCloud2 é»˜è®¤å…³é—­ï¼›å¼€å¯æ—¶
ä¹Ÿåªåœ¨å­˜åœ¨è®¢é˜…è€…ä¸”æ»¡è¶³é™é¢‘æ¡ä»¶æ—¶ç”Ÿæˆã€‚

## Profile å‚æ•°

RTAB-Map profile ä½äºŽ
`src/multi_slam_uav_sim/config/d435i_rtabmap_feature_aligned.yaml`ã€‚
å…³é”®ä¸å˜é‡ï¼š

- `frame_id=base_link`ï¼Œ`use_sim_time=true`ï¼Œexact syncï¼›
- `Kp/DetectorStrategy=6`ï¼Œ`Vis/FeatureType=6`ï¼›
- `Mem/UseOdomFeatures=true`ï¼›
- `Vis/MinInliers=10`ï¼Œ`Rtabmap/LoopThr=0.11`ï¼›
- launch æ‹’ç»é™ä½Ž MinInliers/LoopThr æˆ–å¼€å¯ approximate syncã€‚

å¸¸ç”¨çŽ¯å¢ƒå¼€å…³å‡ä¸º `0` æˆ– `1`ï¼š

| å˜é‡ | é»˜è®¤ | ä½œç”¨ |
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

`D435I_BRIDGE_IMPL=python` å¯åˆ‡æ¢åˆ°å…¼å®¹ bridgeï¼›æ­£å¼åŸºçº¿ä½¿ç”¨ `cpp`ã€‚

## éªŒè¯å…¥å£

```bash
# bridge åžåã€RTAB å»¶è¿Ÿã€ATE/RPE
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/profile_d435i_visual_pipeline.sh

# A-G è§†è§‰å‹å¥½èˆªçº¿
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_visual_slam_flight.sh

# feature alignment å’Œé€Ÿåº¦åŒ…çº¿çŸ©é˜µ
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_feature_alignment_matrix.sh
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_speed_envelope_matrix.sh

# å¯¹å·²æœ‰æ•°æ®åº“æ‰§è¡Œåªè¯»è¯Šæ–­
ros2 run multi_slam_uav_sim rtabmap_database_diagnostics --help
```

æ€§èƒ½ç»“æžœã€é™åˆ¶å’Œå¤çŽ°å£å¾„åˆ†åˆ«è§
[D435I_VISUAL_SLAM_BENCHMARK.md](D435I_VISUAL_SLAM_BENCHMARK.md) ä¸Ž
[D435I_VISUAL_SLAM_STATUS.md](D435I_VISUAL_SLAM_STATUS.md)ã€‚

