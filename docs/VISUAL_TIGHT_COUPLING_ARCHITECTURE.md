# Visual 紧耦合架构

## 数据路径

1. `d435i_rgbd_bridge_cpp` 规范化精确 RGB、`16UC1` depth、CameraInfo 以及经过标定的 camera TF。
2. `uf_visual_frontend/rgbd_feature_frontend` 只接受 ROS timestamp 完全一致的 RGB 和 depth。Keyframe 默认频率为 10 Hz。
3. KLT 前向/后向跟踪生成稳定的 ID 和 age。利用前一帧 depth 将 anchor 反投影。PnP/RANSAC 提供几何内点和实测 reprojection error，而不是 estimator pose。
4. `uf_reliability/reliability_monitor` 发布现有的唯一 `/reliability/vision_score`；现有 scheduler 负责切换、权重和 covariance inflation。
5. `uf_backend_fusion` 将 feature batch 匹配到相邻的 LiDAR-keyed window state，应用固定的 camera time offset/extrinsic，并在 Native LiDAR、IMU、GNSS 和 optical flow 旁插入 robust reprojection factor。

## 接口

输入：

- `/sensors/rgbd/color`（`sensor_msgs/Image`，由 bridge 规范化为 `bgr8`）
- `/sensors/rgbd/depth`（`sensor_msgs/Image`，`16UC1` mm 或 `32FC1` m）
- `/sensors/rgbd/camera_info`（`sensor_msgs/CameraInfo`）
- 现有 Native LiDAR、IMU、GNSS、optical-flow 和 scheduler topic

输出：

- `/vision/feature_tracks`（`uf_interfaces/VisualFeatureTracks`）
- `/reliability/vision_score`（`uf_interfaces/ReliabilityScore`）
- 现有的 `/fusion/unified/odom`、path 和 diagnostics

feature message 同时记录两个 timestamp、normalized 与 pixel 坐标、depth/inverse-depth variance、track age、KLT error、grid cell、PnP inlier 和 reprojection error，使每次 admission decision 都可审计。

## 配置与 launch

稳定的 four-source 默认设置保持不变：`visual_factor_mode: disabled`。要只针对已经运行的 fusion stack 启动 visual frontend：

```bash
ros2 launch uf_visual_frontend visual_tight_coupling.launch.py \
  enabled:=true start_fusion_stack:=false
```

要通过该 launch 同时启动 reliability monitor、five-modality scheduler 和 unified backend：

```bash
ros2 launch uf_visual_frontend visual_tight_coupling.launch.py \
  enabled:=true start_fusion_stack:=true camera_time_offset_s:=0.0
```

使用硬件前，将两个 camera extrinsic 参数替换为实测的 body-from-camera transform。不要仅根据 SLAM 成功来调节 transform。

## 互斥的视觉模式

- `disabled`：精确的 stable-tag four-source 行为。
- `paper_reprojection`：本分支中唯一在线 visual factor。
- `legacy_rtab_relative` 仅作为确定性的 factor-level A/B helper（`add_legacy_visual_odometry`）存在，不接入 online node。

RTAB 仍可用于 mapping、loop 和 cross-session relocalization。当同一图像产生的 sparse feature 已形成 reprojection factor 时，不会再插入其 frame-to-frame odometry。
