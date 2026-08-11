# Visual tight-coupling architecture

## Data path

1. `d435i_rgbd_bridge_cpp` normalizes exact RGB, `16UC1` depth, CameraInfo and
   the calibrated camera TF.
2. `uf_visual_frontend/rgbd_feature_frontend` accepts only RGB and depth with
   identical ROS timestamps. Keyframes default to 10 Hz.
3. KLT forward/backward tracking produces stable IDs and ages. Previous-frame
   depth unprojects anchors. PnP/RANSAC provides geometric inliers and measured
   reprojection errors, not an estimator pose.
4. `uf_reliability/reliability_monitor` publishes the single existing
   `/reliability/vision_score`; the existing scheduler owns switching, weights
   and covariance inflation.
5. `uf_backend_fusion` matches a feature batch to adjacent LiDAR-keyed window
   states, applies the fixed camera time offset/extrinsic, and inserts the
   robust reprojection factor beside Native LiDAR, IMU, GNSS and optical flow.

## Interfaces

Inputs:

- `/sensors/rgbd/color` (`sensor_msgs/Image`, `bgr8` normalized by bridge)
- `/sensors/rgbd/depth` (`sensor_msgs/Image`, `16UC1` mm or `32FC1` m)
- `/sensors/rgbd/camera_info` (`sensor_msgs/CameraInfo`)
- Existing Native LiDAR, IMU, GNSS, optical-flow and scheduler topics

Outputs:

- `/vision/feature_tracks` (`uf_interfaces/VisualFeatureTracks`)
- `/reliability/vision_score` (`uf_interfaces/ReliabilityScore`)
- Existing `/fusion/unified/odom`, path and diagnostics

The feature message records both timestamps, normalized and pixel coordinates,
depth/inverse-depth variance, track age, KLT error, grid cell, PnP inlier and
reprojection error. This keeps every admission decision auditable.

## Configuration and launch

Stable four-source defaults remain unchanged: `visual_factor_mode: disabled`.
To run the visual frontend only against an already running fusion stack:

```bash
ros2 launch uf_visual_frontend visual_tight_coupling.launch.py \
  enabled:=true start_fusion_stack:=false
```

To start the reliability monitor, five-modality scheduler and unified backend
from this launch as well:

```bash
ros2 launch uf_visual_frontend visual_tight_coupling.launch.py \
  enabled:=true start_fusion_stack:=true camera_time_offset_s:=0.0
```

Before hardware use, replace both camera extrinsic parameters with a measured
body-from-camera transform. Do not tune a transform from SLAM success alone.

## Mutually exclusive visual modes

- `disabled`: exact stable-tag four-source behavior.
- `paper_reprojection`: the only online visual factor in this branch.
- `legacy_rtab_relative` exists only as a deterministic factor-level A/B helper
  (`add_legacy_visual_odometry`) and is not wired into the online node.

RTAB remains available for mapping, loops and cross-session relocalization. Its
frame-to-frame odometry is intentionally not inserted when sparse features from
the same images already form the reprojection factor.
