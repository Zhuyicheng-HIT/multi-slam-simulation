# HybridFusion map-level fusion 架构

## 与 PR #6 和 PR #8 的隔离

`hybridfusion_map_fusion` 是新的 leaf package。不会更改 PR #6 backend、reliability、FAST-LIO、RTAB-Map、D435i bridge、scheduler、TF owner、flight controller 或 launch default。该 package 不在现有 integration launch 中，只有提供 `enabled:=true` 时才会在自身 launch 中启用。

```text
现有 PR #8 visual stack                    现有 PR #6 LiDAR stack
RGB + 16UC1 depth + CameraInfo                 /fastlio_denoised_map
RTAB pose + existing TF                                    |
             |                                             |
             v                                             v
  rgbd_map_exporter（无 cloud topic）             lidar_map_exporter
             |                                             |
 visual PCD + keyframes + calibration             LiDAR PCD + frame/stamp
             +--------------------+------------------------+
                                  |
                    hybridfusion_offline（独立 process）
                                  |
       result JSON + SE(3) YAML + aligned copy + fused PCD
```

Export 和 registration error 无法停止或重新配置 publisher。该模块不会广播估计出的 transform；任何未来下游使用前，operator 必须先审核 offline result。

## 从论文到实现的追踪

| 论文步骤 | 本地实现 | 状态/来源 |
|---|---|---|
| Module A visual reconstruction | 现有 D435i/RTAB stack；`rgbd_map_exporter.cpp` 使用 CameraInfo 和带 timestamp 的 TF 对精确 RGB-D keyframe 进行 back-project | 通过复用实现，不使用第二套 visual odometry |
| Module B LiDAR SLAM | 现有 MID360、FAST-LIO 和可靠的 denoised map | 原样复用 |
| GNSS/existing-pose rough match | `dataset.yaml: initial_lidar_to_visual` | 已实现；来源可为 GNSS 或 calibrated pose |
| VoxelGrid preprocessing | `voxel_downsample()` | 已实现；leaf size 为工程参数 |
| Grid patches | 使用 common origin 的 `make_blocks()` | 已实现；按论文文字自动设为 scene-size/10 |
| Significant/valid blocks | `grid.min_points` | 已实现；论文未给出 count，已参数化 |
| Radial/angle candidate range | centroid radius、lambda ring 和 angle filter | 已实现；lambda/theta 未在论文中给出，已参数化 |
| ESF640 patch filtering | PCL ESF 加 Pearson correlation | 已实现；threshold 不能低于论文 Eq. (6) 的 0.60 |
| Neighbor filter | 相同 offset 邻居的 ESF correlation | 等效实现；数值 threshold 已参数化 |
| Spliced patch neighborhoods | `collect_neighborhood()` | 已实现 |
| Ground removal and XY boundary | height quantile、`d>h`、occupancy boundary cell | 等效的 PCL-compatible 实现；h/raster 已参数化 |
| 2D NDT | Gaussian-cell likelihood 和 SE(2) 中的 Gauss-Newton，输出约束为 XY/yaw | 已实现；避免 zero-Z 输入导致的奇异 PCL 3D-NDT neighborhood |
| 3D NDT | 局部 3D neighborhood 上的 PCL NDT | 已实现 |
| Local transformation set K | 每个收敛的 2D-3D registration 一个 SE(3) | 已实现，保留失败项 |
| Translation/rotation clustering | 在 epsilon/omega 下的 connected component | 已实现；两个 threshold 均参数化 |
| Pose fusion | translation mean 加 iterative quaternion SLERP | 按论文 Eq. (7) 后描述实现 |
| Minor full-map adjustment | 受保护的 full-map 3D NDT | 已实现；若 NN fitness 超过配置比例而恶化则拒绝 |
| Fused map and supplement metric | 将 source 变换到 target frame，构建 union voxel map | 已实现，不修改任一输入 |

## 坐标与文件 contract

- 估计 transform 的方向始终为 `visual_frame <- lidar_frame`。
- live visual point 直接存储在选定的 RTAB/global TF frame 中。
- FAST-LIO PCD 保留在其 message frame，通常是 `camera_init`。
- Dataset manifest 和 transform YAML 使用 metre、radian，顺序为 `[x,y,z,roll,pitch,yaw]`。
- `transform.yaml` 明确记录 `published_as_tf: false`。
- Keyframe 和 map metadata 保留 source topic、ROS stamp、frame、intrinsic、distortion coefficient 和 voxel size。

## 评估定义

- Translation/rotation error：估计值与真实 SE(3) 之间的距离。
- Overlap error：source-to-target 最近邻 mean 和 RMSE，受配置的 overlap distance 截断。
- Boundary error：两组提取的 XY boundary cloud 之间的最近邻误差。
- Inlier ratio：在配置 target distance 内的对齐 LiDAR 点比例。
- Supplement growth：`(union occupied voxels - visual occupied voxels) / visual occupied voxels`，类似论文中的 octree leaf volume metric。
- Runtime：每个独立 method process 内的 steady-clock 时长。
- Memory：通过 `getrusage` 获取的 process peak resident set。
- Block accounting：descriptor candidate、neighbor-consistent candidate、收敛的 local transformation、失败的 local registration 以及选定 cluster size 均会记录。

## 已知范围

此 v1 只验证 offline 的 map-level fusion。它有意排除实时 D435i point-cloud display、backend factor、重复 TF、flight-time feedback、官方 Ultra-Fusion binary，以及写回 RTAB 或 FAST-LIO map。

`collect_hybridfusion_simulation.sh` 是可选的 owner-aware orchestration wrapper。它使用现有 guided rectangle 启动未修改的 PR #6/PR #8 headless stack，在另一个 process group 中仅运行本 package 的两个 exporter，着陆后调用其 save service，并将 stack cleanup 交给现有 active-run lifecycle。它不会更改任何 launch default。
