# Visual SLAM 到 Ultra-Fusion 的集成

## 支持边界

当前集成面向官方 binary-only Ultra-Fusion ROS 2 Humble v0.2.2 runtime。已确认的 visual path 是将原始 D435i RGB-D 输入 Ultra-Fusion 内部 visual frontend；无需创建或虚构外部 feature-track message。

```text
D435i RGB-D ── raw RGB -> compressed-image bridge -> Ultra-Fusion frontend
              │                                      -> local visual factors
              └── RTAB-Map -> loop/relocalization/map->odom global correction

MID360 PointCloud2 + D435i IMU ---------------------> Ultra-Fusion LVIO
```

D435i point cloud 和 `/rtabmap/cloud_map` 不作为输入。MID360 原始 PointCloud2 提供 LiDAR 几何。

## 准备完整 profile

不要编写精简 YAML。应从 v0.2.2 安装的完整官方 ROS 2 LVWIO profile，或从经过验证的 Debian package 中提取：

```bash
python3 tools/prepare_ultrafusion_d435i_config.py \
  /opt/ultrafusion/config/m3dgr/uf_m3dgr_ros2_lvwio.yaml \
  /tmp/ultrafusion_d435i
```

generator 保留所有 upstream 字段，只修改已确认的 sensor mode、topic、camera file、frequency、禁用 map export 和 simulation extrinsic。它选择：

- native LVIO：IMU + LiDAR + image，禁用 wheel；
- `/front/d435i/imu` 作为 estimator body IMU；
- `/sim/mid360/points_raw` 作为 LiDAR；
- D435i RGB 流的 compressed derivative；
- 对齐的 `16UC1` depth；
- 固定的 simulation extrinsic 和零 simulated camera delay。

生成的 extrinsic 仅适用于 `models/iris_apm_rgbd/model.sdf`。实体 aircraft 必须使用实测的 Camera–IMU/LiDAR calibration，并应从禁用 online calibration 开始。

## 运行 bridge

simulation stack 已运行且官方 binary 已安装时：

```bash
tools/run_ultrafusion_d435i_bridge.sh \
  --config /tmp/ultrafusion_d435i/ultrafusion_d435i_mid360_lvio.yaml
```

使用 `--dry-run` 可在不启动 binary 的情况下验证实时 topic 类型和 depth encoding。wrapper 检查 RGB、depth、D435i IMU、MID360 PointCloud2 及 visual reliability，然后启动受限的 raw-to-compressed image transport。其 trap 只会向它创建的两个精确 process group 发送信号。

本次任务刻意未安装官方 runtime。真正的 LVIO replay 仍取决于：在受支持的 Humble runtime 中安装经过验证的 v0.2.2 package，并确认其 `preprocess.lidar_type: 7` 如何解释 simulator PointCloud2 的 `time` 字段。

## 记录并检查 baseline

启用一个完整 stack 后：

```bash
tools/record_ultrafusion_visual_inputs.sh \
  --duration 90 \
  --output artifacts/.../bags/ultrafusion_visual_input_baseline

python3 tools/check_ultrafusion_visual_inputs.py \
  artifacts/.../bags/ultrafusion_visual_input_baseline
```

recorder 只发现精确的 allow-list。它排除 D435i point cloud、RTAB colored cloud 和 FAST-LIO registered visualization cloud，同时保留 MID360 原始几何。checker 离线读取 bag，验证计数、频率、单调 timestamp、RGB/depth 配对、CameraInfo、frame、TF、depth encoding 和 reliability，并以有意义的 exit code 写入 JSON 与 Markdown。

## Reliability 与重复计数策略

当前的 `/vision/reliability_*` topic 是有价值的 side-channel 证据，但 v0.2.2 没有公开 subscriber。将这些 score 连接到内部 factor scheduler 属于未来 upstream 工作，不能通过 remap 完成。

RTAB-Map odometry 与 Ultra-Fusion visual track 来自同一组 RGB-D 测量。不要把两者都作为独立的高权重 local factor 注入。如果保留 RTAB，应限制为低频 global information：

- GlobalClosure 和 LocalSpaceClosure event；
- cross-session relocalization；
- `map→odom` global correction；
- 相关性分析后明确进行 covariance-inflated 的 constraint。

下一步工程工作是针对官方 binary 直接进行 LVIO replay，而不是新建 KLT frontend。只有未来 upstream release 发布外部 feature-track contract 时，独立 feature frontend 才有必要。
