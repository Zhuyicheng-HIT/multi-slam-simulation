# D435i RGB-D 视觉 SLAM Benchmark

## 固定条件

- 仿真 D435i：640×480，RGB + aligned depth；
- C++ bridge，`16UC1`，reliable/depth=1；
- RGB/depth exact sync，共享时间戳，`frame_id=base_link`；
- RTAB-Map `feature_aligned`：
  `Kp/DetectorStrategy=6`、`Vis/FeatureType=6`、
  `Mem/UseOdomFeatures=true`；
- `Vis/MinInliers=10`、`Rtabmap/LoopThr=0.11` 未降低；
- GPS/GUIDED 控制飞行，RTAB-Map 仅用于评估；
- 纹理世界、每次独立数据库和日志目录。

结果来自稳定提交 `727d6e0` 的已完成实验。原始数据库、bag 和大体积日志
不进入 PR。

## 结果摘要

| 项目 | 结果 |
|---|---|
| RGB-D pair rate | 约 28–29 Hz |
| 正式样本 RTAB odometry | 约 16 Hz |
| A-G aligned ATE / RPE | 2.32 / 1.17 cm |
| 推荐水平测试速度 | 0.35 m/s |
| 0.35 m/s 直线 | 3/3 PASS |
| 0.75 m/s 直线 | 3/3 PASS，压力档 |
| 最高已验证偏航 | 40 deg/s |
| 最高已验证垂直 | 0.20 m/s |
| 最高已验证组合 | 0.50 m/s + 25 deg/s |
| lost / reset / TF backward | 0 / 0 / 0（有效正式样本） |
| 错误闭环 | 未观测 |

0.75 m/s 证明当前仿真至少能承受该档，但路线长度、稳态样本和安全裕量均
不如 0.35 m/s，因此不替代推荐速度。没有被持续执行到目标速度的档位严格
标为 `NOT_EXERCISED`。

## Feature alignment

`baseline_mismatch` 保留为复现对照；正式默认使用 `feature_aligned`。
两者算法数据值只在 `Vis/FeatureType` 的 8/6 上不同。对齐后：

- `Mem/UseOdomFeatures=true` 不再因类型不匹配被 RTAB-Map core 禁用；
- OdomInfo 持续携带视觉 word，数据库出现有效 Word 和正 word reference；
- 三个独立 loop-return 数据库产生 4、4、3 条有效 GlobalClosure；
- 没有出现 lost、reset、TF 时间倒退或大于 1 m 的错误轨迹跳变。

不需要降低 LoopThr 或 inlier threshold 就能形成有效候选和几何闭环。

## 速度口径

有效速度样本必须连续至少 1 秒达到指令速度的 80%。推荐工作区：

- 水平：0.35 m/s；
- 偏航：不高于 25 deg/s；
- 垂直：不高于 0.20 m/s；
- 每个 RTAB 处理帧平移 p95：不高于约 3 cm；
- 每个 RTAB 处理帧 yaw p95：不高于约 2 deg。

0.50–0.75 m/s 水平和 40 deg/s 偏航仅作为仿真压力档。本轮未观测到可声明
的视觉 FAIL 边界，不能外推更高速度。

## 复现

```bash
cd "$HOME/projects/multi-slam-simulation"
source /opt/ros/humble/setup.bash
source install/setup.bash

# 单次性能、延迟和轨迹质量
DURATION_S=60 PROFILE_LABEL=repro \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/profile_d435i_visual_pipeline.sh

# Feature alignment 独立数据库矩阵
MATRIX_ID=feature_alignment_repro \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_feature_alignment_matrix.sh

# 速度包线和汇总
MATRIX_ID=speed_envelope_repro \
  bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_speed_envelope_matrix.sh
```

工具输出到被 Git 忽略的 `logs/d435i_visual_slam/`。数据库诊断使用 SQLite
`mode=ro` 打开目标文件。

## 限制

当前仍使用 `kms_swrast`，尚未验证真实 D435i。仿真结果不能覆盖真实曝光、
运动模糊、rolling shutter、USB 抖动、机身振动和控制延迟，也不能作为
多源融合或实机飞行验收结论。
