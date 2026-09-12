# Visual 紧耦合验证

## 确定性因子级测试

解析形式的视觉 Jacobian 与中心流形有限差分结果一致。测试还覆盖无效深度/协方差、位姿校正、精确 RGB-D KLT/PnP 跟踪、空间分布、按来源冲突拒绝，以及 RGB-D 不得覆盖 LiDAR 几何的规则。

三次运行的确定性 A/B harness 对每种模式使用完全相同的噪声输入。数值为 seed 0、1、2 的均值：

| 模式 | 平移 RMSE (m) | 旋转 RMSE (rad) | 最终平移 (m) | 运行时间 (s) |
|---|---:|---:|---:|---:|
| Tagged four-source factor set | 0.02453 | 0.01911 | 0.03385 | 0.0479 |
| Legacy RTAB-style relative SE(3) | 0.01078 | 0.00567 | 0.01715 | 0.1876 |
| Paper reprojection | 0.00206 | 0.00056 | 0.00294 | 0.1421 |

这是确定性的因子级回归测试，不代表飞行证据。测量值和噪声均为合成数据，不能作为真实世界精度结果引用。

按来源的地图 harness 产生了 0.6712、0.6599、0.6689 的颜色覆盖率，以及 1.0068、1.0113、1.0068 的补充体积增长率。这些数值只用于验证确定性和指标处理流程。

## 构建与测试命令

```bash
source /opt/ros/humble/setup.bash
source /home/zyc/multi-slam-deps/mid360_ws/install/setup.bash
colcon build --symlink-install
colcon test
python3 -m pytest -q \
  src/ultra_fusion_nav/uf_backend_fusion/test/test_visual_reprojection.py \
  src/ultra_fusion_nav/uf_visual_frontend/test/test_feature_tracker.py \
  src/ultra_fusion_nav/uf_shared_mapping/test/test_voxel_map.py
```

## 仍需满足的运行门槛

发布前，代表性的 headless 运行必须显示非零的 `visual_factors`，Native LiDAR/IMU/GNSS/flow 因子保持非零，并且优化错误/回滚为零。如果本地 simulator 无法安全运行，则完整的 small_rectangle、relocalization 矩阵、camera perturbation 实验和 online-map 指标会记录为 BLOCKED；不会通过放宽阈值来人为制造通过结果。
