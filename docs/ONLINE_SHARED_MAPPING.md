# Online RGB-D 与 LiDAR 共享建图

`uf_shared_mapping` 为可选功能，独立于飞行 estimator。它不发布 TF、不修改
FAST-LIO 的 source map，也不会阻塞 backend。

## 拓扑与一致性策略

有界 hash map 使用可配置的立方体 voxel。每个 voxel 保存几何质心、RGB
均值、LiDAR/RGB-D 观测计数、颜色计数和最后时间戳。数据源归属明确：

- LiDAR 观测创建或更新 primary geometry；
- 与已有 LiDAR geometry 一致的 RGB-D 点只增加颜色和支持计数，不移动 LiDAR 质心；
- 空 voxel 中的 RGB-D 点仅在 visual reliability weight 高于配置的最小值时创建 supplementary geometry；
- 与 primary geometry 距离超过 `conflict_distance_m` 的 RGB-D 点会被拒绝，以避免 ghosting 或动态物体；
- 只有超过 `maximum_voxels` 后才淘汰最旧 voxel。

节点将 RGB-D 帧与 `/fusion/unified/odom` 关联，使用相同的 body-camera
extrinsic，以可配置 stride 采样深度，并消费已注册的 LiDAR 点。结果发布到
`/mapping/shared/points`，且仅通过 `/mapping/shared/export` 写出。

```bash
ros2 launch uf_shared_mapping shared_mapping.launch.py \
  enabled:=true output_directory:=shared_map_output
ros2 service call /mapping/shared/export std_srvs/srv/Trigger '{}'
```

输出目录包含 `lidar_map.pcd`、`rgbd_map.pcd`、`joint_map.pcd` 和
`metrics.json`。YAML 与 launch 中的默认值均保持禁用。

## 指标

- LiDAR/RGB-D/joint/supplementary voxel 数量；
- RGB-D 冲突率（ghosting 代理指标）；
- LiDAR 颜色覆盖率；
- supplementary volume 增长率（完整性代理指标）；
- 淘汰次数与原始接受观测数。

这些是在线一致性代理指标，不能替代测量得到的地图误差。真实地图评估还需
在完整任务期间测量最近邻重叠、边界误差、重复经过产生的 ghosting 以及内存/CPU。
