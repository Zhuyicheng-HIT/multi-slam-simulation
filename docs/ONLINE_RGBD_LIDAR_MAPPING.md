# Online RGB-D 与 LiDAR 建图

`uf_shared_mapping` 是一个有界、感知数据源的局部 voxel map。默认禁用，
不发布 TF，不修改 FAST-LIO 的 map，只写入明确选择的输出目录。

- LiDAR 拥有 primary geometry 并更新其质心；
- 可靠 RGB-D 为 LiDAR voxel 着色，不移动 LiDAR 质心；
- 可靠 RGB-D 可以补充空 voxel；
- 与附近 LiDAR geometry 不一致的 RGB-D 点会被拒绝，不会覆盖 geometry；
- 每个 voxel 记录 source counts、颜色支持度和最后时间戳；
- 通过 `/mapping/shared/export` 导出 LiDAR-only、RGB-D-only、joint PCD 文件及 JSON metrics。

输入为已注册 LiDAR 点、exact RGB/16UC1 depth/CameraInfo、unified body pose
和 D_V。输出为 `/mapping/shared/points`、`lidar_map.pcd`、`rgbd_map.pcd`、
`joint_map.pcd` 以及 `metrics.json`。

三次确定性运行产生 441 个 LiDAR voxel、885/887/885 个 joint voxel，颜色覆盖率
为 0.6712/0.6599/0.6689，supplementary volume 增长为 1.0068/1.0113/1.0068。
由于该数据集刻意保持一致，冲突率为零；独立 unit tests 验证冲突的 RGB-D
不会覆盖 LiDAR geometry。这些是 software-contract 指标，不代表测量得到的地图精度。

与 HybridFusion 不同，本模块在地图生成过程中使用当前 unified pose 融合源观测。
HybridFusion 仍是离线、事后 cross-map registration baseline，使用 block descriptors、
NDT 和 transform clustering。
