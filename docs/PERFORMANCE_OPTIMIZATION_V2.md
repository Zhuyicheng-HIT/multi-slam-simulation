# Ultra-Fusion 性能 V2 改动

## 保留的改动

1. 为 backend callbacks、nonlinear factors、process groups 和 shared-map updates 添加有界、可选的 stage profiling。默认不采样，也不生成逐 factor JSON stream。
2. 将 dense marginal-prior block-diagonal Jacobian 构造及 `J.T @ H @ J` 运算替换为代数等价的 3x3 rotation block transform。直接 dense 与 block 相等性测试保护该改动。
3. 将不可变 factor payload arrays 的事务性 `deepcopy` 替换为复制 factor dictionaries。State objects 仍为 deep copy；isolation test 覆盖 marginalization 与 rollback 时的 index replacement。
4. 使用 NumPy 对 analytic RGB-D reprojection residuals 和 Jacobians 向量化。运行时不使用 finite differences；finite-difference Jacobian equality test 仍在测试套件中。
5. 对 shared-map finite filtering、voxel-index 计算和颜色裁剪进行 batch 化，同时保留逐 voxel 的 sequential confidence 与 source update 语义。
6. 使用 structured NumPy arrays 打包 PointCloud2 数据，避免构造 Python tuple 列表。
7. 为窄范围 Pareto scan 增加 `balanced_light`（0.24 s）和 `balanced_plus`（0.16 s）。生产环境仍使用现有 `balanced` 0.20 s cadence。
8. benchmark/build handoff 选择 `RelWithDebInfo`，保留 native optimization 和 symbols；未移除 assert、integrity 或 rollback logic。

## 已回退的实验

原地 full-Hessian factor accumulation 实验将 graph linearization P50 从 7.764 提高到
8.406 ms，将 solver median 从 52.741 提高到 59.869 ms，并把 RTF 从约 0.436 降至 0.413，
因此已完全回退。

## 隔离效果

精确的 marginal transform 将 graph linearization P50 从 8.406 降至 5.851 ms。Snapshot
改动将自身 P50 从 3.943 降至 0.303 ms（92.3%）。向量化 reprojection 将 visual-factor
P50 从 1.822 降至 0.475 ms（73.9%）。Map batching 在受控 A/B 中将 LiDAR integration
P50 从 12.511 降至 6.678 ms、RGB-D integration 从 70.757 降至 26.023 ms；structured
publication 随后将 publication P50 从 15.011 降至 7.894 ms。

所有 algorithmic changes 均可迁移到真实硬件。没有改动 Gazebo physics、sensor rate、
timestamp、LiDAR geometry sample cap、D_V threshold、association gate、integrity
threshold、rollback policy 或 map parameters。
