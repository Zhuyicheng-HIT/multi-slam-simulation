# HybridFusion map-level fusion 验证

## 范围与可复现性

验证于 2026-08-05 基于未修改的 PR #6/PR #8 baseline `bbe5fe1f068c09244e25e6ce7e80165c05d57928` 完成。生成的 map 和 run artifact 在 Git 中被有意忽略，路径为 `logs/hybridfusion/formal_20260805`。已提交的 generator 使用 seed `20260805` 确定性地重建输入：

- 沿 building-scale route 的 30 个 front-and-side RGB-D keyframe；
- 主 building、annex、ground、roof、facade、curb 和 column；
- 20,953 个 visual 点和 37,539 个 LiDAR 点，具有不同的 visibility、density 和 noise model；
- 真实 LiDAR-to-visual pose：`[1.2, -0.8, 0.22, 1 deg, -1.5 deg, 8 deg]`；
- coarse pose：`[0.86, -0.46, 0.10, 0 deg, 0 deg, 4.5 deg]`；
- visual map SHA-256：`8f66f9c4888e905a58ac44e0bd949100fd9c23c9f914ff76d9ee35f6af4a41a3`；
- LiDAR map SHA-256：`d5d03e31d274bbf6ca30f3c576898ca051835aee967108dcff7cf1941e8c1de9`。

这是生成的 simulation data，不是实测的 Gazebo 或 hardware 结果。之所以使用它，是因为任务明确允许自动生成，且它提供精确的 SE(3) truth。可选择启用的 live collection wrapper 另外经过 syntax/lifecycle 检查，并复用现有 headless stack 和 guided rectangle，不修改任一 baseline package。

## 三种方法、三次运行的结果

每种 method 都在新 process 中运行三次，全部九个结果均已纳入。`+/-` 表示总体 standard deviation，而不是 cherry-picked spread。

| 方法 | 收敛 | 平移误差 (m) | 旋转误差 (deg) | Overlap NN 均值 (m) | Boundary error (m) | Inlier ratio | Voxel supplement growth | 运行时间 (ms) | Peak RSS (MiB) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Coarse pose | 3/3 | 0.49558 +/- 0.00000 | 3.94849 +/- 0.00000 | 0.34066 +/- 0.00000 | 0.54381 +/- 0.00000 | 0.47412 +/- 0.00000 | 1.35555 +/- 0.00000 | 149.00 | 56.42 |
| GICP baseline | 3/3 | 0.00281 +/- 0.00000 | 0.02352 +/- 0.00000 | 0.17504 +/- 0.00000 | 0.45368 +/- 0.00000 | 0.67105 +/- 0.00000 | 1.18654 +/- 0.00000 | 18619.22 | 62.70 |
| HybridFusion | 3/3 | 0.08376 +/- 0.08027 | 0.31432 +/- 0.11152 | 0.20209 +/- 0.02761 | 0.46378 +/- 0.00897 | 0.66895 +/- 0.00230 | 1.14139 +/- 0.03084 | 142349.90 | 61.73 |

HybridFusion 每次运行的平移误差为 `0.03356`、`0.02069` 和 `0.19703` m；旋转误差为 `0.42545`、`0.16185` 和 `0.35567` deg。成功/失败的 local block 分别为 `16/2`、`15/3` 和 `16/2`。因此虽然收敛率为 3/3，但在该确定性场景中稳定性较低，速度也明显慢于 GICP baseline。GICP 在此处精度最高。HybridFusion 将 coarse pose 的平均平移误差改善了 0.412 m，将 overlap mean 降低了 0.139 m，并将 inlier ratio 提高了 0.195。

HybridFusion fused map 增加的 occupied voxel 数量相当于 visual map 数量的 114.14%，代表互补的 LiDAR ground/facade coverage。coarse 结果 135.55% 的更大增长由错位造成，不能报告为更好的地图质量。GICP 的 118.65% 结果是本数据集中对齐最佳的补充比较。

## 实现/验证迭代

1. 第一版实现将 planar zero-Z boundary cloud 传给 PCL 3D NDT，并在 `KdTreeFLANN::radiusSearch` 中可重复崩溃。未接受任何结果。
2. 专用 SE(2) Gaussian-grid NDT 修复了奇异性，但对每个重复 descriptor pair 进行 registration 超过 8 分钟。所属 process 在 523 秒后停止，未产生可接受输出。
3. Neighborhood consistency 现在为每个 source block 选择 combined descriptor score 最高的项，并设置 18 个 local registration 的上限。论文明确的 ESF correlation threshold 仍为 0.60。正式矩阵随后完成 9/9 个 process，HybridFusion 达到 3/3。

失败 block 是真实的 local 2D/3D registration rejection，原因包括 boundary support 不足、未收敛或未达到配置的 local fitness；它们会被计数，不会通过降低 threshold 转换为成功。

## 隔离与回归 contract

- 两个 source PCD 均不修改；aligned 与 fused cloud 为新文件。
- Offline executable 不发布 topic 或 TF，并记录 `published_as_tf: false`。
- 两个 launch file 默认 `enabled:=false`；禁用的 exporter 不订阅任何 source data。
- Live wrapper 会拒绝已有 lifecycle marker，并只清理其拥有的 process group。
- 不修改 PR #6 backend、FAST-LIO、GNSS、optical-flow、D_V、RTAB、flight 或 TF code。完整 build/test 和 lifecycle 证据记录在与此 commit 关联的最终任务报告中。
