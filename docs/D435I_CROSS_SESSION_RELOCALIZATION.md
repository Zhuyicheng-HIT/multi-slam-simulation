# D435i RTAB-Map 跨会话重定位基线

## 范围与定义

本实验对现有 D435i RGB-D RTAB-Map baseline 进行真正的进程边界测试。Session 1
完全停止；Session 2 启动全新的 Gazebo/SITL/MAVROS/D435i stack 和新的 RTAB-Map
进程。唯一保留的状态是磁盘上的 Session 1 RTAB-Map database。

这不是同进程 loop closure、连续建图、保留 TF 状态或 FAST-LIVO2 辅助定位。
RTAB-Map visual odometry 仍是内部 odometry source。

## Database 与定位契约

Session 1 使用 `d435i_rtabmap_feature_aligned.yaml`，以 0.20 m/s、0.50 m 飞行
高度、yaw 0 执行固定 4.50 m 的 `long_loop_return`。只有满足以下条件才接受
参考数据库：数据库可读、包含 nodes 和 visual words、包含 GlobalClosure link、
存在至少 10 个 geometry inliers 的 live closure event，且 lost/reset 为 0/0。

接受的 database 被复制为 mode-0444 mother file。每个 Session 2 在 RTAB-Map 启动
前将 mother 复制为自己的 writable database。每次尝试前后都会检查 mother 的 SHA-256。

Session 2 使用 `d435i_rtabmap_localization.yaml`：

- feature detector、feature type、odometry 和 `Vis/MinInliers=10` 与 feature-aligned mapping baseline 匹配；
- `Mem/IncrementalMemory=false` 启用 localization mode；
- `Mem/InitWMWithAllNodes=true` 将小型参考图加载到 working memory；
- `Mem/LocalizationDataSaved=false` 避免把 Session 2 当作连续 mapping；
- `delete_db_on_start=false` 保留每次尝试的 database copy；
- RGB/depth exact synchronization 和 RTAB-Map visual odometry 保持启用。

在可丢弃的 child copy 上，`Mem/LocalizationReadOnly` 特意设为 false。不可变
对象是 mother database，而不是 RTAB-Map 的运行时 SQLite handle。

## 冻结条件

机器可读定义位于 `config/d435i_relocalization_conditions.yaml`，在正式矩阵前
已经冻结。坐标是相对于公共 Gazebo spawn point 的 MAVROS local offsets。D435i
朝向前方：yaw 0 时 camera +X 与 vehicle +X 同向。

带纹理的场景在参考 corridor 周围保持开阔。所有目标都在已验证的 safety box
内，并远离墙壁和障碍物。参考地图在 y=0 处沿 x=0 至 x=4.50 m 的中心线建图。

| 条件 | x (m) | y (m) | z (m) | yaw | 用途 |
|---|---:|---:|---:|---:|---|
| `start_same` | 0.00 | 0.00 | 0.50 | 0° | 已建图起点悬停，相同视角 |
| `start_reverse` | 0.00 | 0.00 | 0.50 | 180° | 相同位置，大幅改变视角 |
| `route_middle` | 2.25 | 0.00 | 0.50 | 0° | 已建图路线中点 |
| `route_end` | 4.25 | 0.00 | 0.50 | 0° | 接近已建图终点 |
| `mapped_edge` | 3.75 | 0.45 | 0.50 | 0° | 接近已建图 corridor 边缘 |
| `similar_geometry` | 2.25 | 0.60 | 0.50 | 0° | 重复网格，横向几何偏移 |

每个目标执行 12° 或 18° 的有界 yaw sweep，然后保持原始视角。只有飞行器到达
目标后才启动 RTAB-Map，因此 Session 2 不会对定位飞行过程建图或进行 visual
odometry integration。

## 成功标准与证据

只有当 RTAB-Map 报告接受的 global 或 proximity match 且至少有 10 个 visual
inliers 时，事件才算 geometry-accepted。一次尝试只有同时满足以下条件才算
重定位成功：

- 接受的 node 与 GT-relative mapped position 的距离小于 1.25 m；
- map-aligned pose 连续五个 sample 的位置误差不超过 0.75 m、yaw 误差不超过 45°；
- 不存在视觉相似但几何错误的候选项；
- 对齐后没有超过 1 m 的位置跳变；
- lost/reset 和 TF backward-jump 计数均为零；
- child database 可读且 mother hash 未改变；
- 飞行、着陆、进程清理、active-marker 清理和端口审计全部完成。

监视器记录 candidate IDs、matched IDs、map IDs、posterior/likelihood、visual
matches、visual words、geometry inliers、closure transform、map-to-odom、RTAB
odometry、GT、TF 以及事件时间。单独一行 GlobalClosure 日志不能作为成功标准。

## 冒烟结果

实验 `cross_session_v1_smoke_20260730_01` 通过：

- Session 1：160 个 nodes、3915 个 words、34 个 GlobalClosure database links、
  20 个经过 geometry 验证的 live closure events，最多 70 个 inliers，lost/reset 0/0；
- Session 2 `start_same`：在 map 0 中匹配 node 133，76 个 geometry inliers 和 214 个 visual words；
- 从第一个 RTAB-Map Info event 起，稳定对齐延迟 0.178 s；
- 稳定位置/yaw 误差 0.066 m / 0.064°；
- map-to-odom 最大平移跳变 0.050 m；
- 异常对齐后跳变、lost、reset 和 TF backward jump 均为零；
- 参考 mother SHA-256 未改变，两个 session 均完成 active/PID/port 清理。

## 运行

构建并 source 工作空间后，使用独立 reference map 运行冒烟测试：

```bash
MATRIX_ID=cross_session_v1_smoke \
CROSS_SESSION_CONDITIONS=start_same \
VALID_RUNS_PER_CONDITION=1 \
MAX_ATTEMPTS_PER_CONDITION=3 \
REQUIRE_SUCCESS=1 \
bash install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_d435i_cross_session_matrix.sh
```

正式矩阵使用同一脚本，包含默认的六个条件，每个条件三次有效尝试。可通过
传入 `REFERENCE_DB` 和 `REFERENCE_METADATA` 复用已验证的 mother；每个 Session 2
仍会获得全新的 child copy。
