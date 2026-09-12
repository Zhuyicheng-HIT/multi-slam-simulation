# Ultra-Fusion 论文差距分析

## 审计基线

唯一代码基线是标注 tag `v0.1.0-four-source-reloc-calibration`：tag object
`4325b5bdcbe588110039a1b578e282a9e7d03c89`，解析后的 commit
`57930c86d7d96468b3416f84f8e6f504f527df8a`，tree
`1eca12f9266ad0e318d6ffa7453109ea01547256`。更新的 Stage3 tip 已审计但未合并。

| 能力 | 稳定 tag | 当前分支 | 分类 |
|---|---|---|---|
| SO(3) sliding-window 状态 `R,p,v,b_a,b_g` | 已实现 | 不变 | PASS，紧凑复现 |
| Native LiDAR point-to-plane factor | 已实现 | 不变 | PASS |
| IMU preintegration | 已实现 | 不变 | PASS |
| GNSS | ENU position anchor | 不变 | PARTIAL：不是论文伪距/Doppler |
| Optical flow | body displacement constraint | 不变 | PARTIAL：论文适配 |
| FRS/OAI | scheduler 与 observability gates | vision 加入同一 FRS | PARTIAL 复现 |
| LiDAR-IMU calibration | shadow-only | 不变 | PARTIAL；应用仍锁定 |
| Visual reprojection | 无 | inverse-depth two-state factor | PASS，V1 几何 |
| Optimized visual landmarks | 无 | 固定测量的 RGB-D depth | PARTIAL |
| Camera time offset | 无 | 固定修正接口 | PARTIAL；未在线估计 |
| Camera-IMU extrinsic | 无 | 测量参数接口 | PARTIAL；未在线估计 |
| Relocalization | LiDAR workflow | 保留 RTAB persistence workflow | 工具 PASS；当前 runtime 重跑 PARTIAL |
| Geometric/color map | FAST-LIO 几何 | source-aware RGB-D/LiDAR voxel map | PASS，确定性 V1 |

视觉残差使用论文的双帧 inverse-depth 几何：`p_Ci = [x_i,y_i,1]^T/rho_i`，
`p_Cj = T_BC^-1 T_WBj^-1 T_WBi T_BC p_Ci`，以及
`r = pi(p_Cj)-[x_j,y_j]^T`。实现了 right-local analytic pose Jacobian、normalized
pixel covariance、inverse-depth uncertainty、Huber loss 和 FRS information scaling。
Depth 有意保持为固定测量值，不宣称是优化 landmark。

## 相关系统设计评审

FAST-LIVO2 与 R3LIVE 参考了几何/颜色地图单一所有权；LVI-SAM 参考模块化 factor
ownership；VINS-RGBD 参考 depth-aided inverse-depth 初始化；Ground-Fusion 与
LIC-Fusion 参考显式 spatiotemporal calibration gates。未复制外部 source code。
许可证：FAST-LIVO2/R3LIVE 为 GPL-2.0，LVI-SAM 为 BSD-3-Clause，
VINS-RGBD/Ground-Fusion 为 GPL-3.0。

## 尚存的论文差距

- 尚未实现原始 GNSS pseudorange/Doppler model。
- 尚未联合优化 inverse-depth landmarks。
- 尚无可观的 online camera extrinsic/time-offset estimator。
- 尚未完成 full cross-factor covariance propagation。
- 共享地图 conflict rule 属于 engineering V1；论文未定义这套具体 RGB-D voxel policy。
- 最终 headless flight 在三轮修复/验证后仍无法提交第一个 native-triggered state，
  因此 flight-level visual acceptance、ATE/RPE 和 degraded-scene 结果保持 PARTIAL，
  不虚构结果。
