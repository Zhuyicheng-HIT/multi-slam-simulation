# Ultra-Fusion 论文到代码审计

## 基线与范围

本工作仅基于 tag `ultra-fusion-four-source-reloc-stable-20260806`，解析到 commit `57930c86d7d96468b3416f84f8e6f504f527df8a`。当时 Stage3 branch 后来还有一个 map-visualization commit，本审计有意未使用。只有不替换 tagged backend 的部分才迁移现有 D435i/RTAB 与 HybridFusion code。

## 论文步骤实现矩阵

| 论文组件 | Tagged baseline | 当前分支 | 状态 |
|---|---|---|---|
| Shared navigation state `R,p,v,b_a,b_g` | 15-state SO(3) fixed-lag window | 不变 | real，compact reproduction |
| IMU preintegration | bias-aware manifold residual | 不变 | real |
| LiDAR point-to-plane factor | raw/condensed `NativeLidarFactor` | 不变 | real |
| GNSS pseudorange/Doppler | 仅 ENU position anchor | 不变 | paper gap |
| Optical/wheel factor | compensated optical-flow displacement | 不变 | adapted，不是 wheel odometry |
| Visual reprojection Eq. (10) | 无 | RGB-D inverse-depth reprojection between two window states | real V1 |
| Visual landmark state | 无 | depth 是带 variance 的 fixed measured anchor | partial |
| Visual FRS Eq. (20) | image/depth proxy | real tracks、grid occupancy、reprojection、KLT evidence | real/adapted |
| Camera temporal offset | 无 | configured timestamp correction | interface real，calibration 未估计 |
| Camera extrinsic | 无 | calibrated body-camera transform parameter | interface real，calibration 未估计 |
| LiDAR-IMU online calibration | shadow diagnostics | 不变 | prototype，application locked off |
| OAI | startup/observability gates | 不变 | partial reproduction |
| Relocalization | static LiDAR keyframes + migrated RTAB workflow | 保留旧 workflow 修复 | real workflow，非 paper-identical |
| Geometric/color map | FAST-LIO map | opt-in source-aware LiDAR/RGB-D voxel map | engineering V1 |

## 已实现的视觉方程

以 camera frame `C_i` 中、normalized coordinate `u_i=[x_i,y_i,1]^T` 和 measured inverse depth `rho_i` 锚定的 feature，其固定 3D anchor 为 `p_Ci = u_i / rho_i`。使用 calibrated `T_BC` 与 optimized body pose `T_WBi,T_WBj`，预测为 `p_Cj = T_BC^-1 T_WBj^-1 T_WBi T_BC p_Ci`，残差为 `r_ij = pi(p_Cj) - u_j`。这是适配项目 compact window 的论文双帧 visual geometry。使用 analytic right-local SE(3) pose Jacobian，并用 central finite difference 检查；每个 normalized image coordinate 应用 Huber loss。有效 information 仍乘以现有 FRS decision `reliability_weight / covariance_inflation`。

Depth uncertainty 显式计入 factor variance：configured normalized pixel variance 加 scaled measured inverse-depth variance。Depth 不会静默升级为 optimized landmark，因此不宣称 full bundle adjustment。

## Visual reliability

主 degradation score 保留 tagged Eq. (20) adaptation：`D_V = 0.30 D_count + 0.25 D_grid + 0.25 D_reprojection + 0.20 D_depth`。Forward-backward KLT consistency 是显式扩展：`D_V_final = 0.85 D_V + 0.15 (1-r_KLT)`。

PnP/RANSAC 仅作为 geometric validity gate 和 reprojection evidence source，其 pose 不会作为第二 factor 插入。Paper-reprojection mode 不启用 RTAB odometry，避免同源 double weighting。

## Calibration 与 observability 契约

- `visual_time_offset_s` 在匹配两个 backend state 前修正 camera timestamp，V1 中固定。
- `visual_rotation_body_camera` 与 `visual_translation_body_camera_m` 是测量得到的 body-from-camera extrinsic，Identity 仅是 placeholder。
- Online camera calibration 有意延期。解锁前需满足 multi-axis rotation、translation parallax、有界 Hessian condition number、split window 中可重复估计，以及与 held-out reprojection set 一致。
- LiDAR-IMU calibration 仍是 tagged shadow-only implementation。
- Timestamp mismatch、depth-valid KLT+PnP inlier 太少、invalid covariance、non-finite geometry、scheduler disable 或 point 投影到 camera 后方时拒绝 factor。Threshold 属于 configuration，不是论文结论。

## 仅作架构参考的外部系统

- FAST-LIVO2：unified voxel map 与 direct visual/LiDAR update；GPL-2.0。
- R3LIVE：FAST-LIO geometry + image colorization；GPL-2.0。
- LVI-SAM：modular visual-inertial 与 LiDAR-inertial smoothing；BSD-3-Clause。
- VINS-RGBD：RGB-D inverse-depth/depth-aided VINS；GPL-3.0。
- Ground-Fusion：RGB-D/IMU/wheel/GNSS factor graph；GPL-3.0。
- LIC-Fusion：带 online spatiotemporal calibration 的 sparse camera/LiDAR/IMU fusion。

未复制外部 implementation code。Repository 仍为 Apache-2.0，只有 high-level design comparison 影响本工作。

## 已知非论文项与未完成项

Tagged backend 仍缺 original pseudorange/Doppler GNSS、optimized visual landmark、full cross-factor covariance propagation 和 observable online camera calibration。Shared RGB-D map 是 engineering extension；论文描述 geometric/color mapping architecture，但没有定义这套 exact depth-conflict policy。这些限制必须继续出现在后续 PR description 中。
