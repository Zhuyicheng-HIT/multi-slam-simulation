# Ultra-Fusion paper-to-code audit

## Baseline and scope

This work is based only on tag `ultra-fusion-four-source-reloc-stable-20260806`,
which dereferences to commit `57930c86d7d96468b3416f84f8e6f504f527df8a`.
The then-current Stage3 branch had one later map-visualization commit; it was
intentionally not used. Existing D435i/RTAB and HybridFusion code was migrated
only where it did not replace the tagged backend.

## Paper-step implementation matrix

| Paper component | Tagged baseline | This branch | Status |
|---|---|---|---|
| Shared navigation state `R,p,v,b_a,b_g` | 15-state SO(3) fixed-lag window | unchanged | real, compact reproduction |
| IMU preintegration | bias-aware manifold residual | unchanged | real |
| LiDAR point-to-plane factor | raw or condensed `NativeLidarFactor` | unchanged | real |
| GNSS pseudorange/Doppler | ENU position anchor only | unchanged | paper gap |
| Optical/wheel factor | compensated optical-flow displacement | unchanged | adapted, not wheel odometry |
| Visual reprojection Eq. (10) | absent | RGB-D inverse-depth reprojection between two window states | real V1 |
| Visual landmark state | absent | depth is a fixed measured anchor with variance | partial |
| Visual FRS Eq. (20) | image/depth proxy | real tracks, grid occupancy, reprojection and KLT evidence | real/adapted |
| Camera temporal offset | absent | configured timestamp correction | interface real, calibration not estimated |
| Camera extrinsic | absent | calibrated body-camera transform parameter | interface real, calibration not estimated |
| LiDAR-IMU online calibration | shadow diagnostics | unchanged | prototype; application remains locked off |
| OAI | startup and observability gates | unchanged | partial reproduction |
| Relocalization | static LiDAR keyframes plus migrated RTAB workflow | final old workflow fixes retained | real workflow, not paper-identical |
| Geometric/color map | FAST-LIO map | opt-in source-aware LiDAR/RGB-D voxel map | engineering V1 |

## Implemented visual equations

For a feature anchored in camera frame `C_i` with normalized coordinate
`u_i=[x_i,y_i,1]^T` and measured inverse depth `rho_i`, the fixed 3-D anchor is

`p_Ci = u_i / rho_i`.

With calibrated `T_BC` and optimized body poses `T_WBi,T_WBj`, prediction is

`p_Cj = T_BC^-1 T_WBj^-1 T_WBi T_BC p_Ci`,

`r_ij = pi(p_Cj) - u_j`.

This is the paper's two-frame visual geometry, adapted to the project's compact
window. Analytic right-local SE(3) pose Jacobians are used and checked against
central finite differences. A Huber loss is applied per normalized image
coordinate. The effective information is still multiplied by the existing FRS
decision `reliability_weight / covariance_inflation`.

Depth uncertainty is not hidden: the factor variance is the configured
normalized pixel variance plus a scaled measured inverse-depth variance. Depth
is not silently promoted to an optimized landmark, so this branch does not
claim full bundle adjustment.

## Visual reliability

The primary degradation score keeps the tagged Eq. (20) adaptation:

`D_V = 0.30 D_count + 0.25 D_grid + 0.25 D_reprojection + 0.20 D_depth`.

Forward-backward KLT consistency is then an explicitly documented extension:

`D_V_final = 0.85 D_V + 0.15 (1-r_KLT)`.

PnP/RANSAC is only a geometric validity gate and reprojection-evidence source;
its pose is never inserted as a second factor. RTAB odometry is not enabled in
the paper-reprojection mode, preventing same-source double weighting.

## Calibration and observability contract

- `visual_time_offset_s` corrects camera timestamps before matching the two
  backend states. It is fixed in V1.
- `visual_rotation_body_camera` and `visual_translation_body_camera_m` are
  measured body-from-camera extrinsics. Identity is only a placeholder.
- Online camera calibration is deliberately deferred. Before unlocking it,
  require multi-axis rotation, translation parallax, a bounded Hessian
  condition number, repeatable estimates in split windows, and consistency
  against a held-out reprojection set.
- LiDAR-IMU calibration remains the tagged shadow-only implementation.
- A factor is rejected for timestamp mismatch, too few depth-valid KLT+PnP
  inliers, invalid covariance, non-finite geometry, scheduler disable, or point
  projection behind the camera. Thresholds are configuration, not paper claims.

## External systems used only as architectural references

- FAST-LIVO2: unified voxel map and direct visual/LiDAR update; GPL-2.0.
- R3LIVE: FAST-LIO geometry plus image colorization; GPL-2.0.
- LVI-SAM: modular visual-inertial and LiDAR-inertial smoothing; BSD-3-Clause.
- VINS-RGBD: RGB-D inverse-depth/depth-aided VINS; GPL-3.0.
- Ground-Fusion: RGB-D/IMU/wheel/GNSS factor graph; GPL-3.0.
- LIC-Fusion: sparse camera/LiDAR/IMU fusion with online spatiotemporal
  calibration.

No external implementation code was copied. The repository remains
Apache-2.0 and only the high-level design comparisons informed this work.

## Known non-paper and incomplete items

The tagged backend still lacks original pseudorange/Doppler GNSS, optimized
visual landmarks, full cross-factor covariance propagation, and observable
online camera calibration. The shared RGB-D map is an engineering extension:
the paper describes a geometric/color mapping architecture but does not specify
this exact depth-conflict policy. These limitations must remain visible in any
future PR description.
