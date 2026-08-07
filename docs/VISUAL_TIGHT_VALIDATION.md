# Visual tight-coupling validation

## Deterministic factor-level tests

The analytic visual Jacobian matches a central manifold finite difference. The
tests also cover invalid depth/covariance, pose correction, exact RGB-D KLT/PnP
tracking, spatial distribution, source-aware conflict rejection and the rule
that RGB-D cannot overwrite LiDAR geometry.

The three-run deterministic A/B harness uses identical noisy inputs for every
mode. Values are means across seeds 0, 1 and 2:

| Mode | Translation RMSE (m) | Rotation RMSE (rad) | Final translation (m) | Runtime (s) |
|---|---:|---:|---:|---:|
| Tagged four-source factor set | 0.02453 | 0.01911 | 0.03385 | 0.0479 |
| Legacy RTAB-style relative SE(3) | 0.01078 | 0.00567 | 0.01715 | 0.1876 |
| Paper reprojection | 0.00206 | 0.00056 | 0.00294 | 0.1421 |

This is a deterministic factor-level regression, not flight evidence. The
measurements and noise are synthetic and the table must not be quoted as a
real-world accuracy result.

The source-aware map harness produced color coverage ratios 0.6712, 0.6599 and
0.6689, and supplementary-volume growth ratios 1.0068, 1.0113 and 1.0068.
Those values verify determinism and metric plumbing only.

## Build and test commands

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

## Remaining run gates

Before publication, a representative headless run must show nonzero
`visual_factors`, Native LiDAR/IMU/GNSS/flow factors remaining nonzero, and zero
optimization errors/rollbacks. The full small_rectangle, relocalization matrix,
camera perturbation experiments and online-map surveyed metrics are recorded as
BLOCKED if the local simulator cannot safely run; no threshold is relaxed to
manufacture a pass.
