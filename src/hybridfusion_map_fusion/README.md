# HybridFusion map-level fusion

This package is an opt-in, offline visual-LiDAR map fusion module derived from
Wang et al., *HybridFusion: LiDAR and Vision Cross-Source Point Cloud Fusion*
(2023). It does not replace or modify FAST-LIO, RTAB-Map, D_V, the PR #6
unified localization backend, or any flight-control path.

Both launches set `enabled:=false` by default. The package publishes no TF and
never overwrites either source map. A registration failure returns a non-zero
offline exit code and writes only to its selected result directory.

## Existing inputs reused

| Input | Default interface | Stored result |
|---|---|---|
| Normalized D435i RGB | `/sensors/rgbd/color` | RGB-D keyframe PCD files |
| Normalized D435i depth | `/sensors/rgbd/depth` (`16UC1` or `32FC1`) | `visual_map.pcd` |
| D435i calibration | `/front/d435i/color/camera_info` | `camera_calibration.yaml` |
| RTAB pose and calibration TF | `odom -> front_d435i_color_optical_frame` | `keyframes.csv` |
| FAST-LIO map | `/fastlio_denoised_map`, normally `camera_init` | `lidar_map.pcd` |

The RGB-D exporter performs pinhole back-projection itself. It does not depend
on, start, or display a live D435i `PointCloud2` topic.

## Live map export (explicit opt-in)

```bash
ros2 launch hybridfusion_map_fusion hybridfusion_export.launch.py \
  enabled:=true output_root:=/absolute/output/export

ros2 service call /hybridfusion_rgbd_map_exporter/save std_srvs/srv/Trigger '{}'
ros2 service call /hybridfusion_lidar_map_exporter/save std_srvs/srv/Trigger '{}'
```

The visual exporter accepts only exact RGB/depth stamps, requires CameraInfo
and the requested TF at the same timestamp, applies keyframe translation/time/
rotation gates, stores calibration and poses, and then downsamples a copy. The
LiDAR exporter stores the latest complete FAST-LIO denoised map and its frame,
stamp and topic metadata. Source messages and maps are never mutated.

To collect both maps while reusing the existing PR #6/PR #8 headless stack,
flight lifecycle and 6 m x 4 m guided rectangle, use the opt-in wrapper below.
It refuses to start when that stack already owns an active marker, enables no
point-cloud display, and terminates only the process groups it launched.

```bash
ros2 run hybridfusion_map_fusion collect_hybridfusion_simulation.sh \
  /absolute/output/live_building_route
```

The wrapper starts the exporters before the existing stack and flight route,
saves both maps after landing, writes a relative-path `dataset.yaml` with an
identity coarse pose, and keeps the full underlying lifecycle logs. That pose
is an explicit engineering starting point, not a ground-truth transform.

## Offline registration

The dataset manifest contains relative PCD paths plus the initial and optional
ground-truth transforms as `[x,y,z,roll,pitch,yaw]` in metres/radians. When
truth is absent, `ground_truth_available` is false and pose-error fields are
JSON `null`; geometric overlap, boundary, inlier and supplement metrics remain
available.

```bash
ros2 run hybridfusion_map_fusion hybridfusion_offline \
  --dataset /data/dataset.yaml \
  --config $(ros2 pkg prefix hybridfusion_map_fusion)/share/hybridfusion_map_fusion/config/hybridfusion.yaml \
  --method hybrid --output /results/hybrid
```

Supported comparison methods are `initial`, `gicp`, and `hybrid`. Every output
directory contains `result.json`, `transform.yaml`, `aligned_lidar.pcd`,
`fused_map.pcd`, and `run_manifest.yaml`. No TF is broadcast.

## Reproducible building-scale benchmark

```bash
export HYBRIDFUSION_WORKSPACE_SETUP=/workspace/install/setup.bash
ros2 run hybridfusion_map_fusion run_hybridfusion_benchmark.sh \
  /workspace/logs/hybridfusion/benchmark_v1
```

The generator executes a front-and-side 30-keyframe route around a scene with
ground, two connected buildings, roofs, façades, a curb and columns. Visual and
LiDAR maps have distinct density, noise and missing-surface patterns but share
a known overlap and SE(3) truth. It is a deterministic regression simulator,
clearly marked `generated_not_measured`; the live exporters remain the path for
Gazebo or hardware data. All three runs and failures are retained.

## Algorithm and threshold provenance

1. PCL VoxelGrid preprocessing and metadata-provided/GNSS coarse alignment.
2. Common XY grid; `cell_size_m: 0` implements the paper's scene-size/10 rule.
3. Minimum-point valid blocks and GNSS-origin radial/angle candidate filtering.
4. PCL ESF640 and Pearson correlation. The code refuses a threshold below the
   paper's explicit `0.60` criterion.
5. Descriptor neighborhood consistency filter.
6. Ground-height filtering, XY projection, and occupancy boundary extraction.
7. A dedicated Gaussian-grid SE(2) NDT on planar boundaries. This avoids the
   singular-neighborhood failure caused by feeding zero-Z clouds to PCL's 3D
   NDT while preserving the paper's XY/yaw registration contract.
8. PCL 3D NDT for each surviving local patch pair.
9. Translation/rotation clustering, largest-cluster translation mean and
   quaternion SLERP mean, followed by guarded full-map 3D NDT refinement.
10. Final LiDAR-to-visual SE(3), aligned LiDAR copy and voxelized fused map.

The paper does not publish numerical voxel size, valid-point count, lambda,
theta, neighbor threshold, ground height, NDT settings, or clustering epsilon/
omega. Each is an annotated YAML parameter. Defaults are conservative values
for the existing 0.10 m simulated MID360 map and building-scale world; they are
engineering choices, not claimed paper results.

After neighborhood filtering, only the highest combined descriptor/neighborhood
score for each source block is registered, with a configurable global cap. This
does not change the paper's 0.60 acceptance criterion; it bounds duplicate NDT
hypotheses that share the same source patch.

## Evaluation

`result.json` records convergence, SE(3) translation/rotation error, overlapping
nearest-neighbor mean/RMSE, boundary error, inlier ratio, occupied-voxel map
growth, runtime, peak RSS, and candidate/success/failure block counts. The
aggregator includes every run instead of selecting the best one.

Detailed system ownership and the paper-to-code trace are in
`docs/HYBRIDFUSION_MAP_FUSION_ARCHITECTURE.md`.
