# HybridFusion map-level fusion architecture

## Isolation from PR #6 and PR #8

`hybridfusion_map_fusion` is a new leaf package. No PR #6 backend, reliability,
FAST-LIO, RTAB-Map, D435i bridge, scheduler, TF owner, flight controller or
launch default is changed. The package is absent from the existing integration
launch and its own launch is disabled unless `enabled:=true` is supplied.

```text
existing PR #8 visual stack                    existing PR #6 LiDAR stack
RGB + 16UC1 depth + CameraInfo                 /fastlio_denoised_map
RTAB pose + existing TF                                    |
             |                                             |
             v                                             v
  rgbd_map_exporter (no cloud topic)             lidar_map_exporter
             |                                             |
 visual PCD + keyframes + calibration             LiDAR PCD + frame/stamp
             +--------------------+------------------------+
                                  |
                    hybridfusion_offline (separate process)
                                  |
       result JSON + SE(3) YAML + aligned copy + fused PCD
```

Export and registration errors cannot stop or reconfigure the publishers. The
module does not broadcast the estimated transform; an operator must review an
offline result before any future downstream use.

## Paper-to-implementation trace

| Paper step | Local implementation | Status/provenance |
|---|---|---|
| Module A visual reconstruction | Existing D435i/RTAB stack; `rgbd_map_exporter.cpp` back-projects exact RGB-D keyframes using CameraInfo and timestamped TF | Implemented by reuse, no second visual odometry |
| Module B LiDAR SLAM | Existing MID360, FAST-LIO and reliable denoised map | Reused unchanged |
| GNSS/existing-pose rough match | `dataset.yaml: initial_lidar_to_visual` | Implemented; source may be GNSS or calibrated poses |
| VoxelGrid preprocessing | `voxel_downsample()` | Implemented; leaf size is an engineering parameter |
| Grid patches | `make_blocks()` with common origin | Implemented; automatic scene-size/10 follows paper text |
| Significant/valid blocks | `grid.min_points` | Implemented; count absent from paper and parameterized |
| Radial/angle candidate range | centroid radius, lambda ring and angle filter | Implemented; lambda/theta absent from paper and parameterized |
| ESF640 patch filtering | PCL ESF plus Pearson correlation | Implemented; threshold cannot be below paper Eq. (6) value 0.60 |
| Neighbor filter | same-offset neighboring ESF correlations | Equivalent implementation; numerical threshold parameterized |
| Spliced patch neighborhoods | `collect_neighborhood()` | Implemented |
| Ground removal and XY boundary | height quantile, `d>h`, occupancy boundary cells | Equivalent PCL-compatible implementation; h/raster parameterized |
| 2D NDT | Gaussian-cell likelihood and Gauss-Newton in SE(2), output constrained to XY/yaw | Implemented; avoids singular PCL 3D-NDT neighborhoods on zero-Z input |
| 3D NDT | PCL NDT on local 3D neighborhoods | Implemented |
| Local transformation set K | one SE(3) per converged 2D-3D registration | Implemented with failures retained |
| Translation/rotation clustering | connected components under epsilon/omega | Implemented; both thresholds parameterized |
| Pose fusion | translation mean plus iterative quaternion SLERP | Implemented as described after paper Eq. (7) |
| Minor full-map adjustment | guarded full-map 3D NDT | Implemented; rejected if NN fitness degrades beyond configured ratio |
| Fused map and supplement metric | source transformed into target frame, union voxel map | Implemented without altering either input |

## Coordinate and file contracts

- Estimated transform direction is always `visual_frame <- lidar_frame`.
- Live visual points are stored directly in the selected RTAB/global TF frame.
- The FAST-LIO PCD remains in its message frame, normally `camera_init`.
- Dataset manifests and transform YAML use metres and radians in
  `[x,y,z,roll,pitch,yaw]` order.
- `transform.yaml` explicitly records `published_as_tf: false`.
- Keyframe and map metadata preserve source topics, ROS stamps, frames,
  intrinsics, distortion coefficients and voxel sizes.

## Evaluation definitions

- Translation/rotation error: distance between estimated and truth SE(3).
- Overlap error: source-to-target nearest-neighbor mean and RMSE, capped by the
  configured overlap distance.
- Boundary error: nearest-neighbor error between the two extracted XY boundary
  clouds.
- Inlier ratio: aligned LiDAR points within the configured target distance.
- Supplement growth: `(union occupied voxels - visual occupied voxels) /
  visual occupied voxels`, analogous to the paper's octree leaf volume metric.
- Runtime: steady-clock duration inside each standalone method process.
- Memory: process peak resident set from `getrusage`.
- Block accounting: descriptor candidates, neighbor-consistent candidates,
  converged local transformations, failed local registrations and selected
  cluster size are all recorded.

## Known scope

This v1 validates map-level fusion offline. It deliberately excludes real-time
D435i point-cloud display, backend factors, repeated TF, flight-time feedback,
official Ultra-Fusion binaries and any writeback into RTAB or FAST-LIO maps.

`collect_hybridfusion_simulation.sh` is an optional owner-aware orchestration
wrapper. It starts the unchanged PR #6/PR #8 headless stack with the existing
guided rectangle, runs only this package's two exporters in another process
group, invokes their save services after landing, and delegates stack cleanup
to the existing active-run lifecycle. It does not alter any launch default.
