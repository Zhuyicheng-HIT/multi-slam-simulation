# Online RGB-D and LiDAR mapping

`uf_shared_mapping` is a bounded, source-aware local voxel map. It is disabled
by default, publishes no TF, does not mutate FAST-LIO's map, and writes only to
an explicitly selected output directory.

- LiDAR owns primary geometry and updates its centroid.
- Reliable RGB-D colors LiDAR voxels without moving the LiDAR centroid.
- Reliable RGB-D may supplement empty voxels.
- RGB-D points inconsistent with nearby LiDAR geometry are rejected instead of
  overwriting geometry.
- Each voxel records source counts, color support and last timestamp.
- LiDAR-only, RGB-D-only and joint PCD files plus JSON metrics are exported by
  `/mapping/shared/export`.

Inputs are registered LiDAR points, exact RGB/16UC1 depth/CameraInfo, unified
body pose and D_V. Outputs are `/mapping/shared/points`, `lidar_map.pcd`,
`rgbd_map.pcd`, `joint_map.pcd` and `metrics.json`.

Three deterministic runs produced 441 LiDAR voxels, 885/887/885 joint voxels,
color-coverage 0.6712/0.6599/0.6689 and supplementary-volume growth
1.0068/1.0113/1.0068. Conflict ratio was zero for this deliberately consistent
dataset; separate unit tests verify that conflicting RGB-D cannot overwrite
LiDAR geometry. These are software-contract metrics, not surveyed map accuracy.

Unlike HybridFusion, this module fuses source observations during map
generation using the current unified pose. HybridFusion remains an offline,
post-hoc cross-map registration baseline with block descriptors, NDT and
transform clustering.
