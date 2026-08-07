# Online RGB-D and LiDAR shared mapping

`uf_shared_mapping` is opt-in and independent from the flight estimator. It
does not publish TF, modify FAST-LIO's source map, or block the backend.

## Topology and consistency policy

The bounded hash map uses configurable cubic voxels. Each voxel stores a
geometry centroid, RGB mean, LiDAR/RGB-D observation counts, color count and
last timestamp. Source ownership is explicit:

- LiDAR observations create/update primary geometry.
- RGB-D points consistent with existing LiDAR geometry add color and support,
  but never move the LiDAR centroid.
- RGB-D points in empty voxels create supplementary geometry only when the
  visual reliability weight is above the configured minimum.
- RGB-D points farther than `conflict_distance_m` from primary geometry are
  rejected as possible ghosting/dynamic objects.
- Oldest voxels are evicted only after `maximum_voxels` is exceeded.

The node associates RGB-D frames with `/fusion/unified/odom`, applies the same
body-camera extrinsic, samples depth at a configurable stride and consumes
registered LiDAR points. Results are published at `/mapping/shared/points` and
written only through `/mapping/shared/export`.

```bash
ros2 launch uf_shared_mapping shared_mapping.launch.py \
  enabled:=true output_directory:=shared_map_output
ros2 service call /mapping/shared/export std_srvs/srv/Trigger '{}'
```

The output directory contains `lidar_map.pcd`, `rgbd_map.pcd`,
`joint_map.pcd` and `metrics.json`. Defaults remain disabled in both YAML and
launch.

## Metrics

- LiDAR/RGB-D/joint/supplementary voxel counts
- RGB-D conflict ratio (ghosting proxy)
- LiDAR color coverage ratio
- supplementary-volume growth ratio (completeness proxy)
- evictions and raw accepted observation counts

These are online consistency proxies, not a replacement for surveyed map error.
Real-map evaluation must additionally measure nearest-neighbor overlap,
boundary error, repeat-pass ghosting and memory/CPU over a full mission.
