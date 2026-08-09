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
- A current registered LiDAR scan supplies a conservative azimuth/elevation
  depth buffer. RGB-D returns hidden behind that surface are rejected before
  insertion. Keeping elevation in the key prevents a roof/high return from
  erasing a low obstacle on the same XY bearing.
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

The occlusion check uses the source timestamps; it neither rewrites timestamps
nor waits for future scans. It is applied only when the registered LiDAR scan
is in `map_frame` and no older than `occlusion_lidar_tolerance_s`. Its angular
bin sizes and range margin are configurable. The defaults (0.5 degree bins,
no neighboring-bin dilation, 0.40 m margin) are engineering defaults selected
by the deterministic visible-surface A/B test, not values claimed by a paper.
Set `occlusion_filter_enabled:=false` for a control run.

Map integration remains active without subscribers, while full PointCloud2
serialization is skipped by default until `/mapping/shared/points` has a
subscriber. This removes an avoidable periodic full-map rebuild without
changing stored geometry or exports. Set `publish_when_unsubscribed:=true` only
for workflows which deliberately need unobserved publications.

## Metrics

- LiDAR/RGB-D/joint/supplementary voxel counts
- RGB-D conflict ratio (ghosting proxy)
- occlusion candidate/rejection count and ratio
- low/middle/high height voxel counts
- LiDAR color coverage ratio
- supplementary-volume growth ratio (completeness proxy)
- evictions and raw accepted observation counts

These are online consistency proxies, not a replacement for surveyed map error.
Real-map evaluation must additionally measure nearest-neighbor overlap,
boundary error, repeat-pass ghosting and memory/CPU over a full mission.
