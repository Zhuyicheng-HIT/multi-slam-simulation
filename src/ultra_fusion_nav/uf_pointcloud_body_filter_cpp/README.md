# C++ PointCloud2 body filter

`uf_pointcloud_body_filter_cpp` is the allocation-efficient implementation of
the PR21 LiDAR self-body/range filter. The production `uf_sensor_pipeline`
launch uses this executable while retaining the Python executable for frozen
A/B comparisons.

## Runtime contract

- Node name: `pointcloud_body_filter`
- Default input: `/sim/mid360/points_raw`
- Default output: `/sensors/lidar/points_body_filtered`
- Removed-body ratio: `/sensors/lidar/body_removed_ratio`
- Input, output, and ratio use ROS 2 Sensor Data QoS.
- `/livox/lidar` is neither remapped nor modified by this package.

The existing parameter names and defaults are preserved, including
`body_min_*_m`, `body_max_*_m`, `min_range_m`, `max_range_m`,
`lidar_to_body_rotation`, and `lidar_to_body_translation`.

The filter discovers `x`, `y`, and `z` from the `PointCloud2` field layout,
supports all scalar ROS `PointField` datatypes and either endianness, and copies
the complete record of each retained point. Consequently Livox-specific fields
such as timestamp, line, tag, and reflectivity remain byte-identical.

`enable_profiling` is an opt-in diagnostic parameter. It logs bounded callback
P50/P95/max measurements and does not add publishers or subscriptions.
