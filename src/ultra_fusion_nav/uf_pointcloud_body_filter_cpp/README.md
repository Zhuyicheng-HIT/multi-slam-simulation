# C++ PointCloud2/Livox body filter

`uf_pointcloud_body_filter_cpp` is the allocation-efficient implementation of
the PR21 LiDAR self-body/range filter. The production `uf_sensor_pipeline`
launch uses this executable while retaining the Python executable for frozen
A/B comparisons.

## Runtime contract

- Node name: `pointcloud_body_filter`
- Default input: `/sim/mid360/points_raw`
- Default output: `/sensors/lidar/points_body_filtered`
- Removed-body ratio: `/sensors/lidar/body_removed_ratio`
- Health state: `/sensors/lidar/body_filter_status`
- Input, output, and ratio use ROS 2 Sensor Data QoS.
- `/livox/lidar` is neither remapped nor modified by this package.

The existing parameter names and defaults are preserved, including
`body_min_*_m`, `body_max_*_m`, `min_range_m`, `max_range_m`,
`lidar_to_body_rotation`, and `lidar_to_body_translation`.

Real hardware can set `input_message_type=livox_custom` and
`geometry_contract_file=package://uf_sensor_pipeline/config/real_mid360s_d435i_geometry.yaml`.
Retained `CustomPoint` coordinates, `offset_time`, `reflectivity`, `tag`, and
`line` are copied without semantic conversion. The raw `/livox/lidar` publisher
is never replaced.

`geometry_mode=composite` supports a union of oriented boxes and cylinders.
The real contract currently supplies one provisional conservative box with
body-FLU bounds x/y `[-0.28, 0.28] m` and z `[-0.30, 0.06] m`. It filters only
the separate localization/mapping copy; `filter_enabled=false` provides a
one-switch byte-preserving bypass, and invalid geometry/runtime failures remain
fail-open. Simulation retains the legacy AABB configuration.

The filter discovers `x`, `y`, and `z` from the `PointCloud2` field layout,
supports all scalar ROS `PointField` datatypes and either endianness, and copies
the complete record of each retained point. Consequently Livox-specific fields
such as timestamp, line, tag, and reflectivity remain byte-identical.

`enable_profiling` is an opt-in diagnostic parameter. It logs bounded callback
P50/P95/max measurements and does not add publishers or subscriptions.
