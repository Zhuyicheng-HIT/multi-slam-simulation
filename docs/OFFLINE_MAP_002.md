# OFFLINE-MAP-002: bounded online map and offline historical maintenance

## Scope and invariants

This change is based on PR #18 head
`89a1b1525f284db124f130a9978d85a16f083c39` plus the
DESKTOP-MAP-BASELINE-001 documentation commit. It does not modify FAST-LIO,
the fixed-lag fusion backend, sensor factors, Dynamic, ExternalNav,
relocalization transactions, or control ownership. It does not implement loop
closure.

The implementation keeps one online map: the existing
`mid360_reliable_mapper::FastlioCloudMapper::map_`. The new package is offline
and owns no online map publisher.

## Existing mechanisms retained

The mapper already performed finite/range/Z filtering, hash-voxel scan
downsampling, a radius-neighbor outlier filter, odometry/overlap quality gates,
centroid accumulation, frame-window expiration, capacity bounding, and map
publication. Its compiled defaults are 0.08 m scan voxels, 0.12 m map voxels,
two minimum map hits, 160,000 voxels, and a 90-frame window. The deployed YAML
uses 0.05/0.10 m, one hit, 220,000 voxels, and a 300-frame window.

Before OFFLINE-MAP-002, capacity overflow first erased low-hit voxels in
unordered-map iteration order and then erased arbitrary `map_.begin()` entries.
That made retention nondeterministic and could discard stable walls or ground.

The reference `MID360-Reliable-Mapper` mapper source is byte-identical to the
project copy at the audited reference commit. Its useful mechanisms were
already present; no LGPL source was copied into the Apache-2.0 offline package.

## Online bounded-map improvement

Timeout expiration is unchanged. Only capacity-pressure selection changed.
Each current voxel receives bounded evidence:

- multi-frame support count;
- last-observed frame/age;
- occupied 26-neighborhood count;
- whether connected structure exists below it;
- deterministic integer voxel key.

Eviction order is low support, older evidence, spatial isolation, missing lower
support, then voxel key. `map_stable_support_hits` and
`map_isolated_neighbor_threshold` are configuration-owned. Stable multi-frame
geometry is protected without assuming a ground height. The factor graph and
all input/output topics are unchanged.

## Disk archive

`uf_map_maintenance` records these streams directly into rosbag2:

- `/livox/lidar` and `/livox/imu`;
- `/Odometry` and `/fusion/unified/odom`;
- `/fusion/epoch`;
- `/tf` and `/tf_static`.

The manifest uses relative paths and stores frame IDs, the current FAST-LIO
`pose_child_from_scan` extrinsic, size, and SHA256. Recording holds no scan
history in RAM. Materialization performs two sequential bag passes, retaining
only the pose index and the current scan. Livox `offset_time`, `line`, `tag`,
reflectivity, `timebase`, `lidar_id`, reserved bytes, and source timestamp
remain available in the per-scan cache; the raw bag remains authoritative.

## Offline aggregation and cleanup

For each pose revision, each body/sensor-frame scan is transformed through the
recorded extrinsic and the selected map pose. Voxel support counts distinct
scan IDs instead of points. Provenance IDs are bounded while the support count
continues to grow. Cleanup removes:

1. voxels below `minimum_scan_support`;
2. spatially isolated, not-yet-stable voxels;
3. small connected components without stable multi-frame evidence.

Stable wall and ground components survive even when locally small. Outputs are
a cleaned PCD, per-voxel evidence/decision CSV, and machine-readable metrics.
The builder verifies input scan hashes before and after every build.

## Loop-closure boundary

A later global graph must write a new pose revision CSV with the same scan IDs,
timestamps, and epochs. The builder then regenerates the map from immutable
scans. The future graph remains separate from the eight-state fixed-lag backend:

`verified loop constraints -> independent batch graph -> corrected pose CSV -> offline rebuild`.

No historical loop edge, online correction, or relocalization behavior is
introduced here.

## Validation

Automated tests cover deterministic retention ordering, stable structure
protection, archive schema/SHA validation, source-time/epoch association,
non-finite and timestamp-regression rejection, bounded provenance, isolated
noise and floating-component cleanup, and original/corrected pose rebuilds
without scan mutation. Installation-state CLI smoke and the related package
build/test matrix are part of the final verification record for this commit.

Final local verification on Ubuntu 22.04 / ROS 2 Humble:

- five related packages built: `uf_interfaces`, `mid360_reliable_mapper`,
  `uf_map_maintenance`, `uf_shared_mapping`, and `uf_relocalization`;
- 72 test cases passed, with zero errors, failures, or skips;
- installed `ros2 run` entry points were discovered and executed;
- archive dry-run emitted the complete Raw MID360/IMU/pose/TF command;
- a real synthetic rosbag2 containing Livox `CustomMsg`, Odometry, and
  `FusionEpoch` was validated, materialized, and rebuilt end to end: 4 scans,
  17 points, epoch 7 preserved, 8 retained voxels, 1 low-support removal;
- deterministic original/corrected-pose rebuild smoke passed;
- the smoke retained 3/4 stable voxels respectively while removing 5/1
  low-support voxels from 17 input points, demonstrating that the pose revision
  changes aggregation but never mutates the scan cache.
