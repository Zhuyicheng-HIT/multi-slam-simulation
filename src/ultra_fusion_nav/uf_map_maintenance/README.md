# uf_map_maintenance

`uf_map_maintenance` is an offline companion to the existing bounded
`mid360_reliable_mapper`. It does not publish an online map, replace FAST-LIO,
or add loop-closure constraints.

## Data contract

The archive keeps the authoritative raw rosbag2 on disk. A materialized cache
contains one Livox scan per deterministic `NPZ` file and one immutable
`poses/original.csv`. Raw `offset_time`, `line`, `tag`, reflectivity,
`timebase`, `lidar_id`, reserved bytes, and source timestamp are retained. The
manifest records relative paths, frames,
the FAST-LIO scan-to-pose-child extrinsic, byte sizes, and SHA256 values.

Corrected trajectory estimates are new CSV revisions under `poses/`; neither
the raw bag nor `poses/original.csv` is overwritten. This is the interface a
future independent global pose graph will use.

## Archive and materialize

Use a new output directory. Recording streams directly to rosbag2 and runs
until interrupted with Ctrl-C:

```bash
ros2 run uf_map_maintenance archive_mid360_session \
  --output /data/map_sessions/session-001 \
  --session-id session-001 \
  --config install/uf_map_maintenance/share/uf_map_maintenance/config/offline_map_maintenance.yaml

ros2 run uf_map_maintenance materialize_mid360_archive \
  --archive /data/map_sessions/session-001 \
  --scan-topic /livox/lidar \
  --pose-topic /Odometry
```

Materialization uses two sequential bag passes: the first indexes finite poses;
the second writes one scan at a time. Raw scans are never accumulated in RAM.
Rejected associations are written to `poses/rejected.csv` with reason codes.

For a recorded MID360 `PointCloud2` bag, first produce a derived CustomMsg bag
without changing the source archive:

```bash
ros2 run uf_map_maintenance normalize_mid360_pointcloud2_bag \
  --source /data/source-bag \
  --output /data/derived-custom-bag

ros2 run uf_map_maintenance materialize_mid360_archive \
  --archive /data/derived-custom-bag \
  --pose-bag /data/fastlio-odometry-bag \
  --scan-topic /livox/lidar \
  --pose-topic /Odometry \
  --output /data/map-sessions/session-001
```

The normalizer maps `intensity` to Livox `reflectivity`, preserves `line`,
`tag`, XYZ, point order, and source header, and derives `offset_time` by
subtracting the header time in the source float64 timestamp domain before
rounding. It never restamps, clamps, or synthesizes point times.

## Build cleaned map

```bash
ros2 run uf_map_maintenance offline_map_builder \
  --session /data/map_sessions/session-001 \
  --poses /data/map_sessions/session-001/poses/original.csv \
  --trajectory /data/map_sessions/session-001/poses/trajectory_original.csv \
  --output /data/map_sessions/session-001-results/original \
  --config install/uf_map_maintenance/share/uf_map_maintenance/config/offline_map_maintenance.yaml
```

After loop optimization, repeat with `poses/loop-0001.csv` and a new output
directory. Outputs are `cleaned_global_map.pcd`, `voxel_evidence.csv`, and
`metrics.json`.

The builder emits `raw_scan_pose_map.pcd`, `deskewed_map.pcd`,
`voxelized_map.pcd`, `cleaned_map.pcd`, `voxel_evidence.csv`, and
`metrics.json`. The deskew path uses only bracketing FAST-LIO poses, linear
translation interpolation, shortest-path quaternion SLERP, and the recorded
LiDAR-to-body extrinsic. It does not extrapolate a pose.

Cleanup is conservative: support counts distinct scans, not duplicate points;
low-support voxels are removed first; isolated voxels and small low-support
components are rejected; components with stable multi-frame evidence are
preserved. Every voxel receives an auditable decision.

## Single-session loop smoke

`export_offline_keyframes` writes deskewed body-frame descriptor clouds and
map-frame registration targets. The `uf_relocalization` executable
`offline_loop_smoke` reuses the project's ESF retrieval and point-to-plane
verification code. This is deliberately only candidate retrieval plus
geometric verification; it does not add a loop constraint or global pose
graph.
