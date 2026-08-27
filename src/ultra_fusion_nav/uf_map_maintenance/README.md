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

## Build cleaned map

```bash
ros2 run uf_map_maintenance offline_map_builder \
  --session /data/map_sessions/session-001 \
  --poses /data/map_sessions/session-001/poses/original.csv \
  --output /data/map_sessions/session-001-results/original \
  --config install/uf_map_maintenance/share/uf_map_maintenance/config/offline_map_maintenance.yaml
```

After loop optimization, repeat with `poses/loop-0001.csv` and a new output
directory. Outputs are `cleaned_global_map.pcd`, `voxel_evidence.csv`, and
`metrics.json`.

Cleanup is conservative: support counts distinct scans, not duplicate points;
low-support voxels are removed first; isolated voxels and small low-support
components are rejected; components with stable multi-frame evidence are
preserved. Every voxel receives an auditable decision.
