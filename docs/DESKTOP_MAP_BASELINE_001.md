# DESKTOP-MAP-BASELINE-001

Date: 2026-08-27

## Frozen research context

- Repository: `Zhuyicheng-HIT/multi-slam-simulation`
- Pull request: `#18`, draft, `feat/lidar-horizontal-degeneracy-v1`
- PR head: `89a1b1525f284db124f130a9978d85a16f083c39`
- PR base: `main@c7c1adcd92a7fdd3b5b38aa47e48a10ea3552981`
- Relation: base is an ancestor; PR #18 is 19 commits ahead and 0 behind.
- Isolated worktree: `/home/zyc/projects/multi-slam-desktop-map-baseline-20260827`
- Local branch: `codex/desktop-map-baseline-pr18-20260827`

The pre-existing root worktree remains untouched. It is on
`deploy/dynamic-localization-v1` and contains uncommitted Dynamic deployment
work. Build, install, and log output for this research line is confined to the
new worktree and is ignored by Git.

`git fetch --all --tags --prune` fetched the current branches, including PR
#18, but did not replace the local tag `v0.1.0-four-source-reloc-calibration`
with a different remote object. The PR ref and its named branch both resolve to
the SHA above; the tag conflict is deliberately left unresolved.

## PR #18 mapping and loop-closure audit

PR #18 itself does not change `mid360_reliable_mapper`, `uf_shared_mapping`, or
`uf_relocalization` relative to its current `main` base. Its HXY, MID360 IMU,
Dynamic V2, and startup-contract changes can affect upstream data quality, but
the mapping and relocalization components below are inherited unchanged.

### LiDAR reliable mapping

`mid360_reliable_mapper/fastlio_cloud_mapper_node.cpp` consumes
`/cloud_registered` and `/Odometry`. It provides:

- finite/range/height filtering;
- per-scan hash-voxel centroid downsampling;
- a radius-neighbour outlier filter;
- frame admission gates for odometry speed/jump/Z jump and scan-map overlap;
- a bounded rolling voxel map with hit counts and last-update indices;
- filtered scan, reliable scan, and denoised-map topics.

This is an online rolling display/navigation map, not a historical archive. A
registered cloud is already in `camera_init`; the code does not retain the raw
sensor-frame scan, per-point time, exact pose association, calibration, or a
link from a voxel back to its contributing scans. It may restamp outputs, ages
out old voxels, and performs an irreversible aggregate update.

### Shared mapping

`uf_shared_mapping` is disabled by default and does not publish TF or modify
FAST-LIO. It maintains a bounded source-aware voxel map:

- LiDAR owns geometry centroids;
- reliable RGB-D can color LiDAR geometry;
- RGB-D may add supplementary geometry in empty voxels;
- conflicting RGB-D cannot move LiDAR geometry;
- only exported PCD and JSON summaries are persisted.

This is useful source-ownership policy, but it does not preserve raw
observations or support correcting old poses and rebuilding the map.

### Keyframes and relocalization

`uf_relocalization` already has strong reusable primitives:

- timestamp synchronization among `/lio/local_map`, `/lio/odom`,
  `/fusion/unified/map_pose`, and `/lidar/points_deskewed`;
- static-keyframe quality admission using scheduler status, repeatability,
  dynamic ratio, LiDAR degradation, and pose spacing;
- deep-copied clouds and bounded in-memory keyframe storage;
- normalized 640-bin PCL ESF descriptors and cosine-distance retrieval;
- point-to-plane/ICP/NDT registration wrappers, reciprocal support,
  observability/condition diagnostics, ambiguity rejection, and a three-query
  consistency gate;
- a safe `RelocalizationResult -> backend epoch` handoff.

The database has no serialization or reload path. Stored keyframe clouds are
dense map-frame submaps, while the descriptor is derived from a synchronized
body-frame scan. Keyframes are not represented as persistent graph nodes with
odometry or loop edges.

### What the current automatic loop closure really does

The current automatic path searches old, spatially nearby keyframes while the
scheduler is healthy, geometrically verifies a candidate, bounds the proposed
correction, and then requests the same backend epoch transition used by
relocalization. It does **not** build or optimize a global pose graph and does
not correct all historical poses. It is therefore a safe local drift-trimming
mechanism, not full historical loop-closure mapping.

The online backend is a fixed-lag optimizer (`window_size: 8` in the current
configuration). Marginalized historical states are unavailable. Historical
loop edges must not be injected into it as if those old states still existed.

## MID360-Reliable-Mapper reference audit

- Read-only clone: `/home/zyc/projects/references/MID360-Reliable-Mapper`
- Reference head: `c8c20962c1b720f161f19a1f4dd971a88154b209`
- The reference mapper source is byte-identical to the project's
  `mid360_reliable_mapper/src/fastlio_cloud_mapper_node.cpp`.
- The reference repository has no top-level license detected by GitHub. Its
  `uav_slam_sim/package.xml` declares `LGPL-3.0-only`; the corresponding package
  already vendored in this project also has an LGPL-3.0-only license file.

### Designs worth reusing

- separate filtering, reliable-frame, and map outputs;
- explicit frame-admission reasons;
- hash-voxel centroids with observation counts;
- cheap neighbour filtering suited to edge hardware;
- bounded memory and decoupled map publish cadence;
- one-shot real-session runner that records raw LiDAR/IMU, odometry, TF,
  filtered clouds, maps, logs, and a reproducible session directory;
- offline PCD analysis, voxel occupancy/coverage comparison, readiness reports,
  headless deployment guidance, and a record-then-replay workflow.

### Designs not to copy into the offline map

- accepting an asynchronously cached odometry sample without a persisted
  timestamp association;
- replacing invalid `dt` with a constant;
- restamping data that will later be used for causal reconstruction;
- treating an already registered cloud as the only source of truth;
- irreversible voxel averaging without scan provenance;
- a 90-frame rolling window as a long-term map;
- arbitrary unordered-map eviction when a hard size limit is reached;
- scan-map overlap as proof that every point is static;
- hard-coded workstation paths in session scripts;
- copying new reference code without resolving its package-level LGPL
  provenance and preserving notices.

## Recommended ownership boundary

```text
recorded raw MID360 + IMU + calibration
                    |
                    v
        immutable session archive
  raw scan + deskewed body cloud + original pose
  + quality + epoch + hashes + versioned manifest
                    |
          +---------+----------+
          |                    |
          v                    v
offline map builder     keyframe materializer
voxel evidence          descriptor + local submap
support/outlier clean           |
          |              candidate retrieval
          |              geometric verification
          |                    |
          |                 loop edge
          |                    v
          |          independent global pose graph
          |                    |
          +<--------- corrected keyframe poses
          |
          v
versioned cleaned historical map
```

The archive is immutable. Every cleaned map is a derived artifact identified
by archive hash, pose-revision hash, configuration hash, and tool version. A
loop correction changes only the corrected-pose table and triggers a rebuild;
it never mutates raw scans or original poses.

`uf_relocalization` should continue to own retrieval and geometric verification.
It should emit a verified relative-pose measurement plus information/covariance
and provenance, not apply historical corrections itself. A new offline/global
mapping package should own graph persistence, batch optimization, corrected
poses, and rebuild jobs. The existing fixed-lag backend continues to own live
state estimation and epoch continuity.

## Proposed module layout

```text
src/ultra_fusion_nav/uf_map_maintenance/
  include/uf_map_maintenance/
    archive_manifest.hpp
    scan_pose_association.hpp
    voxel_evidence_map.hpp
    map_filters.hpp
  src/
    archive_session.cpp
    offline_map_builder.cpp
    voxel_evidence_map.cpp
    map_filters.cpp
  config/offline_map_maintenance.yaml
  launch/archive_session.launch.py
  tools/validate_archive.py
  tools/compare_map_revisions.py
  test/data/tiny_session/
  test/

src/ultra_fusion_nav/uf_global_pose_graph/
  include/uf_global_pose_graph/
    graph_types.hpp
    pose_graph.hpp
    graph_store.hpp
  src/
    pose_graph.cpp
    optimize_pose_graph.cpp
  config/global_pose_graph.yaml
  test/

session_<id>/
  manifest.json
  calibration.yaml
  raw/session.db3
  scans/<stamp>_body.pcd
  poses/original.csv
  quality/keyframes.csv
  derived/<revision>/corrected_poses.csv
  derived/<revision>/cleaned_map.pcd
  derived/<revision>/metrics.json
```

For the first implementation, rosbag2 is the immutable raw source. Body-frame
deskewed PCDs are a deterministic cache, not a replacement for raw
`livox_ros_driver2/CustomMsg` and IMU. Each pose record includes source stamp,
frame IDs, epoch/reset counter, original SE(3), covariance/quality, and hashes.

## Offline maintenance algorithm boundary

The first map builder should:

1. associate each deskewed body scan to an original timestamped pose;
2. transform points to the selected map frame;
3. aggregate by voxel while counting distinct supporting scan IDs, not raw
   point count;
4. retain centroid, covariance, intensity, first/last stamp, view-direction
   coverage, and contributing scan IDs or a bounded provenance index;
5. require configurable multi-frame support for permanent output;
6. remove isolated voxels and small floating connected components while
   reporting every removal reason;
7. write a new derived revision and reproducibility manifest.

No truth data, future pose, RGB-D, or online estimator feedback is needed for
this stage. All thresholds are configuration values, and the raw archive is
never changed.

## Validation completed for this baseline

The following were built from the isolated PR #18 worktree:

- `uf_interfaces`
- `mid360_reliable_mapper`
- `uf_shared_mapping`
- `uf_relocalization`

Targeted `colcon test` result: **50 tests, 0 errors, 0 failures, 0 skipped**.
The only build diagnostic was the existing developer warning that CMake policy
`CMP0074` is unset while `PCL_ROOT=/usr`; it did not affect the build.

No Gazebo, sensor model, positioning algorithm, HXY, Flow, Visual, GNSS/IMU,
Dynamic, or remote branch was changed.
