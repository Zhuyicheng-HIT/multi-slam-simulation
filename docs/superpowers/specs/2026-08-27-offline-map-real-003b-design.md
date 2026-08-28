# OFFLINE-MAP-REAL-003B Design

## Goal

Validate OFFLINE-MAP-002 on the recovered real MID360/D435i ROS 2 bag by
preserving the raw acquisition contract, generating a reproducible FAST-LIO
trajectory, rebuilding the map with per-point motion compensation, and then
running one single-session loop-candidate and geometric-verification smoke
test. Global pose-graph optimization is explicitly out of scope.

## Frozen inputs and evidence

The source ZIP and extracted SQLite bag remain immutable:

- ZIP: `C:/Users/pc/Desktop/recovered_demo.zip`
- ZIP SHA256: `30D93315264D7E6576CE01C496DF52D8A566937A826DCD9D7073147B57329C63`
- bag database SHA256:
  `96A8075145817013C5FF6FF1008081FF4E1DE0227FA9EDC8B5A3B90CC9CD9A56`
- duration: 65.582488262 seconds
- `/livox/lidar`: 656 `sensor_msgs/msg/PointCloud2` messages at 10.002 Hz
- `/livox/imu`: 13,118 messages at 200.015 Hz
- source bag contains no usable pose, dynamic TF, or GNSS

The point-cloud schema is `x/y/z/intensity/tag/line/timestamp`. The observed
`timestamp` field is an absolute Unix timestamp in nanoseconds stored as a
`float64`; each scan spans about 100 ms. Four line values, full azimuth
coverage, and the observed elevation range are consistent with MID360 output.

## Architecture

The implementation uses a lossless, project-owned compatibility layer rather
than changing FAST-LIO preprocessing mathematics:

```text
immutable PointCloud2 bag
  -> normalized Livox CustomMsg bag
  -> existing FAST-LIO CustomMsg path
  -> immutable original Odometry trajectory
  -> scan-level and per-point map reconstruction
  -> voxel aggregation and existing conservative cleanup
  -> existing keyframe retrieval and geometric verification smoke test
```

All generated artifacts live outside Git under the BAG-AUDIT output root.
Manifests contain source hashes, command/config identities, output hashes,
counts, rejection reasons, elapsed time, and maximum resident memory.

## PointCloud2 normalization contract

The normalizer reads only the source bag and writes a new ROS 2 bag.

- Preserve the original message header and frame ID; never restamp data.
- Map finite `intensity` values to `reflectivity` by rounding and clamping to
  `[0, 255]`.
- Preserve `line` and `tag` byte values exactly.
- Interpret the observed `timestamp` value as absolute nanoseconds and compute
  `offset_time = round(float64(timestamp) - float64(scan_header_stamp_ns))`.
  The subtraction deliberately happens before rounding in the same float64
  precision domain. At this recording's Unix epoch one float64 ULP is 256 ns;
  independently rounding the point timestamp before subtracting the exact
  integer header creates artificial frame-start offsets in `[-127, 127]` ns.
- Accept only finite timestamps whose derived offset is representable by the
  Livox `uint32 offset_time` field. Negative or overflowing offsets are
  rejected and counted; they are never clamped or restamped. The run manifest
  records the observed minimum and maximum so the physical scan interval stays
  auditable without inventing a data-dependent threshold.
- Preserve finite XYZ values, including zero returns, in the normalized raw
  archive. Range/blind filtering remains the responsibility of the existing
  FAST-LIO and map-builder contracts, and zero-return counts remain auditable.
- Preserve source point order. Interleaved MID360 lines mean global point
  timestamps need not be monotonically ordered within the message.
- Copy `/livox/imu` without modifying timestamps or measurements.

The output CustomMsg uses the scan header timestamp as `timebase`, keeps the
four observed lines, and stores each derived offset as an unsigned integer.
The source PointCloud2 remains authoritative and is never deleted.

## FAST-LIO trajectory generation

The normalized CustomMsg and original Livox IMU are replayed through the
existing real MID360 configuration and its `lidar_type=1` CustomMsg path. The
task does not tune estimator parameters. The project-pinned extrinsic is used
and recorded in the run manifest; because the bag has no LiDAR-to-IMU TF, the
report must retain this calibration provenance as a limitation.

The run records `/Odometry` and relevant diagnostics into a separate bag.
Acceptance requires finite, monotonic poses, no estimator reset or timestamp
loopback, and explicit accounting for initialization scans without poses.

## Per-point offline deskew

The original FAST-LIO trajectory is immutable. For a point with source time
`t_i = scan_stamp + offset_time`, the map point is

```text
p_map_i = T_map_body(t_i) * T_body_lidar * p_lidar_i
```

Translation uses linear interpolation between the two adjacent trajectory
states. Rotation uses shortest-path quaternion SLERP after quaternion sign
normalization. Interpolation is allowed only when both bracketing poses exist
and their interval is within a configured maximum. There is no extrapolation;
unbracketed points/scans are rejected with explicit reason counts.

For comparison, scan-level reconstruction uses `T_map_body(scan_stamp)` for
every point in that scan. Both paths apply identical finite/range/body filters
and the same extrinsic so deskew is the only changed variable.

## Four map products

1. `raw_map.pcd`: scan-level pose reprojection after common validity filters;
   no voxel aggregation or evidence cleanup.
2. `deskewed_map.pcd`: per-point SE(3) reprojection of the same accepted raw
   points; no voxel aggregation or cleanup.
3. `voxelized_map.pcd`: deterministic centroid aggregation of the deskewed
   points using the OFFLINE-MAP-002 voxel size.
4. `cleaned_map.pcd`: the existing multi-scan support, age, isolation, and
   small-component cleanup applied to deskewed voxel evidence.

Processing is streaming/chunked. Raw scans and the whole unvoxelized map are
not retained in RAM when a file-backed output can be written incrementally.
Deterministic ordering and content hashes make repeated rebuilds comparable.

## Evaluation

The report records:

- input, accepted, rejected, raw, deskewed, voxel, and cleaned point counts;
- voxel compression ratio and cleanup counts by reason;
- isolated/small-component removal without conflating those removals with
  voxel deduplication;
- ground and wall preservation using identical geometric ROIs and local plane
  classification on voxelized versus cleaned maps;
- plane-normal residual/thickness and revisit nearest-neighbor error before
  and after deskew as ghosting proxies;
- elapsed time, maximum RSS, input/output bytes, and deterministic hashes for
  every stage.

The real bag already implies a 100 ms scan motion of about 5.96 cm translation
and 1.20 degrees rotation at P95. The deskew decision is evidence-driven:
deskew must reduce repeated-surface thickness or revisit error without reducing
stable wall/ground support. No estimator threshold is changed to improve the
map metrics.

## Loop smoke test

After map maintenance completes, keyframes are selected from the immutable
trajectory with the existing project conventions. Retrieval must use the
existing descriptor/keyframe framework rather than pose proximity. Pose is
used only for keyframe spacing and offline evaluation.

Candidates must be temporally separated, then pass existing geometric
verification. The smoke report includes candidate rank, temporal separation,
descriptor score, convergence, inliers/overlap, fitness/residual, estimated
relative transform, and consistency with the original trajectory. The known
45.6-second revisit is an evaluation target, not a hard-coded candidate.

This proves only single-session candidate retrieval and geometric verification.
It does not claim loop insertion, trajectory correction, global pose-graph
success, or multi-session relocalization.

## Failure behavior

- Source hash mismatch, SQLite read failure, missing required fields, invalid
  point timestamps, non-monotonic scan headers, or incomplete output causes a
  non-zero exit and an incomplete manifest.
- Existing output directories are never overwritten.
- Missing pose brackets reject points/scans; the system never invents poses.
- The original ZIP, source bag, and original generated pose revision remain
  immutable throughout retries.

## Test strategy

Implementation follows red-green-refactor cycles:

1. PointCloud2 field conversion tests cover intensity mapping, exact line/tag
   preservation, absolute timestamp conversion, invalid offsets, zero returns,
   and interleaved line timestamps.
2. SE(3) interpolation tests cover endpoints, midpoint translation, quaternion
   sign normalization, SLERP, missing brackets, and excessive pose gaps.
3. A deterministic moving-plane fixture must show lower surface thickness with
   deskew while preserving point count.
4. Four-map tests verify stage semantics, deterministic hashes, evidence-based
   cleanup reasons, and unchanged stable wall/ground support.
5. Loop tests verify temporal exclusion, descriptor-only retrieval, and
   geometric rejection/acceptance on deterministic fixtures.
6. Package tests run before the real-bag pipeline; the final run repeats output
   hash and repository cleanliness checks.

## Non-goals

- no global pose graph;
- no loop constraint insertion into the fixed-lag backend;
- no localization parameter tuning;
- no changes to HXY, Flow, Visual, GNSS/IMU, Dynamic, or production topics;
- no replacement or deletion of the source bag;
- no push or remote PR changes.
