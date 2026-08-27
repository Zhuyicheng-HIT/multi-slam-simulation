# Offline Map Maintenance V1 Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use superpowers:test-driven-development for each implementation task and superpowers:verification-before-completion before committing.

**Goal:** Build a deterministic offline map-maintenance MVP that preserves raw scans and original poses and can rebuild a cleaned historical map from either original or corrected poses.

**Architecture:** Add a new `uf_map_maintenance` ROS 2/C++ package. Rosbag2 remains the immutable source; an archive manifest indexes source topics and cached body-frame scans. The builder aggregates distinct-scan voxel evidence and writes versioned derived outputs. It does not modify the estimator, relocalization node, or current map publishers.

**Tech Stack:** ROS 2 Humble, C++17, PCL 1.12, Eigen 3.4, ament_cmake, Python 3 for manifest validation and metric comparison.

---

### Task 1: Define and validate the immutable archive schema

**Files:**
- Create: `src/ultra_fusion_nav/uf_map_maintenance/package.xml`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/CMakeLists.txt`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/include/uf_map_maintenance/archive_manifest.hpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/src/archive_manifest.cpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/tools/validate_archive.py`
- Test: `src/ultra_fusion_nav/uf_map_maintenance/test/test_archive_manifest.cpp`
- Test data: `src/ultra_fusion_nav/uf_map_maintenance/test/data/tiny_session/`

1. Write failing tests for schema version, required topics, frame IDs, monotonic
   scan stamps, finite poses, epoch IDs, calibration, and SHA256 entries.
2. Implement the smallest manifest reader/validator.
3. Add a validator CLI that emits machine-readable reasons.
4. Run package unit tests and malformed-manifest cases.
5. Commit only the schema, validator, and tests.

### Task 2: Add deterministic scan-pose association and archive extraction

**Files:**
- Create: `src/ultra_fusion_nav/uf_map_maintenance/include/uf_map_maintenance/scan_pose_association.hpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/src/scan_pose_association.cpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/src/archive_session.cpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/config/offline_map_maintenance.yaml`
- Test: `src/ultra_fusion_nav/uf_map_maintenance/test/test_scan_pose_association.cpp`

1. Test exact, within-tolerance, missing-side, stale, duplicate, epoch-crossing,
   timestamp-regression, and non-finite cases.
2. Associate by source timestamps and declared frames; do not use arrival time.
3. Preserve raw rosbag2 and write body-frame PCD only as a cache.
4. Record rejected scans and provenance hashes in the manifest.
5. Run deterministic extraction twice and compare manifests/hashes.

### Task 3: Implement evidence-preserving voxel aggregation and cleanup

**Files:**
- Create: `src/ultra_fusion_nav/uf_map_maintenance/include/uf_map_maintenance/voxel_evidence_map.hpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/include/uf_map_maintenance/map_filters.hpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/src/voxel_evidence_map.cpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/src/map_filters.cpp`
- Test: `src/ultra_fusion_nav/uf_map_maintenance/test/test_voxel_evidence_map.cpp`
- Test: `src/ultra_fusion_nav/uf_map_maintenance/test/test_map_filters.cpp`

1. Test that support counts distinct scan IDs rather than duplicate points.
2. Store centroid, covariance, intensity, temporal support, view diversity, and
   bounded provenance.
3. Implement configurable multi-frame admission, radius isolation, and small
   floating-component filtering with explicit reason counters.
4. Test static wall/corner preservation, duplicate suppression, isolated noise,
   and suspended clusters.
5. Benchmark memory and runtime on a deterministic synthetic session.

### Task 4: Build original/corrected pose revisions

**Files:**
- Create: `src/ultra_fusion_nav/uf_map_maintenance/src/offline_map_builder.cpp`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/tools/compare_map_revisions.py`
- Test: `src/ultra_fusion_nav/uf_map_maintenance/test/test_offline_rebuild.cpp`

1. Test original-pose and corrected-pose builds from the same immutable scans.
2. Require pose-revision and configuration hashes in every output manifest.
3. Write into a temporary revision and publish it atomically only on success.
4. Verify byte-identical repeat builds and unchanged source hashes.
5. Report support histogram, removals, completeness proxy, runtime, and memory.

### Task 5: Connect existing relocalization verification to an offline pose graph

**Files:**
- Create later: `src/ultra_fusion_nav/uf_global_pose_graph/`
- Modify later: `src/ultra_fusion_nav/uf_relocalization/` only to expose a
  verified loop-constraint record; do not change current production behavior.

1. Freeze an interface containing keyframe IDs, relative SE(3), information or
   covariance, descriptor and registration metrics, and provenance.
2. Add sequential odometry edges and verified loop edges to an independent
   batch graph.
3. Test no-loop identity, one valid loop, bad-loop rejection, disconnected
   graph, gauge anchoring, and optimizer non-finite failure.
4. Produce corrected poses and invoke Task 4 rebuild.
5. Compare candidate recall, false loops, trajectory change, map consistency,
   and static preservation before considering any online graph service.

## Completion gate

- No changes to HXY, Flow, Visual, GNSS/IMU, Dynamic, fixed-lag factor
  ownership, or ExternalNav.
- Raw archive and original poses remain byte-identical after every rebuild.
- All thresholds are configuration-owned and all removals are auditable.
- Relevant package build/tests and a deterministic end-to-end tiny-session
  replay pass from a clean worktree.
- No Git push until the user explicitly requests publication.
