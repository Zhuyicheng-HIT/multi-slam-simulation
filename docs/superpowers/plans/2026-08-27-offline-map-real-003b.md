# OFFLINE-MAP-REAL-003B Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Normalize the recovered MID360 PointCloud2 recording without losing per-point semantics, replay it through the existing FAST-LIO configuration, rebuild four auditable map products with causal per-point deskew, and run one single-session loop-candidate/geometric-verification smoke test.

**Architecture:** Keep the source bag immutable. Convert only the LiDAR transport from PointCloud2 to Livox CustomMsg in a derived bag, preserve IMU bytes, and record the derived artifact manifest. Materialize scans plus the full FAST-LIO pose trajectory to disk. A bounded interpolation layer applies translation interpolation and shortest-path quaternion SLERP at each original point timestamp. A streaming builder emits raw, deskewed, voxelized, and cleaned PCDs without retaining the full session in RAM. Loop smoke reuses the existing PR18 ESF database and registration core and does not create a global pose graph.

**Tech Stack:** ROS 2 Humble, rosbag2 sqlite3, Python 3/numpy, Livox ROS driver messages, existing FAST-LIO ROS2, PCL/Eigen, existing `uf_relocalization` C++ libraries, pytest/gtest/colcon.

**Spec:** `docs/superpowers/specs/2026-08-27-offline-map-real-003b-design.md`

## Global Constraints

- Never modify or overwrite the recovered source zip or its extracted source bag.
- Preserve original point order, XYZ, `line`, `tag`, absolute point timestamp, and reflectivity semantics.
- Reject invalid timestamp contracts explicitly; never clamp, rewrite, or synthesize timestamps silently.
- Use only bracketing FAST-LIO poses. Never extrapolate or use future truth.
- Do not tune the localization estimator, add a global pose graph, push, or modify unrelated algorithms.

### Task 1: PointCloud2 MID360 normalization contract

**Files:**
- Create: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/pointcloud_adapter.py`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/normalize.py`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/tools/normalize_mid360_pointcloud2_bag`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/test/test_pointcloud_adapter.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/CMakeLists.txt`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/package.xml`

1. Write failing unit tests for the exact 26-byte source layout, endian handling, intensity rounding/clamping, line/tag preservation, same-float64-domain absolute-nanosecond to `offset_time` conversion, invalid negative/overflow offsets, non-finite coordinates, and organized row stepping.
2. Implement a dependency-light binary decoder and normalization validator.
3. Implement a rosbag2 streaming converter that writes a derived CustomMsg LiDAR bag and copies selected IMU messages unchanged; write source/output hashes and counters atomically.
4. Run the targeted tests and the existing map-maintenance tests.

### Task 2: Causal trajectory interpolation and per-point deskew

**Files:**
- Create: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/trajectory.py`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/test/test_trajectory.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/materialize.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/test/test_materialize.py`

1. Write failing tests for exact-sample lookup, linear translation, shortest-path SLERP, sign-equivalent quaternions, timestamp regression, missing left/right state, and maximum bracket span.
2. Implement immutable pose trajectory loading/interpolation and vectorized point deskew using `T_map_body(t_i) * T_body_lidar * p_i`.
3. Extend materialization to accept a separate pose bag, persist the complete trajectory revision, and retain the scan-level association CSV for rebuild compatibility.
4. Verify that raw NPZ hashes are stable and unbracketed points receive explicit rejection reasons.

### Task 3: Four streaming map products and metrics

**Files:**
- Create: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/pcd.py`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/evaluation.py`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/test/test_pcd.py`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/test/test_evaluation.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/builder.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/voxel_map.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/test/test_rebuild.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/test/test_voxel_evidence.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/config/offline_map_maintenance.yaml`

1. Write failing tests for deterministic streaming binary PCD, all-voxel centroids, deskew/no-deskew product separation, cleanup decision accounting, structural-plane retention, and ghosting proxy.
2. Implement the streaming PCD writer and expose all voxelized centroids without changing cleanup semantics.
3. Extend the builder to output `raw_scan_pose_map.pcd`, `deskewed_map.pcd`, `voxelized_map.pcd`, `cleaned_map.pcd`, evidence CSV, and JSON metrics.
4. Measure point/voxel compression, cleanup reasons, wall/ground retention, ghosting proxy, elapsed time, peak RSS, and immutable-source hashes.

### Task 4: Existing PR18 loop-candidate and geometry smoke tool

**Files:**
- Create: `src/ultra_fusion_nav/uf_relocalization/src/offline_loop_smoke.cpp`
- Create: `src/ultra_fusion_nav/uf_relocalization/test/test_offline_loop_smoke.cpp`
- Modify: `src/ultra_fusion_nav/uf_relocalization/CMakeLists.txt`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/uf_map_maintenance/keyframes.py`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/tools/export_offline_keyframes`
- Create: `src/ultra_fusion_nav/uf_map_maintenance/test/test_keyframes.py`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/CMakeLists.txt`

1. Write tests for deterministic pose-spaced keyframe selection and exclusion of recent frames.
2. Export deskewed keyframe PCDs and metadata from the immutable archive.
3. Add a small C++ CLI that reuses `compute_esf_descriptor`, `StaticKeyframeDatabase`, and PR18 point-to-plane/GICP registration; report descriptor rank, convergence, overlap, inliers, RMSE, transform, and acceptance reason as JSON.
4. Test with deterministic synthetic revisit and non-revisit clouds.

### Task 5: Run recovered data end to end

**Artifacts outside Git:**
- Source extraction: use the read-only BAG-AUDIT-001 extraction root.
- Derived workspace: use a separate OFFLINE-MAP-REAL-003B output root outside Git.

1. Normalize the PointCloud2 LiDAR bag and verify point counts, timestamps, line/tag, hashes, and absence of FAST-LIO field warnings.
2. Replay normalized LiDAR plus original IMU through the existing `lidar_type: 1` FAST-LIO configuration without estimator tuning; record `/Odometry`.
3. Materialize scans and full pose trajectory; build all four map products.
4. Run the keyframe exporter and loop smoke on automatically retrieved non-recent candidates.
5. Record exact metrics and failures; do not substitute the known revisit pair as a hard-coded success.

### Task 6: Full verification, report, and local commit

**Files:**
- Create: `docs/OFFLINE_MAP_REAL_003B.md`
- Modify: `src/ultra_fusion_nav/uf_map_maintenance/README.md`

1. Run targeted pytest, all package tests, `colcon build`, `colcon test`, `colcon test-result --verbose`, syntax/static checks, and `git diff --check`.
2. Document the recovered bag provenance, adapter contract, pose generation, deskew, four-map comparison, loop smoke, limitations, and next global-pose-graph boundary.
3. Confirm generated bags/PCDs/logs remain outside Git and scan the diff for large files, absolute local paths, credentials, and unrelated changes.
4. Create clear local commit(s); leave the worktree clean. Do not push.
