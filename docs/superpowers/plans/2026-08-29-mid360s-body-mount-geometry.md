# MID360S Body Mount Geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement one validated real-hardware geometry contract for MID360S IMU normalization, D435i/LiDAR closure validation, and a type-preserving composite self-body filter without modifying raw Livox topics or FAST-LIO's internal extrinsic.

**Architecture:** A pure Python geometry-contract module loads the one hardware YAML, validates transform provenance/completeness, and generates parameters for consumers. The existing sensor relay and fault-injection paths share one IMU normalization function. The C++ body filter retains its PointCloud2 legacy path and adds a Livox CustomMsg path plus composite box/cylinder geometry and fail-open forwarding.

**Tech Stack:** ROS 2 Humble, Python 3, PyYAML, NumPy, C++17, rclcpp, sensor_msgs, livox_ros_driver2, ament_cmake, pytest, GoogleTest.

**Spec:** `docs/MID360S_D435I_GEOMETRY_CONTRACT.md`

## Global Constraints

- `/livox/lidar` and `/livox/imu` remain immutable raw topics.
- Do not create `/livox/lidar_imu`.
- Do not modify FAST-LIO `lidar_to_imu` translation or rotation.
- Use `R_body_lidar = R_y(+15 deg)`; the former 20-degree draft is invalid.
- Do not substitute zero for the missing `t_body_lidar`.
- Hardware body removal remains fail-open and disabled until translation and CAD primitives are complete.
- Simulation legacy AABB behavior remains unchanged.
- No push or merge; produce one local commit after fresh verification.

---

### Task 1: Canonical geometry contract

**Files:**
- Create: `src/ultra_fusion_nav/uf_sensor_pipeline/uf_sensor_pipeline/geometry_contract.py`
- Create: `src/ultra_fusion_nav/uf_sensor_pipeline/config/real_mid360s_d435i_geometry.yaml`
- Create: `src/ultra_fusion_nav/uf_sensor_pipeline/test/test_geometry_contract.py`
- Modify: `src/ultra_fusion_nav/uf_sensor_pipeline/setup.py`

**Interfaces:**
- Produces: `load_geometry_contract(path)`, `imu_parameters(contract)`, `body_filter_parameters(contract)`, and `closure_report(contract)`.
- Missing body translation is represented by YAML `null` and a structured `INCOMPLETE` closure result.

- [x] Write tests for exact 15-degree rotation, calibration quaternion/matrix validation, inverse round-trip, missing-translation status, and rejection of malformed rotations.
- [x] Run the focused pytest and verify failures arise because the module/config do not exist.
- [x] Implement the minimal immutable dataclasses, parser and validation functions.
- [x] Run the focused pytest and verify all tests pass.
- [x] Refactor only after the focused tests remain green.

### Task 2: MID360 IMU normalization and routing

**Files:**
- Modify: `src/ultra_fusion_nav/uf_sensor_pipeline/uf_sensor_pipeline/fault_models.py`
- Modify: `src/ultra_fusion_nav/uf_sensor_pipeline/uf_sensor_pipeline/sensor_relay_manager.py`
- Modify: `src/ultra_fusion_nav/uf_sensor_pipeline/uf_sensor_pipeline/fault_injector.py`
- Modify: `src/ultra_fusion_nav/uf_sensor_pipeline/uf_sensor_pipeline/fcu_observation_bridge.py`
- Modify: `src/ultra_fusion_nav/uf_sensor_pipeline/launch/sensor_pipeline.launch.py`
- Modify: `src/ultra_fusion_nav/uf_sensor_pipeline/config/real_mid360_imu_units.yaml`
- Modify: existing sensor-pipeline tests and `src/multi_slam_uav_sim/test/test_mid360_imu_route_contract.py`.

**Interfaces:**
- Consumes: rotation and topic parameters returned by Task 1.
- Produces: `/sensors/imu` in `base_link`, m/s^2 and rad/s, with vectors and known covariances rotated and orientation explicitly unavailable.

- [x] Replace the invalid 20-degree assertions with tests for the 15-degree static-gravity sign, covariance rotation, timestamp preservation, orientation-unavailable semantics, unchanged input messages, and identical production/fault-injection normalization.
- [x] Add launch-contract tests proving `/livox/imu` is not remapped and the real profile cannot be overridden back to acceleration scale 1.0.
- [x] Run focused tests and verify the intended failures.
- [x] Implement the minimal shared normalizer/routing changes and remove the stale `/livox/lidar_imu` launch assumption without changing FAST-LIO's internal extrinsic.
- [x] Run focused tests and verify all tests pass.

### Task 3: Composite and Livox CustomMsg body filter

**Files:**
- Modify: `src/ultra_fusion_nav/uf_pointcloud_body_filter_cpp/include/uf_pointcloud_body_filter_cpp/pointcloud_filter.hpp`
- Modify: `src/ultra_fusion_nav/uf_pointcloud_body_filter_cpp/src/pointcloud_filter.cpp`
- Modify: `src/ultra_fusion_nav/uf_pointcloud_body_filter_cpp/src/pointcloud_body_filter_node.cpp`
- Modify: `src/ultra_fusion_nav/uf_pointcloud_body_filter_cpp/test/test_pointcloud_filter.cpp`
- Modify: `src/ultra_fusion_nav/uf_pointcloud_body_filter_cpp/test/test_body_filter_node.py`
- Modify: `src/ultra_fusion_nav/uf_pointcloud_body_filter_cpp/CMakeLists.txt`
- Modify: `src/ultra_fusion_nav/uf_pointcloud_body_filter_cpp/package.xml`

**Interfaces:**
- Consumes: flattened box/cylinder primitives and completeness/enabled flags returned by Task 1.
- Produces: unchanged legacy PointCloud2 filtering and an optional type-preserving Livox CustomMsg filtered copy.

- [x] Write GoogleTests for union-of-box/cylinder inclusion, rotated primitives, gaps between components, near-wall/ground preservation, proper-rotation rejection, disabled pass-through and invalid-geometry fail-open policy.
- [x] Write a ROS smoke test proving retained CustomMsg points preserve coordinates, `offset_time`, `reflectivity`, `tag`, and `line`, and that no input mutation or scan drop occurs.
- [x] Run focused tests and verify the intended failures.
- [x] Implement composite geometry, CustomMsg filtering and fail-open publishing with bounded diagnostics.
- [x] Run focused GoogleTest and ROS smoke suites until green.

### Task 4: D435i closure, TF ownership and production guards

**Files:**
- Modify: `src/mid360_reliable_mapper/launch/real_mid360_fastlio.launch.py`
- Modify: `src/mid360_reliable_mapper/config/mid360_mount_extrinsic.yaml`
- Modify: `src/mid360_reliable_mapper/README.md`
- Inspect only: `src/ultra_fusion_nav/uf_backend_fusion/config/online_backend.yaml`
- Inspect only: `src/ultra_fusion_nav/uf_shared_mapping/config/shared_mapping.yaml`
- Add or modify contract tests in the owning packages.

**Interfaces:**
- Consumes: Task 1 completeness and closure results.
- Produces: canonical `livox_frame` naming, no raw IMU remap, no hardware body/camera TF when required translations are absent, and explicit simulation-only labels on existing visual transforms.

- [x] Write tests that reproduce the stale `/livox/lidar_imu`, ambiguous sensor-frame naming, duplicated hardware transform and incomplete-TF hazards.
- [x] Run them and verify the intended failures.
- [x] Apply the smallest launch/config/documentation changes that enforce one publisher/one frame contract while preserving simulation defaults.
- [x] Run the focused contract tests and verify all pass.

### Task 5: Full verification and local commit

**Files:**
- Update: `docs/MID360S_D435I_GEOMETRY_CONTRACT.md` only if implementation details differ from the approved contract.

**Interfaces:**
- Produces: a clean local branch at one reviewed commit.

- [x] Build the affected packages with `colcon build` using isolated build/install/log directories.
- [x] Run affected package tests, full xUnit checks, Python/YAML/XML/Shell checks, and `git diff --check`.
- [x] Run a launch/static-gravity smoke test and a Livox CustomMsg body-filter smoke test.
- [x] Audit the final diff for raw topic mutation, guessed translations, 20-degree remnants, giant real-hardware AABB, generated artifacts and unrelated changes.
- [x] Commit only the audited files locally and verify `git status --short --branch` is clean.
